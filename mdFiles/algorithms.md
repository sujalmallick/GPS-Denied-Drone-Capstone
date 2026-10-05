# Algorithms

Per-algorithm reference for the swarm pipeline as implemented. All
algorithms except pose and control live in `coordination/algorithms.py`
(pure numpy, no `rclpy`), are unit-tested in
`coordination/test_swarm_coordination.py`, and are called unchanged by the
ROS 2 nodes. [`mapping.md`](mapping.md), [`exploration.md`](exploration.md)
and [`localization.md`](localization.md) give the narrative context;
[`../README.md`](../README.md) has the system overview.

Complexities use $W \times H$ for the grid (200 × 200), $K$ for drones (3),
$F$ for frontier cells and $C$ for clusters.

## Parameters at a Glance

| Constant | Value | Where |
|---|---|---|
| Grid | 200 × 200 cells, 0.2 m, origin $(-20, -20)$ | `depth_mapper` |
| Depth range used | 0.35 – 8 m, obstacle band $0.3 \le z \le 2.7$ m | `depth_mapper` |
| Log-odds | $l_{\text{occ}} = 0.85$, $l_{\text{free}} = -0.4$, clamp $[-2.0, 3.5]$, thresholds occ $\ge 0.5$ / free $\le -0.3$ | `OccupancyLogOdds` |
| Teammate filter | 0.6 m | `depth_mapper` |
| Fusion occupied threshold | 65 | `OCCUPIED_THRESHOLD` |
| Frontier clearance | 0.4 m | `task_allocator` |
| Min cluster size | 5 cells | `task_allocator` (function default 3) |
| Utility weights | $\alpha = \beta = 1$ | `task_allocator` |
| Sensor radius (discount) | 5.0 m | `task_allocator` |
| Lock (exclusion) radius | 2.0 m | `task_allocator`, launch arg `lock_radius` |
| Arrival radius | 0.8 m | `task_allocator` |
| Progress watchdog | 20 s without getting 0.5 m closer; blacklist radius 1.0 m | `task_allocator` |
| Min commit time | 5 s | `task_allocator` |
| A* inflation / unknown cost | 0.5 m / 2× | `flight_controller`, `plan_path` |
| Cruise speed / lookahead | 0.8 m/s / 0.8 m | `flight_controller` |
| Separation radius | 1.5 m | `flight_controller` |
| Command timeout | 2 s | `flight_controller` |
| Altitude | 1.5 m | launch arg `altitude` |

## 1. Pose Source

* **Input:** `/{ns}/odom` (`nav_msgs/Odometry`, world frame `map`, 30 Hz).
* **Output:** Position and orientation used by every node.
* **Logic:** Currently Gazebo's `OdometryPublisher`, i.e. exact ground
  truth. No estimator runs. See [`localization.md`](localization.md) for
  the GPS-denied replacement path (VIO / VSLAM / PX4 EKF2 vision fusion).
* **Limitations:** Perfect pose hides the drift and frame-alignment
  problems a real estimator introduces.

## 2. Depth Image to Rays (`depth_image_to_rays`)

* **Input:** `32FC1` depth image, horizontal FOV (1.5184 rad), camera
  offset (0.13 m forward), drone rotation and position.
* **Output:** Sensor origin, per-column azimuth and free length, obstacle
  hit points.
* **Logic:** Sample every `stride`-th pixel (about 80 columns per frame);
  back-project with $f_x = \frac{w/2}{\tan(\text{hfov}/2)}$; drop returns
  below 0.35 m and at the far clip; keep points with
  $0.3 \le z \le 2.7$ m as obstacles; the nearest obstacle per column is
  the hit; free space runs to the hit or to 8 m. Floor-only columns are
  free only up to the farthest floor point seen.
* **Complexity:** $O(V \cdot U)$ for $V \times U$ sampled pixels.
* **Assumptions:** Planar (z-buffer) depth, camera looking along body $+x$.
* **Limitations:** Collapses 3D to one range per column; overhangs outside
  the band are invisible.
* **Tests:** `test_depth_rays_map_wall_ahead`, `test_depth_rays_ignore_own_propellers`.

## 3. Log-Odds Occupancy (`OccupancyLogOdds`)

* **Input:** Rays from §2.
* **Output:** Local grid with values $\{-1, 0, 100\}$, published on `/{ns}/map` at 2 Hz.
* **Logic:** Cells along each ray (sampled every 0.1 m, stopping half a
  cell before the hit) get $l_{\text{free}} = -0.4$; hit cells get
  $l_{\text{occ}} = +0.85$; each cell updates at most once per frame, and
  values are clipped to $[-2.0, 3.5]$. Publish 100 if $l \ge 0.5$, 0 if
  $l \le -0.3$, otherwise −1. Hits within 0.6 m of a teammate are dropped
  before integration; frames below 0.8 m altitude are skipped.
* **Complexity:** $O(R \cdot S)$ per frame for $R$ rays of $S$ samples.
* **Limitations:** Static-world assumption; no time alignment between the
  depth frame and the pose (the latest odometry is used).

## 4. Conservative Map Fusion (`fuse_occupancy`)

* **Input:** $K$ local grids with identical geometry.
* **Output:** `/global_map` (latched, 2 Hz).
* **Logic:**

$$M_{\text{global}}(c) = \begin{cases}
100 & \text{if } \exists i : M_i(c) \ge 65 \quad \text{(obstacle priority)} \\
0 & \text{if every drone that observed } c \text{ reports } 0 \text{ (at least one did)} \\
-1 & \text{otherwise}
\end{cases}$$

* **Complexity:** $O(K \cdot W \cdot H)$.
* **Assumptions:** All local maps share one world frame (true while pose is
  ground truth). Maps with a different geometry are rejected.
* **Limitations:** Any misalignment between drones survives as phantom
  obstacles. That is safe, but it can block passages.
* **Tests:** `test_conservative_map_fusion`.

## 5. Swarm Metrics (`coverage_pct`, `overlap_rate`)

* **Coverage:** known cells inside the arena $[-12, 12] \times [-9, 11]$ m ÷ arena cells × 100 %.
* **Overlap:** cells observed by ≥ 2 drones ÷ cells observed by ≥ 1 drone × 100 %.
* **Complexity:** $O(K \cdot W \cdot H)$.
* **Tests:** `test_coverage_and_overlap_metrics`, `test_region_mask_matches_bounds`.

## 6. Frontier Detection (`frontier_mask`)

* **Input:** Fused grid.
* **Output:** Boolean frontier mask.
* **Logic:** $F = \{c \mid M(c)=0 \wedge \exists n \in N_8(c): M(n)=-1\}$,
  computed with 8 shifted array comparisons; the border is padded as
  occupied. `task_allocator` then removes cells within 0.4 m of an obstacle
  or outside the arena shrunk by 0.4 m.
* **Complexity:** $O(W \cdot H)$ per cycle (full rescan, vectorized).
* **Future improvements:** Incremental updates over changed cells only.

## 7. Frontier Clustering (`extract_frontier_clusters`)

* **Input:** Frontier mask.
* **Output:** Clusters with `centroid`, `target` (cluster cell nearest the
  centroid), `size`, and `gain` = size × resolution (frontier length in m).
* **Logic:** 8-connected BFS flood fill (the same connectivity as
  detection); drop clusters below 5 cells.
* **Complexity:** $O(F)$.
* **Limitations:** Long L-shaped frontiers become one cluster whose target
  may sit far from most of its cells.
* **Tests:** `test_frontier_extraction_uses_8_neighbourhood`.

## 8. Utility with Information-Gain Discount (`frontier_utility`, `sensing_discount`)

* **Logic:**

$$U_i(f) = \alpha \cdot \text{IG}(f) \cdot \prod_k \min\!\left(1, \tfrac{\lVert f - q_k \rVert}{r_s}\right) - \beta \cdot \lVert p_i - f \rVert$$

  with $q_k$ every other drone's position and locked target, $r_s = 5$ m,
  $\alpha = \beta = 1$. The discount follows Burgard et al. (IEEE T-RO 2005).
* **Complexity:** $O(K)$ per (drone, frontier) pair.
* **Limitations:** IG is frontier length, not a ray-cast estimate of
  visible unknown area; the distance is Euclidean, not path length.
* **Tests:** `test_utility_trades_gain_against_distance`, `test_sensing_discount_spreads_the_swarm`.

## 9. Algorithm 1: Greedy No-Overlap Assignment (`greedy_assign`)

* **Input:** Drone positions, drones needing a target, clusters, locks held
  by other drones, blacklist.
* **Output:** `{ns: (x, y)}` assignments.
* **Logic:** Drop candidates within 1.0 m of a blacklisted point. Repeat:
  over all (drone, frontier) pairs, reject $f$ within 2.0 m of another
  drone or another drone's lock; pick the globally best $U_i(f)$; assign
  it, lock it, and drop candidates within 2.0 m of it.
* **Complexity:** $O(K^3 \cdot C)$ in the worst case. That is trivial for $K = 3$.
* **Limitations:** Greedy and one step ahead; it does not sequence several
  frontiers per drone.
* **Tests:** `test_algorithm_1_greedy_no_overlap`, `test_algorithm_1_rejects_frontiers_near_other_drones_and_locks`.

## 10. Target Management (`task_allocator`, 1 Hz)

* **Logic:** Drones below 1.0 m are not tasked. Release the lock on arrival
  ($\le 0.8$ m). Blacklist the target after 20 s without getting 0.5 m
  closer. Keep a target for at least 5 s, then release it once no
  cluster target lies within 2.0 m of it. Publish `/{ns}/cmd_pose` at
  $z = 1.5$ m.
* **Termination:** No valid cluster and no active target → log
  **EXPLORATION COMPLETE**; the drones hold position. Exploration resumes if
  frontiers reappear. There are no coverage, time or distance stop
  conditions.

## 11. A* Path Planning (`plan_path`)

* **Input:** Fused grid, start, goal.
* **Output:** Smoothed waypoint list, or `None`.
* **Logic:** Inflate cells $\ge 65$ by 0.5 m (impassable); unknown cells
  cost 2×; 8-connected moves (cost 1 or $\sqrt 2$ times cell cost),
  Euclidean heuristic. Snap the start to free space within 1.0 m and the
  goal within 1.5 m. Then line-of-sight smoothing, with segments checked
  every quarter cell.
* **Complexity:** $O(N \log N)$ for $N = W \cdot H$ cells; inflation
  $O(W \cdot H \cdot r^2)$ for radius $r$ cells.
* **Tests:** `test_astar_avoids_inflated_wall`.

## 12. Path Following and Separation (`flight_controller`, 20 Hz)

* **Logic:** Pure pursuit with a 0.8 m lookahead. Speed
  $\min(0.8,\ 0.8\, d_{\text{goal}} + 0.15)\cdot\max(0,\cos e_\psi)^2$.
  Yaw rate $1.5\, e_\psi$ clipped to ±0.8 rad/s. Altitude
  $v_z = 1.2(1.5 - z)$ clipped to ±0.6 m/s. Inside 1.5 m of another drone
  add a repulsion $(1.5 - d)/1.5$ m/s away from it, then cap the speed at
  0.8 m/s. Hold position after 2 s without `cmd_pose`. Output is a
  body-frame twist on `/{ns}/cmd_vel` for Gazebo's
  `MulticopterVelocityControl`.
* **Complexity:** $O(K)$ per tick.
* **Limitations:** Separation is purely reactive and planning ignores
  teammates, so two drones can momentarily push each other off their paths.
  The flight stack is Gazebo's velocity controller, not PX4.

## Earlier Implementations

The C++ packages in `src/mapping` and `src/exploration` implement the
earlier single-drone, PX4-SITL version of mapping and frontier selection.
They are not built by the current launcher and are not described here.
