# Exploration

How the three drones decide where to go: frontier detection on the fused
map, Algorithm 1 (greedy no-overlap frontier assignment), A* path planning
and pure-pursuit flight. The formulas live in `coordination/algorithms.py`
and run in two ROS 2 nodes:

* `coordination/task_allocator.py`: one instance for the swarm, publishes a goal per drone on `/{ns}/cmd_pose` at 1 Hz.
* `coordination/flight_controller.py`: one instance per drone, turns that goal into `/{ns}/cmd_vel`.

The map these operate on is described in [`mapping.md`](mapping.md). See
[`algorithms.md`](algorithms.md) for a compact reference and
[`../README.md`](../README.md) for the system overview.

## 1. Frontier Definition

> A frontier is a boundary between known free space and unknown space.

On the fused grid $M$ (`/global_map`), a frontier cell is a FREE cell with
at least one UNKNOWN cell among its 8 neighbours:

$$F = \{c \mid M(c) = 0 \ \wedge\ \exists n \in N_8(c) : M(n) = -1\}$$

Cells outside the grid count as occupied, not unknown, so the grid border
never produces frontiers.

## 2. Valid Frontier Cells

Before clustering, `task_allocator` masks out frontier cells that the
drones could not safely reach or should not chase:

* cells within 0.4 m (`obstacle_clearance`) of a mapped obstacle
  (obstacles inflated by $0.4 / 0.2 = 2$ cells);
* cells outside the operating area, the arena bounds
  $[-12, 12] \times [-9, 11]$ m shrunk by the same 0.4 m margin. This drops
  frontiers seen through gaps in a partially mapped outer wall.

## 3. Frontier Clustering

`extract_frontier_clusters()` groups the valid frontier cells into
8-connected components (BFS flood fill, the same connectivity as
detection) and drops clusters smaller than `min_cluster_size` = 5 cells.
Each cluster yields:

| Field | Meaning |
|---|---|
| `centroid` | mean position of the cluster cells |
| `target` | the cluster cell closest to the centroid, so the goal is always a known free cell |
| `size` | number of cells |
| `gain` | information-gain proxy: frontier length in metres, $\text{IG}(f) = \text{size} \times 0.2$ |

## 4. Utility and Coordinated Information Gain

For drone $i$ at $p_i$ and frontier target $f$:

$$U_i(f) = \alpha \cdot \text{IG}(f) \cdot \text{discount}_i(f) - \beta \cdot \lVert p_i - f \rVert,
\qquad \alpha = \beta = 1$$

The discount lowers the gain of frontiers that a teammate will sense
anyway:

$$\text{discount}_i(f) = \prod_{k} \min\!\left(1, \frac{\lVert f - q_k \rVert}{r_s}\right),
\qquad r_s = 5\ \text{m}$$

where $q_k$ runs over every other drone's current position and every other
drone's locked target. A frontier 2.5 m from a teammate keeps half its
gain; one 5 m or more away keeps all of it. This follows the coordination
idea of Burgard et al. (*Coordinated Multi-Robot Exploration*, IEEE T-RO
2005): reduce the utility of targets near areas other robots will cover,
which spreads the swarm out. The unit test
`test_sensing_discount_spreads_the_swarm` checks that it changes the choice
compared with plain nearest-frontier selection.

Euclidean distance is used for the cost, not A* path length.

## 5. Algorithm 1: Greedy No-Overlap Assignment

`greedy_assign()` assigns targets to the drones that need one, given the
drones that keep their current target (their *locked* targets):

```text
locks      = targets held by drones that keep their goal
candidates = clusters not within 1.0 m of a blacklisted point
while drones still need a target and candidates remain:
    best = None
    for each drone i needing a target:
        for each candidate f:
            reject f if ||f - p_j|| < 2.0 m for any other drone j
            reject f if ||f - lock_j|| < 2.0 m for any other drone's lock
            u = U_i(f) with the sensing discount
            keep the globally best (u, i, f)
    if best is None: stop
    assign f to i, lock f, remove i
    drop every candidate within 2.0 m of f
```

The exclusion radius $r_{\text{lock}} = 2.0$ m (launch argument
`lock_radius`) guarantees that no two drones' targets are closer than
2 m and that no drone is sent onto a teammate. Taking the globally best
pair first, instead of iterating drones in a fixed order, avoids a fixed
drone always getting first choice.

## 6. Target Lifecycle

Each 1 Hz allocation cycle in `task_allocator`:

1. **Airborne check.** Drones below 1.0 m are not yet tasked.
2. **Arrival.** Within 0.8 m of its target, the drone's lock is released
   and it asks for a new target.
3. **Progress watchdog.** If the drone has not got 0.5 m closer to its
   target within 20 s, the target is blacklisted as unreachable (no new
   target within 1.0 m of it is assigned again) and the lock is released.
4. **Persistence.** A target is kept while a frontier cluster target still
   lies within 2.0 m of it. After at least 5 s (`min_commit_time`), a
   target whose frontier has disappeared is released. The 5 s minimum stops
   small frontier shifts from making the drone zig-zag.
5. **Assignment.** Algorithm 1 runs for the drones that need a target.
6. **Publish.** Every active target goes out on `/{ns}/cmd_pose` at
   $z = 1.5$ m, together with RViz2 markers for frontiers
   (`/swarm/markers/frontiers`) and exclusion zones
   (`/swarm/markers/locked_regions`).

## 7. Exploration Completion

Exploration is complete when no valid frontier cluster remains and no
drone holds a target. The allocator logs **EXPLORATION COMPLETE** and stops
publishing goals, and the drones hold position (see §9). If new frontiers
appear later, it logs that exploration is resuming and tasks the drones
again.

There is no coverage, time or distance stop condition in the stack.
Coverage is reported as a metric (see [`mapping.md`](mapping.md)) but does
not end the mission.

## 8. Path Planning: A*

`flight_controller` replans at 1 Hz, and immediately when the goal moves
by more than 0.3 m, using `plan_path()` on `/global_map`:

* Occupied cells ($\ge 65$) are inflated by 0.5 m (`inflation_radius`) and are impassable.
* Unknown cells are traversable at double cost (`unknown_cost` = 2), so the
  drone can plan into unexplored space but prefers known free space.
* 8-connected moves cost $1$ or $\sqrt 2$ times the cell cost; the
  heuristic is the Euclidean distance in cells.
* A start inside an inflated band (e.g. right after take-off near a wall)
  snaps to the nearest unblocked cell within 1.0 m, and the goal snaps within
  1.5 m. This also covers frontier targets, which only need 0.4 m clearance,
  that land inside the 0.5 m planning inflation.
* The cell path is shortened by line-of-sight smoothing: from each
  waypoint, jump to the farthest later cell reachable in a straight line
  that crosses no blocked cell (checked every quarter cell so diagonals
  cannot clip corners).

If no path exists, the drone gets an empty path and hovers; the
allocator's 20 s watchdog then blacklists the target.

## 9. Path Following

The controller runs at 20 Hz with three states: **ARMING** (publish
`/{ns}/enable` until the Gazebo bridge is listening), **TAKEOFF** (climb to
1.5 m) and **MISSION**. In MISSION:

* **Pure pursuit:** the carrot is the first path point at least 0.8 m
  (`lookahead`) away.
* **Speed:** $v = \min(0.8,\ 0.8\, d_{\text{goal}} + 0.15)$ m/s, scaled by
  $\max(0, \cos e_\psi)^2$, where $e_\psi$ is the heading error. The drone
  turns before it translates, so the forward depth camera always sees
  where it is flying.
* **Yaw rate:** $1.5\, e_\psi$, clipped to ±0.8 rad/s.
* **Altitude hold:** $v_z = 1.2\,(1.5 - z)$, clipped to ±0.6 m/s.
* **Separation:** inside 1.5 m of another drone, a repulsion of magnitude
  $(1.5 - d)/1.5$ m/s pushes the drone directly away from it. The total
  horizontal speed is then capped at 0.8 m/s.
* **Hold:** with no `cmd_pose` for 2 s, the goal is dropped and the drone
  hovers.

The world-frame velocity is rotated into the yaw-aligned body frame and
sent to Gazebo's `MulticopterVelocityControl` on `/{ns}/cmd_vel`.

## 10. Exploration Loop

```text
every 0.2 s   depth_mapper     depth + odom -> log-odds local map (published at 2 Hz)
every 0.5 s   map_merger       local maps -> /global_map (conservative fusion)
every 1 s     task_allocator   frontiers -> clusters -> Algorithm 1 -> /{ns}/cmd_pose
every 1 s     flight_controller  A* on /global_map -> path
every 0.05 s  flight_controller  pure pursuit + separation -> /{ns}/cmd_vel
```

The stages are decoupled by topics, so each can be replaced on its own:
the utility, the clustering or the planner can change without touching
the others, and the pose source can change from ground truth to an
estimator without changing this algorithm (see
[`localization.md`](localization.md)).

## 11. Tests

`coordination/test_swarm_coordination.py` exercises the production
functions directly: 8-neighbour frontier extraction (target is a free
cell, gain = length in metres), Algorithm 1 (targets ≥ 2 m apart,
rejection near other drones, locks and blacklisted points), the sensing
discount, the utility trade-off, and A* around an inflated wall
(path stays clear of the inflation; `None` when the gap is closed).

## 12. Earlier Implementation

`src/exploration` (C++ `frontier_detector`, `frontier_clustering`,
`frontier_selector`) is the earlier single-drone implementation that sent
one waypoint at a time to PX4. It is not built or launched by the current
launcher; `task_allocator` and `flight_controller` replace it.
