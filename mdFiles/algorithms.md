# Algorithms

Per-algorithm specification for every algorithmic component in the
pipeline. See [`exploration.md`](exploration.md) and
[`mapping.md`](mapping.md) for the narrative/architectural context this
summarizes into an implementation-focused reference.

## 1. Pose Estimation

* **Input:** Ground-truth pose topic (dev mode) OR camera + IMU streams
  (VSLAM/VIO mode, from M11).
* **Output:** Pose estimate (position + orientation) in the map frame.
* **Logic:** Dev mode simply republishes/relays Gazebo's ground-truth pose.
  VSLAM/VIO mode runs visual-inertial odometry (e.g., ORB-SLAM3) to fuse
  camera and IMU data into a pose estimate; see
  [`localization.md`](localization.md) for candidate selection.
* **Complexity:** Ground truth: O(1) per update. VSLAM/VIO: dominated by
  the chosen library's per-frame feature extraction/tracking cost —
  library-dependent, not specified here.
* **Assumptions:** A single pose source is active at a time; both modes
  publish on the same topic/frame contract.
* **Limitations:** Ground truth is unrealistic (perfect) and only valid for
  development. VSLAM/VIO is subject to drift and can lose tracking in
  low-texture or fast-motion scenarios.
* **Future improvements:** Loop closure / full SLAM (not just VIO) to
  bound drift; sensor fusion with additional modalities if added later.

## 2. Occupancy Mapping

* **Input:** Pose estimate, depth/point-cloud observation.
* **Output:** Updated occupancy grid (FREE/OCCUPIED/UNKNOWN per cell).
* **Logic:** Transform observation into map frame using current pose;
  ray-cast from sensor origin to each observed point, marking traversed
  cells FREE and the terminal cell OCCUPIED. See
  [`mapping.md`](mapping.md) §2 for full pseudocode.
* **Complexity:** O(N) per observation, where N is the number of
  observed points times the average ray length in cells.
* **Assumptions:** Pose is valid and time-aligned with the sensor
  observation; depth sensor provides metrically accurate range data.
* **Limitations:** Errors in pose (e.g., late-stage VSLAM drift) directly
  corrupt the map; no explicit handling of dynamic obstacles (the
  environment is assumed static within Phase 1's scope).
* **Future improvements:** Probabilistic (log-odds) cell updates instead of
  binary/ternary updates; 3D voxel mapping (see
  [`mapping.md`](mapping.md) §9); dynamic-obstacle handling.

## 3. Frontier Detection

* **Input:** Occupancy grid.
* **Output:** Set of frontier cells.
* **Logic:** A FREE cell is a frontier cell if any adjacent cell is
  UNKNOWN. See [`exploration.md`](exploration.md) §2.
* **Complexity:** O(W×H) for a W×H grid with a full scan; can be reduced to
  amortized near-O(changed cells) with an incremental approach that only
  re-checks cells whose state changed since the last update — worth
  considering if full-grid scans become a performance bottleneck.
* **Assumptions:** Grid connectivity (4- vs 8-adjacency) is fixed and
  consistent with the clustering step.
* **Limitations:** Naive full-grid scanning is wasteful on large, mostly-
  explored maps.
* **Future improvements:** Incremental frontier maintenance instead of
  full-grid rescans; spatial indexing for large maps.

## 4. Frontier Clustering

* **Input:** Set of frontier cells.
* **Output:** List of clusters (connected groups) with member cells.
* **Logic:** Connected-component labeling over the frontier-cell set using
  a fixed connectivity definition; discard clusters below
  `min_cluster_size`. See [`exploration.md`](exploration.md) §3.
* **Complexity:** O(F) for F frontier cells using a standard
  union-find or flood-fill connected-components algorithm.
* **Assumptions:** Connectivity definition matches frontier detection's
  adjacency definition.
* **Limitations:** Simple connectivity-based clustering can merge frontiers
  that are geometrically close but conceptually distinct (e.g., across a
  thin wall gap) or split frontiers that a human would consider one region
  if there's a one-cell gap.
* **Future improvements:** Distance-based or hierarchical clustering as an
  alternative to pure connectivity; merging/splitting heuristics tuned
  against real exploration runs.

## 5. Frontier Scoring

* **Input:** List of clusters, current drone pose.
* **Output:** Each cluster annotated with a scalar score.
* **Logic:** `score = information_gain - distance_cost` (optionally
  `- safety_cost`, see [`exploration.md`](exploration.md) §4.3). Both
  terms are individually swappable functions.
* **Complexity:** O(C) for C clusters, assuming O(1) or O(cluster size)
  per-cluster gain/cost computation.
* **Assumptions:** `information_gain` and `distance_cost` are expressed in
  comparable, weighted units (see weight parameters in
  [`ros2-architecture.md`](ros2-architecture.md) §2.5) so neither term
  trivially dominates.
* **Limitations:** The initial information-gain estimate (cluster size /
  adjacent unknown area) is a heuristic, not a true expected-information
  computation (e.g., ray-casting-based visibility estimation).
* **Future improvements:** Ray-casting-based information-gain estimation;
  learned or adaptive scoring weights; explicit safety-cost term.

## 6. Frontier Selection

* **Input:** Scored, filtered (valid) clusters.
* **Output:** A single selected cluster, or `None`.
* **Logic:** Argmax over score among valid clusters. See
  [`exploration.md`](exploration.md) §5.
* **Complexity:** O(C) for C valid clusters.
* **Assumptions:** Ties are broken deterministically (e.g., by cluster ID
  or centroid position) to keep runs reproducible for evaluation.
* **Limitations:** Purely greedy, single-step-ahead selection; does not
  plan multiple exploration steps ahead.
* **Future improvements:** Multi-step lookahead / sequencing of frontier
  visits; re-scoring previously-rejected clusters as the map changes.

## 7. Waypoint Generation

* **Input:** Selected cluster.
* **Output:** Validated waypoint (position + frame), or a rejection signal.
* **Logic:** Pick a representative point, verify it is FREE (or snap to the
  nearest FREE point), verify clearance from OCCUPIED cells meets a
  configured minimum. See [`exploration.md`](exploration.md) §6.
* **Complexity:** O(1) for the representative-point pick; O(k) for a local
  clearance check over a k-cell neighborhood.
* **Assumptions:** The occupancy grid used for validation is reasonably
  current (not from a stale map snapshot).
* **Limitations:** Purely local clearance check; does not verify a
  collision-free path to the waypoint, only that the waypoint itself is
  safe.
* **Future improvements:** Full path validation/planning between current
  position and the waypoint, not just endpoint validation.

## 8. Navigation

* **Input:** Validated waypoint.
* **Output:** Drone motion toward the waypoint; arrival confirmation.
* **Logic:** PX4 Interface Node streams the waypoint as an offboard
  setpoint at the rate PX4 requires; arrival is declared when the drone's
  pose is within a configured tolerance of the waypoint. See
  [`ros2-architecture.md`](ros2-architecture.md) §2.8.
* **Complexity:** O(1) per control-loop tick.
* **Assumptions:** PX4 is in offboard mode and armed; setpoints are
  streamed continuously enough to avoid PX4 falling back out of offboard
  mode.
* **Limitations:** No dynamic obstacle avoidance beyond what the
  already-validated waypoint provides; a straight-line/PX4-internal path to
  the setpoint is assumed safe because the map was checked, not because a
  path was explicitly planned.
* **Future improvements:** Explicit local path planning with obstacle
  avoidance between current position and waypoint, rather than relying on
  PX4's internal position controller alone.

## 9. Exploration Termination

* **Input:** Current exploration state (frontier set, coverage, elapsed
  time, distance traveled, safety flags).
* **Output:** Boolean exploration-complete signal.
* **Logic:** OR of configured stop conditions. See
  [`exploration.md`](exploration.md) §7 for the full condition list and
  pseudocode.
* **Complexity:** O(1) given precomputed state values.
* **Assumptions:** Each stop-condition input (coverage, time, distance,
  safety flags) is being correctly tracked by the Exploration Manager
  throughout the run.
* **Limitations:** Coverage-based stopping assumes the map's total
  explorable area is knowable/estimable; in an unbounded or very large
  world this criterion may never trigger on its own, which is why multiple
  independent stop conditions are recommended.
* **Future improvements:** Confidence-weighted coverage (accounting for
  map cells with low observation confidence); adaptive stop thresholds
  based on diminishing returns in coverage growth rate.
