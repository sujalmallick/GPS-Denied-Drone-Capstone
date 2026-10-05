# Troubleshooting

General-purpose guide organized by symptom. Where a fix depends on the
exact installed version of PX4, ROS 2, Gazebo, or the uXRCE-DDS agent, this
is marked **VERSION-DEPENDENT** rather than stated as a fixed procedure —
verify against the installed versions before applying.

## PX4 Not Starting

* Confirm the PX4 SITL build target matches the installed Gazebo version
  (Gazebo Harmonic support is **VERSION-DEPENDENT** on the PX4 release in
  use — verify compatibility before debugging further).
* Check for a stale PX4 build; a clean rebuild resolves many "won't start"
  issues after a toolchain or dependency update.
* Confirm no leftover PX4 SITL process is holding required ports from a
  previous run.

## Gazebo Not Starting

* Confirm the world file referenced in the launch command exists at the
  expected path under `simulation/worlds/`.
* Check for GPU/rendering driver issues if Gazebo fails specifically at the
  rendering stage rather than at world load.
* Confirm Gazebo Harmonic (not an incompatible Gazebo Classic install) is
  the one actually being invoked, if both are present on the system.

## ROS 2 Package Build Failures

* Confirm the package's `package.xml`/`CMakeLists.txt` dependencies are
  installed (`rosdep install` against the workspace is the standard first
  step).
* Confirm the ROS 2 Humble environment is sourced before building with
  `colcon`.
* Isolate the failing package with `colcon build --packages-select
  <name>` rather than rebuilding the whole workspace.

## DDS Communication Failures

* Confirm the uXRCE-DDS agent is running and pointed at the correct
  transport (serial vs. UDP) matching how PX4 SITL is configured — this
  pairing is **VERSION-DEPENDENT** on the PX4 release's default transport.
* Confirm ROS 2's DDS domain ID is consistent across all nodes; a mismatch
  silently prevents nodes from seeing each other without an explicit error.
* Confirm firewall/localhost networking isn't blocking DDS discovery
  traffic if running across multiple processes/containers on the same
  machine.

## PX4 Topics Unavailable in ROS 2

* Confirm the uXRCE-DDS agent successfully established a session with the
  PX4 SITL client (check agent logs for a connected-client message).
* Confirm the specific topic is included in the active uXRCE-DDS
  topic-mapping configuration — not all PX4 uORB topics are bridged to
  ROS 2 by default. **VERSION-DEPENDENT** — check the mapping file shipped
  with the installed PX4/uXRCE-DDS version.
* Use `ros2 topic list` to confirm what is actually visible before assuming
  a bridging configuration problem vs. a subscriber-side problem.

## Offboard Control Problems

* Confirm setpoints are being streamed at or above PX4's required minimum
  rate before attempting the mode switch into offboard — PX4 will reject
  or exit offboard mode if setpoints stop arriving frequently enough.
  **VERSION-DEPENDENT** exact rate — confirm against the installed PX4
  version (see [`ros2-architecture.md`](ros2-architecture.md) §2.8).
* Confirm the drone is armed before or as part of the offboard mode
  transition, per PX4's expected arming/mode sequence.
* Confirm the setpoint's coordinate frame matches what PX4 expects for
  offboard position setpoints (see coordinate-frame problems below).

## Drone Not Moving

* Confirm PX4 is actually armed and in offboard mode (not still in a
  manual/hold mode) — a very common cause of "waypoint sent but nothing
  happens."
* Confirm the PX4 Interface Node is receiving a valid waypoint at all
  (log it) before assuming the flight-control side is at fault.
* Confirm Gazebo physics is not paused.

## Map Not Updating

* Confirm the Mapping Node is actually receiving both a current pose
  estimate and depth/sensor data — a stale or missing pose is a common
  silent cause (see [`system-design.md`](system-design.md) §8, "pose
  estimate unavailable or stale").
* Confirm depth sensor topic is publishing (`ros2 topic hz`) before
  assuming a mapping-logic bug.
* Confirm the map is not being reset/reinitialized unintentionally on each
  update cycle.

## Frontier Detector Finding Nothing

* Confirm the occupancy grid actually contains a mix of FREE, OCCUPIED,
  and UNKNOWN cells — if the map is entirely UNKNOWN (nothing observed
  yet) or entirely FREE/OCCUPIED (fully explored), zero frontiers is the
  *correct* result, not a bug.
* Confirm the adjacency/connectivity definition used in detection isn't
  overly strict in a way that misses valid boundary cells (see
  [`exploration.md`](exploration.md) §2).
* Confirm map bounds/edges aren't being treated as artificial frontiers or
  artificial non-frontiers incorrectly — edge-of-map handling is a common
  off-by-one source.

## Invalid Waypoint

* Confirm the representative point chosen from the cluster is being
  correctly snapped to a FREE cell, not left on/near the FREE/UNKNOWN
  boundary (see [`exploration.md`](exploration.md) §6).
* Confirm the clearance check is using the same, current occupancy grid
  the rest of the pipeline is using, not a stale snapshot.
* Confirm the waypoint's coordinate frame matches what downstream (PX4
  Interface Node) expects.

## VSLAM Failure

* Confirm the environment has sufficient visual texture — texture-poor
  Gazebo worlds are a known stress case for VSLAM/VIO tracking (see
  [`simulation.md`](simulation.md) §5).
* Confirm camera and IMU data are correctly time-synchronized; VIO is
  highly sensitive to timestamp misalignment between the two streams.
* Confirm this failure is isolated to localization and not actually a
  downstream mapping/exploration bug being misattributed — this is
  exactly the class of ambiguity the ground-truth-first strategy (see
  [`localization.md`](localization.md) §4) is designed to avoid. If in
  doubt, re-run the same scenario in ground-truth mode to confirm the rest
  of the pipeline is healthy.

## RViz Problems

* Confirm the RViz2 fixed frame is set to the correct map frame (see
  [`system-design.md`](system-design.md) §6) — a mismatched fixed frame is
  the most common cause of "nothing shows up" in RViz2.
* Confirm the relevant topics (occupancy grid, frontier markers, pose) are
  actually being published before assuming a display-configuration issue.

## Coordinate-Frame Problems

* Confirm all components agree on the same map frame name and definition
  (see [`system-design.md`](system-design.md) §6 and
  [`localization.md`](localization.md) §7) — this is a common source of
  silent misalignment (map appears "shifted" or "rotated" relative to the
  real world, sensor data appears offset from the drone's visualized
  position).
* Confirm the transform tree (map → odom → base_link, or equivalent) is
  fully published and consistent; a missing or duplicate transform
  publisher is a common cause of intermittent frame-lookup failures.
* When switching from ground-truth pose to VSLAM/VIO (M11), re-verify the
  frame contract hasn't silently changed — see
  [`localization.md`](localization.md) §6 on why this interface must stay
  fixed across that transition.
