# System Design

> Describes the running three-drone stack (Gazebo Harmonic + ROS 2 Jazzy, Python `coordination`
> package). Component layout and data flow are in [`architecture.md`](architecture.md); exact
> topic and parameter names are in [`ros2-architecture.md`](ros2-architecture.md).

## 1. Functional Requirements

| ID | Requirement | Status |
|---|---|---|
| FR-1 | Simulate three quadrotors with rotor physics and onboard sensors (depth camera, IMU, odometry) in a walled indoor arena. | Done: Gazebo Harmonic, `quadrotor_px4.sdf` ×3 |
| FR-2 | Estimate each drone's pose without GPS. | Partial: Gazebo odometry (ground truth) stands in; VSLAM/VIO is future work ([`localization.md`](localization.md)) |
| FR-3 | Build a 2D occupancy grid per drone from that drone's own depth images and pose. | Done: `depth_mapper` |
| FR-4 | Fuse the per-drone grids into one global grid, never overwriting an observed obstacle with free space. | Done: `map_merger`, conservative fusion |
| FR-5 | Detect frontier cells (free cells with an unknown 8-neighbour) and cluster them. | Done: `task_allocator` |
| FR-6 | Assign frontiers to drones so that no two drones target the same region (2.0 m exclusion, Algorithm 1). | Done: `greedy_assign` |
| FR-7 | Score frontiers with a replaceable utility, `α·IG·discount − β·distance`. | Done: weights and sensor radius are parameters |
| FR-8 | Plan a collision-free path to the assigned frontier on the fused map and fly it. | Done: A* + pure pursuit in `flight_controller` |
| FR-9 | Keep drones apart in flight. | Done: 1.5 m reactive separation |
| FR-10 | Detect unreachable targets and recover. | Done: 20 s progress watchdog, blacklist |
| FR-11 | Stop exploring when no worthwhile frontier is left. | Done: allocator logs EXPLORATION COMPLETE, drones hold position |
| FR-12 | Report coverage and overlap live. | Done: `/swarm/metrics/coverage_pct`, `/swarm/metrics/overlap_rate` |
| FR-13 | Provide offline evaluation (coverage, overlap, ATE). | Partial: `validation/`; ATE needs a VSLAM estimate ([`evaluation.md`](evaluation.md)) |
| FR-14 | Run the mapping and exploration code unchanged whether the pose comes from ground truth or VSLAM. | Design rule; only ground truth exists today |

Stop conditions other than "no frontiers" (coverage threshold, max time, max distance) from the
single-drone design are not implemented.

## 2. Non-Functional Requirements

| ID | Requirement |
|---|---|
| NFR-1 | Software only. No hardware is needed to run or evaluate the system. |
| NFR-2 | Autonomy code is Python (`rclpy` + numpy). Algorithms live in `coordination/algorithms.py` with no ROS dependency, so they can be unit-tested directly. |
| NFR-3 | Modular: utility function, clustering, map fusion and pose source are replaceable without rewriting the surrounding nodes. |
| NFR-4 | Every node uses simulation time (`use_sim_time`), so behaviour does not change when the simulation runs slower than real time. |
| NFR-5 | Near real time on a laptop: the `realtime` sensor profile, the 100 Hz clock relay and the opt-in RGB bridge exist to keep the three-drone stack fast enough under WSL2. On a native Linux host with an NVIDIA GPU it runs in real time. |
| NFR-6 | Reproducible: one command (`run_single_window.sh` / `.ps1`, or `docker/sim/run_docker.sh`) builds and starts everything. |
| NFR-7 | Interfaces (topics, messages, frames) are documented in [`ros2-architecture.md`](ros2-architecture.md) before more than one component relies on them. |

## 3. System Boundaries

**In scope:**
* Three simulated drones, centralized allocation, per-drone mapping and control.
* Gazebo Harmonic physics and sensors; ROS 2 Jazzy.
* 2D occupancy-grid mapping and frontier exploration at a fixed altitude (1.5 m by default).
* Local runs (Ubuntu 24.04 native or WSL2) and the cloud-GPU container
  ([`../docker/sim/README.md`](../docker/sim/README.md)).

**Out of scope for the running stack (future work):**
* PX4 SITL or any other flight stack; Gazebo's velocity controller flies the drones.
* VSLAM / VIO localization.
* Physical hardware (the `vslam/` launch files target a Jetson + Pixhawk + RealSense setup, but
  nothing in the running stack depends on them).
* 3D / voxel mapping.
* Decentralized allocation and communication-range limits; all nodes share one ROS 2 graph.

## 4. Component Responsibilities

See [`architecture.md`](architecture.md) §2. Each node owns one stage and communicates only
through topics. The one shared dependency is `coordination/algorithms.py`, which holds the pure
functions every node calls.

## 5. Interfaces

All boundaries are ROS 2 topics. There are no services or actions in the running stack.

| Boundary | Interface |
|---|---|
| Gazebo → ROS 2 | `ros_gz_bridge`: `/{ns}/odom`, `/{ns}/ground_truth_pose`, `/{ns}/imu`, `/{ns}/depth/image`, `/{ns}/depth/camera_info` |
| Gazebo clock → ROS 2 | `clock_relay` (gz-transport subscriber) → `/clock` |
| Depth mapper → map merger | `/{ns}/map` (`nav_msgs/OccupancyGrid`) |
| Map merger → allocator, controllers, overlay | `/global_map` (latched) |
| Allocator → flight controller | `/{ns}/cmd_pose` (`geometry_msgs/PoseStamped`) |
| Flight controller → Gazebo | `/{ns}/cmd_vel` (`Twist`, body frame), `/{ns}/enable` (`Bool`) via `ros_gz_bridge` |
| Flight controller → overlay | `/{ns}/plan` (`nav_msgs/Path`) |
| Allocator → overlay / RViz2 | `/swarm/markers/frontiers`, `/swarm/markers/locked_regions` |
| Overlay → Gazebo GUI | gz-transport request to `/marker_array` |

In-process (function calls into `algorithms.py`, not topics): frontier extraction and clustering,
greedy assignment, obstacle inflation, A*, ray casting and log-odds updates.

## 6. Coordinate Frames

| Frame | Definition |
|---|---|
| `map` | Fixed world frame. Gazebo odometry is published in it (`odom_frame` = `map`), and every grid, path, goal and marker uses it. |
| `{ns}/base_link` | Drone body frame (`drone_1/base_link`, ...). TF `map → {ns}/base_link` is broadcast by `swarm_sim_bridge`, so it exists only with `rviz:=true`. Nodes read the pose from `/{ns}/odom` directly, not from TF. |
| Body velocity frame | `/{ns}/cmd_vel` is expressed in the yaw-rotated body frame expected by `MulticopterVelocityControl`; `flight_controller` rotates its world-frame command before publishing. |
| Depth camera | Fixed at 0.13 m forward of `base_link` (`camera_offset` parameter, must match the SDF), 87° HFOV. |

Grid geometry shared by all maps: 200 × 200 cells, 0.2 m resolution, origin (−20, −20) in `map`.
The merger ignores a local map whose geometry differs.

Any future VSLAM pose must be expressed in the same `map` frame, so the mapping and exploration
code do not need to know which pose source is active (FR-14).

## 7. Data Flow

See [`architecture.md`](architecture.md) §4–§8.

## 8. Failure Handling

| Failure | Implemented handling |
|---|---|
| No depth or odometry yet | `depth_mapper` skips integration and publishes no map until one frame is integrated. |
| Drone too low to map | Depth frames are ignored while the drone is below 0.8 m (`min_mapping_altitude`). |
| Teammate or own propellers in the depth image | Returns within 0.6 m of a teammate or closer than 0.35 m are not written as obstacles. |
| Local map with a different grid geometry | `map_merger` ignores it and warns. |
| Drone not yet airborne | `task_allocator` does not task drones below 1.0 m. |
| Target unreachable | No 0.5 m progress for 20 s: target blacklisted (1.0 m radius), lock released, drone reassigned. |
| No collision-free path | `flight_controller` logs a warning and clears the path; the drone stops moving towards the goal until a replan (1 Hz) succeeds. |
| Allocator silent | No `cmd_pose` for 2 s: goal dropped, drone holds position. |
| Drones converging | Reactive repulsion inside 1.5 m, on top of the 2.0 m target exclusion. |
| `enable` published before the bridge subscribes | Controller waits for a subscriber before counting arming ticks. |
| No frontiers left | Not an error: EXPLORATION COMPLETE, drones hold. Exploration resumes if new frontiers appear. |
| Stale Gazebo server from an earlier run | `run_gazebo_swarm.sh` kills leftover `gz sim` and bridge processes before launching. |
| Gazebo GUI absent (`headless:=true`) | `gz_overlay` is not started. |

Not handled: pose-estimate loss (ground truth cannot fail), battery, and stalled sensor streams
(a frozen depth stream leaves the map unchanged rather than raising a fault). These matter once
the future VSLAM / PX4 backend is in place. Symptom-based guidance:
[`troubleshooting.md`](troubleshooting.md).
