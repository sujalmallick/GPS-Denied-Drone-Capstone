# GPS-Denied Multi-Drone Swarm Exploration: Documentation Index

Design and reference docs for the three-drone indoor exploration swarm. The repository root
[`../README.md`](../README.md) is the authoritative quick start (topics, rates, algorithms, launch
arguments, known limitations). The files in this folder explain the design behind it.

Repository: <https://github.com/sujalmallick/GPS-Denied-Drone-Capstone>

## Project Objective

Three simulated quadrotors (`drone_1`, `drone_2`, `drone_3`) explore an unknown, walled indoor
arena together: each drone maps what its own depth camera sees, the maps are fused into one
global occupancy grid, and a central allocator sends each drone to a different frontier so the
swarm covers the space without duplicating effort. Everything runs in simulation (Gazebo Harmonic
+ ROS 2 Jazzy). There is no physical hardware in the running system.

## Problem Statement

Outdoor drones localize with GPS. Indoors, underground, in urban canyons and in collapsed
buildings the satellite signal is blocked, reflected or jammed, so position from GPS is missing or
wrong. A GPS-denied drone has to rely on what it carries: it must sense obstacles with onboard
sensors (depth camera, IMU), build its own map, and decide where to go next using only what it has
observed, without a pre-built map or an external position fix.

A single drone doing this is slow, limited by battery, and a single point of failure. A swarm can
split the work, but only if the drones share what they have mapped and avoid exploring the same
space. That coordination problem (map merging plus no-overlap frontier allocation) is the core of
this project.

## Current System (what runs today)

| Layer | Implementation |
|---|---|
| Simulator | Gazebo Harmonic (gz-sim 8), DART physics, 2 ms step; `simulation/worlds/simple_world.sdf`, a walled 24 m × 20 m arena with partitions, boxes and pillars |
| Vehicles | 3 × `simulation/models/quadrotor_px4.sdf`: rotor physics (`MulticopterMotorModel` per propeller) flown by Gazebo's `MulticopterVelocityControl`. **Not PX4.** |
| Sensors | Forward depth camera (87° HFOV, 0.1–20 m), 640×480 RGB camera, 200 Hz IMU with noise and bias, 30 Hz odometry |
| Middleware | ROS 2 Jazzy, `ros_gz_bridge`, `coordination/clock_relay.py` (`/clock` at 100 Hz) |
| Localization | Gazebo odometry (ground truth). VSLAM is future work. |
| Autonomy | Python package `coordination/`: `depth_mapper`, `map_merger`, `task_allocator` (Algorithm 1), `flight_controller` (A* + pure pursuit) |
| Visualization | Swarm overlay drawn inside the Gazebo window (`gz_overlay`); RViz2 optional (`rviz:=true`) |
| Deployment | Ubuntu 24.04 (native or WSL2), or the cloud-GPU Docker image in [`../docker/sim/README.md`](../docker/sim/README.md) |

High-level loop:

```text
Gazebo (rotor physics + depth/IMU/odometry)
   → ros_gz_bridge → per-drone depth_mapper → /{ns}/map
   → map_merger → /global_map (+ coverage / overlap metrics)
   → task_allocator (Algorithm 1) → /{ns}/cmd_pose
   → flight_controller (A* on /global_map, pure pursuit) → /{ns}/cmd_vel
   → MulticopterVelocityControl → rotor speeds → drone moves → new depth frames (repeat)
```

The C++ packages under `src/` (control, exploration, localization, mapping) are from the earlier
single-drone PX4 phase. The current launcher (`run_gazebo_swarm.sh`) builds only the `coordination`
and `simulation` packages, so `src/` is not part of the running stack.

## How to Run

```bash
./run_single_window.sh          # Linux / WSL2: one Gazebo window with the swarm overlay
```
```powershell
.\run_single_window.ps1         # Windows host, runs inside WSL2 Ubuntu
```

Launch arguments (`world`, `headless`, `rviz`, `sensor_profile`, `rgb_bridge`, `lock_radius`,
`altitude`), unit tests and validation commands are documented in [`../README.md`](../README.md).
For a cloud GPU with a browser desktop (noVNC on port 6080) see
[`../docker/sim/README.md`](../docker/sim/README.md).

## Documentation Index

| Document | Contents |
|---|---|
| [`architecture.md`](architecture.md) | Layers, components, launch sequence, data and control flow |
| [`system-design.md`](system-design.md) | Requirements and their status, boundaries, interfaces, coordinate frames, failure handling |
| [`ros2-architecture.md`](ros2-architecture.md) | Nodes, topics, QoS, parameters, bridges, launch arguments |
| [`simulation.md`](simulation.md) | Gazebo world, drone model, sensors, sensor profiles |
| [`localization.md`](localization.md) | Ground-truth odometry today, VSLAM/VIO plan |
| [`mapping.md`](mapping.md) | Per-drone log-odds depth mapping and conservative map fusion |
| [`exploration.md`](exploration.md) | Frontier extraction, Algorithm 1 allocation, completion |
| [`algorithms.md`](algorithms.md) | Per-algorithm specification (`coordination/algorithms.py`) |
| [`evaluation.md`](evaluation.md) | Coverage, overlap and ATE metrics; validation tooling |
| [`troubleshooting.md`](troubleshooting.md) | Common failure modes and fixes |
| [`development-roadmap.md`](development-roadmap.md) | What is done, what is next |

Related files outside this folder:

* [`../README.md`](../README.md): quick start and reference.
* [`../simulation/launch/README.md`](../simulation/launch/README.md): launch notes (its PX4 SITL section is the legacy single-drone workflow).
* [`../REALTIME_GAZEBO_SWARM_BLUEPRINT.md`](../REALTIME_GAZEBO_SWARM_BLUEPRINT.md): design spec written during the move to Gazebo Harmonic. It still lists PX4 SITL as the flight target, which the running stack does not use.

## Future Work

The PX4 SITL / MAVROS offboard backend (`simulation/launch/start_px4_*.sh`, `src/control`,
`vslam/mavrospy.launch.py`) and the Isaac ROS Visual SLAM pipeline (`vslam/`, `sim/`) exist as
starting points but are not wired into the running stack. See the "Future work: PX4 / VSLAM
backend" sections of [`architecture.md`](architecture.md) and
[`ros2-architecture.md`](ros2-architecture.md), and [`development-roadmap.md`](development-roadmap.md).

## Historical Milestone Reports

These describe the earlier single-drone phase (PX4 SITL, uXRCE-DDS, ROS 2 Humble, C++ packages in
`src/`). They are kept for the record and do not describe the current system.

* [`M1_README.md`](M1_README.md), [`M1_IMPLEMENTATION_SUMMARY.md`](M1_IMPLEMENTATION_SUMMARY.md), [`M1_TEST_PROCEDURE.md`](M1_TEST_PROCEDURE.md), [`M1_COMPLETION_REPORT.md`](M1_COMPLETION_REPORT.md)
* [`M2_IMPLEMENTATION_SUMMARY.md`](M2_IMPLEMENTATION_SUMMARY.md), [`M2_TEST_PROCEDURE.md`](M2_TEST_PROCEDURE.md), [`M2_COMPLETION_REPORT.md`](M2_COMPLETION_REPORT.md)
* [`M3_README.md`](M3_README.md), [`M4_README.md`](M4_README.md), [`M5_M6_M7_README.md`](M5_M6_M7_README.md)
* [`M3_M7_IMPLEMENTATION_SUMMARY.md`](M3_M7_IMPLEMENTATION_SUMMARY.md), [`M3_M7_COMPLETION_REPORT.md`](M3_M7_COMPLETION_REPORT.md)
* [`PROJECT_STATUS_M3_M7.md`](PROJECT_STATUS_M3_M7.md)
