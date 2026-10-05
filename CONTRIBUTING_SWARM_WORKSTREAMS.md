# Multi-Drone Swarm System — 3-Person Team Workstream Division & PR Guide

This guide establishes the exact division of responsibilities for our **3-person Capstone team**:
**Project**: Multi-Drone Swarm System for Indoor Exploration in GPS-Denied Environments  
**Primary Simulation Engine**: **Gazebo Harmonic (`gz sim`) + ROS 2 Jazzy** (Running 3 Drones Concurrently).

> **Status (Oct 2026):** the running stack flies the drones with Gazebo's `MulticopterVelocityControl`
> and uses Gazebo odometry as pose; mapping and the local planner are the Python nodes in `coordination/`.
> The PX4 SITL / MAVROS / stereo-IR VSLAM tasks below are still planned work. The world is
> `simple_world.sdf` (not a Curved Gridroom), and the drone model has depth, RGB and IMU sensors but no
> stereo IR pair yet. See [mdFiles/development-roadmap.md](mdFiles/development-roadmap.md) for current status.
> Code lives in https://github.com/sujalmallick/GPS-Denied-Drone-Capstone.

---

## 👥 3-Person Team Ownership Matrix (Zero Merge Conflicts)

To guarantee **zero merge conflicts when opening Pull Requests**, each person has strict, exclusive ownership over designated directories. No person edits files outside their assigned folders.

| Team Member | Git Branch | Owned Directories | Focus Area & Deliverables (All for Gazebo) |
|---|---|---|---|
| **Member 1: YOU (Lead)** | `feature/realtime-simulation-lead` | `coordination/`, `validation/`, root launchers | **Swarm Intelligence & Mission Control**: Conservative map merger node (`/global_map`), Algorithm 1 task allocator (`/{ns}/cmd_pose` with 2.0m locked zones), RViz2 mission control, validation & PR integration. |
| **Member 2: Teammate A** | `feature/sim-environment-physics` | `sim/`, `simulation/`, `docker/` | **Gazebo Physics, World & Multi-Drone Spawn**: 3-drone Gazebo Harmonic world (Curved Gridroom), quadrotor SDF models with RealSense D435i plugins, PX4 SITL multi-instance MAVLink ports (14540, 14541, 14542), and cloud Docker setup. |
| **Member 3: Teammate B** | `feature/vslam-perception-px4` | `vslam/`, `src/control/`, `src/localization/`, `src/mapping/` | **Perception, VSLAM & PX4 Offboard Control**: Visual SLAM / Odometry graph, `ros_gz_bridge` sensor piping, MAVROS 3-drone bridge, PX4 EKF2 GPS-denied tuning, local grid raycasting (`/{ns}/map`), and offboard waypoint follower. |

---

## 📋 Exact Instructions for Each Team Member

### 👤 Member 1: YOU (Lead & Swarm Intelligence)
- **Branch**: `feature/realtime-simulation-lead` (active)
- **Owned Folders**: `coordination/`, `validation/`, root scripts
- **Your Responsibilities**:
  1. **Shared Global Occupancy Grid (`coordination/map_merger.py`)**: Fuses `/{drone_1,drone_2,drone_3}/map` into `/global_map` using conservative fusion (free only if all agree, occupied if any reports obstacle).
  2. **Swarm Task Allocator (`coordination/task_allocator.py`)**: Implements **Algorithm 1** (Greedy No-Overlap Frontier Assignment) running at 1.0 Hz with 2.0m circular locked regions ($R_i$).
  3. **RViz2 Mission Control (`coordination/swarm_sim_bridge.py`, `coordination/rviz/`)**: Live Gazebo drones, arena, fused map, frontiers, exclusion zones and flight trails in one window (the former `presentation_app/` web mock has been removed).
  4. **Validation Suite (`validation/validate.py`)**: Benchmarks Absolute Trajectory Error (RMSE $< 0.10\text{ m}$), coverage progression ($> 90\%$), and overlap rate ($< 5.0\%$).
  5. **PR Merging**: Review and merge Member 2 and Member 3 PRs into `main`.

---

### 👤 Member 2: Teammate A (Gazebo Simulation, Physics & Multi-Drone Spawn)
- **Branch**: `feature/sim-environment-physics`
- **Owned Folders**: `sim/`, `simulation/`, `docker/`
- **Core Tasks for Gazebo**:
  1. Build the indoor Curved Gridroom facility in Gazebo Harmonic (`simulation/worlds/curved_gridroom.sdf`).
  2. Create a ROS 2 launch file (`simulation/launch/gazebo_swarm.launch.py`) that spawns 3 Iris quadrotors into Gazebo:
     - `drone_1`: PX4 Instance 0, MAVLink UDP 14540, spawn position `(-4.0, -6.0, 0.2)`
     - `drone_2`: PX4 Instance 1, MAVLink UDP 14541, spawn position `( 0.0, -6.0, 0.2)`
     - `drone_3`: PX4 Instance 2, MAVLink UDP 14542, spawn position `( 4.0, -6.0, 0.2)`
  3. Attach Intel RealSense D435i plugins (stereo infrared + IMU) to `simulation/models/quadrotor_px4.sdf` using `gz-sim-sensors-system` and `gz-sim-imu-system`.
  4. Ensure `docker/sim/Dockerfile` and `docker/sim/run_docker.sh` run Gazebo headless on cloud GPUs (RunPod or AWS EC2).

---

### 👤 Member 3: Teammate B (Perception, Visual SLAM & PX4 Offboard Flight Control)
- **Branch**: `feature/vslam-perception-px4`
- **Owned Folders**: `vslam/`, `src/control/`, `src/localization/`, `src/mapping/`
- **Core Tasks for Gazebo**:
  1. Configure `ros_gz_bridge` to bridge Gazebo camera streams and IMU to ROS 2 topics (`/{ns}/camera/infra1`, `/{ns}/camera/infra2`, `/{ns}/camera/imu`).
  2. Setup Visual SLAM / VIO tracking in ROS 2 Jazzy to output estimated drone odometry.
  3. Configure MAVROS bridge (`vslam/mavrospy.launch.py`) with 3 namespaces forwarding visual pose to PX4 `/mavros/vision_pose/pose`.
  4. Set PX4 EKF2 parameters for vision-only flight (disable GPS and baro drift: `EKF2_HGT_REF=Vision`, `EKF2_GPS_CTRL=0`).
  5. Generate local 2D occupancy grid `/{ns}/map` via depth camera raycasting (feeds into Lead's map merger).
  6. Implement the offboard position controller accepting waypoints on `/{ns}/cmd_pose` (which the Lead's task allocator publishes).
---


## 🔗 ROS 2 Standardized Interface Contracts

To guarantee that components developed on separate branches connect seamlessly at runtime, use these standardized topics:

| Topic Name | Message Type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `/{ns}/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | Member 2 (Sim) | Member 3 (VSLAM) | Left stereo IR camera stream |
| `/{ns}/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | Member 2 (Sim) | Member 3 (VSLAM) | Right stereo IR camera stream |
| `/{ns}/camera/imu` | `sensor_msgs/Imu` | Member 2 (Sim) | Member 3 (VSLAM) | 200 Hz IMU sensor data |
| `/{ns}/visual_slam/tracking/vo_pose_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | Member 3 (VSLAM) | Member 3 & Lead | Estimated VSLAM drone pose |
| `/{ns}/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Member 3 (Control) | Member 1 (Lead) | Current drone pose in map frame |
| `/{ns}/map` | `nav_msgs/OccupancyGrid` | Member 3 (Mapping) | Member 1 (Lead) | Local 2D occupancy grid per drone |
| `/global_map` | `nav_msgs/OccupancyGrid` | Member 1 (Lead) | Member 1 & 3 | Merged global occupancy grid (2 Hz) |
| `/{ns}/cmd_pose` | `geometry_msgs/PoseStamped` | Member 1 (Lead) | Member 3 (Control) | Assigned exploration waypoint |

---

## 🚀 Conflict-Free Pull Request Workflow

1. **Commit only inside your folders**:
   ```bash
   git add <your_folders>
   git commit -m "feat(<component>): brief description"
   ```
2. **Rebase onto main before opening PR**:
   ```bash
   git fetch origin
   git rebase origin/main
   ```
3. **Open Pull Request to `main`**:
   Because each person's file paths are completely isolated, GitHub will merge automatically with **0 merge conflicts**.
