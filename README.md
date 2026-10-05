# Multi-Drone Swarm System for Indoor Exploration in GPS-Denied Environments

A real-time robotics simulation and autonomy stack for three-UAV collaborative mapping, frontier
exploration and no-overlap task allocation in GPS-denied indoor environments.

**Stack:** ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8, DART physics) · ros_gz_bridge · RViz2 (optional)

---

## 📌 Project Overview

In GPS-denied indoor environments (underground facilities, warehouses, collapsed buildings) satellite
positioning is unavailable and a single drone is slow, battery-limited and a single point of failure.
This project runs a **three-UAV swarm** end to end on simulated physics and sensors:

- **Physics & sensors (Gazebo Harmonic):** three quadrotors fly on real rotor dynamics
  (`MulticopterMotorModel` per propeller + `MulticopterVelocityControl`) inside a walled 24 m × 20 m
  arena. Each carries a forward RealSense-like depth camera, an RGB camera, a BMI055-class IMU
  (200 Hz, noise + bias) and 30 Hz odometry.
- **Per-drone perception (`coordination/depth_mapper.py`):** turns each drone's own depth images into a
  log-odds local occupancy grid `/{drone_i}/map`. Teammates in view and the drone's own propellers
  (returns closer than 0.35 m) are not written as obstacles.
- **Map merging (`coordination/map_merger.py`):** conservative fusion of the three local maps into
  `/global_map` at 2 Hz, plus live coverage and overlap metrics.
- **Swarm task allocation (`coordination/task_allocator.py`):** Algorithm 1, greedy no-overlap frontier
  assignment with 2.0 m exclusion zones and coordinated (discounted) information gain.
- **Flight control (`coordination/flight_controller.py`):** A* path planning on the fused map, pure-pursuit
  velocity commands and inter-drone separation, sent to Gazebo as `/{drone_i}/cmd_vel`.
- **One-window mission view (`coordination/gz_overlay.py`):** the swarm's merged map, frontiers,
  exclusion zones, paths and mission gauges are drawn inside the Gazebo window next to the drones.

---

## 🏗️ Architecture

```text
┌──────────────────────────── GAZEBO HARMONIC ────────────────────────────┐
│ simple_world.sdf (DART) · 3 × quadrotor_px4.sdf (rotor physics, sensors)  │
└──────────────┬──────────────────────────────────────────────▲───────────┘
   clock, odometry, pose, IMU, depth                /{ns}/cmd_vel, /{ns}/enable
┌──────────────▼──────── ros_gz_bridge + clock_relay ──────────┴───────────┐
└──────────────┬──────────────────────────────────────────────▲───────────┘
┌──────────────▼──────────────── ROS 2 JAZZY ───────────────────┴───────────┐
│ per drone:  depth_mapper ──/{ns}/map──┐          flight_controller (A*)  │
│                                       ▼                    ▲             │
│ swarm:      map_merger ──/global_map──► task_allocator ──/{ns}/cmd_pose  │
│             gz_overlay ──► Gazebo GUI markers    (swarm_sim_bridge ──► RViz2) │
└───────────────────────────────────────────────────────────────────────────┘
```

### Topics

| Topic | Type | Rate |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` (relayed from Gazebo) | 100 Hz |
| `/{drone_i}/odom` | `nav_msgs/Odometry` (world frame) | 30 Hz |
| `/{drone_i}/ground_truth_pose` | `geometry_msgs/PoseStamped` | 30 Hz |
| `/{drone_i}/imu` | `sensor_msgs/Imu` | 200 Hz |
| `/{drone_i}/depth/image` | `sensor_msgs/Image` (32FC1, 0.1–20 m) | 10 Hz `realtime` / 30 Hz `full` |
| `/{drone_i}/camera/image` | `sensor_msgs/Image` (640×480, only with `rgb_bridge:=true`) | 30 Hz |
| `/{drone_i}/map` | `nav_msgs/OccupancyGrid` (200×200, 0.2 m, origin −20, −20) | 2 Hz |
| `/{drone_i}/cmd_pose` | `geometry_msgs/PoseStamped` (z = 1.5 m) | 1 Hz |
| `/{drone_i}/cmd_vel`, `/{drone_i}/enable` | `geometry_msgs/Twist`, `std_msgs/Bool` | 20 Hz |
| `/{drone_i}/plan` | `nav_msgs/Path` | 1 Hz |
| `/global_map` | `nav_msgs/OccupancyGrid` (latched) | 2 Hz |
| `/swarm/metrics/coverage_pct`, `/swarm/metrics/overlap_rate` | `std_msgs/Float32` | 2 Hz |
| `/swarm/markers/{frontiers,locked_regions,drones,trails,environment}` | `visualization_msgs/MarkerArray` | — |

---

## ⚡ Algorithms

All formulas live in `coordination/algorithms.py` (pure numpy) and are unit-tested directly.

### 1. Conservative map merging

$$M_{\text{global}}(c) = \begin{cases}
100 & \text{if } \exists i : M_i(c) \ge 65 \quad \text{(obstacle priority)} \\
0 & \text{if every drone that observed } c \text{ reports } 0 \\
-1 & \text{otherwise}
\end{cases}$$

### 2. Algorithm 1: greedy no-overlap frontier assignment

1. Frontier cells: free cells with an unknown 8-neighbour, $F = \{c \mid M(c)=0 \wedge \exists n \in N_8(c): M(n)=-1\}$,
   excluding cells within 0.4 m of a mapped obstacle or outside the operating area.
2. Group them into 8-connected clusters (≥ 5 cells). Each cluster's target is the frontier cell closest to
   its centroid, so goals are always known-free cells.
3. Utility $U(f) = \alpha \cdot \text{IG}(f) \cdot \text{discount}(f) - \beta \cdot \lVert p_i - f \rVert$, where IG is
   the frontier length in metres. The discount is $\prod_k \min(1, d_k / r_s)$ over teammate positions and
   targets (sensor radius $r_s = 5$ m): space a teammate will sense anyway is worth less. This is the
   coordination idea of Burgard et al. (IEEE T-RO 2005), and it spreads the drones out.
4. Reject $f$ if it lies within $r_{\text{lock}} = 2.0$ m of another drone or of another drone's locked region.
5. Assign the best (drone, frontier) pair, lock a disc of radius $r_{\text{lock}}$ around it, and repeat.
6. Keep a target for at least 5 s and while it still borders unknown space. Release the lock on arrival
   ($\le 0.8$ m) or after 20 s without progress, which blacklists the target as unreachable.
7. When no frontier is left, log **EXPLORATION COMPLETE**. The drones hold position.

### 3. Path planning & control

A* on the fused map, with occupied cells inflated by 0.5 m and unknown cells traversable at double cost,
followed by line-of-sight smoothing. The controller does pure pursuit at ≤ 0.8 m/s, yaws towards the
direction of travel (so the forward depth camera sees where the drone flies), adds repulsion inside 1.5 m
of another drone, and holds position if no `cmd_pose` arrives for 2 s.

### 4. Metrics

- **Coverage:** known cells inside the arena bounds ÷ arena cells × 100 %.
- **Overlap rate:** explored cells observed by ≥ 2 drones ÷ explored cells × 100 %.
- **ATE RMSE** (offline, `validation/validate.py`): $\sqrt{\tfrac{1}{N}\sum \lVert p^{gt}_i - p^{est}_i \rVert^2}$.

---

## 🚀 Running the simulation

**Requirements:** Ubuntu 24.04 (native or WSL2) with ROS 2 Jazzy and `ros-jazzy-ros-gz`. RViz2 is optional.

**Windows (runs inside WSL2 Ubuntu):**
```powershell
.\run_single_window.ps1
```

**Linux / WSL shell:**
```bash
./run_single_window.sh
```

This opens **one Gazebo window**. The drones arm, climb to 1.5 m and start exploring. Coverage
typically passes 90 % in about 40 s of simulated time, and the allocator then reports EXPLORATION COMPLETE.

Optional arguments (they also work with `run_gazebo_swarm.ps1` / `.sh`):

| Argument | Effect |
|---|---|
| `rviz:=true` | also open RViz2 (raw maps, TF, topics) |
| `headless:=true rviz:=true` | no Gazebo window, RViz2 only |
| `sensor_profile:=full` | 320×240 depth at 30 Hz (for GPU machines) |
| `rgb_bridge:=true` | publish the 640×480 RGB cameras to ROS |

### What you see in the Gazebo window

| Element | Meaning |
|---|---|
| Blue / amber / violet drones | drone_1 / drone_2 / drone_3. Body, battery stripe and front propellers in the drone's colour (front = direction of flight, green LED), with a halo ring, heading line and altitude stem |
| Blue / amber / violet floor tiles | free space mapped by that drone (map merging) |
| White floor tiles | space mapped by 2 or more drones (overlap) |
| Red floor tiles | obstacles detected by the depth cameras |
| Gold spheres | frontiers: boundary between explored and unexplored space |
| Coloured rings | 2 m Algorithm 1 exclusion zone around each drone's target |
| Coloured lines | planned A* path and flown trail of each drone |
| Bars above the north wall | coverage (green) and overlap (orange), full length = 100 % |

### Unit tests
```bash
python3 coordination/test_swarm_coordination.py
```

### Metrics & validation
The live metrics (`/swarm/metrics/coverage_pct`, `/swarm/metrics/overlap_rate`, RTF) are measured. The
offline scripts below are **placeholders**: `parse_rosbag.py` does not read the bag yet (it writes
synthetic trajectories), and `validate.py` plots hard-coded coverage/overlap/speedup values. Don't cite
their numbers until they are rebuilt (see [mdFiles/evaluation.md](mdFiles/evaluation.md)).
```bash
python3 validation/parse_rosbag.py --name <path_to_bag>
```
```bash
python3 validation/validate.py --dir validation/output
```

### Known limitations
- **Speed on a laptop:** under WSL2 the full stack runs at about 0.6–0.75× real time, limited by
  software rendering of the depth cameras. All nodes use simulation time, so behaviour is unchanged and
  only wall-clock speed is affected. A native Linux host with an NVIDIA GPU (see `docker/sim`) is
  expected to run in real time, but this has not been measured yet.
- **Overlap:** the 87° / 8 m depth cameras see a large part of a 24 × 20 m room from almost anywhere, so
  cells seen by two drones are common (about 30–45 % by the time coverage reaches 90 %). Algorithm 1 and
  gain discounting keep the drones' *targets and paths* apart, which is what the 2 m exclusion zones
  guarantee.
- **Flight stack:** Gazebo's built-in velocity controller, not PX4 SITL. `simulation/launch/start_px4_*.sh`
  and `src/control` are the starting points for a PX4 offboard backend.
- **Localization:** Gazebo odometry (ground truth). The Isaac ROS VSLAM launch (`vslam/`) is not wired in yet.

---

## 📁 Repository Structure

```text
GPS-Denied-Drone/
├── simulation/                    # Gazebo Harmonic package
│   ├── worlds/simple_world.sdf    # Walled indoor arena (single source of truth for RViz too)
│   ├── models/quadrotor_px4.sdf   # Rotor-physics quadrotor template (__NS__, __COLOR__, depth profile)
│   ├── gui/swarm_gui.config       # Gazebo window layout and camera
│   └── launch/gazebo_swarm.launch.py  # Gazebo server + GUI + spawn + bridges + swarm stack
├── coordination/                  # ROS 2 swarm autonomy package
│   ├── algorithms.py              # Fusion, frontiers, Algorithm 1, A*, depth mapping (pure numpy)
│   ├── depth_mapper.py            # Depth camera -> local occupancy grid
│   ├── flight_controller.py       # cmd_pose -> A* path -> cmd_vel
│   ├── map_merger.py              # /global_map + coverage / overlap metrics
│   ├── task_allocator.py          # Algorithm 1 frontier assignment
│   ├── gz_overlay.py              # Swarm view drawn inside the Gazebo window
│   ├── clock_relay.py             # Gazebo clock -> ROS /clock at 100 Hz
│   ├── swarm_sim_bridge.py        # Gazebo odometry + world SDF -> RViz2 markers and TF
│   ├── test_swarm_coordination.py # Unit tests for algorithms.py
│   ├── launch/swarm_coordination.launch.py
│   └── rviz/swarm_coordination.rviz
├── validation/                    # Offline metrics (rosbag -> CSV -> ATE / coverage plots)
├── vslam/, sim/                   # Isaac ROS VSLAM + MAVROS launch files (hardware / future work)
├── src/                           # C++ packages (PX4 control, mapping, exploration, localization)
├── docker/sim/                    # Cloud-GPU container
├── run_single_window.{sh,ps1}     # Launch: one Gazebo window with the swarm overlay
└── run_gazebo_swarm.{sh,ps1}      # Launch with any gazebo_swarm.launch.py arguments
```
