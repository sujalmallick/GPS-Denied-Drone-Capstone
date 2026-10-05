# Real-Time Multi-UAV Swarm Gazebo Simulation Master Specification
**Project**: GPS-Denied Autonomous Indoor Exploration (3-Drone Swarm System)  
**Target Platform**: ROS 2 Jazzy + Gazebo Harmonic (8.15+) + PX4 SITL  
**Primary Architecture**: Distributed Decentralized Mapping & Greedy Spatial-Exclusion Coordination  

---

## 1. Executive Summary & Repository Scan

### 1.1 The Core Discrepancy: Mock UI vs. True Robotics Simulation
A detailed scan of the codebase reveals two competing paradigms:

| Component | Current State in Repo | Real Robotics Target Architecture | Action Required |
| :--- | :--- | :--- | :--- |
| **User Interface / Visualization** | `presentation_app/` (Node.js + WebGL browser dashboard with hardcoded mock telemetry) | **RViz2** (Native ROS 2 GCS) + **Gazebo Harmonic GUI** (Physics visualizer) | **Deprecate / Remove `presentation_app/`** |
| **Physics & Sensor Simulation** | Fragments of Isaac Sim scripts (`sim/`, `vslam/`) alongside Gazebo SDFs | **Gazebo Harmonic** with DART physics, depth camera, IMU, and 30 Hz odometry plugins | **Standardize on Gazebo Harmonic** |
| **Multi-Robot Coordination** | `coordination/` with `map_merger.py` and `task_allocator.py` | Algorithm 1 Greedy No-Overlap Frontier Assignment ($r = 2.0\text{ m}$) & Conservative Grid Fusion | **Keep & Connect to live Gazebo topics** |
| **Vehicle Flight Control** | Stubbed/open-loop cmd_pose in bridge | PX4 SITL multi-instance offboard position controller | **Integrate PX4 SITL / Offboard bridge** |

---

## 2. Target End-to-End System Architecture

```text
                                 ┌─────────────────────────────────────────┐
                                 │             GAZEBO HARMONIC             │
                                 │     (Physics, Obstacles, Dynamics)      │
                                 └────────────────────┬────────────────────┘
                                                      │
                       Simulation Clock & Sensors     │  /clock (gz.msgs.Clock)
                       /model/drone_{i}/odometry      │  /world/.../depth_camera
                       /world/.../imu/imu             │  /model/drone_{i}/pose
                                                      ▼
                                 ┌─────────────────────────────────────────┐
                                 │              ROS-GZ BRIDGE              │
                                 │         (ros_gz_bridge package)         │
                                 └────────────────────┬────────────────────┘
                                                      │
                                                      │ ROS 2 Native Topics
                                                      ▼
                      ┌────────────────────────────────────────────────────────┐
                      │                 ROS 2 JAZZY WORKSPACE                  │
                      │                                                        │
                      │  Per-Drone Stacks (/drone_1, /drone_2, /drone_3):      │
                      │    ├── Sensor Perception: /depth/image, /imu           │
                      │    ├── VSLAM / Local Mapping: /map, /odom              │
                      │    └── Flight Controller: /cmd_pose                    │
                      │                                                        │
                      │  Swarm Mission Control:                                │
                      │    ├── map_merger.py -> /global_map (2.0 Hz)           │
                      │    ├── task_allocator.py -> Algorithm 1 Frontier Alloc │
                      │    └── validation/ -> Live Coverage / Overlap Metrics  │
                      └───────────────────────────────┬────────────────────────┘
                                                      │
                                                      ▼
                                 ┌─────────────────────────────────────────┐
                                 │             UNIFIED VISUALIZER          │
                                 │         (RViz2 Swarm Dashboard)         │
                                 │  • 3D Drone Models with Spinning Rotors │
                                 │  • 3D Environment Obstacles & Pads      │
                                 │  • Fused Global Map & Frontier Spheres  │
                                 │  • 2.0m Safety Exclusion Cylinders      │
                                 └─────────────────────────────────────────┘
```

---

## 3. Standardized ROS 2 Topic Specifications

### 3.1 Simulation Core Topics
- **Simulation Clock**: `/clock` (`rosgraph_msgs/msg/Clock`) — Synchronizes wall time across all ROS 2 nodes and Gazebo.

### 3.2 Per-Drone Namespace Topics (`/{drone_i}/...` where `i` $\in \{1, 2, 3\}$)
- **Odometry**: `/{drone_i}/odom` (`nav_msgs/msg/Odometry`, 30 Hz) — Body velocity and position relative to local starting frame.
- **Ground Truth Pose**: `/{drone_i}/ground_truth_pose` (`geometry_msgs/msg/PoseStamped`, 30 Hz) — Exact pose in world frame.
- **IMU**: `/{drone_i}/imu` (`sensor_msgs/msg/Imu`, 250 Hz) — Angular velocity and linear acceleration with Gaussian sensor noise.
- **RGB Camera**: `/{drone_i}/camera/image` (`sensor_msgs/msg/Image`, 30 Hz) — Forward-facing optical stream ($640 \times 480$).
- **Depth Camera**: `/{drone_i}/depth/image` (`sensor_msgs/msg/Image`, 30 Hz) — 32-bit float metric depth ($0.1\text{ m} - 20.0\text{ m}$).
- **Local Occupancy Grid**: `/{drone_i}/map` (`nav_msgs/msg/OccupancyGrid`, 2 Hz) — Local $200 \times 200$ grid ($0.2\text{ m}$ resolution, $[-1, 0, 100]$).
- **Target Flight Waypoint**: `/{drone_i}/cmd_pose` (`geometry_msgs/msg/PoseStamped`) — Dispatched exploration waypoint at altitude $z = 1.5\text{ m}$.

### 3.3 Swarm Coordination Topics (Global Namespace)
- **Fused Global Map**: `/global_map` (`nav_msgs/msg/OccupancyGrid`, 2.0 Hz) — Unified swarm belief map via conservative fusion.
- **Frontier Markers**: `/swarm/markers/frontiers` (`visualization_msgs/msg/MarkerArray`) — Gold spheres marking active exploration boundaries.
- **Locked Exclusion Regions**: `/swarm/markers/locked_regions` (`visualization_msgs/msg/MarkerArray`) — Translucent colored cylinders ($r = 2.0\text{ m}$) preventing swarm collisions.
- **3D Quadcopter Models**: `/swarm/markers/drones` (`visualization_msgs/msg/MarkerArray`) — 3D body, rotor arms, and floating labels.
- **3D Environment Obstacles**: `/swarm/markers/environment` (`visualization_msgs/msg/MarkerArray`) — 3D obstacles and landing pads matching Gazebo world.
- **Exploration Coverage Metric**: `/swarm/metrics/coverage_pct` (`std_msgs/msg/Float32`) — Numerical percentage of arena mapped ($0\% - 100\%$).
- **Inter-Drone Overlap Metric**: `/swarm/metrics/overlap_rate` (`std_msgs/msg/Float32`) — Ratio of cells observed by multiple drones ($\le 15\%$).

---

## 4. Algorithmic Formulations

### 4.1 Conservative Multi-Robot Map Fusion
For grid cell $c$ across active local maps $M_1, M_2, \dots, M_K$:
$$M_{\text{global}}(c) = \begin{cases}
100 & \text{if } \exists i \text{ s.t. } M_i(c) \ge 65 \quad \text{(Obstacle Priority)} \\
0 & \text{if } \forall i \text{ with valid data, } M_i(c) = 0 \text{ and } \sum [M_i(c) \ge 0] \ge 1 \\
-1 & \text{otherwise (Unknown)}
\end{cases}$$

### 4.2 Algorithm 1: Greedy No-Overlap Frontier Assignment
1. Extract frontier cells $\mathcal{F} = \{c \mid M_{\text{global}}(c) = 0 \land \exists n \in \mathcal{N}_8(c): M_{\text{global}}(n) = -1\}$.
2. Cluster into centroid targets $\{f_1, f_2, \dots, f_m\}$.
3. For each drone $i \in \{1, \dots, K\}$ needing a task:
   - Compute utility $U(f_j) = \alpha \cdot \text{InformationGain}(f_j) - \beta \cdot \|p_i - f_j\|_2$.
   - Reject any $f_j$ where $\|f_j - p_k\| < r_{\text{lock}}$ for any other drone $k \neq i$.
   - Assign $f^* = \arg\max U(f_j)$ and lock region $R_i = \{x \in \mathbb{R}^2 \mid \|x - f^*\| \le r_{\text{lock}}\}$.
4. Release lock $R_i$ when $\|p_i - f^*\| \le 0.8\text{ m}$ (Waypoint Arrival).

---

## 5. Component Lifecycle Matrix: Keep, Adapt, Purge

### KEEP AS-IS (High-Value Swarm Logic)
- `coordination/map_merger.py`: Thread-safe, shape-validated conservative map fusion.
- `coordination/task_allocator.py`: Algorithm 1 implementation with arrival lock release and persistent waypoints.
- `coordination/test_swarm_coordination.py`: 4/4 passing unit tests verifying mathematical guarantees.
- `validation/validate.py`: Mathematical verification of coverage and trajectory logs.

### ADAPT (Already Completed & Hardened)
- `simulation/worlds/simple_world.sdf`: Added DART physics, Ogre2 sensors, dedicated GUI camera, and colored landing pads.
- `simulation/models/quadrotor_px4.sdf`: Added 30 Hz odometry publisher, carbon-fiber arms, canopy, and IMU.
- `simulation/launch/gazebo_swarm.launch.py`: Configured exact Gazebo Harmonic topic bridges and headless execution.
- `coordination/swarm_sim_bridge.py`: Added 3D environment obstacles and landing pads streaming to RViz2.

### PURGE / REMOVE (Detours & Dead Code)
- `presentation_app/`: Web browser JavaScript mock presentation. (Not ROS 2, not Gazebo, synthetic hardcoded data).
- `run_presentation.ps1` & `run_presentation.sh`: Launchers for the web mock app.
- `sim/isaac_sim.py` & `vslam/isaac_ros_vslam_realsense.py`: Legacy Isaac Sim test stubs superseded by Gazebo Harmonic.
