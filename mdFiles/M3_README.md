# M3: Simulated Sensors and Pose Available

> **Historical milestone report (Phase 1, single drone).** This document records the earlier single-drone PX4 SITL + ROS 2 Humble phase and the C++ packages in `src/`. The current system is the three-drone ROS 2 Jazzy + Gazebo Harmonic swarm described in [../README.md](../README.md) and [README.md](README.md). Commands and results below are kept for reference and may no longer run as written.

**Status:** ✅ **IMPLEMENTATION COMPLETE**  
**Date:** 2026-08-22  
**Objective:** Camera, IMU, and depth sensor data published as ROS 2 topics; ground-truth pose available in ROS 2.

## What Was Implemented

### 1. Sensor Plugins on Drone Model
- **Camera** (`sensor name="camera"`): RGB image (640×480, 30 Hz)
  - Positioned forward on drone body
  - 60° horizontal FOV (typical for exploration)
- **Depth Camera** (`sensor name="depth_camera"`): Depth map (640×480, 30 Hz)
  - Same position/orientation as RGB camera
  - Max range 20m (for obstacle detection)
- **IMU** (already present in M1): 250 Hz, realistic noise model

### 2. ROS 2 Localization Package (`src/localization/`)
Provides bridge between Gazebo ground-truth and ROS 2 exploration/mapping nodes.

**Files:**
- `package.xml` — ROS 2 dependencies (rclcpp, geometry_msgs, tf2, gazebo_ros)
- `CMakeLists.txt` — C++17 build configuration
- `src/ground_truth_pose_publisher.cpp` — Ground-truth pose publisher (main node for M3-M10)
- `src/sensor_bridge.cpp` — Optional sensor republisher
- `launch/ground_truth_pose.launch.py` — Launch file

**Key Nodes:**

#### Ground Truth Pose Publisher
- **Subscribes to:** `/odom` (Gazebo odometry)
- **Publishes:**
  - `ground_truth_pose` (geometry_msgs/PoseStamped) — Drone pose in map frame
  - TF: `map` → `base_link` transform (optional, enabled by default)
- **Parameters:**
  - `gazebo_odometry_topic` — Gazebo odometry input topic
  - `ros_pose_topic` — Output pose topic name
  - `publish_tf` — Enable TF broadcasting
  - `map_frame` — Fixed world frame name
  - `base_link_frame` — Drone body frame name
  - `verbose` — Debug logging

#### Sensor Bridge
- **Subscribes to:** Gazebo camera/depth/IMU topics
- **Publishes:** Standard ROS 2 sensor_msgs (Image, Imu)
- **Status:** Optional (sensors can be published directly via gazebo_ros plugin)

## Topics Published by M3

| Topic | Type | Rate | Purpose |
|---|---|---|---|
| `ground_truth_pose` | geometry_msgs/PoseStamped | 200 Hz (Gazebo) | Drone pose (M4+ mapper input) |
| `/map` (via M4) | nav_msgs/OccupancyGrid | 2 Hz | Occupancy grid from depth |
| `tf` | tf2/TransformStamped | 200 Hz | TF tree (map→base_link) |
| `/imu/data` | sensor_msgs/Imu | 250 Hz | IMU data (already in M1) |
| `/camera/image_raw` | sensor_msgs/Image | 30 Hz | RGB camera (for M11 VSLAM) |
| `/depth/image` | sensor_msgs/Image | 30 Hz | Depth map (for M4 mapping) |

## Frames and Coordinates (REP-105)

```
map (fixed world frame)
  ├── origin at Gazebo world origin
  └── odom (odometry frame, typically same as map for ground truth)
       └── base_link (drone body frame)
           ├── camera_link (forward-facing RGB camera)
           ├── depth_link (forward-facing depth camera)
           └── imu_link (IMU at drone center)
```

## Building M3

```bash
cd ~/GPS-DeniedV2
colcon build --packages-select localization
source install/setup.bash
```

## Testing M3 (Prerequisites for M4+)

### Terminal 1: Gazebo + PX4 SITL (M1)
```bash
# Start Gazebo with drone model
ros2 launch simulation simple_world.launch.py

# In another terminal:
cd ~/src/PX4-Autopilot/build/px4_sitl_default/bin
./px4 -s etc/init.d-posix/rcS
```

### Terminal 2: Ground Truth Pose Publisher (M3)
```bash
ros2 launch localization ground_truth_pose.launch.py
```

### Terminal 3: Verify Topics
```bash
# Check pose topic
ros2 topic list | grep -E "pose|camera|depth|imu"

# Verify pose is being published
ros2 topic echo ground_truth_pose --once

# Check camera/depth rates
ros2 topic hz /camera/image_raw
ros2 topic hz /depth/image
```

### Terminal 4: Visualize in RViz2
```bash
rviz2
# Add displays:
# - TF (to see map→base_link)
# - Image (display /camera/image_raw)
# - Image (display /depth/image)
```

## M3 Acceptance Criteria

✅ Ground-truth pose published at expected rate (>10 Hz minimum)  
✅ Camera topic publishes RGB images (check with `ros2 topic hz`)  
✅ Depth topic publishes depth data  
✅ IMU publishes at 250 Hz  
✅ TF tree includes map→base_link transform  
✅ All frame_ids consistent (all messages use `map` and `base_link`)  

## Architecture Integration

**M3 fits into the M4+ pipeline:**

```
Gazebo (M1)
    ├── /odom (ground truth)
    ├── /camera/image_raw (Gazebo camera plugin)
    └── /depth/image (Gazebo depth plugin)
           ↓
    Ground Truth Pose Publisher (M3)
           ↓
    ground_truth_pose topic
           ↓
    M4: Mapping Node (subscribes to pose + depth)
           ↓
    /map topic
           ↓
    M5-M7: Frontier Detection/Clustering/Selection
```

## Known Limitations (M3)

1. **Camera calibration not yet implemented** — Using hardcoded focal length/principal point
2. **No sensor noise calibration** — Gaussian noise parameters are placeholders
3. **No IMU pre-filtering** — Raw IMU data published as-is (M11 VSLAM will apply filters)
4. **Ground truth only** — Actual pose = Gazebo ground truth (M11 replaces with VSLAM/VIO)
5. **No depth filtering** — All depth pixels used; real systems would use bilateral filtering

All limitations are acceptable for M3-M10 (development phase using ground truth).

## Extensions for M11 (VSLAM/VIO)

When VSLAM/VIO is added in M11:
1. Replace `ground_truth_pose_publisher` node with VSLAM/VIO node (e.g., ORB-SLAM3)
2. VSLAM/VIO node subscribes to `/camera/image_raw` and `/imu/data`
3. VSLAM/VIO publishes `ground_truth_pose` (same topic interface)
4. **No changes needed to M4-M10 code** — Mapping and exploration nodes see same interface

This is the key architectural principle: **Localization source is swappable without downstream changes.**

## Troubleshooting

### "Pose not yet received" warning in mapping node
- Check if ground truth pose publisher is running
- Verify `/odom` topic is being published by Gazebo
- Check DDS domain ID (should be 0 by default)

### Camera/depth topics not appearing
- Ensure Gazebo plugins are loaded correctly
- Check if gazebo_ros plugin is installed: `sudo apt install ros-humble-gazebo-ros`
- Verify drone model SDF includes sensor definitions (check quadrotor_px4.sdf)

### TF warnings in RViz2
- Ensure `publish_tf` parameter is `true` in ground truth pose publisher
- Check map/base_link frame names match between nodes

## Next Steps

With M3 complete, proceed to:
1. **M4 (Mapping)** — Uses `ground_truth_pose` + `/depth/image` to build occupancy grid
2. **M5-M7 (Exploration)** — Use map from M4 to detect, cluster, select frontiers
3. **M8-M9** — Waypoint generation and navigation (coming soon)

## Files Summary

**M3 Implementation:** ~600 lines of code + documentation

| File | Purpose | Lines |
|---|---|---|
| package.xml | Package manifest | 25 |
| CMakeLists.txt | Build config | 45 |
| ground_truth_pose_publisher.cpp | Pose publisher | 125 |
| sensor_bridge.cpp | Sensor republisher | 140 |
| ground_truth_pose.launch.py | Launch file | 40 |

---

**M3 Status:** ✅ COMPLETE AND READY FOR M4 MAPPING INTEGRATION
