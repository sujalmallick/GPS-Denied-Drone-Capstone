# Localization

What "GPS-denied" means for this project, what the stack uses for pose
today, and the planned path to replace simulator ground truth with an
onboard estimate. See [`../README.md`](../README.md) for the system
overview and [`mapping.md`](mapping.md) / [`exploration.md`](exploration.md)
for the components that consume the pose.

## 1. Current Status, Stated Plainly

**The autonomy currently uses Gazebo ground-truth pose.** Each drone's
pose comes from Gazebo's `OdometryPublisher` system on `/{ns}/odom`. It is
exact, with no noise and no drift. No state estimator runs in the current
launcher.

The stack is GPS-denied in the sense that **no GPS, motion-capture or
other external positioning signal is modelled or consumed by the
autonomy**. It does not yet estimate its own pose from onboard sensors.
Closing that gap is the localization work described in §6.

## 2. The GPS-Denied Localization Problem

Target environments such as warehouses, underground facilities and
collapsed buildings block or corrupt satellite signals. GPS is excluded
from the autonomy entirely, not even as a fallback. Each drone must
therefore estimate its pose from onboard sensors (camera, depth, IMU)
alone, and the three drones must agree on a common frame. The pose is
needed to:

* place depth rays into the local occupancy grid (`depth_mapper`);
* fuse the three local maps cell by cell (`map_merger`), which assumes
  all drones share one world frame;
* score frontiers by distance, apply the 2.0 m exclusion zones, and detect
  arrival (`task_allocator`);
* plan and follow A* paths and keep 1.5 m separation (`flight_controller`);
* filter teammates out of each drone's depth images (`depth_mapper`).

Onboard estimators drift. Errors grow with distance travelled and are
worst in low-texture scenes, at high speed, and with fast yaw. In a swarm,
each drone's errors are also independent, so their maps disagree unless
the drones are aligned to each other.

## 3. What the Current Stack Does

| Topic | Source | Rate | Used by |
|---|---|---|---|
| `/{ns}/odom` (`nav_msgs/Odometry`, frame `map`) | Gazebo `OdometryPublisher`, ground truth | 30 Hz | `depth_mapper` (own pose and teammates), `task_allocator`, `flight_controller` (own pose and teammates) |
| `/{ns}/ground_truth_pose` (`geometry_msgs/PoseStamped`) | Gazebo `PosePublisher` | 30 Hz | nothing in the autonomy; available as an evaluation reference |
| `/{ns}/imu` (`sensor_msgs/Imu`) | Gazebo IMU, BMI055-class white noise + turn-on and drifting bias | 200 Hz | nothing yet; intended input for VIO |
| `/{ns}/depth/image` (`32FC1`, 87° HFOV) | Gazebo depth camera | 10 Hz `realtime` / 30 Hz `full` | `depth_mapper` |
| `/{ns}/camera/image` (640 × 480 RGB, 60° HFOV) | Gazebo camera, only with `rgb_bridge:=true` | 30 Hz | nothing yet |

`task_allocator` also subscribes to `/{ns}/mavros/local_position/pose`, so
on hardware it can take the PX4 estimate instead of Gazebo odometry.

Sensor parameters are in `simulation/models/quadrotor_px4.sdf`. The IMU
white noise follows the BMI055 datasheet (gyro $3.45 \times 10^{-3}$ rad/s,
accel $2.08 \times 10^{-2}$ m/s² per sample at 200 Hz). The bias terms are
marked in the SDF as estimates to be replaced by an Allan-variance fit.

## 4. Why Ground Truth First

Localization (estimating pose from noisy sensors) and exploration
(deciding where to go given a map) are separate problems. Debugging both
at once makes it impossible to tell an exploration bug from pose drift.
With ground truth, mapping, fusion, Algorithm 1 and planning were validated
against a perfect pose. When an estimator is swapped in, any new failure
can be attributed to localization.

```text
now:     Gazebo odometry (exact) ─────────────► /{ns}/odom ─► mapping, allocation, control
planned: camera + IMU ─► VIO / VSLAM (─► EKF2) ─► /{ns}/odom ─► mapping, allocation, control
                                       Gazebo ground truth ─► evaluation only (ATE)
```

Ground truth must stay out of the autonomy loop once an estimator exists.
In simulation it remains available as the reference for ATE (§8).

## 5. Options for GPS-Denied Pose

| Approach | What it does | Strengths | Weaknesses |
|---|---|---|---|
| **Visual-inertial odometry (VIO)** | Tracks visual features and integrates the IMU in a filter or sliding window | Low latency, metric scale from the IMU, cheap | Drifts without bound; no loop closure |
| **Visual SLAM (with IMU)** | VIO plus a persistent map, relocalization and loop closure | Bounded drift when revisiting places; reusable map | Heavier; loop-closure jumps must be handled by downstream nodes |
| **PX4 EKF2 external-vision fusion** | The flight controller fuses an external pose (VIO/VSLAM via MAVROS `vision_pose`) with its own IMU | High-rate, smooth estimate the flight controller trusts for offboard control | Only as good as the vision input; needs time sync and frame conventions right |

Candidates: **Isaac ROS Visual SLAM** (cuVSLAM: stereo + IMU, GPU, the one
the repo already has launch files for), **ORB-SLAM3** (mono/stereo/RGB-D,
optionally inertial, with loop closure), and lighter VIO such as
**OpenVINS** or **VINS-Fusion**. These are complementary: the realistic
design is VIO/VSLAM feeding EKF2, with the PX4 estimate used as the
drone's pose.

For a swarm there is a further requirement: **a shared frame**. Each
estimator starts its own frame at its own take-off pose. The options are
known start poses (the spawn offsets in simulation, surveyed positions on
hardware), inter-drone map alignment (matching local maps or visual place
recognition between drones), or a multi-robot SLAM back end.
`map_merger` assumes the local maps are already aligned.

## 6. Planned Path to Replace Ground Truth

The repo contains hardware-side starting points that are **not wired into
the simulation launcher**:

* `vslam/isaac_ros_vslam_realsense.py`: per-drone namespace, RealSense
  D435i stereo IR (640 × 480 at 30 Hz) and IMU (200 Hz) into Isaac ROS
  Visual SLAM with IMU fusion. It publishes
  `/<ns>/visual_slam/tracking/vo_pose_covariance`. `sim/isaac_ros_vslam.sh`
  launches it for one namespace.
* `vslam/mavrospy.launch.py`: MAVROS to a Pixhawk plus a `topic_tools`
  relay from the VSLAM pose to `mavros/vision_pose/pose_cov`, the input of
  EKF2's external-vision fusion. The EKF2 parameters themselves (vision
  position/yaw fusion enabled, GPS disabled) still have to be set on the
  flight controller; the launch file does not set them.

Steps:

1. **Give the simulated drone the sensors the estimator needs.** The SDF
   has a mono RGB camera, a depth camera and an IMU, but no stereo IR pair,
   so Isaac ROS VSLAM cannot run on it unchanged. Either add a stereo
   camera pair to `quadrotor_px4.sdf` or choose an estimator that accepts
   RGB-D + IMU or mono + IMU.
2. **Run one estimator per drone** under `/{ns}/` and publish its result as
   `nav_msgs/Odometry` in a world frame. The simplest version transforms it
   by the known spawn pose.
3. **Swap the input:** point `depth_mapper`, `task_allocator` and
   `flight_controller` at the estimated odometry instead of Gazebo's. The
   nodes only need a `nav_msgs/Odometry` in a common `map` frame, so no
   algorithm changes are required.
4. **Time-align depth and pose** in `depth_mapper` (look up the pose at the
   depth image stamp instead of using the latest message) once the pose is
   no longer exact.
5. **Measure** ATE against `/{ns}/ground_truth_pose` (§8), then evaluate
   how drift affects coverage and the fused map.
6. **Hardware / PX4:** feed the estimate to EKF2 through MAVROS and use
   `/{ns}/mavros/local_position/pose` (already supported by
   `task_allocator`) as the pose source. The PX4 SITL scripts in
   `simulation/launch/start_px4_*.sh` and `src/control` are the starting
   point for a PX4 backend in simulation.

## 7. Coordinate Frames

* **`map`:** the fixed world frame. Gazebo publishes odometry directly in
  it (`odom_frame` = `map` in the SDF), and all grids, frontiers, goals and
  paths are expressed in it.
* **`{ns}/base_link`:** the body frame of each drone. The depth camera sits
  0.13 m ahead of it, looking along $+x$.
* **`odom`:** not used today. With an estimator, follow REP-105
  (`map → odom → base_link`). The Isaac ROS launch already names
  `map`, `odom` and `base_link`.

## 8. Evaluating Localization

`validation/validate.py` computes, per drone, the absolute trajectory
error between ground truth and the estimate:

$$\text{ATE}_i = \lVert p^{gt}_i - p^{est}_i \rVert_2, \qquad
\text{RMSE}_{\text{ATE}} = \sqrt{\tfrac{1}{N}\textstyle\sum_i \text{ATE}_i^2}, \qquad
\text{RMSE}_{\psi} = \sqrt{\tfrac{1}{N}\textstyle\sum_i \operatorname{wrap}(\psi^{gt}_i - \psi^{est}_i)^2}$$

It reads `validation/output/{ns}_trajectory.csv` (columns `gt_*` and
`vslam_*`) and plots the trajectories and the error over time. The target
printed by the script is ATE RMSE < 0.10 m.

**Caveat:** since no estimator runs yet, there is no real data to
evaluate. `validation/parse_rosbag.py` does not parse a bag today. It
writes synthetic circular trajectories with added noise, so any ATE it
reports is illustrative, not a measurement. To make it real, record
`/{ns}/ground_truth_pose` and the estimator's pose, align the first poses
and interpolate both onto common timestamps. Neither step is implemented
in `validate.py` yet. Then correlate ATE with coverage and fused-map
quality. See [`evaluation.md`](evaluation.md) for the full metric set.
