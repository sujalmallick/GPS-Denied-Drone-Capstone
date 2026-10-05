# Simulation Launch Guide

Launch files for the three-drone GPS-denied exploration swarm (ROS 2 Jazzy + Gazebo Harmonic).

## Files

| File | Purpose |
|---|---|
| `gazebo_swarm.launch.py` | **Main launch.** Gazebo server + GUI, three drones, ROS ↔ Gazebo bridges, swarm autonomy stack |
| `simple_world.launch.py` | Legacy M1: Gazebo with one drone, for the PX4 SITL workflow |
| `start_px4_sitl.sh`, `start_px4_swarm.sh` | Legacy / future work: PX4 SITL (one or three instances). Not used by the current stack |

## Prerequisites

- Ubuntu 24.04 (native or WSL2)
- ROS 2 Jazzy with `ros-jazzy-ros-gz` (Gazebo Harmonic). RViz2 is optional.
- Or no local install at all: use the cloud-GPU container in [`docker/sim`](../../docker/sim/README.md).

## Quick start

From the repository root:

```bash
./run_single_window.sh
```

This builds `coordination` and `simulation` with colcon, stops any leftover Gazebo processes, then
runs this launch file. To launch it directly:

```bash
source /opt/ros/jazzy/setup.bash
```
```bash
colcon build --packages-select coordination simulation && source install/setup.bash
```
```bash
ros2 launch simulation gazebo_swarm.launch.py
```

Build only these two packages. A bare `colcon build` fails, because `src/control` needs `px4_msgs`
and `src/localization` needs `gazebo_ros`.

## Arguments

| Argument | Default | Effect |
|---|---|---|
| `world` | `share/simulation/worlds/simple_world.sdf` | World file (24 × 20 m walled arena) |
| `headless` | `false` | `true`: no Gazebo GUI client (the server and swarm stack still run) |
| `rviz` | `false` | Also start RViz2 mission control |
| `sensor_profile` | `realtime` | Depth cameras: `realtime` = 160×120 @ 10 Hz, `full` = 320×240 @ 30 Hz (GPU hosts) |
| `rgb_bridge` | `false` | Bridge the 640×480 RGB cameras to `/{ns}/camera/image` |
| `lock_radius` | `2.0` | Algorithm 1 exclusion radius (m) |
| `altitude` | `1.5` | Exploration altitude (m) |

## What starts, and when

| Time | Action |
|---|---|
| t = 0 s | `gz sim -s -r --headless-rendering <world>`: physics server. Sensors render off-screen with EGL |
| t = 0 s | `ros_gz_bridge` (odometry, pose, IMU, depth, `cmd_vel` / `enable`) and `clock_relay` (`/clock` at 100 Hz) |
| t = 3 s | `ros_gz_sim create` spawns `drone_1` (−4, −6), `drone_2` (0, −6), `drone_3` (4, −6), facing +y |
| t = 5 s | `coordination/launch/swarm_coordination.launch.py`: depth mappers, flight controllers, map merger, task allocator, RViz2 bridge, Gazebo overlay (not in headless mode) and RViz2 (`rviz:=true`) |
| t = 7 s | `gz sim -g`: GUI client, unless `headless:=true`. It starts after spawning because a GUI that starts earlier can miss the spawned models |

The drones arm and climb to the exploration altitude about 10–20 s after launch.

Topics, rates and node parameters are listed in [`mdFiles/ros2-architecture.md`](../../mdFiles/ros2-architecture.md).
To check that a running simulation is healthy, use the smoke test in
[`docker/sim`](../../docker/sim/README.md#4-smoke-test) (`bash docker/sim/smoke_test.sh` also works outside Docker).

## Legacy: PX4 SITL workflow

The current stack flies the drones with Gazebo's `MulticopterVelocityControl`, not PX4.
`simple_world.launch.py` and the `start_px4_*.sh` scripts belong to the earlier single-drone PX4 phase
(see the historical M1 report, [`mdFiles/M1_README.md`](../../mdFiles/M1_README.md)). They are kept
as the starting point for a PX4 offboard backend. Set `PX4_SOURCE_DIR` to your PX4-Autopilot checkout
before using them.

## References

- [Project README](../../README.md)
- [Simulation design](../../mdFiles/simulation.md) · [Troubleshooting](../../mdFiles/troubleshooting.md)
- [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
