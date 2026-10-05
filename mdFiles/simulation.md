# Simulation

> Scope: this file documents the simulation stack (Gazebo Harmonic + ROS 2
> Jazzy) for the three-drone swarm. No physical hardware is involved.
> For running the stack, launch arguments and what the Gazebo window shows,
> see [`../README.md`](../README.md); for the cloud-GPU container see
> [`../docker/sim/README.md`](../docker/sim/README.md).

## 1. Requirements

* Ubuntu 24.04 (native or WSL2).
* ROS 2 Jazzy installed in `/opt/ros/jazzy`, plus `ros-jazzy-ros-gz`
  (Gazebo Harmonic / gz-sim 8 and `ros_gz_bridge`). RViz2 is optional.
* ROS 2 Humble does not work: it pairs with Gazebo Fortress, and
  `run_gazebo_swarm.sh` exits if `/opt/ros/jazzy/setup.bash` is missing.

## 2. Running

```bash
./run_single_window.sh                      # one Gazebo window with the swarm overlay
./run_gazebo_swarm.sh rviz:=true            # Gazebo window + RViz2
./run_gazebo_swarm.sh headless:=true rviz:=true   # no Gazebo window, RViz2 only
```

On Windows, `.\run_single_window.ps1` and `.\run_gazebo_swarm.ps1` run the
same scripts inside the WSL distribution named `Ubuntu`.

`run_gazebo_swarm.sh` does the following, in order:

1. Sources `/opt/ros/jazzy/setup.bash`.
2. Builds only the two packages this stack needs:
   `colcon build --packages-select coordination simulation`. A bare
   `colcon build` fails, because `src/control` needs `px4_msgs` and
   `src/localization` needs `gazebo_ros`.
3. Stops Gazebo servers and `parameter_bridge` processes left over from an
   earlier run.
4. Runs `ros2 launch simulation gazebo_swarm.launch.py "$@"`.

`run_single_window.sh` calls `run_gazebo_swarm.sh rviz:=false` and passes
on any extra arguments.

## 3. Launch Arguments

Declared in `simulation/launch/gazebo_swarm.launch.py`:

| Argument | Default | Effect |
|---|---|---|
| `world` | `simulation/worlds/simple_world.sdf` | World file to load |
| `headless` | `false` | No Gazebo GUI client and no in-window overlay |
| `rviz` | `false` | Also start RViz2 |
| `sensor_profile` | `realtime` | `realtime` = depth 160×120 @ 10 Hz; `full` = 320×240 @ 30 Hz (for GPU hosts) |
| `rgb_bridge` | `false` | Bridge the 640×480 RGB cameras to `/{ns}/camera/image` |
| `lock_radius` | `2.0` | Algorithm 1 exclusion radius (m) |
| `altitude` | `1.5` | Exploration altitude (m) |

## 4. Launch Sequence

`gazebo_swarm.launch.py` starts these processes (times are wall-clock
`TimerAction` delays after launch):

| Time | Process |
|---|---|
| 0 s | Gazebo server: `gz sim -s -r --headless-rendering <world>`. Sensors render off-screen (EGL) in this process |
| 0 s | `clock_relay` (Gazebo clock → ROS `/clock`, at most 100 Hz) and `ros_gz_bridge` `parameter_bridge` (plus a second bridge for RGB when `rgb_bridge:=true`) |
| 3 s | `ros_gz_sim create` spawns `drone_1`, `drone_2`, `drone_3` |
| 5 s | Swarm stack (`coordination/launch/swarm_coordination.launch.py`) with `use_sim_time:=true` |
| 7 s | Gazebo GUI client: `gz sim -g --gui-config simulation/gui/swarm_gui.config` (skipped when `headless:=true`) |

The GUI is a separate client process so that drawing the 3D view never
stalls the physics loop. It starts after spawning because a GUI client that
is still initialising when a model is created can miss that model.

## 5. World

`simulation/worlds/simple_world.sdf`:

* DART physics, `max_step_size` 0.002 s (2 ms), target real-time factor 1.
* Systems: Physics, UserCommands, SceneBroadcaster, Sensors (ogre2), Imu.
* Walled arena of 24 m × 20 m (x −12…12, y −9…11), 3 m high walls, with two
  partition walls, four 1 m obstacles, four 1.6 m pillars and a central
  block.
* Three coloured landing pads at the spawn points.

The world file is also the source of truth for the RViz2 environment
markers: with `rviz:=true`, `coordination/swarm_sim_bridge.py` reads it.

## 6. Drone Model

`simulation/models/quadrotor_px4.sdf` is a template. The launch file
replaces `__NS__`, `__COLOR__` and the depth profile tokens
(`__DEPTH_W__`, `__DEPTH_H__`, `__DEPTH_HZ__`) and spawns it three times:

| Drone | Spawn (x, y) | Colour |
|---|---|---|
| `drone_1` | (−4.0, −6.0) | blue |
| `drone_2` | (0.0, −6.0) | amber |
| `drone_3` | (4.0, −6.0) | violet |

All three spawn at z = 0.1 m facing +y. Despite its file name, the model
does not use PX4: it flies on Gazebo's own plugins.

* **Rotor physics:** one `MulticopterMotorModel` per propeller.
* **Flight control:** `MulticopterVelocityControl`, listening on
  `/{ns}/cmd_vel` (`geometry_msgs/Twist`) and `/{ns}/enable`
  (`std_msgs/Bool`).
* **State:** `OdometryPublisher` at 30 Hz and `PosePublisher`.

## 7. Simulated Sensors

| Sensor | Configuration | ROS 2 topic |
|---|---|---|
| Depth camera | 87° (1.5184 rad) horizontal FOV, 0.1–20 m, 32-bit float; 160×120 @ 10 Hz (`realtime`) or 320×240 @ 30 Hz (`full`) | `/{ns}/depth/image`, `/{ns}/depth/camera_info` |
| RGB camera | 640×480 @ 30 Hz, 60° FOV, Gaussian noise | `/{ns}/camera/image` (only with `rgb_bridge:=true`) |
| IMU | 200 Hz, BMI055-class noise and bias | `/{ns}/imu` |
| Odometry | 30 Hz, world frame (ground truth from Gazebo) | `/{ns}/odom` |
| Pose | Ground truth | `/{ns}/ground_truth_pose` |

The RGB cameras are opt-in because rendering all three at 30 Hz alone
drops a laptop below real time. The depth mapper samples about the same
number of rays per frame in both profiles and uses an 8 m range.

## 8. How Simulation Data Flows into ROS 2

```text
Gazebo server (physics, sensors)
   → ros_gz_bridge  /model/{ns}/odometry → /{ns}/odom
                    /model/{ns}/pose     → /{ns}/ground_truth_pose
                    /{ns}/imu, /{ns}/depth/image, /{ns}/depth/camera_info
   → clock_relay    Gazebo clock → /clock (≤ 100 Hz)
   → depth_mapper, flight_controller, map_merger, task_allocator,
     gz_overlay (unless headless), swarm_sim_bridge + RViz2 (rviz:=true)

flight_controller
   → /{ns}/cmd_vel, /{ns}/enable → ros_gz_bridge → MulticopterVelocityControl
```

There is a single bridge path. PX4, uXRCE-DDS and MAVROS are not part of
the running stack. The full topic table is in [`../README.md`](../README.md).

## 9. Performance

* Under WSL2 the full stack runs at about 0.6–0.75× real time. The limit is
  software rendering of the depth cameras. Every node uses simulation time,
  so behaviour is the same and only wall-clock speed changes.
* An NVIDIA GPU host running the `docker/sim` image is expected to reach
  real time with `sensor_profile:=full`. This has **not** been verified
  yet: the Docker image has not been built or run on a GPU host.
* Check the real-time factor while the simulation runs:

```bash
gz topic -e -t /stats
```

## 10. Cloud GPU (Docker)

`docker/sim/` builds an Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic image
and serves Gazebo and RViz2 in a browser through noVNC on port 6080, or runs
headless. It also includes `smoke-test` (topic rates and real-time factor)
and `check-gpu`. See [`../docker/sim/README.md`](../docker/sim/README.md).

```bash
bash docker/sim/run_docker.sh
```

## 11. How to Add Another Environment

1. Add a world file under `simulation/worlds/` with the same systems as
   `simple_world.sdf` (Physics, SceneBroadcaster, Sensors with ogre2, Imu,
   UserCommands) and the world name `simple_world`. The spawners address
   that name (`WORLD_NAME` in `gazebo_swarm.launch.py`).
2. Keep the spawn points (§6) free, or edit `DRONES` in the launch file.
3. Update `arena_bounds` in `coordination/map_merger.py` and
   `coordination/task_allocator.py` (default `[-12, 12, -9, 11]`), since
   coverage and frontier filtering use them.
4. Select it at launch:

```bash
./run_gazebo_swarm.sh world:=/absolute/path/to/your_world.sdf
```

5. For evaluation runs keep the world deterministic (fixed obstacle
   placement) so results are comparable (see [`evaluation.md`](evaluation.md)).

## 12. Legacy and Future Work

`simulation/launch/simple_world.launch.py`, `start_px4_sitl.sh` and
`start_px4_swarm.sh` belong to the earlier single-drone PX4 SITL phase.
They are kept as a starting point for a PX4 offboard backend (see
[`development-roadmap.md`](development-roadmap.md)) and are not used by
`gazebo_swarm.launch.py`. [`../simulation/launch/README.md`](../simulation/launch/README.md)
describes that legacy workflow.
