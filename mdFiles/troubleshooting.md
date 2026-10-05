# Troubleshooting

General-purpose guide for the current stack (ROS 2 Jazzy + Gazebo Harmonic,
three drones, `run_gazebo_swarm.sh` / `gazebo_swarm.launch.py`), organized
by symptom. Running instructions are in [`../README.md`](../README.md); the
cloud-GPU container is documented in
[`../docker/sim/README.md`](../docker/sim/README.md).

## Quick Health Check

A healthy run looks like this:

* About 10–20 s after launch the flight controllers log `armed, taking off`
  and then `at altitude, starting exploration`, and the drones climb to
  1.5 m.
* The task allocator logs `drone_i -> frontier (x, y)` lines.
* Coverage keeps rising:

```bash
ros2 topic echo /swarm/metrics/coverage_pct
```

* Gazebo reports its real-time factor:

```bash
gz topic -e -t /stats
```

## Build Fails

* Use the launcher scripts, or build only the two packages the stack uses:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select coordination simulation
```

* A bare `colcon build` fails by design: `src/control` depends on
  `px4_msgs` and `src/localization` on `gazebo_ros`, and neither is
  installed for this stack.
* `Error: ROS 2 Jazzy not found in /opt/ros/jazzy`: install ROS 2 Jazzy and
  `ros-jazzy-ros-gz` on Ubuntu 24.04. ROS 2 Humble pairs with Gazebo
  Fortress and cannot run this stack.

## `bad interpreter` or `$'\r': command not found`

* The scripts have Windows (CRLF) line endings. `.gitattributes` forces LF
  for `*.sh`, `*.py`, `*.sdf`, `*.xml`, `*.rviz`, `*.cfg`, `*.yaml` and
  `*.txt`, but files checked out before it existed, or edited with a
  Windows editor that writes CRLF, can still carry them.
* Fix a single file inside WSL:

```bash
sed -i 's/\r$//' run_gazebo_swarm.sh run_single_window.sh
```

* To re-normalize the whole checkout, first commit or stash your work,
  then re-check out the files so the `.gitattributes` rules apply.

## Windows Launcher Does Nothing

* `run_gazebo_swarm.ps1` and `run_single_window.ps1` call
  `wsl -d Ubuntu`. Your WSL distribution must be named `Ubuntu`
  (`wsl -l -v` lists the names). Otherwise run the `.sh` scripts from a
  WSL shell.

## Simulation Runs Slowly (Low Real-Time Factor)

* Under WSL2 about 0.6–0.75× real time is expected. Depth cameras are
  rendered in software. Nodes use simulation time, so behaviour does not
  change; only the wall clock is slower.
* Keep the default `sensor_profile:=realtime` (160×120 @ 10 Hz depth) on a
  laptop. `sensor_profile:=full` is meant for GPU hosts.
* Leave `rgb_bridge:=false`. Once the RGB cameras are bridged, Gazebo
  renders all three at 640×480 @ 30 Hz.
* Check for a second Gazebo server from an earlier run (next section). Two
  servers compete for CPU and both publish a clock.
* **WSL GPU:** the launch file does not force the d3d12 (WSL GPU) render
  path, because in our tests it was slower than llvmpipe for depth-sensor
  read-back. Forcing it is not a fix for low RTF.

## Stale Gazebo Processes

* `run_gazebo_swarm.sh` stops leftover `gz sim` and `parameter_bridge`
  processes before launching. If a run was killed uncleanly and you start
  the launch file directly, clean up by hand:

```bash
pgrep -af "gz sim|parameter_bridge"
pkill -INT -f "^gz sim"; sleep 3; pkill -9 -f "^gz sim"
pkill -9 -f "ros_gz_bridge|parameter_bridge"
```

## A Drone Is Missing from the Gazebo Window

* The GUI client starts at t = 7 s, after the drones are spawned at
  t = 3 s. A GUI client that is still initialising when a model is created
  can miss it and never draw it. This can happen on a heavily loaded
  machine, or if you open a GUI yourself before spawning finishes.
* Close the Gazebo window (the server keeps running) and open a new client
  from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
gz sim -g --gui-config simulation/gui/swarm_gui.config
```

* The overlay (merged map, frontiers, rings, HUD) is drawn by
  `coordination/gz_overlay.py` through the GUI's MarkerManager plugin. It is
  disabled with `headless:=true`.

## Drones Do Not Take Off

* Wait 10–20 s. The swarm stack starts at t = 5 s, and each flight
  controller only arms after the bridge has subscribed to `/{ns}/enable`.
* Check that the drones exist and are publishing state:

```bash
ros2 topic hz /drone_1/odom
ros2 topic echo /drone_1/enable --once
```

* If `/clock` is missing, nothing advances: all nodes run with
  `use_sim_time:=true`. Check `ros2 topic hz /clock` (≤ 100 Hz from
  `clock_relay`).

## No Messages on a Topic

* Run the smoke test. It lists missing, silent and slow topics and the
  real-time factor. In the Docker container:

```bash
docker exec -it swarm_sim_cloud smoke-test
```

* On a native or WSL install, run it from a sourced shell. It defaults to
  the `full` profile and a minimum RTF of 0.8, so for a laptop run pass:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 docker/sim/smoke_test.py --profile realtime --min-rtf 0
```

* Minimum sim-time rates: odom 28 Hz, imu 190 Hz, ground_truth_pose
  28 Hz, map and `/global_map` 1.8 Hz, depth 28 Hz (`full`) or 9 Hz
  (`realtime`), RTF 0.8.
* If topics are missing in every shell, check that all terminals use the
  same `ROS_DOMAIN_ID`.

## Map or Coverage Not Updating

* The depth mapper ignores frames while the drone is below 0.8 m and
  returns closer than 0.35 m (its own propellers). Mapping starts once the
  drones have climbed.
* Check the per-drone maps and the merged map:

```bash
ros2 topic hz /drone_1/map
ros2 topic hz /global_map
```

* If the drone maps but the merged map stays empty, check that
  `map_merger` is running (`ros2 node list`).

## Exploration Stops Early or a Drone Hovers

* `EXPLORATION COMPLETE: no frontiers left, drones holding position` is the
  normal end of a run.
* A drone that makes no progress towards its target for 20 s has that
  target blacklisted (`blacklisting target` warning) and gets a new one.
* A flight controller holds position if no `cmd_pose` arrives for 2 s.
  Check `ros2 topic hz /drone_1/cmd_pose` (1 Hz).

## High Overlap Rate

* An overlap of about 30–45 % at 90 % coverage is the current, expected
  behaviour, not a bug. The 87° depth cameras see much of the 24 × 20 m
  room from almost anywhere. Algorithm 1 keeps targets and paths apart,
  not sensor footprints. See [`evaluation.md`](evaluation.md).

## RViz Shows Nothing

* RViz2 only starts with `rviz:=true`. The RViz-only markers and TF
  (`swarm_sim_bridge`) are also only started then.
* The fixed frame is `map`. Confirm `/global_map` is publishing before
  assuming a display problem.

## Docker: Rendering on the CPU (llvmpipe)

* Run the bundled check inside the container:

```bash
docker exec -it swarm_sim_cloud check-gpu
```

* The `GL_RENDERER` lines from `~/.gz/rendering/ogre2.log` should name the
  NVIDIA GPU. `llvmpipe` means software rendering, which usually means the
  container started without `--gpus all` or the NVIDIA Container Toolkit
  is missing. Verify the toolkit on the host:

```bash
docker run --rm --gpus all ubuntu nvidia-smi
```

* If the Gazebo window does not appear in viewer mode, rule out VirtualGL:

```bash
VGLRUN=0 bash docker/sim/run_docker.sh
```

## Docker: Cannot Open the Browser Desktop

* The noVNC desktop is on port 6080 (`NOVNC_PORT` changes the host port).
  It is only published in viewer mode: `headless:=true` without
  `rviz:=true` publishes no port.
* The generated password is printed at start-up. With `--detach`, read it
  from the logs:

```bash
docker logs swarm_sim_cloud 2>&1 | grep "VNC password"
```

* Only the first 8 characters of the password count. Set your own with
  `VNC_PASSWORD=...`.
* With `BIND_ADDR=127.0.0.1` the port is only reachable through an SSH
  tunnel (`ssh -L 6080:localhost:6080 user@<server-ip>`). Otherwise allow
  TCP 6080 in the firewall or security group, from your own IP only.

## Future PX4 Backend

* PX4 SITL, uXRCE-DDS and MAVROS are not part of the current stack: the
  drones fly on Gazebo's `MulticopterVelocityControl`. The legacy scripts
  `simulation/launch/start_px4_sitl.sh` and `start_px4_swarm.sh` and the
  `src/control` package (which needs `px4_msgs`) are the starting points
  for a PX4 offboard backend. Troubleshooting for that path will be added
  when it is wired in (see [`development-roadmap.md`](development-roadmap.md)).
