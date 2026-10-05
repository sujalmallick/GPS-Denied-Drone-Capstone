# Swarm simulation on a cloud GPU (Docker + browser desktop)

The image runs the stack in a container: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, RViz2 and the
`coordination` and `simulation` packages. You view Gazebo and RViz2 in a browser through noVNC on
port 6080, and everything renders on the NVIDIA GPU:

- The **Gazebo server** renders the drones' depth cameras off-screen through EGL. It does this in
  every mode.
- The **Gazebo GUI client** and **RViz2** draw into the VNC desktop through VirtualGL, using its
  EGL back end.

## Requirements on the server

- An NVIDIA driver (`nvidia-smi` works on the host).
- Docker plus the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
  To check it, run `docker run --rm --gpus all ubuntu nvidia-smi`. It should print your GPU.
- An x86_64 or arm64 machine.

## 1. Build and run

Run this from the repository root:

```bash
bash docker/sim/run_docker.sh
```

The script does three things:

1. It builds `swarm_sim:jazzy` from your working tree. Uncommitted changes are included.
2. It starts the container with `--gpus all`, `NVIDIA_VISIBLE_DEVICES=all` and `NVIDIA_DRIVER_CAPABILITIES=all`.
3. It publishes port 6080 and runs `run_gazebo_swarm.sh rviz:=true sensor_profile:=full`. That opens
   the Gazebo window with the swarm overlay, plus RViz2, with depth cameras at 320x240 and 30 Hz.

The first build takes a while. Later builds only re-copy the workspace and re-run `colcon build`.

It prints a line like `VNC password (generated): Ab12Cd34`. To set your own password, pass it in
when you start the script:

```bash
VNC_PASSWORD=mysecret bash docker/sim/run_docker.sh
```

Arguments of the form `name:=value` go to `gazebo_swarm.launch.py`. The available arguments are
`world`, `headless`, `rviz`, `sensor_profile` (`realtime` or `full`), `lock_radius` and `altitude`.
Passing any argument replaces the default `rviz:=true`. `sensor_profile:=full` is still added
unless you give a different profile. For example:

```bash
bash docker/sim/run_docker.sh rviz:=true lock_radius:=2.5
```

Other options are `--no-build`, `--detach` (then read the logs with `docker logs -f swarm_sim_cloud`)
and `--shell` (opens bash in the container instead of the simulation). Stop the run with Ctrl+C or
`docker stop swarm_sim_cloud`.

## 2. Open it in the browser

Go to `http://<server-ip>:6080` and enter the VNC password. The drones arm and climb about 10–20 s
after launch.

Port 6080 has only the VNC password protecting it, and only the first 8 characters of that password
count. If you can, don't open the port to the internet. Instead, keep it on localhost and use an
SSH tunnel:

```bash
BIND_ADDR=127.0.0.1 bash docker/sim/run_docker.sh
```

```bash
ssh -L 6080:localhost:6080 user@<server-ip>
```

Then open `http://localhost:6080` on your machine. If you use a firewall or security group instead,
allow TCP 6080 from your own IP only.

## 3. Headless mode (no viewer)

```bash
bash docker/sim/run_docker.sh headless:=true
```

This mode starts no desktop, publishes no port and starts no Gazebo GUI client. The physics server
and the swarm stack run as normal, and the depth cameras still render on the GPU through EGL. Use
`headless:=true rviz:=true` to keep the browser desktop with RViz2 only.

## 4. Smoke test

While the simulation is running, open a second shell on the server and run:

```bash
docker exec -it swarm_sim_cloud smoke-test
```

It checks that these topics exist and are publishing, and prints their rate in both wall time and
simulation time:

- `/clock`
- `/global_map`
- for each drone: `/{ns}/odom`, `/{ns}/imu`, `/{ns}/depth/image`, `/{ns}/ground_truth_pose` and `/{ns}/map`

It also prints the real-time factor from `/clock` and from Gazebo's `/stats`. The exit status is
non-zero if a topic is missing or silent, or if it's below its minimum. Minimums are checked
against simulation time:

| Topic | Nominal rate | Minimum |
|---|---|---|
| `odom`, `ground_truth_pose` | 30 Hz | 28 Hz |
| `imu` | 200 Hz | 190 Hz |
| `depth/image` | 30 Hz with `sensor_profile:=full`, 10 Hz with `realtime` | 28 Hz / 9 Hz |
| `map`, `/global_map` | 2 Hz | 1.8 Hz |
| real-time factor | | 0.8 |

The depth threshold follows the container's `SENSOR_PROFILE`, which is `full` by default. You can
override it with `--profile realtime`. Other options are `--min-rate imu=150` (repeatable),
`--no-default-rates`, `--min-rtf 0` (turns off the RTF check) and `--duration 20`.

To check things by hand:

```bash
docker exec -it swarm_sim_cloud bash -ic "ros2 topic echo /swarm/metrics/coverage_pct"
```

```bash
docker exec -it swarm_sim_cloud bash -ic "gz topic -e -t /stats"
```

The first command shows coverage, which should keep rising. The second shows the real-time factor.

## 5. Check that the GPU is being used

On the **host**, while the simulation runs:

```bash
nvidia-smi
```

The process table should list the Gazebo processes as type `G` (graphics) with GPU memory in use.
In viewer mode it should also list `rviz2`. The Gazebo processes are the server and the GUI client,
and they may appear as `gz`/`gz sim` or as `ruby`, because the `gz` CLI is a Ruby launcher.
GPU-Util should be above 0 %.

Inside the container, `nvidia-smi` usually can't show process names, because PIDs are in a separate
namespace. Run the bundled check instead:

```bash
docker exec -it swarm_sim_cloud check-gpu
```

It prints the GPU, the renderer VirtualGL sees and the `GL_RENDERER` lines from
`~/.gz/rendering/ogre2.log`. Those lines should name the NVIDIA GPU. If they say **llvmpipe**,
rendering fell back to the CPU. That usually means the container started without `--gpus all` or
the NVIDIA Container Toolkit isn't installed.

If the Gazebo window doesn't come up in viewer mode, try running without VirtualGL to see whether
it's the cause:

```bash
VGLRUN=0 bash docker/sim/run_docker.sh
```

In this mode the GUI renders on the CPU, but the server's sensors still use the GPU.

## How it works

| File | Role |
|---|---|
| `Dockerfile` | `osrf/ros:jazzy-desktop` + `ros-jazzy-ros-gz`, RViz2, numpy/pandas/matplotlib, VirtualGL, TurboVNC, noVNC; builds `coordination` + `simulation` with colcon |
| `entrypoint.sh` | Viewer mode: TurboVNC on `:1` (localhost only) → websockify/noVNC on 6080 → `vglrun -d egl ./run_gazebo_swarm.sh …`. Headless mode: `./run_gazebo_swarm.sh headless:=true …` with no display |
| `xstartup.sh` | Fluxbox desktop with one terminal. Apps you start there need `vglrun` to use the GPU, e.g. `vglrun rviz2` |
| `check_gpu.sh` | `check-gpu` in the container |
| `smoke_test.sh` / `.py` | `smoke-test` in the container |
| `run_docker.sh` | Builds the image and runs the container |

VirtualGL's EGL back end renders on the GPU without an X server on the GPU, so the container
doesn't need host X11 or a `/tmp/.X11-unix` mount. With several GPUs, set `VGL_DISPLAY=egl1` (and
so on) inside the container to pick a different one.
