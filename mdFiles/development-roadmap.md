# Development Roadmap

```text
CURRENT PHASE = 3-drone swarm on Gazebo Harmonic + ROS 2 Jazzy (ground-truth odometry)
NEXT TARGET   = N1: verify the stack on an NVIDIA GPU host (docker/sim)
```

Repository: <https://github.com/sujalmallick/GPS-Denied-Drone-Capstone>

The project started as a single-drone plan (milestones M1–M12: PX4 SITL
over uXRCE-DDS, ROS 2 Humble, Gazebo with `gazebo_ros`). That plan has been
superseded. The current system is a three-drone swarm that runs end to end
on Gazebo Harmonic, with Gazebo's own velocity controller and ground-truth
odometry. Most of what M4–M10 aimed at (mapping, frontiers, selection,
waypoints, the full exploration loop) now exists in multi-drone form in
`coordination/`. M11 (VSLAM/VIO) and M12 (evaluation) remain open as N3
and N4 below.

How to run the stack: [`../README.md`](../README.md). Simulation details:
[`simulation.md`](simulation.md). Metrics: [`evaluation.md`](evaluation.md).

---

## Done

| # | Item | Where |
|---|---|---|
| D1 | Swarm simulation on Gazebo Harmonic: walled 24 m × 20 m arena, DART physics at a 2 ms step, three rotor-physics quadrotors, depth camera, RGB camera, IMU, 30 Hz odometry, `realtime` / `full` sensor profiles | `simulation/` |
| D2 | ROS ↔ Gazebo bridging (`ros_gz_bridge`) and `/clock` relay at 100 Hz | `simulation/launch/gazebo_swarm.launch.py`, `coordination/clock_relay.py` |
| D3 | Per-drone depth mapping into log-odds local grids `/{ns}/map` | `coordination/depth_mapper.py` |
| D4 | Conservative map merging into `/global_map`, plus live coverage and overlap metrics | `coordination/map_merger.py` |
| D5 | Algorithm 1: greedy no-overlap frontier assignment with 2.0 m exclusion zones and discounted information gain | `coordination/task_allocator.py` |
| D6 | A* planning on the fused map, pure-pursuit velocity control, inter-drone separation | `coordination/flight_controller.py` |
| D7 | One-window mission view: merged map, frontiers, exclusion zones, paths and gauges drawn inside Gazebo | `coordination/gz_overlay.py`, `simulation/gui/swarm_gui.config` |
| D8 | Launchers for Linux/WSL and Windows; scoped `colcon` build; stale-process cleanup | `run_gazebo_swarm.{sh,ps1}`, `run_single_window.{sh,ps1}` |
| D9 | Unit tests for fusion, metrics, frontiers, Algorithm 1, A* and depth rays | `coordination/test_swarm_coordination.py` |
| D10 | Cloud-GPU Docker setup: noVNC browser viewer on port 6080, headless mode, `smoke-test`, `check-gpu` | `docker/sim/` |

D10 is written but has **not** been built or run on a GPU host yet (see N1).

---

## Next

### N1 — Verify on an NVIDIA GPU host

**Objective:** Confirm the Docker image builds and the stack runs at real
time with `sensor_profile:=full`.

**Implementation:** Build and run on a cloud GPU (`bash docker/sim/run_docker.sh`).

**Test:**

```bash
docker exec -it swarm_sim_cloud check-gpu
docker exec -it swarm_sim_cloud smoke-test
```

**Success Criteria:** `check-gpu` shows the NVIDIA renderer, not llvmpipe.
The smoke test passes: odom ≥ 28 Hz, imu ≥ 190 Hz, ground_truth_pose
≥ 28 Hz, map and `/global_map` ≥ 1.8 Hz, depth ≥ 28 Hz, RTF ≥ 0.8.

**Common Failure Cases:** Missing NVIDIA Container Toolkit (llvmpipe);
VirtualGL problems in viewer mode (try `VGLRUN=0`).

**Owner:** simulation / Docker workstream.

---

### N2 — Visual verification of the latest GUI polish

**Objective:** Confirm the Gazebo window matches the "What you see in the
Gazebo window" table in [`../README.md`](../README.md).

**Test:** Run `./run_single_window.sh` and check the drone colours,
battery stripe, front propellers, green LED, halo ring, heading line,
altitude stem, the per-drone floor tiles, white overlap tiles, red
obstacle tiles, gold frontiers, exclusion rings, paths and trails, and
the coverage and overlap bars above the north wall.

**Success Criteria:** Every element renders as described, in the GUI on
WSL2 and in the browser viewer on a GPU host.

**Owner:** lead (coordination).

---

### N3 — Replace ground-truth odometry with VSLAM/VIO

**Objective:** Drive the swarm from an estimated pose instead of Gazebo
odometry, without changing the exploration code.

**Implementation:** A VSLAM/VIO node per drone that publishes in place of
`/{ns}/odom`, the topic that `depth_mapper`, `flight_controller`,
`task_allocator`, `gz_overlay` and `swarm_sim_bridge` consume. The Isaac
ROS VSLAM launch in `vslam/` is not wired in yet, and the drone model has
no stereo infrared cameras yet, so either add them to
`simulation/models/quadrotor_px4.sdf` or use a monocular/RGB-D VIO with the
existing RGB, depth and IMU sensors.

**Test:** Run the same scenario as on ground truth and compare coverage,
time to `EXPLORATION COMPLETE` and the maps.

**Success Criteria:** The swarm completes exploration on estimated pose
alone; `/{ns}/ground_truth_pose` is used only for evaluation.

**Common Failure Cases:** Tracking loss in texture-poor areas; drift that
smears the merged map; frame or timestamp mismatches.

**Owner:** perception / VSLAM workstream.

---

### N4 — ATE evaluation once an estimated pose exists

**Objective:** Measure ATE RMSE against the < 0.10 m target and replace the
placeholder offline pipeline.

**Implementation:** Make `validation/parse_rosbag.py` read real bags
(ground truth plus estimated pose, and `/swarm/metrics/*`). Make
`validation/validate.py` align and synchronise the poses, plot measured
coverage and overlap, and check the targets instead of using hard-coded
values and an unconditional `PASS`. Use the live overlap definition
(cells seen by ≥ 2 drones) and add a task-level overlap metric (see
[`evaluation.md`](evaluation.md) §1, §3 and §4).

**Dependencies:** N3.

**Success Criteria:** Every number in the report comes from a recorded
run, and the report can be reproduced from the bag.

**Owner:** lead (validation).

---

### N5 — PX4 offboard backend

**Objective:** Optionally fly the drones with PX4 SITL instead of
`MulticopterVelocityControl`, keeping `/{ns}/cmd_pose` as the interface.

**Implementation:** Start from `src/control` (needs `px4_msgs`) and
`simulation/launch/start_px4_sitl.sh` / `start_px4_swarm.sh`. Keep the
current velocity-control path as the default backend.

**Test:** Three PX4 instances arm, take off and follow `cmd_pose`
waypoints from the task allocator.

**Owner:** perception / control workstream.

---

## Workstreams

`CONTRIBUTING_SWARM_WORKSTREAMS.md` splits ownership by directory:
lead → `coordination/`, `validation/`, root launchers; simulation →
`sim/`, `simulation/`, `docker/`; perception/control → `vslam/`,
`src/control/`, `src/localization/`, `src/mapping/`. Its task lists still
describe the planned PX4 / MAVROS / stereo-IR VSLAM work (Iris models, PX4
MAVLink ports, a `curved_gridroom.sdf` world), which the current code does
not use yet; a status note at its top says so. Trust the code and
[`../README.md`](../README.md).

---

# AI-Agent Friendly Requirements

Any AI coding agent working on this repository must:

1. Read [`../README.md`](../README.md) and this file.
2. Work on the `NEXT TARGET` above unless told otherwise.
3. Build only `coordination` and `simulation`
   (`colcon build --packages-select coordination simulation`).
4. Run `python3 coordination/test_swarm_coordination.py` after changing
   `coordination/algorithms.py` or the nodes that use it.
5. Preserve existing interfaces (topics, message types, frames, launch
   arguments) unless a roadmap item requires changing them.
6. Not report output of the placeholder validation scripts as measured
   results (see [`evaluation.md`](evaluation.md) §4).
7. Update the relevant `mdFiles/*.md` file whenever the architecture
   changes, and explain any deviation in the commit or PR description.
