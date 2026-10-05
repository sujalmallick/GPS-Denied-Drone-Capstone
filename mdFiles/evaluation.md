# Evaluation

This document defines how the three-drone swarm is evaluated, the targets
the project set, and what can be measured today. There are two separate
sources of numbers:

* **(a) Live metrics.** These are real: they are computed from the running
  simulation (§2).
* **(b) Offline pipeline** (`validation/`). This is currently a placeholder
  and mock-up. Its numbers must not be cited (§4).

## 1. Metrics

| Metric | Definition | Source |
|---|---|---|
| Coverage | Known cells (free or occupied) inside the arena bounds ÷ arena cells × 100 % | Live: `/swarm/metrics/coverage_pct` |
| Overlap rate | Cells observed by **≥ 2** drones ÷ cells observed by ≥ 1 drone × 100 % | Live: `/swarm/metrics/overlap_rate` |
| Real-time factor | Simulated time ÷ wall time | Live: Gazebo `/stats`, smoke test |
| Topic rates | Messages per second of simulated time | Live: smoke test |
| Exploration time | Simulated time from launch to the allocator's `EXPLORATION COMPLETE` log line | Live: run log + `/clock` |
| ATE RMSE | $\sqrt{\tfrac{1}{N}\sum \lVert p^{gt}_i - p^{est}_i \rVert^2}$ per drone, plus yaw RMSE | Offline: `validation/validate.py`. Not meaningful yet (§3) |

Coverage and overlap are computed by `coordination/map_merger.py` at 2 Hz on
the 0.2 m, 200 × 200 grids, with arena bounds `[-12, 12, -9, 11]` m. The
formulas are `coverage_pct` and `overlap_rate` in
`coordination/algorithms.py` and are covered by
`coordination/test_swarm_coordination.py`.

**Overlap definition mismatch.** The `validate.py` docstring (its "Eq 12")
defines overlap as cells seen by *all three* drones over cells seen by any
drone. The live metric counts cells seen by *two or more* drones. This
document and the README use the code's live definition. The offline
definition gives lower numbers and should be aligned with the live one.

## 2. Live Metrics (Real)

```bash
ros2 topic echo /swarm/metrics/coverage_pct
ros2 topic echo /swarm/metrics/overlap_rate
gz topic -e -t /stats
```

The smoke test checks topic rates against simulated time and the
real-time factor. Minimums: odom 28 Hz, imu 190 Hz, ground_truth_pose
28 Hz, map and `/global_map` 1.8 Hz, depth 28 Hz (`full`) or 9 Hz
(`realtime`), RTF 0.8. On a laptop, turn the RTF check off:

```bash
python3 docker/sim/smoke_test.py --profile realtime --min-rtf 0
```

In the Docker container, run `docker exec -it swarm_sim_cloud smoke-test`
(see [`../docker/sim/README.md`](../docker/sim/README.md)).

To keep a run for later analysis, record the live topics:

```bash
ros2 bag record /clock /global_map /swarm/metrics/coverage_pct /swarm/metrics/overlap_rate \
  /drone_1/ground_truth_pose /drone_2/ground_truth_pose /drone_3/ground_truth_pose
```

## 3. Targets vs. Current Behaviour

| Metric | Target (team plan) | Current behaviour |
|---|---|---|
| Coverage | > 90 % | Reported as reached (not yet backed by a recorded run): [`../README.md`](../README.md) reports coverage typically passing 90 % in about 40 s of simulated time, followed by `EXPLORATION COMPLETE` |
| Overlap rate | < 5 % | **Not met** under the live definition: about 30–45 % by the time coverage reaches 90 % |
| ATE RMSE | < 0.10 m | **Not meaningful yet**: the stack uses Gazebo ground-truth odometry, so there is no estimated pose to compare |
| Real-time factor | ≥ 0.8 (smoke test) | About 0.6–0.75× under WSL2. Real time on an NVIDIA GPU host is expected but not yet verified |

The coverage, overlap and ATE targets come from
`CONTRIBUTING_SWARM_WORKSTREAMS.md`.

**Why overlap is high.** The forward depth cameras have an 87° field of
view and the depth mapper uses an 8 m range, so in a 24 m × 20 m room a
drone sees much of the room from almost anywhere. Algorithm 1 keeps the
drones' *targets and paths* apart, using 2.0 m exclusion zones and gain
discounting. It does not keep sensor footprints apart, which is what
cell overlap measures. A task-level overlap metric, such as target or
path conflicts, would match the design's guarantee better. That is future
work.

## 4. Offline Pipeline (Placeholder)

```bash
python3 validation/parse_rosbag.py --name <path_to_bag>
python3 validation/validate.py --dir validation/output
```

Do not cite the output of these scripts as results. In the current code:

* `parse_rosbag.py` ignores the bag. It always writes synthetic circular
  trajectories with added noise to
  `validation/output/drone_*_trajectory.csv`. Its docstring names the
  topics `/drone_N/ground_truth/pose` and
  `/drone_N/visual_slam/tracking/vo_pose_covariance`. The first does not
  match the bridged `/{ns}/ground_truth_pose`, and nothing publishes the
  second yet.
* `validate.py` computes ATE and yaw RMSE from those synthetic CSVs. The
  pose alignment and time interpolation described in its docstring are
  not implemented. The coverage curve (94.6 %), the overlap bars (1.84 %
  vs. 24.3 %) and the "2.74x" speedup are hard-coded, and it prints
  `PASS` whatever the inputs are.

Before its numbers can be cited, the pipeline needs:

* real bag parsing (ground truth plus an estimated pose, and
  `/swarm/metrics/*`);
* time synchronisation and pose alignment;
* metrics computed from the data, including the live overlap definition;
* target checks that can fail.

This is roadmap item N4 in [`development-roadmap.md`](development-roadmap.md).

## 5. Methodology

* All nodes run on simulation time, so metrics are reported against
  simulated time. A lower real-time factor changes wall-clock duration,
  not the result.
* Use the default world (`simulation/worlds/simple_world.sdf`) and the
  fixed spawn points so runs are comparable (see
  [`simulation.md`](simulation.md)). Record `sensor_profile`,
  `lock_radius` and `altitude` with each run.
* Ground truth is an evaluation reference only. Once VSLAM/VIO replaces
  ground-truth odometry, ground truth must not feed the autonomy stack.
* Planned comparisons, none of them run yet: swarm vs. a single drone
  (time to 90 % coverage), Algorithm 1 vs. uncoordinated allocation, a
  `lock_radius` sweep, and per-drone ATE with an estimated pose.
