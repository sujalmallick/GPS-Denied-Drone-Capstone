# Evaluation

Evaluation is implemented at milestone M12, using Python, NumPy, Pandas,
and Matplotlib against logged data from exploration runs. This document
defines the metrics and how each is measured; it does not implement the
evaluation code.

## 1. Metrics

| Metric | Definition | How Measured |
|---|---|---|
| Exploration coverage | Fraction of the (estimable) total explorable area that has been marked FREE or OCCUPIED by the end of the run | `(FREE_cells + OCCUPIED_cells) / total_explorable_cells`, computed from the final occupancy grid; `total_explorable_cells` estimated from the known world bounds/geometry for evaluation purposes only |
| Exploration time | Wall-clock or simulated time from mission start to exploration-complete | Timestamp difference between run start and the exploration-complete event logged by the Exploration Manager |
| Total distance traveled | Cumulative path length flown by the drone | Sum of per-tick displacement magnitudes from the logged pose trajectory |
| Number of waypoints | Count of waypoints generated and/or successfully reached | Count of waypoint-generation and waypoint-arrival events in the run log |
| Unexplored area | Fraction of the map still UNKNOWN at run end | `UNKNOWN_cells / total_cells` from the final occupancy grid |
| Redundant exploration | Degree to which the drone revisits already-FREE areas without gaining new coverage | Fraction of flight time/distance spent in cells that were already FREE at the time of traversal, vs. cells that transitioned FREE for the first time |
| Computational time | Per-stage processing latency (mapping update, frontier detection, clustering, scoring, waypoint generation) | Wall-clock timing instrumentation around each pipeline stage, logged per invocation |
| Map quality | Accuracy of the built occupancy grid relative to the true world geometry | Comparison of final occupancy grid against Gazebo's known ground-truth world geometry (available in simulation only, for evaluation purposes) — e.g., cell-wise agreement rate |
| Localization error | Deviation of the pose estimate from Gazebo ground truth | Position error (e.g., Euclidean distance) between VSLAM/VIO pose and ground-truth pose at matched timestamps, logged throughout the run (ground truth used only as an offline evaluation reference, never fed into the autonomy pipeline post-M11 — see [`localization.md`](localization.md)) |
| Collision count | Number of collision events during the run | Collision events reported by Gazebo's physics/contact sensing, counted from the run log |

## 2. Measurement Methodology

* All metrics are computed from **logged data**, not recomputed live during
  the run — the exploration stack should log the necessary raw data
  (pose trajectory, occupancy grid snapshots, event timestamps, per-stage
  timing) during execution, and `evaluation/` scripts consume these logs
  after the fact.
* Ground-truth data (world geometry, ground-truth pose) is available only
  because this is a simulation; it is used exclusively for evaluation and
  must never be fed back into the autonomy pipeline once M11 is
  implemented (see [`localization.md`](localization.md) §4).
* Runs used for evaluation should use deterministic, documented world
  configurations (see [`simulation.md`](simulation.md) §5) so results are
  reproducible and comparable across code changes.
* Each metric should be computed per-run, with support for aggregating
  across multiple runs (e.g., mean/variance across repeated episodes in the
  same world, or across different worlds) once enough runs exist.

## 3. Output Format

Evaluation scripts should produce:

* A per-run metrics table (e.g., a Pandas DataFrame, exportable to CSV).
* Plots (Matplotlib) for time-series metrics where useful — e.g., coverage
  over time, localization error over time/distance.
* A human-readable summary suitable for inclusion in a milestone report
  when M12 is completed.

Exact file formats/paths under `evaluation/` are an implementation detail
to be decided at M12 and are not fixed by this document.

## 4. Future Baseline Comparisons

The single-drone metrics defined above are intended to serve as the
baseline against which the future swarm/multi-drone phase (see
[`development-roadmap.md`](development-roadmap.md), "Future Phase") is
compared. When that phase begins, additional multi-agent metrics (overlap,
workload balance, communication overhead, scalability) will be layered on
top of this same measurement methodology, not implemented now.
