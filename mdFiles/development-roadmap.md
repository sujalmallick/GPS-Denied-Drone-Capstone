# Development Roadmap

```text
CURRENT TARGET = M1
```

Do not implement M2+ before M1 is working and tested. Do not skip ahead to
M10 or M11 under any circumstances — the ground-truth-first strategy (see
[`localization.md`](localization.md)) depends on the exploration algorithm
being validated before localization is made harder.

Each milestone below lists: Objective, Implementation, Dependencies, Test,
Success Criteria, Expected Output, Failure Cases, and What Should NOT Be
Implemented Yet.

---

## M1 — Gazebo + PX4 drone works

**Objective:** A drone model flies (manually or via a basic PX4 command) in
a Gazebo Harmonic world, with PX4 SITL as the flight controller.

**Implementation:** Gazebo world + drone model under `simulation/worlds/`
and `simulation/models/`; launch files under `simulation/launch/`; PX4 SITL
started against the Gazebo instance.

**Dependencies:** Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, PX4 SITL
installed and building.

**Test:** Launch Gazebo + PX4 SITL together; arm the drone and command
takeoff/land through PX4's own tools (e.g., its shell or QGroundControl).

**Success Criteria:** Drone arms, takes off, hovers, and lands in Gazebo
without crashing or diverging.

**Expected Output:** A running Gazebo instance showing the drone
responding to PX4 commands; no ROS 2 involvement required yet.

**Common Failure Cases:** Gazebo/PX4 version mismatch; missing SITL
build target; world fails to load; drone spawns outside world bounds.

**What Should NOT Be Implemented Yet:** ROS 2 nodes, sensors beyond what
PX4 SITL provides by default, any mapping/exploration code.

---

## M2 — ROS 2 ↔ PX4 communication works

**Objective:** ROS 2 can observe PX4 state and send basic commands through
uXRCE-DDS.

**Implementation:** uXRCE-DDS agent running alongside PX4 SITL; a minimal
ROS 2 package (`src/control/` or equivalent) that subscribes to a PX4 status
topic and can send an arm/offboard command.

**Dependencies:** M1 complete; uXRCE-DDS agent/client set up.

**Test:** With PX4 SITL + Gazebo running, start the uXRCE-DDS agent and a
ROS 2 node; confirm PX4 status topics are visible via `ros2 topic list`
and `ros2 topic echo`.

**Success Criteria:** ROS 2 reliably receives PX4 telemetry and can
successfully issue at least one command PX4 acts on (e.g., arm).

**Expected Output:** ROS 2 topics reflecting PX4 state; a command sent from
ROS 2 visibly changes drone state in Gazebo.

**Common Failure Cases:** uXRCE-DDS agent/client not linked; DDS domain ID
mismatch; missing micro-ROS/uXRCE-DDS topic mapping configuration.

**What Should NOT Be Implemented Yet:** Sensor processing, mapping,
exploration logic, waypoint navigation.

---

## M3 — Simulated sensors and pose available

**Objective:** Camera, IMU, and depth sensor data are published as ROS 2
topics, and a pose source (ground truth from Gazebo) is available in ROS 2.

**Implementation:** Sensor plugins configured on the drone model in Gazebo;
ROS 2 bridge/topics exposing image, IMU, depth, and ground-truth pose.

**Dependencies:** M2 complete.

**Test:** With the full stack running, verify each sensor topic publishes
at an expected rate and with sane data (e.g., image not blank, IMU not
NaN).

**Success Criteria:** Camera, IMU, depth, and ground-truth pose are all
available as ROS 2 topics simultaneously, at consistent, documented rates.

**Expected Output:** `ros2 topic hz` shows stable rates for each sensor
topic; RViz2 can display the camera image and IMU data.

**Common Failure Cases:** Sensor plugin misconfiguration; topic name
mismatches between Gazebo bridge and expected ROS 2 names; frame_id not
set on sensor messages.

**What Should NOT Be Implemented Yet:** VSLAM/VIO, occupancy mapping,
frontier logic.

---

## M4 — Occupancy map works

**Objective:** A 2D occupancy grid is built and updated in real time using
ground-truth pose and depth/sensor data.

**Implementation:** Mapping node (`src/mapping/`) consuming pose + depth,
publishing a `nav_msgs/OccupancyGrid`-equivalent map.

**Dependencies:** M3 complete. Uses ground-truth pose, not VSLAM
(see [`localization.md`](localization.md), "ground truth first").

**Test:** Fly the drone manually (or via scripted PX4 commands) around a
Gazebo world with obstacles; observe the map in RViz2.

**Success Criteria:** FREE, OCCUPIED, and UNKNOWN cells are all correctly
represented and update as the drone explores; obstacles in the world appear
as OCCUPIED cells in roughly the correct location.

**Expected Output:** A live-updating occupancy grid visible in RViz2.

**Common Failure Cases:** Map/pose frame mismatch producing a smeared or
offset map; depth data not correctly projected into the map frame; map
not persisting previously observed FREE/OCCUPIED cells.

**What Should NOT Be Implemented Yet:** Frontier detection, clustering,
waypoint generation, VSLAM.

---

## M5 — Frontier detection works

**Objective:** Given an occupancy grid, correctly identify frontier cells
(FREE cells adjacent to UNKNOWN cells).

**Implementation:** Frontier detector (`src/exploration/`) consuming the
occupancy grid from M4 and publishing/returning a list of frontier cells.

**Dependencies:** M4 complete.

**Test:** Run against a partially explored map (from M4) and visually
verify detected frontier cells align with the FREE/UNKNOWN boundary.

**Success Criteria:** All FREE cells adjacent to UNKNOWN cells are
detected; no OCCUPIED-adjacent-only cells are misclassified as frontiers.

**Expected Output:** Frontier cells visualized (e.g., as a marker overlay
in RViz2) matching the visible FREE/UNKNOWN boundary in the map.

**Common Failure Cases:** Off-by-one errors at map edges; treating map
boundary cells incorrectly as frontiers; performance issues on large maps
from naive per-cell scanning.

**What Should NOT Be Implemented Yet:** Clustering, scoring, selection,
waypoint generation.

---

## M6 — Frontier clustering works

**Objective:** Group connected frontier cells into discrete candidate
regions.

**Implementation:** Clustering step (e.g., connected-component labeling)
added after frontier detection.

**Dependencies:** M5 complete.

**Test:** Run on a map with multiple distinct unexplored regions; verify
each region produces one cluster.

**Success Criteria:** Frontier cells that are spatially connected are
grouped into the same cluster; disconnected frontier regions produce
separate clusters.

**Expected Output:** Distinct clusters visualized with different colors or
IDs in RViz2.

**Common Failure Cases:** Overly aggressive connectivity merging unrelated
regions; overly strict connectivity fragmenting one region into many small
clusters; clusters below a meaningful size not filtered out.

**What Should NOT Be Implemented Yet:** Scoring, selection, waypoint
generation.

---

## M7 — Frontier selection works

**Objective:** Score each frontier cluster and select the best one using
the initial scoring model.

**Implementation:** Scoring function `score = information_gain -
distance_cost` applied per cluster; selection logic picks the
highest-scoring valid cluster.

**Dependencies:** M6 complete.

**Test:** Run on a map with multiple clusters of varying size/distance;
verify the selected cluster matches the expected best-scoring one given the
formula.

**Success Criteria:** Selection is deterministic given the same map and
pose, matches the documented scoring formula, and is modular (scoring
function is swappable without changing the selection logic's interface).

**Expected Output:** The selected target cluster is visualized/logged with
its score.

**Common Failure Cases:** Distance cost computed in the wrong units/frame;
information gain not normalized against distance cost, causing one term to
always dominate; ties not handled deterministically.

**What Should NOT Be Implemented Yet:** Waypoint generation, PX4 offboard
navigation.

---

## M8 — Waypoint generation works

**Objective:** Convert a selected frontier cluster into a valid, safe
waypoint.

**Implementation:** Waypoint generator (`src/planning/`) picking a
representative point from the selected cluster, checking map validity and
obstacle clearance, and producing a waypoint in the documented coordinate
frame.

**Dependencies:** M7 complete.

**Test:** Run against several selected clusters, including ones near
obstacles; verify generated waypoints are always in FREE space with
sufficient clearance.

**Success Criteria:** No waypoint is ever generated inside OCCUPIED or
UNKNOWN space; clearance from obstacles meets a configurable minimum.

**Expected Output:** A validated waypoint (position + frame) logged/
visualized for each selected frontier.

**Common Failure Cases:** Representative point chosen exactly on the
FREE/UNKNOWN boundary (too close to unknown space); insufficient clearance
check causing waypoints too close to obstacles; wrong coordinate frame
sent downstream.

**What Should NOT Be Implemented Yet:** Sending the waypoint to PX4 /
closed-loop navigation (that begins at M9); the full loop.

---

## M9 — Drone flies autonomously to a single waypoint

**Objective:** Given one generated waypoint, the drone autonomously
navigates to it using PX4 offboard control and the system detects arrival.

**Implementation:** PX4 interface node sends the waypoint as an offboard
setpoint; arrival is detected via position tolerance check.

**Dependencies:** M8 complete; M2's ROS 2 ↔ PX4 link.

**Test:** Manually supply a single generated waypoint (not yet from the
full loop) and verify the drone flies to it and arrival is correctly
detected.

**Success Criteria:** Drone reaches the commanded waypoint within a
configurable tolerance and the system correctly reports arrival exactly
once.

**Expected Output:** Drone visibly flies to the target in Gazebo; a
"waypoint reached" signal is published/logged.

**Common Failure Cases:** PX4 rejects the offboard setpoint (mode not
correctly entered); drone oscillates near the target without arrival being
detected; arrival check triggers prematurely due to loose tolerance.

**What Should NOT Be Implemented Yet:** The full repeating explore loop;
map updates driving new waypoint selection in a loop.

---

## M10 — Full autonomous exploration loop

**Objective:** Chain M4–M9 into the complete repeating loop: sense → map →
detect → cluster → score → select → waypoint → navigate → repeat, using
ground-truth pose.

**Implementation:** Exploration manager node orchestrating all autonomy
components in a loop, with a configurable stop condition (see
[`exploration.md`](exploration.md) §"Exploration completion").

**Dependencies:** M4–M9 all individually working and tested.

**Test:** Run the full loop in an unexplored Gazebo world until the stop
condition triggers; verify map coverage grows monotonically (or
near-monotonically) over time.

**Success Criteria:** The drone autonomously explores without manual
intervention and correctly stops when the configured completion condition
is met.

**Expected Output:** A fully explored (or coverage-threshold-satisfied) map
at the end of the run, with a log of waypoints visited.

**Common Failure Cases:** Loop gets stuck re-selecting the same
already-visited/invalid frontier; exploration never terminates because the
stop condition is misconfigured; map updates and frontier detection race
against each other.

**What Should NOT Be Implemented Yet:** VSLAM/VIO-based localization
(still using ground truth); multi-drone anything.

---

## M11 — Replace ground-truth pose with VSLAM/VIO

**Objective:** Swap the localization source from Gazebo ground truth to a
VSLAM/VIO pipeline (e.g., ORB-SLAM3) consuming camera + IMU, without
changing the exploration algorithm.

**Implementation:** VSLAM/VIO node (`src/localization/`) replacing the
ground-truth publisher on the same pose topic/frame contract established
in M3–M4.

**Dependencies:** M10 fully working on ground truth. Camera/IMU topics
from M3.

**Test:** Run the same exploration scenario used to validate M10, now
sourced from VSLAM/VIO pose instead of ground truth; compare resulting map
and path quality against the M10 ground-truth run.

**Success Criteria:** The exploration loop runs successfully using
VSLAM/VIO pose alone, with no code changes required in mapping or
exploration components (only the pose source changes).

**Expected Output:** A completed exploration run using only camera+IMU for
localization; a pose-drift comparison against ground truth for evaluation
purposes.

**Common Failure Cases:** VSLAM tracking loss in low-texture areas;
scale/drift issues typical of monocular VIO; pose topic/frame mismatch
between the VSLAM node and what mapping expects.

**What Should NOT Be Implemented Yet:** Multi-drone anything; hardware
deployment.

---

## M12 — GPS-denied exploration evaluation

**Objective:** Systematically evaluate the full GPS-denied exploration
system (M11's VSLAM/VIO-based loop) against the metrics defined in
[`evaluation.md`](evaluation.md).

**Implementation:** Evaluation scripts (`evaluation/`) that consume run logs
and produce metrics/plots: coverage, exploration time, distance traveled,
waypoint count, unexplored area, redundant exploration, computation time,
map quality, localization error, collision count.

**Dependencies:** M11 complete.

**Test:** Run several exploration episodes across different worlds/seeds;
compute and review the full metric set for each.

**Success Criteria:** All metrics in [`evaluation.md`](evaluation.md) can
be computed automatically from a run's logs; results are reproducible
across repeated runs of the same configuration.

**Expected Output:** A metrics report (tables/plots) per run, suitable as a
baseline for future comparisons (including the eventual swarm phase).

**Common Failure Cases:** Missing or incomplete logging from earlier
milestones preventing metric computation; metrics computed inconsistently
between runs due to undocumented parameter changes.

**What Should NOT Be Implemented Yet:** Multi-drone comparison metrics
(single-drone baseline only, per the Future Phase section below).

---

# Future Phase — Multi-Drone / Swarm Exploration (OUT OF SCOPE FOR PHASE 1)

**Do NOT implement any of the following now.** This section exists purely
to document direction so Phase-1 interfaces don't accidentally preclude it.

```text
Multiple drones
      ↓
Multi-agent task allocation
      ↓
Greedy distance-based assignment
      ↓
Region / overlap avoidance
      ↓
Voronoi partitioning
      ↓
Improved coordination algorithm
```

Potential future metrics (in addition to the Phase-1 metrics in
[`evaluation.md`](evaluation.md)):

* Coverage (multi-agent)
* Time
* Distance
* Overlap between agents
* Workload balance across agents
* Communication overhead
* Scalability with agent count

None of this is implemented in Phase 1. Phase 1 delivers a working,
evaluated single-drone GPS-denied exploration system; the swarm phase is a
separate, later effort building on top of it.

---

# AI-Agent Friendly Requirements

> These files are the source of truth for AI coding agents working on the
> project.

Any AI coding agent working on this repository must:

1. Read `README.md`.
2. Read `docs/development-roadmap.md` (this file).
3. Read the architecture documentation (`docs/architecture.md`,
   `docs/system-design.md`, `docs/ros2-architecture.md`).
4. Determine the current milestone from the `CURRENT TARGET` marker above.
5. Implement only the current milestone.
6. Not implement future milestones prematurely, even if they seem easy or
   related.
7. Preserve existing interfaces (topics, message types, frames) unless a
   milestone explicitly requires changing them.
8. Update the relevant `docs/*.md` file whenever the actual architecture
   changes, so documentation never silently drifts from implementation.
9. Test each milestone (per its "Test" and "Success Criteria" above) before
   proceeding to the next.
10. Never silently replace a major technology choice from
    [`README.md`](../README.md)'s technology stack without documenting why.
11. Explicitly explain any deviation from the documented architecture in
    the relevant doc file and, ideally, in the commit/PR description.
