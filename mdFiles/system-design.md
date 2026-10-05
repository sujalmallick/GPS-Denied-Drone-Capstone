# System Design

## 1. Functional Requirements

| ID | Requirement |
|---|---|
| FR-1 | The system shall simulate a single drone in Gazebo, controlled via PX4 SITL. |
| FR-2 | The system shall estimate the drone's pose without using GPS. Early milestones may use simulator ground-truth pose as a development stand-in (see [`localization.md`](localization.md)). |
| FR-3 | The system shall build and maintain a 2D occupancy grid map from simulated sensor observations and the current pose estimate. |
| FR-4 | The system shall detect frontier cells (FREE cells adjacent to UNKNOWN cells) in the occupancy grid. |
| FR-5 | The system shall cluster connected frontier cells into candidate exploration regions. |
| FR-6 | The system shall score candidate frontier clusters using a modular scoring function (initially `information_gain - distance_cost`). |
| FR-7 | The system shall select the highest-scoring valid frontier as the next exploration target. |
| FR-8 | The system shall convert a selected frontier into a waypoint that is validated for map validity and obstacle clearance before being sent to the flight controller. |
| FR-9 | The system shall send waypoints to PX4 via ROS 2 offboard control and verify waypoint arrival. |
| FR-10 | The system shall repeat sense → map → detect → cluster → score → select → waypoint → navigate until an exploration-complete condition is met. |
| FR-11 | The system shall support at least these exploration-complete conditions, each independently configurable: no valid frontiers remain, coverage threshold reached, max flight time reached, max distance reached, safety condition triggered. |
| FR-12 | The system shall be able to run the exploration loop using either ground-truth pose (development mode) or VSLAM/VIO pose (target mode) without changing the exploration algorithm itself. |
| FR-13 | The system shall provide evaluation tooling to compute exploration metrics from a completed run (see [`evaluation.md`](evaluation.md)). |

## 2. Non-Functional Requirements

| ID | Requirement |
|---|---|
| NFR-1 | The system shall run entirely in software simulation; no physical hardware dependency of any kind. |
| NFR-2 | Core autonomy nodes (mapping, frontier detection/clustering/selection, waypoint generation) shall be implemented in C++ for performance and consistency with ROS 2 conventions; launch, configuration, and evaluation scripts shall be Python. |
| NFR-3 | The mapping and exploration algorithms shall be modular: the scoring function, the clustering method, and the localization source shall each be replaceable without rewriting the surrounding pipeline. |
| NFR-4 | The system shall be developed and validated incrementally, milestone by milestone, per [`development-roadmap.md`](development-roadmap.md); later milestones must not be implemented before earlier ones are working. |
| NFR-5 | Interfaces between components (topics, messages, coordinate frames) shall be documented before being relied upon by more than one component. |
| NFR-6 | The system shall not introduce infrastructure (databases, containers, microservices, cloud services) beyond what is needed to run a single-process robotics simulation stack. |
| NFR-7 | Any interface whose exact form depends on the installed PX4/ROS 2/Gazebo version shall be explicitly marked as version-dependent rather than hard-documented as fixed. |

## 3. System Boundaries

**In scope (Phase 1):**
* Single simulated drone.
* Full sense→map→explore→navigate loop in Gazebo + PX4 SITL.
* Ground-truth-pose development mode and VSLAM/VIO target mode.
* 2D occupancy-grid-based frontier exploration.
* Evaluation of a single drone's exploration run.

**Out of scope (Phase 1):**
* Multiple drones / swarm coordination (documented separately as a future
  phase in [`development-roadmap.md`](development-roadmap.md), not
  implemented now).
* Any physical hardware, embedded compute target, or hardware bring-up.
* 3D/voxel mapping as the primary representation (2D is primary; a path
  toward 3D is documented but not implemented in Phase 1).
* Anything beyond a single-process, single-machine simulation stack
  (no distributed systems, cloud, or web components).

## 4. Component Responsibilities

See [`architecture.md`](architecture.md) §2 for the full component table.
Each autonomy component owns exactly one stage of the pipeline and
communicates only through documented topics/messages — components do not
reach into each other's internal state.

## 5. Interfaces

Interfaces between autonomy components are ROS 2 topics/services/actions.
The authoritative list of node names, topics, and message types is
[`ros2-architecture.md`](ros2-architecture.md). This document defines the
*kinds* of interface each boundary needs; exact names are defined there to
avoid two documents disagreeing.

| Boundary | Interface kind |
|---|---|
| Sensors → Localization | Topic (image, IMU, depth streams) |
| Localization → Mapping | Topic (pose estimate) |
| Mapping → Frontier Detector | Topic (occupancy grid) |
| Frontier Detector → Clustering | Topic or in-process call (frontier cell list) |
| Clustering → Selector | Topic or in-process call (cluster list with properties) |
| Selector → Waypoint Generator | Topic or in-process call (selected frontier target) |
| Waypoint Generator → PX4 Interface | Topic (validated waypoint / setpoint) |
| PX4 Interface → PX4 | uXRCE-DDS bridged uORB topics |
| Exploration Manager → all autonomy nodes | Orchestration via topics/services/parameters |

Whether a given boundary is a ROS 2 topic vs. an in-process function call is
an implementation decision to be made at the corresponding milestone and
recorded in [`ros2-architecture.md`](ros2-architecture.md) once decided.

## 6. Coordinate Frames

At minimum, the following frames must be defined and kept consistent
(exact frame names to be finalized in [`ros2-architecture.md`](ros2-architecture.md)
and [`mapping.md`](mapping.md), following standard ROS `REP-105` conventions
where applicable):

* **World/map frame** — fixed frame the occupancy grid is expressed in.
* **Odometry frame** — frame the pose estimate is published in before any
  loop-closure/drift correction is applied.
* **Body frame** — drone-fixed frame (sensors are mounted relative to this).
* **Sensor frames** — camera frame, IMU frame, depth-sensor frame, each with
  a fixed, documented transform to the body frame.

The relationship between the ground-truth pose (used in early milestones)
and the VSLAM/VIO pose (used from M11 onward) must be expressed in the same
map frame so that mapping and exploration code do not need to know which
pose source is active. This is what allows localization and exploration to
be debugged independently — see [`localization.md`](localization.md) §"Why
ground truth first."

## 7. Data Flow

See [`architecture.md`](architecture.md) §3–§7 for the full data-flow
description; not repeated here to avoid duplication drift.

## 8. Failure Handling

| Failure | Expected handling |
|---|---|
| Pose estimate unavailable or stale | Exploration manager pauses navigation; does not command a waypoint based on stale/missing pose. |
| No frontiers found | Treated as a candidate exploration-complete condition (see FR-11), not an error. |
| Selected frontier fails waypoint validation (e.g., inside an obstacle) | Waypoint generator rejects it; frontier selector must supply the next-best candidate. |
| PX4 not in offboard mode / rejects setpoint | PX4 interface node reports failure to the exploration manager; manager retries or aborts per configurable policy. |
| Map update stalls (no new sensor data) | Treated as a fault condition; exploration manager should not proceed to frontier detection on a stale map. |
| VSLAM/VIO tracking loss (post-M11) | Documented explicitly in [`localization.md`](localization.md); exploration manager should hold position rather than continue navigating on an unreliable pose. |

Detailed symptom-based debugging guidance is in
[`troubleshooting.md`](troubleshooting.md).
