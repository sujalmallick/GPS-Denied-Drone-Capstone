# GPS-Denied Autonomous Drone Exploration (Software-Only Simulation)

## Project Objective

Build a **single simulated drone** that can autonomously explore an unknown,
GPS-denied environment: build a map as it flies, detect unexplored regions,
choose where to go next, generate waypoints, and fly there — repeating until
the environment is sufficiently explored. Everything runs in simulation.
There is no physical hardware in this project.

## Problem Statement

Traditional drone navigation relies on GPS for localization. GPS is
unavailable or unreliable indoors, in urban canyons, underground, and in many
disaster-response scenarios. A GPS-denied autonomy stack must localize itself
using onboard sensors (camera + IMU) and must decide where to explore next
using only what it has already observed (an occupancy map), not any
externally supplied position or pre-built map.

This project builds and validates such a stack, entirely in simulation,
starting from a single drone.

## Software-Only Scope

* **No hardware** is designed, targeted, or assumed at any point in Phase 1.
* **No Jetson Orin Nano or any embedded deployment target.** Any reference to
  onboard compute in a hypothetical future phase is explicitly out of scope
  here.
* Everything runs on a development workstation using Gazebo, PX4 SITL, and
  ROS 2.
* The deliverable is a working simulation stack plus the autonomy code that
  could later be ported to hardware — porting itself is not part of Phase 1.

## Main Features (Phase 1 target)

* Single simulated drone in Gazebo, flown by PX4 SITL.
* ROS 2 ↔ PX4 communication via uXRCE-DDS.
* Occupancy-grid mapping built from simulated sensor data.
* Frontier-based exploration: detection → clustering → scoring → selection.
* Waypoint generation and offboard control to autonomously visit frontiers.
* A ground-truth-pose development mode, followed by a VSLAM/VIO pose mode,
  so localization and exploration can be debugged independently (see
  [`docs/localization.md`](docs/localization.md)).
* A full autonomous exploration loop with a configurable stopping condition.
* Evaluation tooling (Python/NumPy/Pandas/Matplotlib) to measure exploration
  performance.

## Explicitly Out of Scope for Phase 1

* Multiple drones / swarm coordination (see
  [`docs/development-roadmap.md`](docs/development-roadmap.md) future-phase
  section).
* Any real hardware, embedded deployment, or Jetson-specific tooling.
* Kubernetes, Docker, cloud services, databases, microservices, or web
  applications.

## Architecture Overview

Two layers, kept clearly separate throughout the documentation:

* **Existing infrastructure (integration work):** Gazebo, PX4 SITL, ROS 2,
  uXRCE-DDS, sensor simulation, flight control.
* **Project-specific autonomy (the actual contribution):** mapping, frontier
  detection, frontier clustering, frontier scoring, frontier selection,
  waypoint generation, exploration manager, evaluation.

Full details: [`docs/architecture.md`](docs/architecture.md) and
[`docs/system-design.md`](docs/system-design.md).

High-level data flow:

```text
Gazebo → Simulated Drone → Camera + IMU + Depth → VSLAM/VIO → Estimated Pose
   → Occupancy Map → Frontier Detection → Frontier Clustering
   → Frontier Selection → Waypoint Generation → ROS 2 Offboard Control
   → PX4 SITL → Drone Movement → Map Update → (repeat)
```

Early in development, "VSLAM/VIO → Estimated Pose" is replaced by Gazebo
ground-truth pose. See [`docs/localization.md`](docs/localization.md) and
milestone M11 in the roadmap.

## Technology Stack

| Area | Choice |
|---|---|
| OS | Ubuntu 22.04 LTS |
| Robotics middleware | ROS 2 Humble |
| Simulator | Gazebo Harmonic |
| Flight stack | PX4 SITL |
| ROS 2 ↔ PX4 bridge | uXRCE-DDS |
| Visualization | RViz2, QGroundControl |
| Core autonomy code | C++ |
| Launch / config / analysis | Python |
| Perception | OpenCV, VSLAM/VIO (ORB-SLAM3 or comparable) |
| Mapping | 2D occupancy grid (path toward 3D/voxel later) |
| Exploration | Frontier detection, clustering, scoring, selection, waypoint generation |
| Build | CMake, colcon |
| Source control | Git, GitHub |
| Evaluation | Python, NumPy, Pandas, Matplotlib |

See [`docs/simulation.md`](docs/simulation.md) and
[`docs/ros2-architecture.md`](docs/ros2-architecture.md) for details.

## Current Development Milestone

```text
CURRENT TARGET = M1 — Gazebo + PX4 drone works
```

The project is developed incrementally, M1 through M12, and later milestones
must not be implemented ahead of earlier ones. Full milestone definitions:
[`docs/development-roadmap.md`](docs/development-roadmap.md).

## How to Run the Project (Eventually)

This section will be filled in as milestones are completed. At minimum, once
M1–M3 exist, running the project will look like:

1. Launch the Gazebo world with the simulated drone (`simulation/launch/`).
2. Start PX4 SITL and confirm the uXRCE-DDS agent/client link is up.
3. Launch the ROS 2 autonomy stack (`launch/`) — localization, mapping,
   exploration, waypoint generation, PX4 interface.
4. Observe the drone take off and begin the explore/map/waypoint loop.
5. Watch progress in RViz2 (map, frontiers, current waypoint, drone pose).

Exact commands will be added once the corresponding milestone's launch files
exist — see [`docs/simulation.md`](docs/simulation.md).

## Expected Simulation Result (Phase 1 Demonstration)

A single simulated drone starts in an unknown Gazebo environment, takes off,
and autonomously builds an occupancy map from its simulated sensors. It
repeatedly detects frontiers (boundaries between known-free and unknown
space), clusters and scores them, selects the best one, generates a
waypoint, and flies there — updating the map as it goes — until an
exploration-complete condition is met (e.g., no valid frontiers remain, a
coverage threshold is reached, or a safety/time/distance limit triggers). In
later milestones this runs on VSLAM/VIO pose estimates instead of simulator
ground truth, demonstrating GPS-denied autonomy.

## Roadmap

| Milestone | Goal |
|---|---|
| M1 | Gazebo + PX4 drone works |
| M2 | ROS 2 ↔ PX4 communication works |
| M3 | Simulated sensors and pose available |
| M4 | Occupancy map works |
| M5 | Frontier detection works |
| M6 | Frontier clustering works |
| M7 | Frontier selection works |
| M8 | Waypoint generation works |
| M9 | Drone flies autonomously to waypoint |
| M10 | Full autonomous exploration loop |
| M11 | Replace ground-truth pose with VSLAM/VIO |
| M12 | GPS-denied exploration evaluation |

Full details, including test procedures and success criteria, are in
[`docs/development-roadmap.md`](docs/development-roadmap.md).

A future swarm/multi-drone phase is documented for context only and is
**not** part of Phase 1 — see the "Future Phase" section of the roadmap.

## Documentation Index

* [`docs/architecture.md`](docs/architecture.md) — system architecture, data/control flow
* [`docs/system-design.md`](docs/system-design.md) — requirements, interfaces, coordinate frames
* [`docs/development-roadmap.md`](docs/development-roadmap.md) — M1–M12 plus future phase
* [`docs/ros2-architecture.md`](docs/ros2-architecture.md) — nodes, topics, messages
* [`docs/simulation.md`](docs/simulation.md) — Gazebo/PX4 simulation setup
* [`docs/localization.md`](docs/localization.md) — ground truth → VSLAM/VIO
* [`docs/mapping.md`](docs/mapping.md) — occupancy grid design
* [`docs/exploration.md`](docs/exploration.md) — frontier exploration algorithm
* [`docs/algorithms.md`](docs/algorithms.md) — per-algorithm specification
* [`docs/evaluation.md`](docs/evaluation.md) — metrics and measurement methodology
* [`docs/troubleshooting.md`](docs/troubleshooting.md) — common failure modes

## For AI Coding Agents

This repository's `.md` files are the source of truth for implementation.
See the "AI-Agent Friendly Requirements" section in
[`docs/development-roadmap.md`](docs/development-roadmap.md) before writing
any code.
