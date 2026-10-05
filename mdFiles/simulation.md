# Simulation

> Scope: this file documents the simulation stack (Gazebo + PX4 SITL) only.
> No physical hardware is involved anywhere in this project.

## 1. Gazebo Environment

* **Simulator:** Gazebo Harmonic.
* **Purpose:** Provide physics, a 3D world with obstacles, and simulated
  sensor data for the drone.
* Worlds live under `simulation/worlds/`; each world is a distinct
  environment used to test exploration under different obstacle layouts,
  sizes, and complexity.
* Early worlds should be simple (few large obstacles, bounded area) so that
  M1–M4 can be validated without exploration-algorithm complexity
  confounding basic sim/flight issues. More complex, maze-like, or larger
  worlds should be introduced once M10 (full loop) is stable.

## 2. PX4 SITL

* **Role:** Software-in-the-loop flight controller — handles attitude
  control, position control, mode management (arm/disarm, offboard,
  takeoff/land), and safety checks.
* Runs as a separate process from Gazebo and from the ROS 2 autonomy stack,
  connected to Gazebo for state/actuation and to ROS 2 via uXRCE-DDS.
* This project does not modify PX4's internal control loops — it only
  interacts with PX4 through its standard offboard control interface (see
  [`ros2-architecture.md`](ros2-architecture.md) §5).

## 3. Drone Model

* A single multirotor model (exact frame type to be selected at M1 — a
  standard PX4-supported quadrotor is the default choice unless there is a
  specific reason to deviate).
* The model carries the sensor suite described below; sensors are attached
  as Gazebo sensor plugins on the model.
* No hardware-specific model variants (e.g., tied to a particular real
  drone's mass/inertia) are required — a standard simulated frame is
  sufficient since this project never targets real hardware.

## 4. Simulated Sensors

| Sensor | Purpose | Consumed by |
|---|---|---|
| Camera (RGB) | Visual input for VSLAM/VIO | Localization Node |
| IMU | Inertial input for VSLAM/VIO and PX4's own state estimation | Localization Node, PX4 |
| Depth sensor (depth camera or simulated point cloud) | Obstacle/free-space observations for mapping | Mapping Node |

Exact sensor plugin configuration (resolution, field of view, noise model,
update rate) is decided at M3 implementation time and should be recorded
here once finalized, since it directly affects VSLAM/VIO quality and
mapping resolution.

## 5. World Design

Guidelines for worlds used in this project:

* Bounded, enclosed, or clearly limited-extent environments (fully open,
  unbounded worlds make "exploration complete" ill-defined).
* A mix of obstacle sizes/shapes to exercise frontier detection and
  clearance checks meaningfully.
* Enough visual texture for VSLAM/VIO to track features (relevant from M11
  onward) — texture-less environments should be avoided or noted as a known
  VSLAM stress case (see [`troubleshooting.md`](troubleshooting.md)).
* Reproducibility: worlds should be deterministic (fixed obstacle
  placement) so exploration runs are comparable across evaluation episodes
  (see [`evaluation.md`](evaluation.md)).

## 6. Physics

Standard Gazebo Harmonic physics engine defaults are the starting point.
Do not tune physics parameters to compensate for autonomy-stack bugs —
physics tuning should only be used for legitimate simulation-fidelity
reasons (e.g., matching PX4's expected update rate) and any deviation from
defaults should be documented with a reason.

## 7. Sensor Configuration

To be finalized at M3. This section should be updated with the actual
chosen values (resolution, rate, noise) once M3 is implemented, since other
documents ([`ros2-architecture.md`](ros2-architecture.md),
[`localization.md`](localization.md)) reference "the sensor topics" without
repeating their configuration.

## 8. Launch Sequence

Conceptual order (exact commands/launch files to be added as milestones are
implemented):

1. Start Gazebo with the target world and drone model.
2. Start PX4 SITL, connected to the Gazebo instance.
3. Start the uXRCE-DDS agent (ROS 2 side of the bridge).
4. Launch the ROS 2 autonomy stack (localization → mapping → exploration →
   PX4 interface → exploration manager), via `launch/` files.
5. (Optional) Launch RViz2 with a project-specific configuration for
   visualization.
6. (Optional) Launch QGroundControl for manual monitoring/override.

A single top-level launch file that performs steps 1–4 should be added once
enough milestones exist to make an end-to-end launch meaningful (around
M9–M10).

## 9. How Simulation Data Flows into ROS 2

```text
Gazebo sensor/state plugins
   → Gazebo-ROS 2 bridge (sensor topics: image, IMU, depth, ground-truth pose)
   → ROS 2 topics consumed by Localization / Mapping nodes

Gazebo (via PX4 SITL)
   → PX4 internal state (uORB)
   → uXRCE-DDS bridge
   → ROS 2 topics consumed by PX4 Interface Node / Exploration Manager
```

Note the two distinct bridging paths: sensor data typically comes through a
Gazebo-ROS 2 sensor bridge, while flight-controller state and commands go
through the separate uXRCE-DDS PX4 bridge. These are not interchangeable
and should not be conflated when debugging (see
[`troubleshooting.md`](troubleshooting.md)).

## 10. How to Create Additional Environments

1. Add a new world file under `simulation/worlds/`.
2. Ensure the drone spawn point and world bounds are set appropriately for
   exploration (bounded, reachable, with a reasonable amount of initially
   unknown space).
3. Reuse the existing drone model under `simulation/models/` unless a
   specific test requires a model variant.
4. Add a corresponding launch configuration or launch argument so the new
   world can be selected without duplicating the entire launch file.
5. If the world is intended for evaluation (M12), keep it deterministic and
   record its name/seed in the evaluation run configuration.
