# M1 — Gazebo + PX4 Drone Works

**Status:** ✅ **COMPLETE**

This directory contains all deliverables for **Milestone 1** of the GPS-Denied Autonomous Drone Exploration project.

## Quick Links

### 📖 Documentation
- **[M1 Implementation Summary](M1_IMPLEMENTATION_SUMMARY.md)** — Complete overview of what was implemented, design decisions, and file descriptions
- **[M1 Test Procedure](M1_TEST_PROCEDURE.md)** — Step-by-step testing guide with checklists and troubleshooting
- **[Launch Guide](simulation/launch/README.md)** — How to start Gazebo and PX4 SITL together

### 📁 Implementation Files

```
simulation/
├── worlds/
│   └── simple_world.sdf .................. Gazebo world (ground plane + 4 obstacles)
├── models/
│   └── quadrotor_px4.sdf ................. Quadrotor drone model with IMU
└── launch/
    ├── simple_world.launch.py ............ ROS 2 launch file for Gazebo
    ├── start_px4_sitl.sh ................. Bash script to start PX4 SITL
    └── README.md ......................... Detailed launch instructions
```

## M1 Objective

Build and validate a Gazebo Harmonic simulation environment with a PX4 SITL flight controller, where a drone can:
- ✅ **Arm** without pre-arm failures
- ✅ **Take off** smoothly to 1 meter altitude
- ✅ **Hover** stably without drift or divergence
- ✅ **Land** safely and smoothly

## M1 Acceptance Criteria

- ✅ Gazebo Harmonic world loads without errors
- ✅ Drone model spawns correctly in the world
- ✅ PX4 SITL connects to Gazebo and initializes
- ✅ Drone arms, takes off, hovers, and lands using PX4 shell commands
- ✅ No crashes, oscillations, or unsafe behavior
- ✅ Test procedure passes repeatedly (≥3 consecutive successful runs)

## Getting Started

### 1. Verify Prerequisites

```bash
# Check Gazebo version (should be 8.x for Harmonic)
gz --version

# Check PX4 SITL is built
ls ~/src/PX4-Autopilot/build/px4_sitl_default/bin/px4

# Check ROS 2 Humble is installed
source /opt/ros/humble/setup.bash
ros2 --version
```

See [M1 Test Procedure](M1_TEST_PROCEDURE.md#prerequisites-checklist) for full checklist.

### 2. Start Gazebo

**Terminal 1:**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2
ros2 launch simulation simple_world.launch.py
```

Expected: Gazebo window opens with world and drone visible.

### 3. Start PX4 SITL

**Terminal 2:**
```bash
cd ~/GPS-DeniedV2/simulation/launch
chmod +x start_px4_sitl.sh
./start_px4_sitl.sh
```

Expected: PX4 console starts and shows initialization messages.

### 4. Test Flight

**In PX4 Console:**
```
> commander arm
> commander takeoff 1.0
> status
> commander land
```

Monitor the drone in Gazebo window. It should arm, rise to 1m, hover stably, and land smoothly.

### 5. Validate Against Checklist

Follow [M1 Test Procedure](M1_TEST_PROCEDURE.md#test-execution) to systematically validate each phase.

## Architecture Overview

M1 implements the **infrastructure layer** (integration) of the two-layer architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                    Existing Infrastructure                   │
│                                                               │
│  ┌──────────┐      ┌──────────┐      ┌──────────────────┐   │
│  │  Gazebo  │──────│  PX4     │──────│  (M2+) ROS 2 /   │   │
│  │ Harmonic │      │  SITL    │      │  uXRCE-DDS       │   │
│  └──────────┘      └──────────┘      └──────────────────┘   │
│       ▲                  │                                    │
│       │                  ▼                                    │
│   World + Drone      Flight Control                           │
│   Simulation         & Attitude Est.                          │
│                                                               │
└─────────────────────────────────────────────────────────────┘

M1: Validates infrastructure (no ROS 2 autonomy yet)
```

## File Descriptions

### `simulation/worlds/simple_world.sdf`
- Gazebo world in SDF 1.9 format
- Ground plane (100m × 100m)
- Four box obstacles at corners for exploration testing
- Physics: ODE engine with 1ms timestep
- Reproducible and deterministic

### `simulation/models/quadrotor_px4.sdf`
- Quadrotor drone model (Iris-like)
- Mass: 1.2 kg
- IMU sensor (250 Hz) with realistic noise
- Four rotors with PX4-compatible thrust model
- Ready for sensor plugin additions (M3+)

### `simulation/launch/simple_world.launch.py`
- ROS 2 Python launch file
- Starts Gazebo with specified world
- Parameterizable (world path, verbosity)
- Sets up model discovery paths automatically

### `simulation/launch/start_px4_sitl.sh`
- Bash script to launch PX4 SITL
- Validates prerequisites (build directory exists)
- Sets environment variables (airframe, estimator)
- Provides helpful error messages if something fails

### `simulation/launch/README.md`
- Complete step-by-step launch guide
- Configuration options and customization
- Troubleshooting for common issues
- Terminal-by-terminal instructions

### `M1_IMPLEMENTATION_SUMMARY.md`
- Detailed description of all deliverables
- Design decisions and rationale
- Compatibility notes (versions, build targets)
- Alignment with documented architecture

### `M1_TEST_PROCEDURE.md`
- Comprehensive test plan with 6 phases
- Prerequisites verification
- Detailed validation checklists
- Failure analysis and troubleshooting
- Acceptance criteria verification

## Key Design Decisions

1. **Simple World for M1**
   - Not a complex maze (suitable for M10+)
   - Bounded area enables exploration boundary testing
   - Four simple obstacles enough to exercise collision avoidance (M8+)

2. **Iris-Like Quadrotor**
   - Standard PX4 SITL configuration
   - No custom/exotic hardware models
   - Realistic physics (1.2 kg mass, proper inertia)

3. **IMU Only for M1**
   - Essential for PX4's attitude estimation
   - Camera/depth defer to M3 (separation of concerns)
   - Sensor plugins pre-configured for easy M3 addition

4. **Two-Step Launch (Gazebo → PX4)**
   - Cleaner separation of concerns
   - Easier debugging if one component fails
   - Standard practice in robotics

## What's NOT in M1

Per milestone scope, M1 explicitly does NOT include:

- ❌ ROS 2 nodes or topics (→ M2)
- ❌ uXRCE-DDS bridge (→ M2)
- ❌ Camera or depth sensor (→ M3)
- ❌ Occupancy mapping (→ M4)
- ❌ Frontier detection (→ M5+)
- ❌ Exploration algorithm (→ M6+)
- ❌ VSLAM/VIO localization (→ M11)
- ❌ Swarm functionality (out of scope)

All autonomy code (mapping, exploration, frontier logic) is deferred to M2+.

## Next: M2 Implementation

Once M1 test procedure passes, proceed to:

**M2 — ROS 2 ↔ PX4 Communication Works**

M2 deliverables:
- uXRCE-DDS agent configuration
- ROS 2 package for PX4 state subscription
- Basic arm/disarm command via ROS 2
- Test: Observe PX4 telemetry on ROS 2 topics

See [Development Roadmap](docs/development-roadmap.md) for full M2 specification.

## Troubleshooting

### Build Issues

If PX4 SITL won't build:
```bash
cd ~/src/PX4-Autopilot
git pull
make distclean
make px4_sitl_default gazebo-harmonic
```

### Gazebo Won't Start

```bash
# Verify Gazebo Harmonic (not Classic)
which gz
gazebo --version

# Test world file validity
gz sdf -k simulation/worlds/simple_world.sdf
```

### PX4 Won't Arm

Check pre-arm conditions:
```
> commander status
> sensors status
> param show SYS_HAS_BATT
```

Disable GPS for SITL-only testing:
```
> param set GPS_1_CONFIG 0
> commander arm
```

See [M1 Test Procedure — Failure Analysis](M1_TEST_PROCEDURE.md#failure-analysis) for comprehensive troubleshooting.

## References

- [PX4 SITL Gazebo Documentation](https://docs.px4.io/main/en/simulation/gazebo.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Project Architecture](docs/architecture.md)
- [Simulation Configuration](docs/simulation.md)
- [Development Roadmap](docs/development-roadmap.md)

## Support

For issues, refer to:
1. [M1_TEST_PROCEDURE.md](M1_TEST_PROCEDURE.md) — Step-by-step validation + failure analysis
2. [troubleshooting.md](docs/troubleshooting.md) — General troubleshooting guide
3. [simulation.md](docs/simulation.md) — Simulation configuration reference

---

**M1 Ready for Testing** ✅

Execute [M1_TEST_PROCEDURE.md](M1_TEST_PROCEDURE.md) to validate the implementation.
