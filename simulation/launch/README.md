# M1 Simulation Launch Guide

> **Swarm simulation:** the 3-drone real-time stack is started with
> `ros2 launch simulation gazebo_swarm.launch.py` (or `./run_gazebo_swarm.sh` from the repo root,
> ROS 2 Jazzy). It spawns `models/quadrotor_px4.sdf` once per drone with Gazebo's own rotor-physics
> velocity controller. The PX4 SITL instructions below are the legacy single-drone M1 workflow.

This directory contains launch files and scripts for M1 testing:
**Gazebo + PX4 drone works**

## Files

- `simple_world.launch.py` — ROS 2 launch file for Gazebo + drone model
- `start_px4_sitl.sh` — Bash script to start PX4 SITL alongside Gazebo

## Prerequisites

1. **Ubuntu 22.04 LTS**
2. **Gazebo Harmonic** installed
3. **PX4 Autopilot** source code cloned and built for SITL
4. **ROS 2 Humble** installed and sourced

## Quick Start

### Terminal 1: Start Gazebo with the drone model

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# From the project root directory
ros2 launch simulation simple_world.launch.py
```

This will start Gazebo with:
- The simple world (with ground plane and obstacles)
- A quadrotor drone model at spawn position (0, 0, 0.1)

Gazebo should start with the world visible and the drone hovering slightly above ground.

### Terminal 2: Start PX4 SITL

```bash
# Source PX4 environment (if using auto-generated setup script)
# Or manually ensure PX4 build artifacts are available

cd simulation/launch
chmod +x start_px4_sitl.sh
./start_px4_sitl.sh
```

This will:
1. Check for PX4 source and build directories
2. Start PX4 SITL connected to the running Gazebo instance
3. Initialize the drone's flight controller

### Terminal 3 (Optional): Monitor with QGroundControl

QGroundControl can connect to PX4 SITL for monitoring:

```bash
qgroundcontrol
```

QGroundControl should automatically detect the PX4 instance on UDP port 14550.

## Testing M1 Acceptance Criteria

Once both Gazebo and PX4 SITL are running, test the following using **PX4 MAVLink shell**:

### In the PX4 shell:

1. **Arm the drone:**
   ```
   arm
   ```
   The drone should arm without errors (check PX4 logs for any safety violations).

2. **Takeoff:**
   ```
   takeoff 1.0
   ```
   The drone should rise to 1 meter above ground and hover.

3. **Monitor altitude:**
   ```
   status
   ```
   Should show position data (altitude increasing, stable hover).

4. **Land:**
   ```
   land
   ```
   The drone should descend smoothly and disarm upon landing.

### Expected Behavior

- **Drone arms smoothly:** No pre-arm failures
- **Takeoff stable:** No erratic pitch/roll, smooth climb
- **Hover stable:** Altitude holding without divergence or oscillation
- **Landing smooth:** Controlled descent, safe contact with ground
- **No crashes:** Drone should not collide with obstacles or world bounds

### Troubleshooting

If the drone fails to arm or behaves erratically, check:

1. **Gazebo and PX4 communication:**
   - Both processes should be running (check process list)
   - No error messages in either terminal

2. **IMU/Sensor data:**
   - In PX4 shell: `commander status` should show all sensors OK
   - Check IMU data is flowing (not NaN or stale)

3. **World/model loading:**
   - Gazebo window should display the world with the drone
   - Drone should be above ground (not clipping into floor)

4. **Physics/Stability:**
   - If the drone oscillates wildly, the physics tuning may need adjustment
   - Verify PX4 SITL is configured for Gazebo Harmonic (not Classic)

## Configuration

### Changing the World

To use a different world, modify `simple_world.launch.py` or pass arguments:

```bash
ros2 launch simulation simple_world.launch.py world:=/path/to/your/world.sdf
```

### Gazebo Model Path

The launch file automatically sets `GZ_SIM_RESOURCE_PATH` to point to `simulation/models/`.
If models are not found, verify the path is correct in the launch file.

### PX4 Build Directory

Edit `start_px4_sitl.sh` to set `PX4_SOURCE_DIR` and `PX4_BUILD_DIR` to match your installation:

```bash
PX4_SOURCE_DIR="/path/to/PX4-Autopilot"
PX4_BUILD_DIR="/path/to/PX4-Autopilot/build/px4_sitl_default"
```

## Next Steps (M2+)

Once M1 is validated (drone arms, takes off, hovers, lands):

1. **M2:** Add ROS 2 ↔ PX4 communication via uXRCE-DDS
2. **M3:** Integrate simulated sensors (camera, depth, IMU)
3. **M4+:** Implement mapping and exploration pipeline

See `docs/development-roadmap.md` for full milestone details.

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/)
- [PX4 SITL Guide](https://docs.px4.io/main/en/simulation/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Project Architecture](../docs/architecture.md)
- [Simulation Configuration](../docs/simulation.md)
