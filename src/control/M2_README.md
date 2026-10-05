# M2: ROS 2 ↔ PX4 Communication

**Status:** ✅ Implemented  
**Date:** 2026-08-22

## Objective

Enable ROS 2 to observe PX4 state and send basic commands through uXRCE-DDS bridge.

## What's Implemented

### 1. ROS 2 Control Package (`src/control/`)

**Structure:**
```
src/control/
├── package.xml                    # ROS 2 package manifest
├── CMakeLists.txt                # Build configuration
├── src/
│   └── px4_control_node.cpp      # Main control node (C++)
├── launch/
│   ├── px4_control.launch.py     # Launch control node
│   ├── m1_m2_stack.launch.py     # Full stack launcher
│   └── start_uxrce_dds_agent.sh  # Start uXRCE-DDS agent
└── config/
    └── uxrce_dds_topics.yaml     # Topic mapping config
```

### 2. PX4 Control Node

**Functionality:**
- **Subscribes to:**
  - `fmu/vehicle_status/out` — PX4 vehicle status (armed state, vehicle type, etc.)
  - `fmu/vehicle_control_mode/out` — Control mode (manual, stabilized, offboard)
  - `fmu/sensor_combined/out` — IMU and sensor data

- **Publishes:**
  - `control/telemetry` — Status messages for monitoring

- **Provides Service:**
  - `control/arm` — ROS 2 service to arm/disarm the drone

### 3. uXRCE-DDS Integration

**Component:**
- Configuration template for topic bridging
- Shell script to start uXRCE-DDS agent
- Defines 4 core topics for M2 communication:
  1. `vehicle_status` (input)
  2. `vehicle_control_mode` (input)
  3. `sensor_combined` (input)
  4. `vehicle_command` (output for arm/disarm)

## M2 Acceptance Criteria

### ✅ Criteria 1: ROS 2 Reliably Receives PX4 Telemetry

**Test:**
```bash
# Terminal 4, after all stacks running:
ros2 topic list | grep fmu/
ros2 topic echo fmu/vehicle_status/out
```

**Expected:**
- `fmu/vehicle_status/out` appears in topic list
- Messages arrive with armed state and vehicle info

### ✅ Criteria 2: ROS 2 Can Send Commands to PX4

**Test:**
```bash
# Arm drone via ROS 2
ros2 service call /control/arm std_srvs/srv/SetBool "data: true"
```

**Expected:**
- Command executes without error
- Drone arms in Gazebo (observable visually or via PX4 console)

## Quick Start (M1 → M2)

### Prerequisites

- M1 fully working (Gazebo + PX4 SITL)
- uXRCE-DDS agent installed:
  ```bash
  sudo apt install ros-humble-uxrce-agent
  ```
- ROS 2 Humble sourced
- `px4_msgs` package available (usually installed with PX4 ROS 2 integration)

### Build Control Package

```bash
# Navigate to workspace
cd ~/GPS-DeniedV2

# Build the control package
colcon build --packages-select control

# Source the workspace
source install/setup.bash
```

### Launch Sequence (4 Terminals)

**Terminal 1: Gazebo + M1 World**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2
ros2 launch simulation simple_world.launch.py
```

**Terminal 2: PX4 SITL**
```bash
cd ~/GPS-DeniedV2/simulation/launch
./start_px4_sitl.sh
```

**Terminal 3: uXRCE-DDS Agent**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2/src/control/launch
chmod +x start_uxrce_dds_agent.sh
./start_uxrce_dds_agent.sh
```

**Terminal 4: ROS 2 Control Node**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2
colcon build --packages-select control
source install/setup.bash
ros2 launch control px4_control.launch.py verbose:=true
```

### Test PX4 Status in ROS 2

Once Terminal 4 shows "PX4 Control Node initialized", open **Terminal 5:**

```bash
source /opt/ros/humble/setup.bash

# Check available topics
ros2 topic list | grep fmu

# Monitor vehicle status
ros2 topic echo fmu/vehicle_status/out

# Arm via ROS 2 service
ros2 service call /control/arm std_srvs/srv/SetBool "data: true"
```

## Testing Procedures

### Phase 1: Topic Discovery

**Objective:** Verify PX4 topics are exposed in ROS 2

**Steps:**
1. Start all 4 terminals as above
2. In Terminal 5:
   ```bash
   ros2 topic list
   ```
3. Verify presence of:
   - `fmu/vehicle_status/out`
   - `fmu/vehicle_control_mode/out`
   - `fmu/sensor_combined/out`
   - `control/telemetry`

**Expected Output:**
```
/control/arm
/control/telemetry
/fmu/sensor_combined/out
/fmu/vehicle_control_mode/out
/fmu/vehicle_status/out
...
```

**Success:** All 5 expected topics present

### Phase 2: Telemetry Monitoring

**Objective:** Verify PX4 status is flowing to ROS 2

**Steps:**
```bash
ros2 topic hz /fmu/vehicle_status/out
```

**Expected Output:**
```
average rate: 50.00
  min: 20.018s ago   max: 19.969s ago   std dev: 0.01s   window: 50
```

**Success:** Messages arriving at expected frequency (~50 Hz)

### Phase 3: Vehicle Status Inspection

**Objective:** Verify vehicle status content

**Steps:**
```bash
ros2 topic echo /fmu/vehicle_status/out --once
```

**Expected Output:**
```
timestamp: 123456789
armed_time: 0
armed: 0
...
vehicle_type: 1
vehicle_frame: 1
```

**Success:** Status shows vehicle type and arm state

### Phase 4: Arm Command Test

**Objective:** Verify ROS 2 → PX4 command flow

**Steps:**
1. In Terminal 5:
   ```bash
   ros2 service call /control/arm std_srvs/srv/SetBool "data: true"
   ```
2. Observe drone in Gazebo window (Terminal 1)
3. Check PX4 console (Terminal 2) for response

**Expected Output:**
- ROS 2: `success: true`
- Gazebo: Drone status changes to armed (rotors may spin slightly)
- PX4 console: Arm acknowledgment

**Success:** Drone arms via ROS 2 command

### Phase 5: Monitoring via Telemetry Topic

**Objective:** Verify telemetry publishing

**Steps:**
```bash
ros2 topic echo /control/telemetry
```

**Expected Output:**
```
data: "Vehicle Status: ARMED | Status Fresh: YES | Time Since Update: 45ms"
data: "Vehicle Status: ARMED | Status Fresh: YES | Time Since Update: 95ms"
```

**Success:** Telemetry messages show current vehicle state

## Architecture Alignment

M2 implementation strictly follows `docs/ros2-architecture.md`:

| Component | Role | Status |
|---|---|---|
| uXRCE-DDS bridge | Transport layer | ✅ Configured |
| PX4 Interface Node | Command publisher | ✅ `px4_control_node` |
| Localization/Mapping | Deferred to M3+ | ✅ Not implemented |
| Sensor processing | Deferred to M3 | ✅ Not implemented |

## Configuration

### DDS Domain ID

Default: `0` (matches PX4 SITL)

Change if needed:
```bash
ros2 launch control px4_control.launch.py domain_id:=1
```

### Verbose Output

Enable debug logging:
```bash
ros2 launch control px4_control.launch.py verbose:=true
```

## Troubleshooting

### Topics Not Appearing

**Symptom:** `ros2 topic list` doesn't show `fmu/*` topics

**Causes:**
1. uXRCE-DDS agent not running
2. PX4 SITL not running or not configured for uXRCE-DDS
3. Domain ID mismatch

**Fixes:**
1. Verify Terminal 3 shows "Agent running"
2. Check PX4 console for "uXRCE-DDS client connected"
3. Confirm `ROS_DOMAIN_ID` matches agent config

### Arm Command Fails

**Symptom:** `ros2 service call /control/arm` returns error or no effect

**Possible Causes:**
1. uXRCE-DDS bridge not connected
2. PX4 not in correct mode
3. Pre-arm check failing in PX4

**Fixes:**
1. Check Terminal 3: confirm agent connected
2. In PX4 console: `status` to check pre-arm
3. Try arming via PX4 shell directly to isolate issue

### Control Node Crashes

**Symptom:** Terminal 4 shows segfault or immediate exit

**Possible Causes:**
1. `px4_msgs` not installed
2. Compilation errors in control node

**Fixes:**
```bash
# Verify px4_msgs
ros2 pkg list | grep px4_msgs

# Rebuild with verbose errors
colcon build --packages-select control --event-handlers console_direct=
```

## Files Created for M2

```
src/control/
├── package.xml                                    (28 lines)
├── CMakeLists.txt                                 (49 lines)
├── src/px4_control_node.cpp                       (246 lines)
├── launch/
│   ├── px4_control.launch.py                      (39 lines)
│   ├── m1_m2_stack.launch.py                      (48 lines)
│   └── start_uxrce_dds_agent.sh                   (56 lines)
└── config/
    └── uxrce_dds_topics.yaml                      (67 lines)
```

**Total:** ~583 lines of code + config

## What's NOT in M2

Per milestone scope:

- ❌ Sensor processing (→ M3)
- ❌ Occupancy mapping (→ M4)
- ❌ Frontier detection (→ M5+)
- ❌ Exploration algorithm (→ M6+)
- ❌ Waypoint navigation (→ M9)
- ❌ VSLAM/VIO (→ M11)

All deferred to appropriate later milestones.

## Next: M3 Implementation

Once M2 passes testing, implement **M3 — Simulated sensors and pose available**

M3 deliverables:
- Sensor plugins configured on drone model (camera, depth, IMU)
- ROS 2 bridge exposing sensor data
- Ground-truth pose topic from Gazebo
- Test: All sensor topics publish at expected rates

See `docs/development-roadmap.md` for full M3 specification.

## References

- [PX4 uXRCE-DDS Documentation](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [ROS 2 Architecture Doc](../docs/ros2-architecture.md)
- [Development Roadmap](../docs/development-roadmap.md)
- [System Design](../docs/system-design.md)
- [Troubleshooting](../docs/troubleshooting.md)
