# M2 Test Procedure and Validation Checklist

> **Historical milestone report (Phase 1, single drone).** This document records the earlier single-drone PX4 SITL + ROS 2 Humble phase and the C++ packages in `src/`. The current system is the three-drone ROS 2 Jazzy + Gazebo Harmonic swarm described in [../README.md](../README.md) and [README.md](README.md). Commands and results below are kept for reference and may no longer run as written.

**Milestone:** M2 — ROS 2 ↔ PX4 communication works  
**Status:** Ready for testing  
**Date:** 2026-08-22

## Objective

Validate that ROS 2 can observe PX4 state and send basic commands through uXRCE-DDS bridge.

## Prerequisites Checklist

Before testing M2, verify:

- [ ] **M1 Complete:** Gazebo + PX4 SITL working (see M1_TEST_PROCEDURE.md)
- [ ] **M1 Tested:** 3 consecutive successful arm/takeoff/land cycles
- [ ] **uXRCE-DDS Installed:**
  ```bash
  ros-humble-uxrce-agent
  ```
- [ ] **px4_msgs Available:**
  ```bash
  ros2 pkg list | grep px4_msgs
  ```
- [ ] **ROS 2 Humble:** Sourced in all terminals
- [ ] **Workspace Built:**
  ```bash
  cd ~/GPS-DeniedV2
  colcon build --packages-select control
  ```

## Environment Setup

### Step 1: Verify File Structure

```bash
cd ~/GPS-DeniedV2
ls -la src/control/
# Expected:
# -rw-r--r-- package.xml
# -rw-r--r-- CMakeLists.txt
# drwxr-xr-x src/
# drwxr-xr-x launch/
# drwxr-xr-x config/
```

### Step 2: Build Control Package

```bash
# Navigate to workspace
cd ~/GPS-DeniedV2

# Clean build (optional, for fresh start)
colcon build --packages-select control --cmake-clean-cache

# Standard build
colcon build --packages-select control

# Source the new build
source install/setup.bash

# Verify build succeeded
ls install/control/lib/control/px4_control_node
```

**Expected:** Build completes without errors, executable exists

### Step 3: Verify Topic Mapping

```bash
# Check uXRCE-DDS configuration
cat src/control/config/uxrce_dds_topics.yaml | head -20
```

**Expected:** Configuration file lists vehicle_status, vehicle_control_mode, sensor_combined, vehicle_command

## Test Execution

### Phase 1: Environment Validation

**Objective:** Verify all prerequisites are met

**Terminal 1: Start Gazebo with M1 World**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2
ros2 launch simulation simple_world.launch.py
```

**Expected Output:**
- Gazebo window opens with world
- No errors in console
- Drone model visible

**Validation Checklist:**
- [ ] Gazebo window appears within 10 seconds
- [ ] Drone model visible in center
- [ ] No error messages about world/model loading
- [ ] Console shows "resource path" configuration

### Phase 2: PX4 SITL Startup

**Terminal 2: Start PX4 SITL**
```bash
cd ~/GPS-DeniedV2/simulation/launch
chmod +x start_px4_sitl.sh
./start_px4_sitl.sh
```

**Expected Output:**
- PX4 initialization messages
- Vehicle airframe loads
- Console prompt appears
- No uXRCE-DDS connection errors yet (expected)

**Validation Checklist:**
- [ ] PX4 starts without build errors
- [ ] Vehicle airframe (iris) loads
- [ ] Sensors initialize (IMU OK)
- [ ] Console prompt active (ready for commands)
- [ ] No fatal errors in output

### Phase 3: uXRCE-DDS Agent Startup

**Terminal 3: Start uXRCE-DDS Agent**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2/src/control/launch
chmod +x start_uxrce_dds_agent.sh
./start_uxrce_dds_agent.sh
```

**Expected Output:**
```
Starting uXRCE-DDS Agent for PX4 SITL
Agent: 127.0.0.1:2019
PX4: 127.0.0.1:2018
Starting MicroXRCEAgent...
Agent started with PID XXXXX
Agent is listening on 127.0.0.1:2019
Waiting for PX4 SITL to connect...
```

**Connection Event (in Terminal 2 — PX4 console):**
Look for a message like:
```
[uxrce_dds_client] INFO: Connected to agent
```

**Validation Checklist:**
- [ ] Agent starts without errors
- [ ] Agent listening on correct port (2019)
- [ ] PX4 console shows connection confirmation
- [ ] No "connection refused" errors
- [ ] Agent remains running (no crashes)

### Phase 4: ROS 2 Control Node Startup

**Terminal 4: Start Control Node**
```bash
source /opt/ros/humble/setup.bash
cd ~/GPS-DeniedV2
source install/setup.bash
ros2 launch control px4_control.launch.py verbose:=true
```

**Expected Output:**
```
[px4_control-1] PX4 Control Node starting (domain_id=0)
[px4_control-1] PX4 Control Node initialized successfully
[px4_control-1] [DEBUG] (status messages follow)
```

**Validation Checklist:**
- [ ] Node launches without errors
- [ ] "PX4 Control Node initialized successfully" appears
- [ ] No missing dependency errors
- [ ] Debug output shows vehicle updates
- [ ] Node continues running (doesn't crash)

### Phase 5: Topic Discovery Test

**Terminal 5: Verify ROS 2 Topics**
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "fmu|control"
```

**Expected Output:**
```
/control/arm
/control/telemetry
/fmu/sensor_combined/out
/fmu/vehicle_control_mode/out
/fmu/vehicle_status/out
```

**Validation Checklist:**
- [ ] All 5 expected topics present
- [ ] Topic names match configuration
- [ ] No truncated or corrupted names

### Phase 6: Topic Message Rate Test

**Objective:** Verify topics are publishing at expected rates

```bash
ros2 topic hz /fmu/vehicle_status/out
```

**Expected Output:**
```
average rate: 50.00
  min: 20.018s ago   max: 19.969s ago   std dev: 0.01s   window: 50
```

Run on each topic:
```bash
ros2 topic hz /fmu/vehicle_status/out    # Should be ~50 Hz
ros2 topic hz /fmu/vehicle_control_mode/out  # Should be ~50 Hz
ros2 topic hz /fmu/sensor_combined/out   # Should be ~250 Hz (IMU)
ros2 topic hz /control/telemetry         # Should be ~2 Hz (periodic status)
```

**Validation Checklist:**
- [ ] `vehicle_status`: Publishing at ~50 Hz
- [ ] `vehicle_control_mode`: Publishing at ~50 Hz
- [ ] `sensor_combined`: Publishing at ~250 Hz (fast for IMU)
- [ ] `control/telemetry`: Publishing at ~2 Hz (periodic)
- [ ] No message drops or gaps in output

### Phase 7: Telemetry Content Inspection

**Objective:** Verify message content is valid

```bash
# Inspect vehicle status once
ros2 topic echo /fmu/vehicle_status/out --once
```

**Expected Output:**
```
timestamp: 1234567890
armed_time: 0
armed: 0
vehicle_type: 1
vehicle_frame: 1
system_id: 1
...
```

**Validation Checklist:**
- [ ] Timestamp is reasonable (not 0 or stale)
- [ ] `armed: 0` (disarmed initially)
- [ ] `vehicle_type: 1` (quadrotor)
- [ ] `vehicle_frame: 1` (quad X)
- [ ] All fields have valid values (no NaN/Inf)

### Phase 8: Sensor Data Verification

**Objective:** Verify IMU data is flowing

```bash
ros2 topic echo /fmu/sensor_combined/out --once
```

**Expected Output:**
```
timestamp: 1234567890
gyro_rad_s: [0.001, -0.002, 0.0005]
accelerometer_m_s2: [0.1, -0.05, 9.81]
...
```

**Validation Checklist:**
- [ ] IMU data has realistic values
- [ ] Acceleration Z ≈ 9.81 (gravity)
- [ ] Gyro values small (hovering)
- [ ] No NaN or very large outliers

### Phase 9: Arm Command Test (Critical)

**Objective:** Verify ROS 2 → PX4 command flow

```bash
# Send ARM command via ROS 2 service
ros2 service call /control/arm std_srvs/srv/SetBool "data: true"
```

**Expected Output (Terminal 5):**
```
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response: std_srvs.srv.SetBool_Response(success=True, message='ARM command sent')
```

**Observation in Terminal 2 (PX4 console):**
Look for vehicle status change or acknowledgment

**Observation in Terminal 1 (Gazebo):**
Drone should arm (subtle visual change or rotor readiness)

**Validation Checklist:**
- [ ] Service call returns `success: true`
- [ ] PX4 console shows arm acknowledgment
- [ ] Gazebo drone status changes (armed)
- [ ] No error messages in any terminal
- [ ] Command completes within 1 second

### Phase 10: Telemetry Monitoring Test

**Objective:** Verify continuous telemetry publishing

```bash
# Monitor telemetry for 5 seconds
ros2 topic echo /control/telemetry &
sleep 5
pkill -f "ros2 topic echo"
```

**Expected Output:**
```
data: "Vehicle Status: DISARMED | Status Fresh: YES | Time Since Update: 45ms"
data: "Vehicle Status: DISARMED | Status Fresh: YES | Time Since Update: 95ms"
data: "Vehicle Status: ARMED | Status Fresh: YES | Time Since Update: 50ms"
data: "Vehicle Status: ARMED | Status Fresh: YES | Time Since Update: 100ms"
...
```

**Validation Checklist:**
- [ ] Messages arrive continuously
- [ ] Status updates reflect vehicle state
- [ ] "Status Fresh: YES" indicates recent data
- [ ] Time Since Update increases gradually
- [ ] No long gaps between messages

### Phase 11: Disarm Command Test

**Objective:** Verify disarm command

```bash
ros2 service call /control/arm std_srvs/srv/SetBool "data: false"
```

**Expected Output:**
```
response: std_srvs.srv.SetBool_Response(success=True, message='DISARM command sent')
```

**Observation:**
- Gazebo: Drone status changes
- Telemetry: "Vehicle Status: DISARMED" appears

**Validation Checklist:**
- [ ] Service call succeeds
- [ ] Vehicle telemetry reflects disarm
- [ ] No errors in PX4 console

## Failure Analysis

### uXRCE-DDS Agent Won't Start

**Symptom:**
```
MicroXRCEAgent: command not found
```

**Cause:** Package not installed

**Fix:**
```bash
sudo apt install ros-humble-uxrce-agent
```

### PX4 Doesn't Connect to Agent

**Symptom:**
- Terminal 2 (PX4): No connection message
- Terminal 3 (Agent): Waiting... no client connection

**Cause:**
1. PX4 not configured for uXRCE-DDS
2. Port mismatch
3. Network/firewall issue

**Fix:**
```bash
# In PX4 console, check if uXRCE-DDS client loaded:
> dmesg | grep uxrce
> param show UXRCE_DDS_CFG
```

### ROS 2 Topics Don't Appear

**Symptom:**
- `ros2 topic list` shows no `fmu/*` topics

**Cause:**
1. Agent not connected
2. Domain ID mismatch
3. Middleware transport issue

**Fix:**
1. Verify agent shows "Client connected"
2. Check DDS domain:
   ```bash
   echo $ROS_DOMAIN_ID
   ```
3. Rebuild control package

### Arm Command Fails or Has No Effect

**Symptom:**
- Service call returns error
- OR returns success but drone doesn't arm

**Cause:**
1. uXRCE-DDS bridge broken
2. PX4 pre-arm checks failing
3. Command not reaching PX4

**Fix:**
1. Verify `fmu/vehicle_command/in` topic exists
2. In PX4 console: `commander status` to check pre-arm
3. Try arming via PX4 shell directly:
   ```
   > commander arm
   ```

### High Message Latency

**Symptom:**
- `ros2 topic hz` shows rates <10 Hz
- "Time Since Update" shows >500ms

**Cause:**
1. Network congestion
2. uXRCE-DDS agent CPU-bound
3. ROS 2 middleware buffer limit

**Fix:**
1. Reduce verbosity: `verbose:=false`
2. Restart agent and control node
3. Check system CPU: `top`

## Acceptance Criteria

M2 is successfully validated when ALL of the following are met:

### ✅ Criterion 1: Topic Discovery
- [ ] `ros2 topic list` shows all 5 expected topics
- [ ] Topics include `fmu/vehicle_status/out`, `fmu/vehicle_control_mode/out`, `fmu/sensor_combined/out`, `control/telemetry`, `/control/arm`

### ✅ Criterion 2: Telemetry Flow
- [ ] Vehicle status topics publish at expected rates (50 Hz+)
- [ ] Message content is valid (no NaN, reasonable values)
- [ ] No message drops over 10-second observation

### ✅ Criterion 3: Arm Command Execution
- [ ] `ros2 service call /control/arm std_srvs/srv/SetBool "data: true"` returns `success: true`
- [ ] Drone arms in Gazebo (observable)
- [ ] Telemetry shows armed state

### ✅ Criterion 4: Disarm Command Execution
- [ ] `ros2 service call /control/arm std_srvs/srv/SetBool "data: false"` succeeds
- [ ] Drone disarms in Gazebo
- [ ] Telemetry reflects disarm

### ✅ Criterion 5: Continuous Operation
- [ ] Stack runs for 5+ minutes without crashes
- [ ] Topics continue publishing at expected rates
- [ ] No latency degradation over time

## Post-Test Validation

Once all phases pass:

1. **Repeatability:** Run full test sequence at least 2 more times. Success should be consistent.
2. **Clean Logs:** No error messages in any terminal after final test.
3. **Graceful Shutdown:** Kill nodes in reverse order (Terminal 5→4→3→2→1) without errors.

## Acceptance Criteria Met

M2 is complete when:

- ✅ ROS 2 reliably receives PX4 telemetry (topics publishing consistently)
- ✅ ROS 2 can send arm/disarm commands that PX4 executes
- ✅ All 11 test phases pass
- ✅ Test procedures repeatable with consistent success

## Next Steps

Once M2 passes testing, implement **M3: Simulated sensors and pose available**

See `docs/development-roadmap.md` M3 specification for next milestone.

## References

- [PX4 uXRCE-DDS Guide](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [ROS 2 Architecture](../docs/ros2-architecture.md)
- [Development Roadmap](../docs/development-roadmap.md)
- [Troubleshooting](../docs/troubleshooting.md)
- [M2 README](M2_README.md)
