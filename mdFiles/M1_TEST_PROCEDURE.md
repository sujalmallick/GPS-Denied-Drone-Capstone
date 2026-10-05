# M1 Test Procedure and Validation Checklist

## Objective
Validate that the Gazebo + PX4 SITL integration works correctly, with the drone capable of:
- Arming without pre-arm failures
- Taking off and hovering stably
- Landing smoothly without divergence

## Prerequisites Checklist

Before proceeding with M1 testing, verify:

- [ ] **System:** Ubuntu 22.04 LTS (or compatible Linux distribution)
- [ ] **Gazebo Harmonic:** Installed and verified
  ```bash
  gz --version  # Should show Gazebo 8.x (Harmonic)
  ```
- [ ] **PX4 SITL:** Source cloned and built
  ```bash
  cd ~/src/PX4-Autopilot
  make px4_sitl_default gazebo-harmonic  # Not gazebo-classic
  ```
- [ ] **ROS 2 Humble:** Installed and sourced
  ```bash
  source /opt/ros/humble/setup.bash
  ```

## Test Environment Setup

### Step 1: Verify File Structure

```bash
cd ~/GPS-DeniedV2  # or your project root
ls -la simulation/
# Expected output:
# drwxr-xr-x worlds/
# drwxr-xr-x models/
# drwxr-xr-x launch/
```

### Step 2: Verify World File Exists

```bash
ls -la simulation/worlds/
# Should contain: simple_world.sdf
cat simulation/worlds/simple_world.sdf | head -10
# Should show valid SDF XML header
```

### Step 3: Verify Drone Model Exists

```bash
ls -la simulation/models/
# Should contain: quadrotor_px4.sdf
cat simulation/models/quadrotor_px4.sdf | head -10
# Should show valid SDF XML header with quadrotor_px4 model
```

## Test Execution

### Phase 1: Gazebo Launch Test

**Terminal 1:**

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Navigate to project
cd ~/GPS-DeniedV2

# Launch Gazebo with the world and drone model
ros2 launch simulation simple_world.launch.py
```

**Expected Output:**
- Gazebo window opens within 5-10 seconds
- World is visible with ground plane and obstacles
- Drone model appears in center of world
- No critical error messages in console

**Validation Checklist:**
- [ ] Gazebo window opens successfully
- [ ] World loads without errors
- [ ] Drone model is visible (blue central body with rotors)
- [ ] No rendering errors or crashes
- [ ] Gazebo reports "world created" in logs

### Phase 2: PX4 SITL Launch Test

**Terminal 2** (while Gazebo is running):

```bash
# Navigate to launch directory
cd ~/GPS-DeniedV2/simulation/launch

# Make script executable and run
chmod +x start_px4_sitl.sh
./start_px4_sitl.sh
```

**Expected Output:**
- PX4 initialization messages appear
- Vehicle frame "iris" is loaded
- IMU/sensor status lines appear
- No "connection failed" messages
- PX4 console prompt becomes active (wait for ">")

**Validation Checklist:**
- [ ] PX4 SITL starts without build errors
- [ ] Vehicle airframe loads correctly
- [ ] No uXRCE-DDS communication errors (yet — M2 feature)
- [ ] PX4 console is responsive (can type commands)
- [ ] No sensor initialization failures

### Phase 3: Drone Arm Test

**In PX4 Console** (Terminal 2):

```
> commander arm
[action_land] Unexpected landing
```

Or check status first:

```
> status
Commander in mode: Stabilized
State Machine: Manual Ready
```

**Expected Behavior:**
- Pre-arm checks pass (or report specific failures, not generic errors)
- If failures, investigate and fix (common issues: battery low, GPS not ready, etc.)
- Once pre-arm passes: `commander arm` succeeds without error

**Validation Checklist:**
- [ ] No pre-arm failures preventing arming
- [ ] Drone arms without error message
- [ ] In Gazebo, drone visual does NOT change (rotation should start next)
- [ ] PX4 status shows "ARMED" state

### Phase 4: Drone Takeoff Test

**In PX4 Console** (Terminal 2):

```
> commander takeoff 1.0
```

This commands the drone to take off to 1 meter altitude.

**Expected Behavior:**
- Rotors spin up smoothly in Gazebo
- Drone rises steadily (not erratically)
- Reaches ~1.0 meter altitude
- Hovers stably without oscillation
- No rapid oscillations or instability

**Validation Checklist:**
- [ ] Rotors spin smoothly (visual in Gazebo)
- [ ] Drone rises without erratic pitch/roll
- [ ] Altitude increases smoothly over ~2-3 seconds
- [ ] No collision with obstacles
- [ ] Hover is stable (no divergence in altitude/position)
- [ ] PX4 altitude reading matches expected height

### Phase 5: Stable Hover Test

**In PX4 Console** (after takeoff):

Wait 5-10 seconds while monitoring the drone in Gazebo.

```
> status
```

Check reported altitude and position.

**Expected Behavior:**
- Drone maintains altitude within ±0.1m
- No significant drift in X/Y position
- Control surface readings (pitch/roll) are near zero
- IMU data is clean (no NaN values)

**Validation Checklist:**
- [ ] Altitude holds steady (within ±0.1m of target)
- [ ] No slow drift in position
- [ ] No oscillation or wobbling
- [ ] Control loops are stable
- [ ] Sensor data is sane (not NaN, not stale)

### Phase 6: Drone Land Test

**In PX4 Console** (Terminal 2):

```
> commander land
```

**Expected Behavior:**
- Rotors begin to slow down
- Drone descends smoothly
- Lands gently on ground plane
- Rotors stop spinning
- Drone disarms automatically (or remains armed, depending on config)

**Validation Checklist:**
- [ ] Descent is smooth (not erratic)
- [ ] Drone does not bounce or oscillate
- [ ] Soft contact with ground (no hard impact)
- [ ] Rotors stop within 1-2 seconds of landing
- [ ] No divergence or instability during descent
- [ ] Position stabilizes at z ≈ 0.0

## Failure Analysis

If any phase fails, investigate:

### Gazebo Fails to Start

**Symptoms:**
- Error: "Failed to load world file"
- Error: "Resource path not found"
- Gazebo crashes or hangs

**Troubleshooting:**
1. Verify world file path in launch file
2. Ensure `GZ_SIM_RESOURCE_PATH` is set correctly
3. Check SDF syntax: `gz sdf -k simulation/worlds/simple_world.sdf`
4. Try launching manually: `gz sim -r simulation/worlds/simple_world.sdf`

### PX4 SITL Fails to Start

**Symptoms:**
- Error: "Build directory not found"
- Error: "command not found: px4"
- Gazebo crashes after PX4 starts

**Troubleshooting:**
1. Verify PX4 build succeeded: `ls ~/src/PX4-Autopilot/build/px4_sitl_default/bin/px4`
2. Ensure build target matches Gazebo version (Harmonic, NOT Classic)
3. Clean rebuild: `cd ~/src/PX4-Autopilot && make clean && make px4_sitl_default gazebo-harmonic`
4. Check physics engine compatibility (ODE/Bullet vs Gazebo Harmonic)

### Drone Fails to Arm

**Symptoms:**
- `commander arm` returns error
- Pre-arm checks fail

**Possible Causes:**
- Battery low (check `param show SYS_HAS_BATT`)
- GPS not initialized (should be OK for SITL)
- IMU not responding (check sensor status)
- Safety switch not triggered

**Fixes:**
```
# In PX4 console, disable GPS requirement:
param set GPS_1_CONFIG 0

# Arm again:
commander arm
```

### Drone Takes Off but Oscillates or Diverges

**Symptoms:**
- Drone spirals out of control
- Altitude diverges (keeps climbing or falls)
- Wild pitch/roll oscillations

**Possible Causes:**
- Physics tuning mismatch between PX4 and Gazebo
- IMU noise model too aggressive
- PID controller tuning not compatible with Gazebo Harmonic
- Propeller thrust model mismatch

**Fixes:**
1. Check PX4 logs: in PX4 console, `log start` / `log stop` / `log upload`
2. Verify IMU noise is reasonable (check `simulation/models/quadrotor_px4.sdf`)
3. Compare motor/propeller parameters with standard PX4 Iris model
4. Reduce IMU noise if too high

### Drone Does Not Respond to Commands

**Symptoms:**
- `commander takeoff` returns OK but nothing happens
- Rotors don't spin

**Possible Causes:**
- Gazebo/PX4 communication is stalled
- Motor plugins not loaded
- PX4 not in offboard/auto mode

**Fixes:**
1. Verify Gazebo is still running (check window)
2. Check PX4 mode: `commander status` should show STABILIZED or AUTO
3. Look for plugin errors in Gazebo logs
4. Try manual control: in PX4 console, `param set COM_RC_IN_MODE 1` (RC input enabled)

## Post-Test Validation

Once all phases pass, confirm:

1. **Repeatable:** Run through phases 3-6 at least 2 more times. Success should be consistent.
2. **Clean logs:** No error messages in either Gazebo or PX4 console after final test.
3. **Files intact:** No corrupted output files or log fragments left behind.

## Acceptance Criteria Met

M1 is successfully validated when:

- ✅ Gazebo launches with world + drone model
- ✅ PX4 SITL connects and initializes without errors
- ✅ Drone arms without pre-arm failures
- ✅ Takeoff is smooth and stable
- ✅ Hover holds altitude and position
- ✅ Landing is smooth and controlled
- ✅ No crashes, divergence, or unsafe behavior
- ✅ All tests repeatable (at least 3 consecutive successful runs)

## Next Steps

Once M1 is validated, proceed to M2:
**ROS 2 ↔ PX4 communication works**

See `docs/development-roadmap.md` for M2 requirements.

## References

- [PX4 SITL Documentation](https://docs.px4.io/main/en/simulation/gazebo.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [MAVLink Command Reference](https://mavlink.io/en/messages/common.html)
- Project Documentation:
  - [Architecture](../docs/architecture.md)
  - [Simulation Configuration](../docs/simulation.md)
  - [Troubleshooting](../docs/troubleshooting.md)
