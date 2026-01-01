# Wheel Encoder Drift - Solutions Guide

## Problem Identified

The robot model drifts in RViz2 because the **DiffDriveController** is publishing odometry transforms based on **noisy or drifting wheel encoder data**, NOT because of theIMU.

**Key Finding**: `enable_odom_tf: true` in `duck_controllers.yaml` line 46

## Root Causes of Wheel Encoder Drift

When a robot is stationary, encoders may still report movement due to:

1. **Electrical noise** on encoder pins
2. **Vibration** causing false pulses
3. **Motor driver noise** affecting encoder signals
4. **Loose encoder connections**
5. **Software debouncing issues**
6. **Encoder resolution too low**

## Solutions (In Order of Recommendation)

### Solution 1: Add Encoder Deadband (Recommended First)

Add a velocity deadband to ignore small encoder noise.

**File**: `duck_control/config/duck_controllers.yaml`

```yaml
duck_control:
  ros__parameters:
    # ... existing parameters ...
    
    # Add these parameters to filter encoder noise
    cmd_vel_timeout: 0.5
    
    # Velocity deadband - ignore velocities below this threshold
    linear:
      x:
        # ... existing limits ...
        min_velocity: -0.7
        max_velocity: 0.7
        # Add deadband
        min_acceleration: -0.7
        max_acceleration: 0.7
    
    # Alternatively, add wheel velocity filter
    wheel_params_file: "$(find duck_control)/config/wheel_params.yaml"
```

Then create a noise filter by setting minimum detectable velocity thresholds.

### Solution 2: Disable Odometry TF Publishing (Quick Fix)

If you're using the IMU for orientation, you don't need the DiffDriveController to publish TF.

**File**: `duck_control/config/duck_controllers.yaml`

**Change line 46:**
```yaml
# Before:
enable_odom_tf: true

# After:
enable_odom_tf: false
```

**Effect**: The controller still publishes odometry on `/duck_control/odom` topic but doesn't update the TF tree, preventing drift visualization in RViz2.

**When to use**: If you plan to use EKF/UKF for sensor fusion later.

### Solution 3: Use Robot Localization EKF

Fuse wheel odometry and IMU together using an Extended Kalman Filter. This filters out noise from both sensors.

**File**: `duck_bringup/launch/real_robot.launch.py`

**Uncomment lines 140-147:**
```python
localization = IncludeLaunchDescription(
    os.path.join(
        get_package_share_directory("duck_localization"),
        "launch",
        "global_localization.launch.py"
    ),
    condition=UnlessCondition(use_slam)
)
```

And add to launch:
```python
return LaunchDescription([
    # ... existing nodes ...
    localization,  # Add this
])
```

**Configure EKF**: Edit `duck_localization/config/ekf.yaml` to:
- Trust wheel odometry for X, Y position (but not orientation)
- Trust IMU for orientation (yaw)
- Filter out noise through sensor fusion

### Solution 4: Hardware Improvements

If software solutions don't work:

1. **Add pull-up resistors** (1-10kΩ) to encoder signal lines
2. **Shield encoder wires** to reduce electromagnetic interference
3. **Add capacitors** (0.1µF) across encoder outputs
4. **Check encoder connections** - ensure secure, no loose wires
5. **Separate encoder wires** from motor power lines
6. **Use twisted pair wiring** for encoder signals

### Solution 5: Software Encoder Filtering

Create a custom encoder filter node that:
- Ignores position changes below a threshold
- Implements median filtering
- Detects and rejects outliers

## Quick Test: Identify the Problem

Run the diagnostic script:

```bash
cd ~/duck_v1_raspberry
python3 duck_bringup/scripts/diagnose_drift.py
```

Keep robot stationary and watch the output:
- If **wheel velocities** show non-zero values → encoder noise
- If **odometry** shows movement → DiffDriveController drift
- If **IMU** shows high values → IMU calibration issue (unlikely now)

## Recommended Action Plan

### Step 1: Quick Fix (Test Immediately)
```yaml
# duck_control/config/duck_controllers.yaml
enable_odom_tf: false
```

### Step 2: If Still Drifting
Enable robot_localization EKF to fuse sensors properly.

### Step 3: If Problem Persists  
Check hardware connections and add noise filtering.

### Step 4: Long-term Solution
Configure EKF properly to fuse wheel odometry + IMU + (eventually) LIDAR/Vision.

## Files to Modify

1. **`duck_control/config/duck_controllers.yaml`** - Disable odom TF or add filtering
2. **`duck_localization/config/ekf.yaml`** - Configure sensor fusion
3. **`duck_bringup/launch/real_robot.launch.py`** - Enable localization

## Expected Results

After fixing:
- ✅ Robot model stays stationary in RViz2 when physical car doesn't move
- ✅ Smooth odometry without jitter
- ✅ Accurate pose estimation during movement

---

**Next Steps**: Try Solution 2 (disable odom TF) first as a quick test, then move to Solution 3 (EKF) for proper sensor fusion.
