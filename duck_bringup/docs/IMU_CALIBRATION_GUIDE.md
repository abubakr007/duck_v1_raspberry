# MPU9250 IMU Calibration Guide

This guide will help you calibrate your MPU9250 IMU sensor to eliminate drift and improve robot localization accuracy.

## Why Calibration is Needed

IMU sensors suffer from:
- **Gyroscope bias**: Constant offset causing orientation drift over time
- **Accelerometer bias**: Offset in acceleration readings
- **Temperature drift**: Sensor values change with temperature
- **Manufacturing variations**: Each sensor is slightly different

Calibration corrects these biases to provide accurate readings when the robot is stationary.

## Prerequisites

1. ✓ Robot assembled and IMU mounted securely
2. ✓ ROS2 installed and workspace built
3. ✓ IMU driver node working (publishing to `/imu/data_raw`)
4. ✓ A **flat, level, stable surface** (very important!)

## Calibration Methods

### Method 1: Automatic Calibration (Built-in Driver Feature)

Your MPU9250 driver has built-in calibration support.

#### Steps:

1. **Enable calibration mode** in the config file:
   ```bash
   nano ~/duck_v1_raspberry/ros2_mpu9250_driver/params/mpu9250.yaml
   ```

2. **Set `calibrate: True`**:
   ```yaml
   mpu9250driver_node:
     ros__parameters:
       calibrate: True  # Enable automatic calibration
   ```

3. **Place robot on level surface** and keep it stationary

4. **Launch the IMU driver**:
   ```bash
   ros2 launch duck_bringup real_robot.launch.py
   ```

5. **The driver will automatically calibrate** on startup
   - Watch the terminal output for calibration messages
   - Keep the robot stationary during this process

6. **After calibration**, the driver will output the bias values
   - Copy these values back to `mpu9250.yaml`
   - Set `calibrate: False` to use the stored values

### Method 2: Manual Calibration with Python Script (Recommended)

This method gives you more control and detailed feedback.

#### Steps:

1. **Make the calibration script executable**:
   ```bash
   chmod +x ~/duck_v1_raspberry/duck_bringup/scripts/imu_calibration.py
   ```

2. **Start the IMU driver** (with calibration disabled):
   ```bash
   # In mpu9250.yaml, set calibrate: False
   ros2 run mpu9250driver mpu9250driver
   ```

3. **In another terminal, run the calibration script**:
   ```bash
   cd ~/duck_v1_raspberry
   python3 duck_bringup/scripts/imu_calibration.py
   ```

4. **Wait for completion** (takes about 5-10 seconds)
   - The script collects 500 samples
   - Shows progress updates
   - Displays calibration results

5. **Copy the output values** to your configuration file:
   ```bash
   nano ~/duck_v1_raspberry/ros2_mpu9250_driver/params/mpu9250.yaml
   ```

6. **Update the launch file** to use the calibrated config:
   See section below for details.

## Important Tips for Good Calibration

### 📍 Placement
- ✓ Use a **solid, flat, level surface** (desk, floor)
- ✗ Do NOT use carpet, soft surfaces, or uneven ground
- ✓ Ensure the robot won't vibrate or move
- ✓ Check with a spirit level if possible

### 🕐 Timing
- ✓ Let the IMU warm up for 1-2 minutes before calibration
- ✓ Keep the robot completely stationary for entire process
- ✓ Don't touch or bump the robot during calibration

### 🌡️ Environment
- ✓ Calibrate in the same environment where the robot will operate
- ✓ Let the electronics reach operating temperature
- ✗ Avoid calibrating in extreme temperatures

### 🔁 Recalibration
- Recalibrate if you notice drift returning
- Recalibrate after physical impacts or drops
- Recalibrate if operating in different temperature environments

## Updating Configuration Files

### Option A: Update driver parameters directly

Edit `ros2_mpu9250_driver/params/mpu9250.yaml`:

```yaml
mpu9250driver_node:
  ros__parameters:
    calibrate: False  # Important: disable auto-calibration
    gyro_x_offset: -0.123456  # Replace with your values
    gyro_y_offset: 0.234567   # Replace with your values
    gyro_z_offset: -0.345678  # Replace with your values
    accel_x_offset: 0.012345  # Replace with your values
    accel_y_offset: -0.023456 # Replace with your values
    accel_z_offset: 0.034567  # Replace with your values
```

### Option B: Update launch file to load calibration

Update `duck_bringup/launch/real_robot.launch.py`:

```python
imu_driver = Node(
    package="mpu9250driver",
    executable="mpu9250driver",
    name="mpu9250driver",
    output="screen",
    parameters=[
        os.path.join(
            get_package_share_directory("ros2_mpu9250_driver"),
            "params",
            "mpu9250.yaml"
        )
    ],
    remappings=[
        ("imu", "imu/data_raw"),
    ],
)
```

## Verification

After calibration, verify the results:

### 1. Check IMU Topic Output

```bash
# View raw IMU data
ros2 topic echo /imu/data_raw

# Watch for angular velocity (should be near 0 when stationary)
# Watch for linear acceleration (z should be ~9.81, x&y near 0)
```

### 2. Monitor in RViz2

```bash
# Launch RViz2 with your robot model
ros2 launch duck_bringup <your_rviz_launch>

# The robot model should remain stationary when robot is not moving
```

### 3. Check for Drift

Place the robot stationary and observe for 1-2 minutes:
- ✓ **Good**: Robot stays in same position/orientation
- ✗ **Bad**: Robot drifts or rotates slowly

## Troubleshooting

### Problem: High noise levels during calibration

**Symptoms**: Script warns about high gyroscope or accelerometer noise

**Solutions**:
1. Ensure robot is on a solid, stable surface
2. Make sure no vibrations from nearby equipment
3. Check IMU is securely mounted (not loose)
4. Let IMU warm up for longer before calibrating

### Problem: Robot still drifts after calibration

**Possible causes**:
1. **Robot was moving during calibration** - Recalibrate
2. **Temperature drift** - Calibrate at operating temperature
3. **Sensor fusion issue** - Check `imu_filter_madgwick` parameters
4. **Loose IMU mounting** - Secure the sensor properly

### Problem: Accelerometer Z-axis not near 9.81 m/s²

**Solutions**:
1. Surface is not level - Use a spirit level
2. IMU is mounted at an angle - Check mounting orientation
3. Wrong axis configuration - Verify IMU frame orientation

### Problem: Values keep changing between calibrations

**This is normal** if:
- Temperature is different
- IMU hasn't warmed up
- Different surface/mounting

**Recalibrate** when:
- Moving to new environment
- After significant temperature changes
- After physical impacts

## Advanced: Multi-Position Calibration

For even better results, you can perform a 6-position calibration:

1. Collect data with IMU in 6 orientations:
   - Face up, face down
   - Left side, right side  
   - Front side, back side

2. Use this data to calculate a full calibration matrix

This is more complex and typically not needed for wheeled robots.

## Next Steps

After calibration:

1. ✓ Set `calibrate: False` in config file
2. ✓ Restart your robot launch file
3. ✓ Verify robot doesn't drift in RViz2
4. ✓ Test robot movement and localization
5. ✓ Fine-tune `imu_filter_madgwick` parameters if needed

## Reference Values

Typical good calibration values for MPU9250:

- **Gyroscope offsets**: -2.0 to +2.0 deg/s
- **Accelerometer X/Y offsets**: -0.5 to +0.5 m/s²
- **Accelerometer Z offset**: -1.0 to +1.0 m/s² (after removing gravity)

If your values are outside these ranges, check:
- IMU mounting and wiring
- Sensor might be damaged
- Extreme temperature conditions

## Support

If you continue to experience drift after calibration:
1. Check the IMU filter parameters (see `real_robot.launch.py`)
2. Verify robot_localization/EKF configuration
3. Consider using only wheel odometry for position estimation
4. Use IMU only for orientation (yaw) estimation

---

**Last Updated**: 2026-01-01
