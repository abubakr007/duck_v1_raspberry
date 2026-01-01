# IMU Calibration - Quick Start

## 🚀 Quick Calibration Steps

### Step 1: Prepare
```bash
# Place robot on a flat, level, stable surface
# Keep it completely stationary
```

### Step 2: Run Calibration
```bash
# Terminal 1: Start IMU driver
cd ~/duck_v1_raspberry
ros2 run mpu9250driver mpu9250driver

# Terminal 2: Run calibration script
cd ~/duck_v1_raspberry
python3 duck_bringup/scripts/imu_calibration.py
```

### Step 3: Copy Values
The script will output calibration values. Copy them to:
```bash
nano ~/duck_v1_raspberry/ros2_mpu9250_driver/params/mpu9250.yaml
```

Update these lines:
```yaml
mpu9250driver_node:
  ros__parameters:
    calibrate: False  # ← Set to False!
    gyro_x_offset: [YOUR_VALUE]
    gyro_y_offset: [YOUR_VALUE]
    gyro_z_offset: [YOUR_VALUE]
    accel_x_offset: [YOUR_VALUE]
    accel_y_offset: [YOUR_VALUE]
    accel_z_offset: [YOUR_VALUE]
```

### Step 4: Test
```bash
# Restart your robot
ros2 launch duck_bringup real_robot.launch.py

# Check in RViz2 - robot should not drift when stationary
```

## ✅ Verification Checklist

- [ ] Robot on flat, level surface
- [ ] Robot kept completely stationary during calibration
- [ ] Calibration script completed without errors
- [ ] Copied values to `mpu9250.yaml`
- [ ] Set `calibrate: False` in config
- [ ] Restarted robot nodes
- [ ] Verified no drift in RViz2

## 📝 Expected Values

- **Gyroscope**: -2.0 to +2.0 deg/s
- **Accel X/Y**: -0.5 to +0.5 m/s²
- **Accel Z**: -1.0 to +1.0 m/s²

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| High noise warning | Use more stable surface, let IMU warm up |
| Still drifting | Recalibrate, check IMU mounting |
| Values wildly different | Check robot was stationary during calibration |

## 📚 Full Documentation

See `duck_bringup/docs/IMU_CALIBRATION_GUIDE.md` for complete details.
