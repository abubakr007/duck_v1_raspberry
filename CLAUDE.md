# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 workspace for "Duck" — a differential-drive mobile robot (2WD + front caster) running on a Raspberry Pi. The stack covers hardware interfacing, localization, path planning, and autonomous navigation.

## Build Commands

```bash
# Build entire workspace
cd ~/duck_ws && colcon build

# Build a single package
cd ~/duck_ws && colcon build --packages-select <package_name>

# Build with dependencies
cd ~/duck_ws && colcon build --packages-up-to <package_name>

# Source after building
source ~/duck_ws/install/setup.bash
```

## Launch

The main entry point launches hardware, navigation, camera, and rosbridge together:
```bash
ros2 launch duck_bringup duck_app.launch.py                    # Full stack (default: camera on)
ros2 launch duck_bringup duck_app.launch.py use_camera:=false  # Without camera
```

This is also what `duck_robot.service` (systemd) runs on boot via `start_duck.sh`.

Alternative sequenced bringup (hardware → localization → navigation → rosbridge with timed delays):
```bash
ros2 launch duck_bringup duck_sequenced_bringup.launch.py
```

Individual subsystems:
```bash
ros2 launch duck_bringup real_robot.launch.py                   # Hardware + lidar + controllers + IMU
ros2 launch duck_bringup real_robot.launch.py use_camera:=true  # … + camera
ros2 launch duck_nav_stack navigation.launch.py map:=/home/abubakr/duck_ws/src/duck_localization/maps/my_house.yaml  # Nav2 + AMCL + map server
ros2 launch duck_nav_stack slam.launch.py                       # SLAM (slam_toolbox) + Nav2 for mapping
ros2 launch duck_vision camera.launch.py                        # libcamera IMX708 camera node
ros2 launch duck_localization global_localization.launch.py     # AMCL + EKF
ros2 launch duck_bringup duck_full_stack.launch.py              # A* planner + smoother + PD motion planner
ros2 launch duck_control joystick_teleop.launch.py              # Joystick manual control
ros2 launch duck_description display.launch.py                  # URDF visualization in RViz2
```

## Testing

Only basic linting tests exist (flake8, pep257, copyright) in ament_python packages:
```bash
cd ~/duck_ws && colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Package Architecture

| Package | Type | Role |
|---|---|---|
| `duck_bringup` | ament_python | Top-level launch files and sequenced bringup |
| `duck_control` | ament_cmake (C++/Python) | Differential drive kinematics, wheel controllers, joystick teleop |
| `duck_description` | ament_cmake | URDF/xacro robot model |
| `duck_firmware` | ament_cmake (C++/Python) | ros2_control hardware plugin (Arduino serial) + MPU6050 IMU driver |
| `duck_localization` | ament_python | AMCL particle filter + EKF sensor fusion |
| `duck_motion` | ament_cmake (C++/Python) | nav2_core Controller plugins: PD controller, Pure Pursuit |
| `duck_nav_stack` | ament_python | Full Nav2 navigation (AMCL + EKF + planners) & SLAM launch/config, waypoint looping node |
| `duck_navigation` | ament_cmake | Nav2 costmap and smoother server configs |
| `duck_planning` | ament_cmake (C++/Python) | A* and Dijkstra path planners |
| `duck_vision` | ament_cmake (C++) | libcamera-based camera driver (Raspberry Pi Camera Module 3 Wide / IMX708) |
| `sllidar_ros2` | ament_cmake | External: RPLiDAR C1 driver |
| `ros2_mpu9250_driver` | ament_cmake | External: MPU9250 driver (unused, replaced by MPU6050) |

## Data Flow

```
[Arduino (L298N + Encoders)]
    | serial /dev/ttyACM0 @ 115200
[DuckInterface — ros2_control HW plugin]
    | /joint_states
[simple_controller]
    | /duck_control/odom          [mpu6050_driver → /imu/out]
[EKF (robot_localization)]  ←────────┘
    | /odometry/local
[AMCL] ← /scan ← [sllidar_c1]
    | map→odom TF
[nav2_costmap_2d] → /costmap
[a_star_planner] ← /goal_pose
    | /a_star/path
[PDMotionPlanner]
    | /duck_control/cmd_vel (TwistStamped)
[simple_controller] → [DuckInterface] → serial → [Arduino]

[Pi Camera Module 3 Wide (IMX708) via libcamera]
    | /camera/image_raw, /camera/camera_info
[camera_node (duck_vision)]
```

## Key Conventions

- **Mixed C++/Python packages**: `duck_control`, `duck_firmware`, `duck_motion`, `duck_planning` use ament_cmake but install Python nodes via `ament_python_install_package()` and `install(PROGRAMS ...)` in CMakeLists.txt.
- **Plugin system**: `DuckInterface` is a `hardware_interface::SystemInterface` plugin (registered via `duck_interface.xml`). `PDMotionPlanner` and `PurePursuit` are `nav2_core::Controller` plugins (registered via `motion_planner_plugins.xml`).
- **Controller selection**: `duck_control/launch/controller.launch.py` has `use_simple_controller` (default True) and `use_python` (default False) launch arguments to switch between controller implementations.
- **real_robot launch arguments**: `use_slam` (default false), `use_camera` (default false). Camera requires `duck_vision` and a connected Pi Camera Module.
- **Topic namespacing**: Robot control topics are under `/duck_control/` (e.g., `/duck_control/cmd_vel`, `/duck_control/odom`).

## Hardware Parameters

| Parameter | Value |
|---|---|
| Drive wheel radius (physical) | 0.0335m |
| Drive wheel radius (effective, under load) | 0.032m |
| Wheel separation | 0.17m |
| Serial port | `/dev/ttyACM0` @ 115200 baud |
| IMU | MPU6050 via I2C (smbus, address 0x68) |
| Lidar | Slamtec RPLiDAR C1 |
| Camera | Raspberry Pi Camera Module 3 Wide (IMX708) via libcamera 0.5 |
| Arduino serial protocol (TX) | `r{p/n}{padded_val},l{p/n}{padded_val},,` |
| Arduino serial protocol (RX) | `r{sign}{vel},l{sign}{vel},` |

## System Dependencies

- `libserial-dev` — C++ serial communication (duck_firmware)
- `python3-smbus` — I2C for MPU6050
- `Eigen3` — Matrix math for kinematics (duck_control)
- `libcamera-dev` (>=0.5) — Camera driver (duck_vision, from Raspberry Pi apt source)
- `camera_info_manager` — ROS2 camera calibration (duck_vision)
- `ros-jazzy-image-transport-plugins` — compressed/theora/zstd image transport (used by image_republisher in duck_app)
- ROS2 packages: `ros2_control`, `nav2_*`, `robot_localization`, `rosbridge_server`, `twist_mux`, `joy`, `joy_teleop`, `image_transport`, `slam_toolbox`, `nav2_simple_commander`
