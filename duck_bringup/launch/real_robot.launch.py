import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    duck_control_pkg = get_package_share_directory('duck_control')
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # hardware_interface = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("duck_firmware"),
    #         "launch",
    #         "hardware_interface.launch.py"
    #     ),
    # )
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("duck_firmware"),
                "launch",
                "hardware_interface.launch.py",
            )
        ),
        # launch_arguments={"use_sim_time": "false"}.items(),  # if that launch supports args
    )
    # laser_driver = Node(
    #         package="rplidar_ros",
    #         executable="rplidar_node",
    #         name="rplidar_node",
    #         parameters=[os.path.join(
    #             get_package_share_directory("duck_bringup"),
    #             "config",
    #             "rplidar_a1.yaml"
    #         )],
    #         output="screen"
    # )
    imu_driver = Node(
        package="mpu9250driver",
        executable="mpu9250driver",
        name="mpu9250driver",  # Must match YAML namespace
        output="screen",
        parameters=[
            # Load calibration parameters from YAML config file
            # After calibration, update ros2_mpu9250_driver/params/mpu9250.yaml
            # with the calibration values
            os.path.join(
                get_package_share_directory("mpu9250driver"),
                "params",
                "mpu9250.yaml"
            )
        ],
        # Remap the driver output to a clear name
        remappings=[
            ("imu", "imu/data_raw"),
        ],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[
            {
                # Magnetometer settings
                "use_mag": False,
                "use_magnetic_field_msg": False,
                
                # Transform settings
                "publish_tf": False,
                "world_frame": "nwu",  # Changed from "enu" to "nwu" for better stability
                
                # Filter gain - lower value = more stable but slower response
                # Increase if response is too slow, decrease if too noisy
                "gain": 0.01,  # Default is 0.1, reduced for less drift
                
                # Stateless mode - helps reduce drift accumulation
                "stateless": False,
                
                # Remove gravitational acceleration from linear acceleration
                "remove_gravity_vector": True,
                
                # Orientation covariance (diagonal values)
                # Higher values indicate less confidence in orientation
                "orientation_stddev": 0.05,  # Increased from default to handle noise
                
                # Set fixed covariance for angular velocity and linear acceleration
                # This helps when the robot is stationary
                "fixed_covariance": True,
                
                # Angular velocity covariance
                "angular_velocity_covariance": [0.02, 0.0, 0.0, 
                                               0.0, 0.02, 0.0, 
                                               0.0, 0.0, 0.02],
                
                # Linear acceleration covariance
                "linear_acceleration_covariance": [0.04, 0.0, 0.0, 
                                                  0.0, 0.04, 0.0, 
                                                  0.0, 0.0, 0.04],
            }
        ],
        # No remapping needed - uses default topics
        # Subscribes to: imu/data_raw (from driver)
        # Publishes to: imu/data (default output)
        remappings=[
            ("imu/data", "imu/filtered"),
        ],
    )

    laser_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sllidar_ros2"),
                "launch",
                "sllidar_c1_launch.py",
            )
        ),
        # Optional: pass args if that launch file supports them
        # launch_arguments={
        #     "serial_port": "/dev/ttyUSB0",
        #     "frame_id": "laser_frame",
        #     "inverted": "false",
        #     "angle_compensate": "true",
        # }.items(),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("duck_control"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    # twist_mux_launch = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("twist_mux"),
    #         "launch",
    #         "twist_mux_launch.py"
    #     ),
    #     launch_arguments={
    #         "cmd_vel_out": "duck_control/cmd_vel_unstamped",
    #         "config_locks": os.path.join(duck_control_pkg, "config", "twist_mux_locks.yaml"),
    #         "config_topics": os.path.join(duck_control_pkg, "config", "twist_mux_topics.yaml"),
    #         "config_joy": os.path.join(duck_control_pkg, "config", "twist_mux_joy.yaml"),
    #         "use_sim_time": LaunchConfiguration("use_sim_time"),
    #     }.items(),
    # )

    # twist_relay_node = Node(
    #     package="duck_control",
    #     executable="twist_relay",
    #     name="twist_relay",
    #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    # )

    # imu_driver_node = Node(
    #     package="duck_firmware",
    #     executable="mpu6050_driver.py"
    # )

    # localization = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("duck_localization"),
    #         "launch",
    #         "global_localization.launch.py"
    #     ),
    #     condition=UnlessCondition(use_slam)
    # )

    # slam = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("duck_mapping"),
    #         "launch",
    #         "slam.launch.py"
    #     ),
    #     condition=IfCondition(use_slam)
    # )
    
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        laser_driver,
        controller,
        imu_driver,        
        imu_filter, 
        # twist_mux_launch,
        # twist_relay_node,
        # imu_driver_node,
        # localization,
        # slam
    ])