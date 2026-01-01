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
        name="mpu9250driver",
        output="screen",
        parameters=[
            {
                "frame_id": "imu_link"
            }
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
                "use_mag": False,
                "publish_tf": False,
                "world_frame": "enu",
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