import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    map_name = LaunchConfiguration("map_name")
    use_sim_time = False
    amcl_config = LaunchConfiguration("amcl_config")
    lifecycle_nodes = ["map_server", "amcl"]

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )


    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(
    #         get_package_share_directory("duck_localization"),
    #         "config",
    #         "ekf.yaml"
    #     )]
    # )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory("duck_localization"),
                "config",
                "ekf.yaml"
            )
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/local')
        ]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("duck_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory("duck_localization"),
        "maps",
        "small_house.yaml"
    ])
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="nav2_costmap_2d",
        output="screen",
        parameters=[
            "/home/abubakr/duck_ws/src/duck_navigation/config/costmap.yaml",
            {
                "use_sim_time": use_sim_time,
                "robot_base_frame": "base_footprint",  # Explicitly set
                "global_frame": "map",
                "transform_tolerance": 3.0 
            },
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 30.0} 
        ],
    )

    # Second lifecycle manager for costmap (delayed start)
    nav2_lifecycle_manager_costmap = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[
            {"node_names": ["nav2_costmap_2d"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 30.0}
        ],
    )

    delayed_lifecycle_manager = TimerAction(
        period=10.0,  # Wait 3 seconds after launch
        actions=[nav2_lifecycle_manager_costmap]
    )


    # Add this node to your launch file
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    # )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_costmap,
        nav2_lifecycle_manager,
        delayed_lifecycle_manager,
        ekf_node,
        #static_tf_node
    ])