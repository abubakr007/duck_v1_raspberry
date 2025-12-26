import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    smoother_params = "/home/abubakr/duck_ws/src/duck_navigation/config/smoother_server.yaml"
    path_to_smoother_script = "/home/abubakr/duck_ws/src/temp/path_to_smoother.py"

    return LaunchDescription([
        # 1) A* planner
        Node(
            package="duck_planning",
            executable="a_star_planner",
            name="a_star_planner",
            output="screen",
        ),

        # 2) Nav2 smoother server (Lifecycle node)
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[smoother_params],
        ),

        # 2b) Lifecycle manager to auto-configure + activate smoother_server
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_smoother",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["smoother_server"],
                # Optional: make it respawn nodes if they die (depends on Nav2 version)
                # "bond_timeout": 4.0,
            }],
        ),

        # 3) Your Python bridge node: /a_star/path -> /smooth_path -> /final/path
        ExecuteProcess(
            cmd=["python3", path_to_smoother_script],
            output="screen",
        ),

        # 4) PD motion planner
        Node(
            package="duck_motion",
            executable="pd_motion_planner.py",
            name="pd_motion_planner",
            output="screen",
        ),
    ])
