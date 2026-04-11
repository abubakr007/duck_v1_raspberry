# Copyright 2024 Abubakr Abdalla
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Navigation launch file for Duck robot.

Launches the full Nav2 stack with AMCL localization and EKF sensor fusion.
Assumes real_robot.launch.py is already running (hardware, lidar, IMU).

Usage:
    ros2 launch duck_nav_stack navigation.launch.py
    ros2 launch duck_nav_stack navigation.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package directories
    nav_stack_dir = get_package_share_directory('duck_nav_stack')
    localization_dir = get_package_share_directory('duck_localization')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Default map path — read from ~/.duck_active_map if it exists
    maps_dir = os.path.join(localization_dir, 'maps')
    active_map_file = os.path.expanduser('~/.duck_active_map')
    default_map = os.path.join(maps_dir, 'my_house.yaml')  # fallback
    if os.path.isfile(active_map_file):
        with open(active_map_file) as f:
            active_name = f.read().strip()
        candidate = os.path.join(maps_dir, f'{active_name}.yaml')
        if active_name and os.path.isfile(candidate):
            default_map = candidate

    # Default params file
    default_params = os.path.join(nav_stack_dir, 'config', 'nav2_params.yaml')

    # EKF config from duck_localization
    ekf_config = os.path.join(localization_dir, 'config', 'ekf.yaml')

    # Rewrite the params file with map path and autostart substitution
    param_substitutions = {
        'yaml_filename': map_yaml,
        'autostart': autostart,
    }
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Force line-buffered stdout for better log output
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # --- Declare launch arguments ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml file',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameters file',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
    )

    # --- EKF Node (not a lifecycle node) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/local')],
    )

    # --- Localization nodes (lifecycle-managed) ---
    localization_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'autostart': autostart,
                    'node_names': ['map_server', 'amcl'],
                    'bond_timeout': 30.0,
                }],
            ),
        ]
    )

    # --- Navigation nodes (lifecycle-managed, delayed) ---
    navigation_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/duck_control/cmd_vel'),
                ],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'autostart': autostart,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'velocity_smoother',
                        'bt_navigator',
                    ],
                    'bond_timeout': 30.0,
                }],
            ),
        ]
    )

    # Delay navigation startup to let localization initialize first
    navigation_delayed = TimerAction(
        period=5.0,
        actions=[navigation_nodes],
    )

    # --- Build launch description ---
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_log_level)

    ld.add_action(ekf_node)
    ld.add_action(localization_nodes)
    ld.add_action(navigation_delayed)

    return ld
