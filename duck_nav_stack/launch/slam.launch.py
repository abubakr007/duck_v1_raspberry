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
SLAM launch file for Duck robot.

Launches slam_toolbox for mapping + Nav2 for navigation while mapping.
Assumes real_robot.launch.py is already running (hardware, lidar, IMU).

Usage:
    ros2 launch duck_nav_stack slam.launch.py

Save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_house
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.descriptions import ParameterFile
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package directories
    nav_stack_dir = get_package_share_directory('duck_nav_stack')
    localization_dir = get_package_share_directory('duck_localization')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Default params files
    default_params = os.path.join(nav_stack_dir, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(
        nav_stack_dir, 'config', 'slam_toolbox_params.yaml'
    )

    # EKF config from duck_localization
    ekf_config = os.path.join(localization_dir, 'config', 'ekf.yaml')

    # Rewrite the params file with autostart substitution
    param_substitutions = {'autostart': autostart}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Force line-buffered stdout
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # --- Declare launch arguments ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameters file',
    )
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='Full path to the SLAM toolbox parameters file',
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

    # --- SLAM Toolbox (lifecycle node — configure + activate via events) ---
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Auto-configure when slam_toolbox starts
    configure_slam = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(event=ChangeState(
                            lifecycle_node_matcher=lambda node: True,
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )),
                    ],
                ),
            ],
        )
    )

    # Auto-activate after configure succeeds
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(event=ChangeState(
                            lifecycle_node_matcher=lambda node: True,
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )),
                    ],
                ),
            ],
        )
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

    # Delay navigation to let SLAM initialize and publish map->odom TF
    navigation_delayed = TimerAction(
        period=5.0,
        actions=[navigation_nodes],
    )

    # --- Build launch description ---
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_slam_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_log_level)

    ld.add_action(ekf_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(configure_slam)
    ld.add_action(activate_slam)
    ld.add_action(navigation_delayed)

    return ld
