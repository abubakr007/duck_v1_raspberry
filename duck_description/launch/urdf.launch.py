from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path
def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('duck_description'), 'urdf', 'my_robot.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    rviz_config_path = os.path.join(get_package_share_path('duck_description'), 'config', 'my_robot.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])