from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get URDF file path (can be .urdf or .xacro)
    urdf_file = os.path.join(
        get_package_share_directory('motor_driver'),
        'urdf',
        'lidar_robot.urdf'  # <-- change this if your file is named differently
    )

    # Parse the URDF file
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])
