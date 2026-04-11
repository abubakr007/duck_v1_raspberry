import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_camera = LaunchConfiguration('use_camera')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
    )

    duck_bringup_dir = get_package_share_directory('duck_bringup')
    duck_nav_stack_dir = get_package_share_directory('duck_nav_stack')

    # Hardware + lidar + controllers + IMU + camera
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(duck_bringup_dir, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={'use_camera': use_camera}.items(),
    )

    # Navigation stack (map server, AMCL, Nav2, etc.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(duck_nav_stack_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': '/home/abubakr/duck_ws/src/duck_localization/maps/my_house.yaml',
        }.items(),
    )

    # Republish raw camera images as compressed JPEG for the Flutter app
    image_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed'),
        ],
        condition=IfCondition(use_camera),
    )

    # Rosbridge WebSocket for Flutter app
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'delay_between_messages': 0.0}
        ],
    )

    # QR landmark relocalization — auto-detects on boot, publishes
    # /initialpose, then shuts down
    qr_landmarks_file = os.path.join(
        get_package_share_directory('duck_localization'),
        'config', 'qr_landmarks.yaml')
    qr_localizer = Node(
        package='duck_localization',
        executable='qr_localizer',
        name='qr_localizer',
        output='screen',
        parameters=[{
            'landmarks_file': qr_landmarks_file,
            'qr_size': 0.097,
            'auto_relocalize': True,
            'auto_timeout': 60.0,
        }],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        use_camera_arg,
        real_robot_launch,
        navigation_launch,
        image_republisher,
        rosbridge_node,
        qr_localizer,
    ])
