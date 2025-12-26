from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Package paths
    duck_bringup_dir = get_package_share_directory('duck_bringup')
    duck_localization_dir = get_package_share_directory('duck_localization')

    # --- First launch: real robot ---
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(duck_bringup_dir, 'launch', 'real_robot.launch.py')
        )
    )

    # --- Second launch: global localization (after 10s) ---
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(duck_localization_dir, 'launch', 'global_localization.launch.py')
        )
    )

    # --- Third launch: full stack (after another 10s) ---
    full_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(duck_bringup_dir, 'launch', 'duck_full_stack.launch.py')
        )
    )

    return LaunchDescription([

        # 1️⃣ Start real robot immediately
        real_robot_launch,

        # 2️⃣ Start localization after 10 seconds
        TimerAction(
            period=10.0,
            actions=[localization_launch]
        ),

        # 3️⃣ Start full stack after 20 seconds total
        TimerAction(
            period=20.0,
            actions=[full_stack_launch]
        ),
    ])
