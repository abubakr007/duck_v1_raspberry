"""Standalone launch for the duck_vision camera node.

Launches the libcamera-based camera_node with parameters from
``config/camera_params.yaml``. Topics:

  /camera/image_raw    sensor_msgs/Image
  /camera/camera_info  sensor_msgs/CameraInfo

Override individual parameters from the command line, e.g.:

  ros2 launch duck_vision camera.launch.py width:=640 height:=480 framerate:=30.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("duck_vision")
    default_params = os.path.join(pkg_share, "config", "camera_params.yaml")
    default_camera_info = "file://" + os.path.join(
        pkg_share, "config", "camera_info.yaml"
    )

    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    framerate = LaunchConfiguration("framerate")
    rotation_deg = LaunchConfiguration("rotation_deg")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")

    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="1280"),
        DeclareLaunchArgument("height", default_value="720"),
        DeclareLaunchArgument("framerate", default_value="15.0"),
        DeclareLaunchArgument(
            "rotation_deg",
            default_value="180",
            description="Software image rotation: 0 or 180. Use 180 if the camera "
                        "module is mounted upside-down.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_optical_link",
            description="frame_id stamped on Image and CameraInfo messages. "
                        "Must match the optical frame in duck_description.",
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value=default_camera_info,
            description="URL of the camera_info YAML. Defaults to the placeholder "
                        "file shipped with this package.",
        ),
        Node(
            package="duck_vision",
            executable="camera_node",
            name="camera",
            output="screen",
            parameters=[
                default_params,
                {
                    "width": width,
                    "height": height,
                    "framerate": framerate,
                    "rotation_deg": rotation_deg,
                    "frame_id": frame_id,
                    "camera_info_url": camera_info_url,
                },
            ],
        ),
    ])
