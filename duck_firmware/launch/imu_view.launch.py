from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # MPU6050 driver
    mpu6050 = Node(
        package="duck_firmware",
        executable="mpu6050_driver.py",
        name="mpu6050_driver",
    )

    # Broadcasts IMU orientation as world -> imu_link TF
    imu_tf = Node(
        package="duck_firmware",
        executable="imu_tf_broadcaster.py",
        name="imu_tf_broadcaster",
    )

    return LaunchDescription([mpu6050, imu_tf])
