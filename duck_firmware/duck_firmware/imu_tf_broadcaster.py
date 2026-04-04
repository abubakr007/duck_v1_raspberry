#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ImuTfBroadcaster(Node):

    def __init__(self):
        super().__init__("imu_tf_broadcaster")
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.create_subscription(Imu, "/imu/out", self.imu_callback, qos_profile_sensor_data)

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "imu_link"
        t.transform.rotation = msg.orientation
        self.tf_broadcaster_.sendTransform(t)


def main():
    rclpy.init()
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
