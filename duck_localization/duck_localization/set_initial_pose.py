import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.count = 0
        self.get_logger().info('Initial Pose Publisher Node Started')

    def publish_pose(self):
        if self.count >= 3: # Publish a few times to ensure it's received
            self.get_logger().info('Finished publishing initial pose.')
            self.destroy_node()
            rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position (0, 0, 0) - Adjust these if your robot starts elsewhere
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Orientation (0, 0, 0, 1) - Identity quaternion (facing East/X-positive)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Set covariance to indicate high confidence
        # msg.pose.covariance[0] = 0.25 # x
        # msg.pose.covariance[7] = 0.25 # y
        # msg.pose.covariance[35] = 0.068 # yaw

        self.get_logger().info('Publishing Initial Pose: x=0.0, y=0.0')
        self.publisher_.publish(msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
