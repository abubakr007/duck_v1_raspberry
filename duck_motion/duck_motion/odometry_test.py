#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time

class OdometryTestNode(Node):
    def __init__(self):
        super().__init__('odometry_test_node')

        # Parameters
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('cmd_vel_topic', '/duck_control/cmd_vel')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.3)

        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # State measurement
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.initial_odom_received = False

        # State machine
        self.state = 'IDLE' # IDLE, MOVE_1, TURN, MOVE_2, DONE
        self.state_start_time = 0
        self.wait_time = 1.0

        # Sub/Pub
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        
        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Odometry Test Node Started. Subscribed to {odom_topic}, publishing to {cmd_vel_topic}')
        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

        if not self.initial_odom_received:
            self.initial_odom_received = True
            self.get_logger().info(f'Received initial reading: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}')
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.state = 'MOVE_1'
            self.get_logger().info('Starting Sequence: MOVE 1 meter')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher.publish(msg)

    def control_loop(self):
        if not self.initial_odom_received or self.state == 'DONE':
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.state == 'MOVE_1':
            distance = math.sqrt(
                (self.current_x - self.start_x)**2 + 
                (self.current_y - self.start_y)**2
            )
            
            if distance >= 1.0:
                self.get_logger().info(f'Completed 1st Move. Distance: {distance:.3f}m')
                self.stop_robot()
                self.state = 'WAIT_1'
                self.state_start_time = time.time()
                # Reset start for next relative move/turn
                self.start_yaw = self.current_yaw # Ref for turn
            else:
                msg.twist.linear.x = self.linear_speed
                self.publisher.publish(msg)

        elif self.state == 'WAIT_1':
            if time.time() - self.state_start_time > self.wait_time:
                self.state = 'TURN'
                self.start_yaw = self.current_yaw
                self.get_logger().info('Starting Sequence: TURN 90 degrees Left')

        elif self.state == 'TURN':
            # Target is +90 degrees (pi/2)
            # Calculate difference
            diff = self.normalize_angle(self.current_yaw - self.start_yaw)
            
            if diff >= (math.pi / 2.0): # Passed 90 degrees
                self.get_logger().info(f'Completed Turn. Turned: {math.degrees(diff):.2f} degrees')
                self.stop_robot()
                self.state = 'WAIT_2'
                self.state_start_time = time.time()
                # Reset start for next move
                self.start_x = self.current_x
                self.start_y = self.current_y
            else:
                msg.twist.angular.z = self.angular_speed
                self.publisher.publish(msg)

        elif self.state == 'WAIT_2':
             if time.time() - self.state_start_time > self.wait_time:
                self.state = 'MOVE_2'
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.get_logger().info('Starting Sequence: MOVE 1 meter (2nd leg)')

        elif self.state == 'MOVE_2':
            distance = math.sqrt(
                (self.current_x - self.start_x)**2 + 
                (self.current_y - self.start_y)**2
            )
            
            if distance >= 1.0:
                self.get_logger().info(f'Completed 2nd Move. Distance: {distance:.3f}m')
                self.stop_robot()
                self.state = 'DONE'
                self.report_final_location()
            else:
                msg.twist.linear.x = self.linear_speed
                self.publisher.publish(msg)

    def report_final_location(self):
        self.get_logger().info('================================')
        self.get_logger().info('SEQUENCE COMPLETE')
        self.get_logger().info(f'Final RViz Location: x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}')
        self.get_logger().info('Please measure physical location and compare.')
        self.get_logger().info('================================')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
