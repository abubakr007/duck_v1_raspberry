#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import tty
import termios

class DuckTeleop(Node):
    def __init__(self):
        super().__init__('duck_teleop')
        
        # Publisher for the duck's velocity command
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/duck_control/cmd_vel', 10)
        
        # --- Control Parameters ---
        # Linear speed when moving forward/backward
        self.linear_speed = 0.5  
        # Angular speed when turning left/right
        self.angular_speed = 0.5 
        
        self.get_logger().info("Duck Keyboard Teleop Node Started")
        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("---------------------------")
        self.get_logger().info("Control Your Duck!")
        self.get_logger().info("---------------------------")
        self.get_logger().info("Moving around:")
        self.get_logger().info("  w    : Move forward")
        self.get_logger().info("  s    : Move backward")
        self.get_logger().info("  a    : Turn left")
        self.get_logger().info("  d    : Turn right")
        self.get_logger().info("  q    : Stop the robot")
        self.get_logger().info("  k    : Exit the program")
        self.get_logger().info("---------------------------")
        self.get_logger().info("Press 'Ctrl+C' to quit")

    def get_key(self):
        # Settings for terminal to read single keypress
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Create a new TwistStamped message
                cmd_vel_msg = TwistStamped()
                cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_vel_msg.header.frame_id = 'base_footprint' # Or 'odom', doesn't strictly matter for this

                if key == 'w':
                    cmd_vel_msg.twist.linear.x = self.linear_speed
                    self.get_logger().info("Moving forward...", throttle_duration_sec=1)
                elif key == 's':
                    cmd_vel_msg.twist.linear.x = -self.linear_speed
                    self.get_logger().info("Moving backward...", throttle_duration_sec=1)
                elif key == 'a':
                    cmd_vel_msg.twist.angular.z = self.angular_speed
                    self.get_logger().info("Turning left...", throttle_duration_sec=1)
                elif key == 'd':
                    cmd_vel_msg.twist.angular.z = -self.angular_speed
                    self.get_logger().info("Turning right...", throttle_duration_sec=1)
                elif key == 'q':
                    # Stop command (both velocities are 0 by default)
                    self.get_logger().info("Stopping robot.")
                elif key == 'k':
                    # Exit the program
                    self.get_logger().info("Exiting program.")
                    # Publish a stop command before exiting
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                    # Break the loop to exit
                    break
                else:
                    # For any other key, do nothing
                    continue
                
                # Publish the command
                self.cmd_vel_pub.publish(cmd_vel_msg)

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down teleop node.")
        finally:
            # Publish a final stop command when exiting
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = DuckTeleop()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
