import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class TwistToMotors(Node):
    def __init__(self):
        super().__init__('twist_to_motors')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Int32MultiArray, '/motor_speeds', 10)
        
        # Parameters: tune for your robot
        self.max_pwm = 255
        self.max_lin_speed = 1.0  # m/s
        self.max_ang_speed = 2.0  # rad/s
        self.wheel_base = 0.2     # meters

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive conversion
        left_speed = linear - (angular * self.wheel_base / 2.0)
        right_speed = linear + (angular * self.wheel_base / 2.0)

        # Scale to PWM range
        left_pwm = int((left_speed / self.max_lin_speed) * self.max_pwm)
        right_pwm = int((right_speed / self.max_lin_speed) * self.max_pwm)

        # Clamp PWM values
        left_pwm = max(min(left_pwm, self.max_pwm), -self.max_pwm)
        right_pwm = max(min(right_pwm, self.max_pwm), -self.max_pwm)

        msg_out = Int32MultiArray()
        msg_out.data = [left_pwm, right_pwm]
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToMotors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
