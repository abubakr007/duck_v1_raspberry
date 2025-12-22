from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
#from tf_transformations import quaternion_from_euler
from transforms3d.euler import euler2quat

import math
import time

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Publishers
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Subscribers
        self.subscription = self.create_subscription(Int32MultiArray, 'motor_speeds', self.speed_callback, 10)

        # Serial connection
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.create_timer(0.05, self.read_serial)  # 20 Hz
        self.create_timer(0.05, self.update_odometry)  # 20 Hz

        # Robot params
        self.wheel_radius = 0.03  # meters
        self.wheel_base = 0.15  # meters

        # State
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def speed_callback(self, msg):
        left, right = msg.data
        cmd = f"L:{left} R:{right}\n"
        self.serial.write(cmd.encode())

    def read_serial(self):
        if self.serial.in_waiting:
            try:
                line = self.serial.readline().decode().strip()
                if line.startswith("L:") and "R:" in line:
                    parts = line.replace("L:", "").split(" R:")
                    self.rpm_l = float(parts[0])
                    self.rpm_r = float(parts[1])
                    msg = Float32MultiArray(data=[self.rpm_l, self.rpm_r])
                    self.rpm_publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to parse: {e}")

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Convert RPM to m/s
        v_l = (self.rpm_l / 60.0) * 2 * math.pi * self.wheel_radius
        v_r = (self.rpm_r / 60.0) * 2 * math.pi * self.wheel_radius

        # Compute linear and angular velocity
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.wheel_base

        # Update pose
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Quaternion from yaw
        #q = quaternion_from_euler(0, 0, self.theta)
        q = euler2quat(0, 0, self.theta)  # returns [w, x, y, z]


        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_publisher.publish(odom)
def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
