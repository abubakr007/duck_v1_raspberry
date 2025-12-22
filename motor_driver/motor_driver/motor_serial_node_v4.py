import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState  # Add this import
from geometry_msgs.msg import Quaternion, TransformStamped
from transforms3d.euler import euler2quat
import math
from tf2_ros import TransformBroadcaster


class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Publishers
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Int32MultiArray, '/motor_speeds', 10)

        # Subscribers
        self.speed_sub = self.create_subscription(Int32MultiArray, 'motor_speeds', self.speed_callback, 10)

        # Serial connection
        self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.create_timer(0.05, self.read_serial)  # 20 Hz
        self.create_timer(0.02, self.update_odometry)  # 50 Hz

        # Robot params
        self.wheel_radius = 0.033605  # meters (actual measured)
        self.wheel_base = 0.131      # meters (actual measured)

        # State
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        # Add this publisher (place it with other publishers)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Track wheel positions (radians)
        self.left_front_wheel_pos = 0.0
        self.left_rear_wheel_pos = 0.0
        self.right_front_wheel_pos = 0.0
        self.right_rear_wheel_pos = 0.0

        self.max_pwm = 255
        self.max_lin_speed = 1.0  # m/s
        self.max_ang_speed = 2.0  # rad/s
        self.wheel_base = 0.2     # meters

        self.v_l = 0.0
        self.v_r = 0.0
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
                    # self.rpm_l = float(parts[0])
                    # self.rpm_r = float(parts[1])
                    self.v_l = float(parts[0])  # Already in m/s
                    self.v_r = float(parts[1])  # Already in m/s
                    msg = Float32MultiArray(data=[self.v_l, self.v_r])
                    self.rpm_publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to parse: {e}")
    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Convert RPM to m/s
        # v_l = (self.rpm_l / 60.0) * 2 * math.pi * self.wheel_radius
        # v_r = (self.rpm_r / 60.0) * 2 * math.pi * self.wheel_radius
        v_l = self.v_l
        v_r = self.v_r

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

        # Quaternion from yaw (NOTE: euler2quat returns [w, x, y, z])
        q = euler2quat(0, 0, self.theta)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_publisher.publish(odom)

        # Publish odom -> base_footprint transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        self.tf_broadcaster.sendTransform(t)

        # Update wheel positions (integrate RPM to radians)
        # self.left_front_wheel_pos += (self.rpm_l / 60.0) * 2 * math.pi * dt
        # self.left_rear_wheel_pos += (self.rpm_l / 60.0) * 2 * math.pi * dt
        # self.right_front_wheel_pos += (self.rpm_r / 60.0) * 2 * math.pi * dt
        # self.right_rear_wheel_pos += (self.rpm_r / 60.0) * 2 * math.pi * dt
        delta_rad_l = (self.v_l / self.wheel_radius) * dt
        delta_rad_r = (self.v_r / self.wheel_radius) * dt

        self.left_front_wheel_pos += delta_rad_l
        self.left_rear_wheel_pos += delta_rad_l
        self.right_front_wheel_pos += delta_rad_r
        self.right_rear_wheel_pos += delta_rad_r


        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = ['left_front_wheel_joint', 'left_rear_wheel_joint', 'right_front_wheel_joint', 'right_rear_wheel_joint']
        joint_msg.position = [self.left_front_wheel_pos, self.left_rear_wheel_pos, self.right_front_wheel_pos, self.right_rear_wheel_pos]
        #joint_msg.velocity = [v_l, v_r]
        joint_msg.velocity = [v_l, v_l, v_r, v_r]
        self.joint_pub.publish(joint_msg)

    # def update_odometry(self):
    #     now = self.get_clock().now()
    #     dt = (now - self.last_time).nanoseconds / 1e9
    #     self.last_time = now

    #     # Convert RPM to m/s
    #     v_l = (self.rpm_l / 60.0) * 2 * math.pi * self.wheel_radius
    #     v_r = (self.rpm_r / 60.0) * 2 * math.pi * self.wheel_radius

    #     # Compute linear and angular velocity
    #     v = (v_r + v_l) / 2.0
    #     omega = (v_r - v_l) / self.wheel_base

    #     # Update pose
    #     delta_x = v * math.cos(self.theta) * dt
    #     delta_y = v * math.sin(self.theta) * dt
    #     delta_theta = omega * dt

    #     self.x += delta_x
    #     self.y += delta_y
    #     self.theta += delta_theta

    #     # Quaternion from yaw
    #     q = euler2quat(0, 0, self.theta)  # returns [w, x, y, z]

    #     # Publish odometry
    #     odom = Odometry()
    #     odom.header.stamp = now.to_msg()
    #     odom.header.frame_id = 'odom'
    #     odom.child_frame_id = 'base_footprint'

    #     odom.pose.pose.position.x = self.x
    #     odom.pose.pose.position.y = self.y
    #     odom.pose.pose.position.z = 0.0
    #     odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    #     odom.twist.twist.linear.x = v
    #     odom.twist.twist.angular.z = omega

    #     self.odom_publisher.publish(odom)

    #     # Publish transform
    #     t = TransformStamped()
    #     t.header.stamp = now.to_msg()
    #     t.header.frame_id = 'odom'
    #     t.child_frame_id = 'base_footprint'
    #     t.transform.translation.x = self.x
    #     t.transform.translation.y = self.y
    #     t.transform.translation.z = 0.0
    #     t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    #     self.tf_broadcaster.sendTransform(t)

    #     # Update wheel positions (integrate RPM to radians)
    #     dt = (now - self.last_time).nanoseconds / 1e9
    #     self.left_wheel_pos += (self.rpm_l / 60.0) * 2 * math.pi * dt
    #     self.right_wheel_pos += (self.rpm_r / 60.0) * 2 * math.pi * dt

    #     # Publish joint states (match URDF joint names!)
    #     joint_msg = JointState()
    #     joint_msg.header.stamp = now.to_msg()
    #     joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
    #     joint_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
    #     joint_msg.velocity = [v_l, v_r]  # Optional but useful for debugging
    #     self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()