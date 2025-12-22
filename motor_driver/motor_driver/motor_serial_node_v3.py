import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from transforms3d.euler import euler2quat
import math
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor


class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.03),
                ('wheel_base', 0.15),
                ('serial_port', '/dev/ttyUSB1'),
                ('baudrate', 115200),
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_footprint'),
            ]
        )

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        # Initialize serial connection with error handling
        try:
            self.serial = serial.Serial(serial_port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to {serial_port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Publishers and Subscribers
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(
            Int32MultiArray, 
            'motor_speeds', 
            self.speed_callback, 
            10
        )

        # Timers
        self.create_timer(0.02, self.read_serial)  # 50 Hz for serial reads
        self.create_timer(0.05, self.update_odometry)  # 20 Hz for odometry

        # State variables
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def speed_callback(self, msg):
        try:
            left, right = msg.data
            cmd = f"L:{left} R:{right}\n"
            self.serial.write(cmd.encode())
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Invalid motor speeds message: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial(self):
        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode().strip()
                if line.startswith("L:") and "R:" in line:
                    parts = line.replace("L:", "").split(" R:")
                    self.rpm_l = float(parts[0])
                    self.rpm_r = float(parts[1])
                    
                    # Publish RPMs
                    msg = Float32MultiArray(data=[self.rpm_l, self.rpm_r])
                    self.rpm_publisher.publish(msg)
        except UnicodeDecodeError:
            self.get_logger().warn("Received invalid serial data (not UTF-8)")
        except ValueError:
            self.get_logger().warn(f"Failed to parse RPM values from: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read failed: {e}")

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
            
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

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Convert to quaternion (note: transforms3d uses w,x,y,z order)
        q = euler2quat(0, 0, self.theta)

        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Add some default covariance values
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        ]

        self.odom_publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorSerialNode()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Node crashed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()