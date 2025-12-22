import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'motor_speeds',
            self.speed_callback,
            10)
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.create_timer(0.05, self.read_serial)  # 20 Hz

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
                    rpm_l = float(parts[0])
                    rpm_r = float(parts[1])
                    msg = Float32MultiArray(data=[rpm_l, rpm_r])
                    self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to parse: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
