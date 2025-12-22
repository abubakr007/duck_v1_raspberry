import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')
        self.publisher_ = self.create_publisher(Int32, 'wheel_rpm', 10)

        # Adjust your port and baud rate
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                rpm = int(line)
                msg = Int32()
                msg.data = rpm
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published RPM: {rpm}")
            except Exception as e:
                self.get_logger().warn(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()