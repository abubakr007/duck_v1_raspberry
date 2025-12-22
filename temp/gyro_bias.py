import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class Bias(Node):
    def __init__(self):
        super().__init__('gyro_bias')
        self.n = 0
        self.sumx = self.sumy = self.sumz = 0.0
        self.target = 500  # samples (~10s at 50 Hz)
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)

    def cb(self, msg: Imu):
        self.sumx += msg.angular_velocity.x
        self.sumy += msg.angular_velocity.y
        self.sumz += msg.angular_velocity.z
        self.n += 1
        if self.n >= self.target:
            bx = self.sumx / self.n
            by = self.sumy / self.n
            bz = self.sumz / self.n
            print(f"gyro_bias_rad_s: x={bx:.8f} y={by:.8f} z={bz:.8f}  (N={self.n})")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Bias()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
