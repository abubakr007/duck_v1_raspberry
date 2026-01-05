import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess

class RebootService(Node):
    def __init__(self):
        super().__init__('reboot_service')
        self.srv = self.create_service(Trigger, 'reboot_pi', self.reboot_callback)

    def reboot_callback(self, request, response):
        response.success = True
        response.message = "Rebooting Raspberry Pi..."
        subprocess.Popen(['sudo', 'reboot'])
        return response

def main():
    rclpy.init()
    node = RebootService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
