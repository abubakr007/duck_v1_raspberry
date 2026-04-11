import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DuckSystemManager(Node):
    def __init__(self):
        super().__init__('duck_system_manager')
        self.create_service(Trigger, '/duck/shutdown', self.shutdown_cb)
        self.create_service(Trigger, '/duck/restart_service', self.restart_cb)
        self.get_logger().info('System manager ready')

    def shutdown_cb(self, request, response):
        self.get_logger().warn('Shutdown requested from app!')
        response.success = True
        response.message = 'Shutting down...'
        subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
        return response

    def restart_cb(self, request, response):
        self.get_logger().warn('Service restart requested from app!')
        response.success = True
        response.message = 'Restarting duck_robot service...'
        subprocess.Popen(['sudo', 'systemctl', 'restart', 'duck_robot'])
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DuckSystemManager()
    rclpy.spin(node)
    rclpy.shutdown()
