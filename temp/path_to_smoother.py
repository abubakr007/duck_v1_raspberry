#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from nav2_msgs.action import SmoothPath


class PathToSmoother(Node):
    def __init__(self):
        super().__init__('path_to_smoother')

        # Parameters (optional)
        self.declare_parameter('input_topic', '/a_star/path')
        self.declare_parameter('output_topic', '/final/path')
        self.declare_parameter('smoother_action', '/smooth_path')
        self.declare_parameter('smoother_id', '')  # usually "" is fine

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        smoother_action = self.get_parameter('smoother_action').get_parameter_value().string_value
        self.smoother_id = self.get_parameter('smoother_id').get_parameter_value().string_value

        # QoS: Path is often latched-like in nav pipelines; keep last.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_final = self.create_publisher(Path, output_topic, qos)
        self.sub_path = self.create_subscription(Path, input_topic, self._on_path, qos)

        self.action_client = ActionClient(self, SmoothPath, smoother_action)

        self._busy = False
        self._latest_path = None

        self.get_logger().info(f"Listening for paths on: {input_topic}")
        self.get_logger().info(f"Publishing smoothed paths on: {output_topic}")
        self.get_logger().info(f"Using smoother action: {smoother_action}")

        # Wait for action server (non-blocking timer check)
        self._server_check_timer = self.create_timer(0.5, self._check_action_server_ready)

    def _check_action_server_ready(self):
        if self.action_client.server_is_ready():
            self.get_logger().info("Smoother action server is ready.")
            self._server_check_timer.cancel()
            return
        self.get_logger().warn("Waiting for smoother action server... (/smooth_path)")

    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received an empty path on /a_star/path. Ignoring.")
            return

        # Save latest always
        self._latest_path = msg

        # If already smoothing, skip (we’ll use latest next time)
        if self._busy:
            self.get_logger().info("Smoother busy; received new path but will ignore until current request finishes.")
            return

        # If server not ready yet, skip
        if not self.action_client.server_is_ready():
            self.get_logger().warn("Smoother action server not ready yet; cannot send goal.")
            return

        self._send_to_smoother(msg)

    def _send_to_smoother(self, path_msg: Path):
        self._busy = True

        goal = SmoothPath.Goal()
        goal.path = path_msg
        goal.smoother_id = self.smoother_id

        self.get_logger().info(
            f"Sending path to smoother (poses={len(path_msg.poses)}, frame={path_msg.header.frame_id})"
        )

        send_future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        # feedback_msg.feedback has fields depending on Nav2 version.
        # Keep it light to avoid log spam.
        self.get_logger().debug("Smoother feedback received.")

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Smoother rejected the goal.")
            self._busy = False
            # If we have a newer path, try again
            self._try_process_latest()
            return

        self.get_logger().info("Smoother goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Failed to get smoother result: {e}")
            self._busy = False
            self._try_process_latest()
            return

        smoothed_path = result.path
        if not smoothed_path.poses:
            self.get_logger().warn("Smoother returned an empty smoothed path.")
        else:
            self.get_logger().info(f"Publishing smoothed path (poses={len(smoothed_path.poses)}) to /final/path")
            self.pub_final.publish(smoothed_path)

        self._busy = False
        self._try_process_latest()

    def _try_process_latest(self):
        """
        If a new path arrived while we were busy, process it now.
        """
        if self._latest_path is None:
            return
        if self._busy:
            return
        if not self.action_client.server_is_ready():
            return

        # Optional: If you want to avoid re-smoothing the same path repeatedly,
        # you can add a timestamp/seq check here.
        # For now, we just do nothing unless a new callback triggers,
        # BUT since we stored latest_path, you can choose to re-run immediately.
        #
        # Uncomment next line if you want to always process the latest right away:
        # self._send_to_smoother(self._latest_path)

        pass


def main():
    rclpy.init()
    node = PathToSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
