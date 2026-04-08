#!/usr/bin/env python3
# Copyright 2026 Abubakr Abdalla
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Loop the Duck robot between two waypoints in the map frame.

On startup the node grabs the robot's current pose (from /amcl_pose) and
uses it as Point A. Point B is supplied as ROS parameters. The node then
keeps sending NavigateToPose goals via Nav2's BasicNavigator, alternating
A -> B -> A -> B ... forever (or until Ctrl+C).

Parameters:
    point_b_x   (double) X coordinate of Point B in the map frame  [required]
    point_b_y   (double) Y coordinate of Point B in the map frame  [required]
    point_b_yaw (double) Yaw of Point B in radians                 [default 0.0]
    point_a_x   (double) Override Point A x (skip auto-capture)    [default NaN]
    point_a_y   (double) Override Point A y                        [default NaN]
    point_a_yaw (double) Override Point A yaw                      [default 0.0]
    loops       (int)    Max A<->B round trips, -1 = infinite      [default -1]

Usage:
    # Capture current pose as A, drive to (1.5, 0.0) and back, forever:
    ros2 run duck_nav_stack loop_waypoints \\
        --ros-args -p point_b_x:=1.5 -p point_b_y:=0.0

    # Explicit A and B:
    ros2 run duck_nav_stack loop_waypoints --ros-args \\
        -p point_a_x:=0.0 -p point_a_y:=0.0 \\
        -p point_b_x:=1.5 -p point_b_y:=0.5
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quat(yaw: float):
    """Return (z, w) of a quaternion for a rotation about Z."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointLooper(Node):
    def __init__(self):
        super().__init__('loop_waypoints')

        # --- declare parameters ---
        self.declare_parameter('point_b_x', 0.0)
        self.declare_parameter('point_b_y', 0.0)
        self.declare_parameter('point_b_yaw', 0.0)
        self.declare_parameter('point_a_x', float('nan'))
        self.declare_parameter('point_a_y', float('nan'))
        self.declare_parameter('point_a_yaw', 0.0)
        self.declare_parameter('loops', -1)

        self.b_x = self.get_parameter('point_b_x').value
        self.b_y = self.get_parameter('point_b_y').value
        self.b_yaw = self.get_parameter('point_b_yaw').value
        self.a_x = self.get_parameter('point_a_x').value
        self.a_y = self.get_parameter('point_a_y').value
        self.a_yaw = self.get_parameter('point_a_yaw').value
        self.max_loops = self.get_parameter('loops').value

        # AMCL publishes its pose with TRANSIENT_LOCAL durability latched
        amcl_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._amcl_pose = None
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            amcl_qos,
        )

        # Distance accumulator (uses EKF-fused odometry)
        self.total_distance = 0.0
        self._last_odom_xy = None
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odometry/local',
            self._odom_cb,
            10,
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_pose = msg.pose.pose

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_xy is not None:
            dx = x - self._last_odom_xy[0]
            dy = y - self._last_odom_xy[1]
            step = math.hypot(dx, dy)
            # Ignore tiny jitter (<1mm) to avoid noise drift
            if step > 0.001:
                self.total_distance += step
        self._last_odom_xy = (x, y)

    def wait_for_amcl_pose(self, timeout_sec: float = 30.0):
        self.get_logger().info('Waiting for /amcl_pose ...')
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self._amcl_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError(
                    'Timed out waiting for /amcl_pose. '
                    'Is AMCL running and is the robot localized?'
                )
        return self._amcl_pose

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        qz, qw = yaw_to_quat(float(yaw))
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p


def main():
    rclpy.init()
    node = WaypointLooper()
    navigator = BasicNavigator()

    try:
        # If Point A wasn't given explicitly, capture the current robot pose.
        if math.isnan(node.a_x) or math.isnan(node.a_y):
            pose = node.wait_for_amcl_pose()
            node.a_x = pose.position.x
            node.a_y = pose.position.y
            node.a_yaw = quat_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            node.get_logger().info(
                f'Captured current pose as Point A: '
                f'x={node.a_x:.3f}, y={node.a_y:.3f}, yaw={node.a_yaw:.3f}'
            )
        else:
            node.get_logger().info(
                f'Using Point A from parameters: '
                f'x={node.a_x:.3f}, y={node.a_y:.3f}, yaw={node.a_yaw:.3f}'
            )

        node.get_logger().info(
            f'Point B: x={node.b_x:.3f}, y={node.b_y:.3f}, yaw={node.b_yaw:.3f}'
        )

        # Wait for Nav2 to be fully up before sending goals.
        node.get_logger().info('Waiting for Nav2 to become active ...')
        navigator.waitUntilNav2Active(localizer='amcl')

        pose_a = node.make_pose(node.a_x, node.a_y, node.a_yaw)
        pose_b = node.make_pose(node.b_x, node.b_y, node.b_yaw)

        targets = [('B', pose_b), ('A', pose_a)]
        idx = 0
        round_trips = 0

        while rclpy.ok():
            label, target = targets[idx]
            target.header.stamp = node.get_clock().now().to_msg()
            node.get_logger().info(f'==> Sending goal: Point {label}')
            navigator.goToPose(target)

            while not navigator.isTaskComplete():
                rclpy.spin_once(node, timeout_sec=0.1)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                node.get_logger().info(
                    f'Reached Point {label}. '
                    f'Total distance traveled: {node.total_distance:.2f} m'
                )
            elif result == TaskResult.CANCELED:
                node.get_logger().warn(f'Goal to Point {label} was canceled.')
            elif result == TaskResult.FAILED:
                node.get_logger().error(
                    f'Goal to Point {label} failed. Retrying in 2s ...'
                )
                # brief pause then retry the same target
                end = node.get_clock().now().nanoseconds + int(2e9)
                while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                continue

            # alternate A <-> B
            idx = 1 - idx
            if idx == 0:
                round_trips += 1
                if 0 <= node.max_loops <= round_trips:
                    node.get_logger().info(
                        f'Completed {round_trips} round trip(s). Stopping.'
                    )
                    break

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'>>> FINAL total distance traveled: {node.total_distance:.2f} m <<<'
        )
        try:
            navigator.lifecycleShutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
