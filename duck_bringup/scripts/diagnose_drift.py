#!/usr/bin/env python3
"""
Odometry Drift Diagnostic Script

This script monitors all odometry and transform sources to identify
which sensor is causing drift when the robot is stationary.

Usage:
1. Keep robot stationary
2. Run: python3 diagnose_drift.py
3. Watch which values are changing
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped
import numpy as np
from collections import deque


class DriftDiagnostic(Node):
    def __init__(self):
        super().__init__('drift_diagnostic')
        
        # Data storage for statistical analysis
        self.window_size = 50
        
        # Wheel encoder data
        self.joint_positions = deque(maxlen=self.window_size)
        self.joint_velocities = deque(maxlen=self.window_size)
        
        # Odometry data
        self.odom_positions = deque(maxlen=self.window_size)
        self.odom_velocities = deque(maxlen=self.window_size)
        
        # IMU data
        self.imu_angular_vel = deque(maxlen=self.window_size)
        self.imu_linear_accel = deque(maxlen=self.window_size)
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/duck_control/odom', self.odom_callback, 10)
        
        self.imu_raw_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_raw_callback, 10)
        
        self.imu_filtered_sub = self.create_subscription(
            Imu, '/imu/filtered', self.imu_filtered_callback, 10)
        
        # Timer for periodic reporting
        self.timer = self.create_timer(2.0, self.report_drift)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('DRIFT DIAGNOSTIC STARTED')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Keep robot STATIONARY and observe which values are changing...')
        self.get_logger().info('')
        
    def joint_callback(self, msg):
        """Monitor joint states (wheel encoders)"""
        if len(msg.position) >= 2 and len(msg.velocity) >= 2:
            # Assume left and right wheel
            self.joint_positions.append([msg.position[0], msg.position[1]])
            self.joint_velocities.append([msg.velocity[0], msg.velocity[1]])
    
    def odom_callback(self, msg):
        """Monitor odometry output"""
        pos = [msg.pose.pose.position.x, 
               msg.pose.pose.position.y,
               msg.pose.pose.orientation.z]
        vel = [msg.twist.twist.linear.x,
               msg.twist.twist.angular.z]
        
        self.odom_positions.append(pos)
        self.odom_velocities.append(vel)
    
    def imu_raw_callback(self, msg):
        """Monitor raw IMU data"""
        gyro = [msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z]
        accel = [msg.linear_acceleration.x,
                 msg.linear_acceleration.y,
                 msg.linear_acceleration.z]
        
        self.imu_angular_vel.append(gyro)
        self.imu_linear_accel.append(accel)
    
    def imu_filtered_callback(self, msg):
        """Monitor filtered IMU data"""
        # Just log that we received it
        pass
    
    def report_drift(self):
        """Analyze and report drift statistics"""
        self.get_logger().info('─' * 70)
        
        # 1. Wheel Encoder Analysis
        if len(self.joint_velocities) > 10:
            vels = np.array(list(self.joint_velocities))
            left_vel_std = np.std(vels[:, 0])
            right_vel_std = np.std(vels[:, 1])
            left_vel_mean = np.mean(vels[:, 0])
            right_vel_mean = np.mean(vels[:, 1])
            
            self.get_logger().info('WHEEL ENCODERS:')
            self.get_logger().info(f'  Left  Wheel: mean={left_vel_mean:+.4f} rad/s, std={left_vel_std:.4f}')
            self.get_logger().info(f'  Right Wheel: mean={right_vel_mean:+.4f} rad/s, std={right_vel_std:.4f}')
            
            # Check for problematic encoders
            if abs(left_vel_mean) > 0.01 or abs(right_vel_mean) > 0.01:
                self.get_logger().warn('  ⚠ ISSUE: Encoders showing velocity when stationary!')
            if left_vel_std > 0.05 or right_vel_std > 0.05:
                self.get_logger().warn('  ⚠ ISSUE: High encoder noise detected!')
        else:
            self.get_logger().warn('WHEEL ENCODERS: Waiting for data...')
        
        # 2. Odometry Analysis
        if len(self.odom_velocities) > 10:
            vels = np.array(list(self.odom_velocities))
            linear_vel_std = np.std(vels[:, 0])
            angular_vel_std = np.std(vels[:, 1])
            linear_vel_mean = np.mean(vels[:, 0])
            angular_vel_mean = np.mean(vels[:, 1])
            
            self.get_logger().info('')
            self.get_logger().info('ODOMETRY OUTPUT:')
            self.get_logger().info(f'  Linear  Vel: mean={linear_vel_mean:+.4f} m/s, std={linear_vel_std:.4f}')
            self.get_logger().info(f'  Angular Vel: mean={angular_vel_mean:+.4f} rad/s, std={angular_vel_std:.4f}')
            
            if abs(linear_vel_mean) > 0.005 or abs(angular_vel_mean) > 0.01:
                self.get_logger().warn('  ⚠ DRIFT SOURCE: Odometry reporting movement!')
        else:
            self.get_logger().warn('ODOMETRY: Waiting for data...')
        
        # 3. IMU Analysis
        if len(self.imu_angular_vel) > 10:
            gyro = np.array(list(self.imu_angular_vel))
            gyro_std = np.std(gyro, axis=0)
            gyro_mean = np.mean(gyro, axis=0)
            
            self.get_logger().info('')
            self.get_logger().info('IMU (RAW):')
            self.get_logger().info(f'  Gyro X: mean={np.degrees(gyro_mean[0]):+.3f}°/s, std={np.degrees(gyro_std[0]):.3f}')
            self.get_logger().info(f'  Gyro Y: mean={np.degrees(gyro_mean[1]):+.3f}°/s, std={np.degrees(gyro_std[1]):.3f}')
            self.get_logger().info(f'  Gyro Z: mean={np.degrees(gyro_mean[2]):+.3f}°/s, std={np.degrees(gyro_std[2]):.3f}')
            
            if any(np.abs(np.degrees(gyro_mean)) > 1.0):
                self.get_logger().warn('  ⚠ IMU may need recalibration')
        else:
            self.get_logger().warn('IMU: Waiting for data...')
        
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DriftDiagnostic()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\nDiagnostic stopped by user')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
