#!/usr/bin/env python3
"""
IMU Calibration Script for MPU9250
This script helps calibrate the gyroscope and accelerometer offsets.

Usage:
1. Place the robot on a flat, level surface
2. Keep the robot completely stationary
3. Run this script: ros2 run duck_bringup imu_calibration.py
4. Wait for calibration to complete
5. Copy the output values to your configuration file
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import sys


class IMUCalibration(Node):
    def __init__(self):
        super().__init__('imu_calibration')
        
        # Calibration parameters
        self.num_samples = 500  # Number of samples to collect
        self.sample_count = 0
        
        # Data storage
        self.gyro_x_samples = []
        self.gyro_y_samples = []
        self.gyro_z_samples = []
        self.accel_x_samples = []
        self.accel_y_samples = []
        self.accel_z_samples = []
        
        # Subscribe to raw IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('IMU Calibration Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('IMPORTANT: Keep the robot COMPLETELY STATIONARY on a LEVEL surface')
        self.get_logger().info(f'Collecting {self.num_samples} samples...')
        self.get_logger().info('=' * 60)
        
    def imu_callback(self, msg):
        """Collect IMU samples for calibration"""
        if self.sample_count < self.num_samples:
            # Store gyroscope data (angular velocity)
            self.gyro_x_samples.append(msg.angular_velocity.x)
            self.gyro_y_samples.append(msg.angular_velocity.y)
            self.gyro_z_samples.append(msg.angular_velocity.z)
            
            # Store accelerometer data (linear acceleration)
            self.accel_x_samples.append(msg.linear_acceleration.x)
            self.accel_y_samples.append(msg.linear_acceleration.y)
            self.accel_z_samples.append(msg.linear_acceleration.z)
            
            self.sample_count += 1
            
            # Progress indicator
            if self.sample_count % 50 == 0:
                progress = (self.sample_count / self.num_samples) * 100
                self.get_logger().info(f'Progress: {progress:.1f}% ({self.sample_count}/{self.num_samples})')
        
        elif self.sample_count == self.num_samples:
            # Calculate calibration values
            self.calculate_calibration()
            self.sample_count += 1  # Increment to prevent repeated calculation
            
    def calculate_calibration(self):
        """Calculate bias offsets from collected samples"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Calibration Complete! Calculating offsets...')
        self.get_logger().info('=' * 60)
        
        # Convert to numpy arrays
        gyro_x = np.array(self.gyro_x_samples)
        gyro_y = np.array(self.gyro_y_samples)
        gyro_z = np.array(self.gyro_z_samples)
        accel_x = np.array(self.accel_x_samples)
        accel_y = np.array(self.accel_y_samples)
        accel_z = np.array(self.accel_z_samples)
        
        # Calculate gyroscope biases (should be near 0 when stationary)
        gyro_x_offset = np.mean(gyro_x)
        gyro_y_offset = np.mean(gyro_y)
        gyro_z_offset = np.mean(gyro_z)
        
        # Convert rad/s to deg/s for gyroscope
        gyro_x_offset_deg = np.degrees(gyro_x_offset)
        gyro_y_offset_deg = np.degrees(gyro_y_offset)
        gyro_z_offset_deg = np.degrees(gyro_z_offset)
        
        # Calculate accelerometer biases
        # X and Y should be near 0, Z should be near 9.81 m/s² (gravity)
        accel_x_offset = np.mean(accel_x)
        accel_y_offset = np.mean(accel_y)
        accel_z_offset = np.mean(accel_z) - 9.81  # Subtract gravity
        
        # Calculate standard deviations to check data quality
        gyro_x_std = np.std(gyro_x)
        gyro_y_std = np.std(gyro_y)
        gyro_z_std = np.std(gyro_z)
        accel_x_std = np.std(accel_x)
        accel_y_std = np.std(accel_y)
        accel_z_std = np.std(accel_z)
        
        # Display results
        self.get_logger().info('')
        self.get_logger().info('GYROSCOPE CALIBRATION RESULTS:')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Gyro X Offset: {gyro_x_offset_deg:+.6f} deg/s (std: {np.degrees(gyro_x_std):.6f})')
        self.get_logger().info(f'Gyro Y Offset: {gyro_y_offset_deg:+.6f} deg/s (std: {np.degrees(gyro_y_std):.6f})')
        self.get_logger().info(f'Gyro Z Offset: {gyro_z_offset_deg:+.6f} deg/s (std: {np.degrees(gyro_z_std):.6f})')
        
        self.get_logger().info('')
        self.get_logger().info('ACCELEROMETER CALIBRATION RESULTS:')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Accel X Offset: {accel_x_offset:+.6f} m/s² (std: {accel_x_std:.6f})')
        self.get_logger().info(f'Accel Y Offset: {accel_y_offset:+.6f} m/s² (std: {accel_y_std:.6f})')
        self.get_logger().info(f'Accel Z Offset: {accel_z_offset:+.6f} m/s² (std: {accel_z_std:.6f})')
        
        # Check data quality
        self.get_logger().info('')
        self.get_logger().info('DATA QUALITY CHECK:')
        self.get_logger().info('-' * 60)
        
        # Gyroscope should have low standard deviation when stationary
        gyro_noise_ok = all([np.degrees(std) < 0.5 for std in [gyro_x_std, gyro_y_std, gyro_z_std]])
        accel_noise_ok = all([std < 0.2 for std in [accel_x_std, accel_y_std, accel_z_std]])
        
        if gyro_noise_ok:
            self.get_logger().info('✓ Gyroscope noise levels: GOOD')
        else:
            self.get_logger().warn('✗ Gyroscope noise levels: HIGH - Robot may have been moving!')
            
        if accel_noise_ok:
            self.get_logger().info('✓ Accelerometer noise levels: GOOD')
        else:
            self.get_logger().warn('✗ Accelerometer noise levels: HIGH - Robot may have been moving!')
        
        # Display configuration to copy
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('COPY THESE VALUES TO YOUR CONFIGURATION FILE:')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('File: ros2_mpu9250_driver/params/mpu9250.yaml')
        self.get_logger().info('')
        self.get_logger().info('mpu9250driver_node:')
        self.get_logger().info('  ros__parameters:')
        self.get_logger().info('    calibrate: False  # Set to False after using these values')
        self.get_logger().info(f'    gyro_x_offset: {gyro_x_offset_deg:.6f}  # [deg/s]')
        self.get_logger().info(f'    gyro_y_offset: {gyro_y_offset_deg:.6f}  # [deg/s]')
        self.get_logger().info(f'    gyro_z_offset: {gyro_z_offset_deg:.6f}  # [deg/s]')
        self.get_logger().info(f'    accel_x_offset: {accel_x_offset:.6f}  # [m/s²]')
        self.get_logger().info(f'    accel_y_offset: {accel_y_offset:.6f}  # [m/s²]')
        self.get_logger().info(f'    accel_z_offset: {accel_z_offset:.6f}  # [m/s²]')
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        
        # Shutdown
        self.get_logger().info('Calibration complete. Shutting down...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibration_node = IMUCalibration()
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        print('\nCalibration interrupted by user')
    except Exception as e:
        print(f'Error during calibration: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
