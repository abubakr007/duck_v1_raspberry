#!/usr/bin/env python3
import rclpy.time
import smbus
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")
        
        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # Calibration
        self.gyro_x_offset_ = 0.0
        self.gyro_y_offset_ = 0.0
        self.gyro_z_offset_ = 0.0
        self.calibrate_gyro(200) # Calibrate with 200 samples

        # Complementary Filter variables
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.yaw_ = 0.0
        self.alpha_ = 0.96 # Filter coefficient (0.96 means 96% Gyro, 4% Accel)
        self.last_time_ = self.get_clock().now()

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link"
        
        # Set covariance matrices
        self.imu_msg_.orientation_covariance = [
            0.0025, 0.0,    0.0,
            0.0,    0.0025, 0.0,
            0.0,    0.0,    0.0025
        ]
        self.imu_msg_.angular_velocity_covariance = [
            0.02, 0.0,  0.0,
            0.0,  0.02, 0.0,
            0.0,  0.0,  0.02
        ]
        self.imu_msg_.linear_acceleration_covariance = [
            0.04, 0.0,  0.0,
            0.0,  0.04, 0.0,
            0.0,  0.0,  0.04
        ]
        
        self.frequency_ = 0.02 # 50Hz for better integration
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        
        # Debug counter
        self.debug_counter_ = 0

    def calibrate_gyro(self, samples):
        """Calibrate gyroscope by calculating offsets"""
        self.get_logger().info("Starting calibration... KEEP ROBOT STATIONARY!")
        x_sum = 0.0
        y_sum = 0.0
        z_sum = 0.0
        
        for i in range(samples):
            try:
                gyro_x = self.read_raw_data(GYRO_XOUT_H)
                gyro_y = self.read_raw_data(GYRO_YOUT_H)
                gyro_z = self.read_raw_data(GYRO_ZOUT_H)
                
                x_sum += (gyro_x / 131.0) * 0.017453293
                y_sum += (gyro_y / 131.0) * 0.017453293
                z_sum += (gyro_z / 131.0) * 0.017453293
            except OSError:
                continue
            
            # Small delay not processed by ROS spin, handled by time.sleep if needed, 
            # but here we rely on the bus speed. A tight loop is okay for calibration 
            # if we don't block too long. 
            pass
        
        self.gyro_x_offset_ = x_sum / samples
        self.gyro_y_offset_ = y_sum / samples
        self.gyro_z_offset_ = z_sum / samples
        
        self.get_logger().info(f"Calibration Complete. Offsets -> X: {self.gyro_x_offset_:.4f}, Y: {self.gyro_y_offset_:.4f}, Z: {self.gyro_z_offset_:.4f}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (x, y, z, w)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
            
            # Calculate dt
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time_).nanoseconds / 1e9
            self.last_time_ = current_time
            
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Convert to proper units
            ax = (acc_x / 16384.0) * 9.81
            ay = (acc_y / 16384.0) * 9.81
            az = (acc_z / 16384.0) * 9.81
            
            self.imu_msg_.linear_acceleration.x = ax
            self.imu_msg_.linear_acceleration.y = ay
            self.imu_msg_.linear_acceleration.z = az
            
            # Gyroscope (rad/s) - Apply Calibration
            gyro_x_rad = ((gyro_x / 131.0) * 0.017453293) - self.gyro_x_offset_
            gyro_y_rad = ((gyro_y / 131.0) * 0.017453293) - self.gyro_y_offset_
            gyro_z_rad = ((gyro_z / 131.0) * 0.017453293) - self.gyro_z_offset_
            
            # Deadzone for very small noise
            if abs(gyro_z_rad) < 0.005: gyro_z_rad = 0.0
            if abs(gyro_x_rad) < 0.005: gyro_x_rad = 0.0
            if abs(gyro_y_rad) < 0.005: gyro_y_rad = 0.0
            
            self.imu_msg_.angular_velocity.x = gyro_x_rad
            self.imu_msg_.angular_velocity.y = gyro_y_rad
            self.imu_msg_.angular_velocity.z = gyro_z_rad

            # Calculate roll and pitch from accelerometer
            acc_roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
            acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            
            # Complementary Filter
            self.roll_ = self.alpha_ * (self.roll_ + gyro_x_rad * dt) + (1.0 - self.alpha_) * acc_roll
            self.pitch_ = self.alpha_ * (self.pitch_ + gyro_y_rad * dt) + (1.0 - self.alpha_) * acc_pitch
            
            # Integrate gyroscope for yaw
            self.yaw_ += gyro_z_rad * dt
            
            # Normalize yaw to [-pi, pi]
            while self.yaw_ > math.pi:
                self.yaw_ -= 2.0 * math.pi
            while self.yaw_ < -math.pi:
                self.yaw_ += 2.0 * math.pi
            
            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(self.roll_, self.pitch_, self.yaw_)
            self.imu_msg_.orientation.x = qx
            self.imu_msg_.orientation.y = qy
            self.imu_msg_.orientation.z = qz
            self.imu_msg_.orientation.w = qw

            # Debug output every 50 messages
            self.debug_counter_ += 1
            if self.debug_counter_ >= 50:
                self.get_logger().info(
                    f"Roll: {math.degrees(self.roll_):.2f}° | Pitch: {math.degrees(self.pitch_):.2f}° | "
                    f"Yaw: {math.degrees(self.yaw_):.2f}° | Gyro_Z: {gyro_z_rad:.3f} rad/s"
                )
                self.debug_counter_ = 0

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError as e:
            self.get_logger().error(f"I2C Error: {e}")
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
            self.get_logger().info("MPU6050 initialized successfully")
        except OSError as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")
            self.is_connected_ = False
        
    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
            value = value - 65536
        return value


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()