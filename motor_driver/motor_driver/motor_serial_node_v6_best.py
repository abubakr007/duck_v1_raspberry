import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
from transforms3d.euler import euler2quat
import math
from tf2_ros import TransformBroadcaster
from datetime import datetime

# ✅ NEW imports
import os
import json
import time

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Publishers
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, '/motor_speeds', 10)

        # Subscribers
        self.speed_sub = self.create_subscription(Int32MultiArray, 'motor_speeds', self.speed_callback, 10)

        # Serial connection
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.create_timer(0.05, self.read_serial)      # 20 Hz
        self.create_timer(0.02, self.update_odometry)  # 50 Hz

        # Robot params
        self.wheel_radius = 0.033605  # meters (actual measured)
        self.wheel_base   = 0.1735    # meters (actual measured)

        # State
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        self.max_pwm = 255
        self.max_lin_speed = 1.0  # m/s
        self.max_ang_speed = 2.0  # rad/s

        self.v_l = 0.0
        self.v_r = 0.0
        self.rpm_lr = 0.0
        self.rpm_lf = 0.0
        self.rpm_rr = 0.0
        self.rpm_rf = 0.0

        # ✅ Calibration fields
        self.pwm_to_speed = {}  # {pwm: avg_mps}
        self.calib_path = '/home/abubakr/ros2_ws/src/motor_driver/motor_driver/logs/calibration.json'

        # ✅ One-time calibration init
        self._init_calibration()

    # ---------- NEW: calibration persistence ----------
    def _init_calibration(self):
        """
        Load calibration from disk if present; otherwise run and save it.
        """
        try:
            if os.path.exists(self.calib_path):
                with open(self.calib_path, 'r') as f:
                    data = json.load(f)
                # keys are strings in JSON; convert back to int
                self.pwm_to_speed = {int(k): float(v) for k, v in data.get('pwm_to_speed', {}).items()}
                if self.pwm_to_speed:
                    self.get_logger().info(f"Loaded motor calibration from: {self.calib_path}")
                else:
                    self.get_logger().warn("Calibration file found but empty. Running calibration...")
                    self._run_and_save_calibration()
            else:
                self.get_logger().info("No calibration file found. Running one-time calibration...")
                self._run_and_save_calibration()
        except Exception as e:
            self.get_logger().warn(f"Calibration load failed ({e}). Falling back to default scaling.")
            self.pwm_to_speed = {}

    def _run_and_save_calibration(self):
        mapping = self.calibrate_motor()
        self.pwm_to_speed = mapping or {}
        try:
            os.makedirs(os.path.dirname(self.calib_path), exist_ok=True)
            with open(self.calib_path, 'w') as f:
                json.dump({
                    "generated_at": datetime.now().isoformat(),
                    "wheel_radius": self.wheel_radius,
                    "wheel_base": self.wheel_base,
                    "pwm_to_speed": self.pwm_to_speed
                }, f, indent=2)
            self.get_logger().info(f"Saved calibration to: {self.calib_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save calibration: {e}")

    # ---------- UPDATED: use calibration to map speed -> PWM ----------
    def _speed_to_pwm(self, mps: float) -> int:
        """
        Map desired linear speed (m/s) to PWM using calibrated curve.
        Handles sign by symmetry; uses linear interpolation between points.
        Falls back to simple scaling if no calibration exists.
        """
        if not self.pwm_to_speed:
            # fallback: linear scale against max speed
            pwm = int((mps / self.max_lin_speed) * self.max_pwm)
            return max(min(pwm, self.max_pwm), -self.max_pwm)

        sign = 1 if mps >= 0 else -1
        target = abs(mps)

        # Build sorted pairs (speed) for positive axis
        pts = sorted([(float(v), int(k)) for k, v in self.pwm_to_speed.items() if v >= 0],
                     key=lambda x: x[0])

        # Edge cases
        if target <= pts[0][0]:
            return sign * pts[0][1]
        if target >= pts[-1][0]:
            return sign * pts[-1][1]

        # Interpolate between neighbors
        for i in range(1, len(pts)):
            s0, p0 = pts[i-1]
            s1, p1 = pts[i]
            if s0 <= target <= s1:
                # linear interpolation on speed
                if s1 == s0:
                    pwm = p0
                else:
                    ratio = (target - s0) / (s1 - s0)
                    pwm = int(round(p0 + ratio * (p1 - p0)))
                return sign * pwm

        # Shouldn’t get here, but just in case
        return sign * pts[-1][1]

    # ---------- (unchanged) ----------
    def write_to_file(self, data, source):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open('/home/abubakr/ros2_ws/src/motor_driver/motor_driver/logs/motor_data_' + source + '.txt', 'a') as f:
            f.write(f"{timestamp}, {data}\n")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive conversion
        left_speed  = linear - (angular * self.wheel_base / 2.0)
        right_speed = linear + (angular * self.wheel_base / 2.0)

        # ✅ Use calibration to get PWM (keeps your existing flow)
        left_pwm  = self._speed_to_pwm(left_speed)
        right_pwm = self._speed_to_pwm(right_speed)

        # If you want to KEEP sending “rpm” downstream, convert PWM→rpm here if you have that relation.
        # For now, we’ll send the PWM values via your existing topic to the serial writer.
        msg_out = Int32MultiArray()
        msg_out.data = [left_pwm, right_pwm]
        self.publisher.publish(msg_out)
        self.write_to_file(msg_out, "from_cmd_vel")

    def speed_callback(self, msg):
        left, right = msg.data
        cmd = f"{left},{right}\n"  # left/right treated as PWM now
        self.serial.write(cmd.encode())

    def calibrate_motor(self):
        """
        Runs a short sweep of PWM values, averages measured speed (m/s) from serial,
        and returns a mapping {pwm: avg_mps}. Called only if no calibration file exists.
        """
        try:
            calibration_pwms = [50, 100, 150, 200, 255]
            actual_speeds = []

            # Ensure motors stopped before starting
            self.serial.write(b"0,0\n")
            time.sleep(1.0)

            for pwm in calibration_pwms:
                # Send symmetric PWM (forward)
                cmd = f"{pwm},{pwm}\n"
                self.serial.write(cmd.encode())
                time.sleep(2.0)  # settle

                speeds = []
                samples = 10
                for _ in range(samples):
                    line = self.serial.readline().decode(errors='ignore').strip()
                    # Expecting "v_l,v_r" in m/s from your firmware
                    if "," in line:
                        try:
                            v_l, v_r = map(float, line.split(","))
                            speeds.append((v_l + v_r) / 2.0)
                        except:
                            pass
                    time.sleep(0.1)

                avg = (sum(speeds) / len(speeds)) if speeds else 0.0
                actual_speeds.append(avg)
                self.get_logger().info(f"[CAL] PWM {pwm} → {avg:.3f} m/s")

            # Stop motors at end
            self.serial.write(b"0,0\n")
            time.sleep(0.5)

            mapping = dict(zip(calibration_pwms, actual_speeds))
            return mapping

        except Exception as e:
            self.get_logger().warn(f"Calibration failed ({e}).")
            return {}

    # ===== rest of your methods remain unchanged =====
    def read_serial(self):
        if self.serial.in_waiting:
            try:
                line = self.serial.readline().decode().strip()
                data = line.split(",")
                if len(data) == 2:
                    self.rpm_l = float(data[0])
                    self.rpm_r = float(data[1])

                    self.v_l = float(data[0])  # if firmware sends m/s already
                    self.v_r = float(data[1])

                    msg = Float32MultiArray(data=[self.rpm_l, self.rpm_r])
                    self.rpm_publisher.publish(msg)
                    self.write_to_file(line, "from_serial")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse serial: {e}")

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        v_l = self.v_l
        v_r = self.v_r

        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.wheel_base

        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        q = euler2quat(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        self.tf_broadcaster.sendTransform(t)

        delta_rad_l = (self.v_l / self.wheel_radius) * dt
        delta_rad_r = (self.v_r / self.wheel_radius) * dt
        self.left_wheel_pos += delta_rad_l
        self.right_wheel_pos += delta_rad_r

        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        vel_rad_l = v_l / self.wheel_radius
        vel_rad_r = v_r / self.wheel_radius
        joint_msg.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_msg.velocity = [vel_rad_l, vel_rad_r]
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
