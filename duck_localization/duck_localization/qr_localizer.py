#!/usr/bin/env python3
"""QR code landmark-based relocalization for the duck robot.

Modes
-----
auto_relocalize=True  (default, for boot):
    Scans for a known QR code on startup.  As soon as one is found, publishes
    /initialpose and shuts down.

auto_relocalize=False (for manual use):
    Keeps running.  Call /qr_relocalize to publish pose on demand.

Services
--------
/qr_record_landmark  (std_srvs/Trigger)
    Record the detected QR code's map position using the robot's current
    (assumed correct) pose.

/qr_relocalize  (std_srvs/Trigger)
    Compute the robot's map pose from the currently visible QR code and
    publish it to /initialpose.
"""

import os
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import tf2_ros


class QRLocalizer(Node):
    def __init__(self):
        super().__init__('qr_localizer')

        # ---------- parameters ----------
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('qr_size', 0.097)          # 9.7 cm
        self.declare_parameter('detection_interval', 0.5)  # seconds
        self.declare_parameter('auto_relocalize', True)    # auto mode
        self.declare_parameter('auto_timeout', 60.0)       # give up after N seconds

        self.landmarks_path = self.get_parameter('landmarks_file').value
        self.qr_size = self.get_parameter('qr_size').value
        self._det_interval = self.get_parameter('detection_interval').value
        self._auto = self.get_parameter('auto_relocalize').value
        self._auto_timeout = self.get_parameter('auto_timeout').value

        # ---------- landmarks ----------
        self.landmarks: dict = {}
        if self.landmarks_path and os.path.exists(self.landmarks_path):
            self._load_landmarks()

        # ---------- vision ----------
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.latest_qr: tuple | None = None   # (content, corners_4x2, stamp)
        self._last_det_time = 0.0
        self._done = False

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- ROS I/O ----------
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._cam_info_cb, 10)
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb,
            qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_service(Trigger, 'qr_relocalize', self._relocalize_cb)
        self.create_service(Trigger, 'qr_record_landmark', self._record_cb)

        # auto-mode timeout
        if self._auto:
            self._start_time = self.get_clock().now()
            self.get_logger().info(
                f'QR Localizer AUTO mode  (timeout={self._auto_timeout}s, '
                f'qr_size={self.qr_size}m, '
                f'landmarks={list(self.landmarks.keys())})')
        else:
            self.get_logger().info(
                f'QR Localizer MANUAL mode  (qr_size={self.qr_size}m, '
                f'landmarks={list(self.landmarks.keys())})')

    # ── config ──────────────────────────────────────────────

    def _load_landmarks(self):
        with open(self.landmarks_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        self.landmarks = data.get('landmarks', {})
        self.get_logger().info(
            f'Loaded {len(self.landmarks)} landmark(s): '
            f'{list(self.landmarks.keys())}')

    def _save_landmarks(self):
        with open(self.landmarks_path, 'w') as f:
            yaml.dump({'landmarks': self.landmarks}, f,
                      default_flow_style=False, sort_keys=False)
        self.get_logger().info(f'Saved landmarks to {self.landmarks_path}')

    # ── subscribers ─────────────────────────────────────────

    def _cam_info_cb(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _image_cb(self, msg: Image):
        if self.camera_matrix is None or self._done:
            return

        # throttle detection
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_det_time < self._det_interval:
            return
        self._last_det_time = now

        # auto-mode timeout check
        if self._auto:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
            if elapsed > self._auto_timeout:
                self.get_logger().warn(
                    f'No QR landmark found after {self._auto_timeout}s — shutting down')
                self._shutdown()
                return

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, points, _ = self.qr_detector.detectAndDecode(cv_img)

        if data and points is not None and len(points) > 0:
            self.latest_qr = (data, points[0], msg.header.stamp)
            self.get_logger().info(f'QR detected: "{data}"')

            # auto-relocalize: try immediately
            if self._auto and data in self.landmarks:
                ok, result_msg, T = self._compute_pose()
                if not ok:
                    self.get_logger().warn(f'Relocalize failed: {result_msg}')
                if ok:
                    x, y = T[0, 3], T[1, 3]
                    yaw = float(np.arctan2(T[1, 0], T[0, 0]))
                    self._publish_pose(x, y, yaw)
                    self.get_logger().info(
                        f'Auto-relocalized via QR "{data}": '
                        f'x={x:.3f} y={y:.3f} yaw={np.degrees(yaw):.1f} deg')
                    self._shutdown()

    def _shutdown(self):
        """Mark done and request graceful shutdown."""
        self._done = True
        self.get_logger().info('QR Localizer shutting down')
        # Use a timer to let the current callback finish before shutdown
        self.create_timer(0.5, lambda: rclpy.shutdown())

    # ── solvePnP ────────────────────────────────────────────

    def _solve_qr_pose(self, corners: np.ndarray, size: float):
        """Return T_cam_qr (4x4) via solvePnP, or None."""
        h = size / 2.0
        obj = np.array([[-h, -h, 0], [h, -h, 0],
                        [h, h, 0], [-h, h, 0]], dtype=np.float64)
        img = corners.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            obj, img, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    # ── TF helpers ──────────────────────────────────────────

    def _tf_to_matrix(self, ts) -> np.ndarray:
        t = ts.transform.translation
        q = ts.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _lookup_tf(self, target: str, source: str):
        """Return 4x4 matrix mapping source-frame -> target-frame, or None."""
        try:
            ts = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return self._tf_to_matrix(ts)
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'TF {target}<-{source}: {e}')
            return None

    def _landmark_to_matrix(self, lm: dict) -> np.ndarray:
        """Build T_map_qr from stored landmark data."""
        T = np.eye(4)
        T[0, 3] = lm['x']
        T[1, 3] = lm['y']
        T[2, 3] = lm['z']
        if 'rotation' in lm:
            T[:3, :3] = np.array(lm['rotation']).reshape(3, 3)
        else:
            # Legacy fallback: reconstruct from yaw only
            yaw = lm['yaw']
            c, s = np.cos(yaw), np.sin(yaw)
            T[:3, :3] = np.array([[ s, 0, c],
                                   [-c, 0, s],
                                   [ 0,-1, 0]])
        return T

    # ── services ────────────────────────────────────────────

    def _relocalize_cb(self, _req, resp):
        ok, msg, T = self._compute_pose()
        if not ok:
            resp.success = False
            resp.message = msg
            return resp

        x, y = T[0, 3], T[1, 3]
        yaw = float(np.arctan2(T[1, 0], T[0, 0]))
        self._publish_pose(x, y, yaw)
        resp.success = True
        resp.message = (f'Relocalized: x={x:.3f} y={y:.3f} '
                        f'yaw={np.degrees(yaw):.1f} deg')
        self.get_logger().info(resp.message)
        return resp

    def _record_cb(self, _req, resp):
        if self.latest_qr is None:
            resp.success = False
            resp.message = 'No QR code in view'
            return resp
        content, corners, _ = self.latest_qr
        size = self.qr_size

        T_cam_qr = self._solve_qr_pose(corners, size)
        if T_cam_qr is None:
            resp.success = False
            resp.message = 'solvePnP failed'
            return resp

        T_map_base = self._lookup_tf('map', 'base_footprint')
        T_base_cam = self._lookup_tf('base_footprint', 'camera_optical_link')
        if T_map_base is None or T_base_cam is None:
            resp.success = False
            resp.message = 'TF unavailable (is AMCL running?)'
            return resp

        # QR position in map: T_map_qr = T_map_base @ T_base_cam @ T_cam_qr
        T_map_qr = T_map_base @ T_base_cam @ T_cam_qr
        normal = T_map_qr[:3, 2]   # QR Z-axis = outward normal
        qr_yaw = float(np.arctan2(normal[1], normal[0]))

        self.landmarks[content] = {
            'x': float(T_map_qr[0, 3]),
            'y': float(T_map_qr[1, 3]),
            'z': float(T_map_qr[2, 3]),
            'yaw': qr_yaw,
            'rotation': T_map_qr[:3, :3].flatten().tolist(),
            'size': float(size),
        }
        self._save_landmarks()

        resp.success = True
        resp.message = (f'Recorded "{content}": '
                        f'x={T_map_qr[0,3]:.3f} y={T_map_qr[1,3]:.3f} '
                        f'z={T_map_qr[2,3]:.3f} yaw={np.degrees(qr_yaw):.1f} deg')
        self.get_logger().info(resp.message)
        return resp

    # ── relocalize logic ────────────────────────────────────

    def _compute_pose(self):
        """Return (ok, message, T_map_base)."""
        if self.latest_qr is None:
            return False, 'No QR code in view', None

        content, corners, _ = self.latest_qr
        if content not in self.landmarks:
            return False, f'QR "{content}" not in landmarks', None

        lm = self.landmarks[content]
        size = lm.get('size', self.qr_size)

        T_cam_qr = self._solve_qr_pose(corners, size)
        if T_cam_qr is None:
            return False, 'solvePnP failed', None

        T_base_cam = self._lookup_tf('base_footprint', 'camera_optical_link')
        if T_base_cam is None:
            return False, 'TF unavailable', None

        T_map_qr = self._landmark_to_matrix(lm)

        # T_map_base = T_map_qr @ inv(T_cam_qr) @ inv(T_base_cam)
        T_map_base = T_map_qr @ np.linalg.inv(T_cam_qr) @ np.linalg.inv(T_base_cam)
        return True, '', T_map_base

    def _publish_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = float(np.sin(yaw / 2))
        msg.pose.pose.orientation.w = float(np.cos(yaw / 2))
        # moderate covariance
        msg.pose.covariance[0] = 0.1    # x variance
        msg.pose.covariance[7] = 0.1    # y variance
        msg.pose.covariance[35] = 0.05  # yaw variance
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QRLocalizer()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
