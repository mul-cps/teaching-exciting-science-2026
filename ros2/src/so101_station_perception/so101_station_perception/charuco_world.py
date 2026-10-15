"""Track a fixed ChArUco board as the visual world while the camera moves."""

import json
import math
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .calibration import matrix_quaternion, rigid_matrix
from .tag_observations import stamp_is_fresh, stamp_seconds


def world_from_opencv_board():
    """Right-handed table world: keep board X, reverse Y, point Z outward/up."""
    return np.diag([1.0, -1.0, -1.0, 1.0])


def board_definition():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    board = aruco.CharucoBoard((7, 5), 0.025, 0.018, dictionary)
    return board, aruco.CharucoDetector(board)


def camera_matrices(info, image_shape):
    height, width = image_shape[:2]
    if (int(info.width), int(info.height)) != (int(width), int(height)):
        raise ValueError("CameraInfo dimensions differ from color image")
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(info.d, dtype=np.float64).reshape(-1, 1)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("CameraInfo has invalid focal lengths or non-finite values")
    return k, d


def estimate_board_pose(image_bgr, info, board, detector, min_corners=6,
                        max_reprojection_error_px=2.5):
    """Return optical_T_board from calibrated PnP, or fail closed with a reason."""
    k, d = camera_matrices(info, image_bgr.shape)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < int(min_corners):
        raise ValueError(f"only {0 if ids is None else len(ids)} ChArUco corners (need {min_corners})")
    ids = np.asarray(ids, dtype=np.int32).reshape(-1)
    pixels = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    object_all = np.asarray(board.getChessboardCorners(), dtype=np.float64).reshape(-1, 3)
    if np.any(ids < 0) or np.any(ids >= len(object_all)):
        raise ValueError("detector returned an invalid ChArUco corner ID")
    object_points = object_all[ids]
    # Limit pathological detector output and keep each solve bounded.
    if len(ids) > 24:
        object_points, pixels = object_points[:24], pixels[:24]
    ok, rvec, tvec = cv2.solvePnP(object_points, pixels, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        raise ValueError("calibrated solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, k, d)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - pixels) ** 2, axis=1))))
    depths = (rotation @ object_points.T + tvec.reshape(3, 1))[2]
    if not np.isfinite(rotation).all() or np.linalg.det(rotation) < 0.99:
        raise ValueError("PnP returned an invalid rotation")
    if np.any(depths <= 0) or float(tvec.reshape(3)[2]) <= 0:
        raise ValueError("board pose is behind the camera")
    if not math.isfinite(rms) or rms > float(max_reprojection_error_px):
        raise ValueError(f"reprojection RMS {rms:.3f}px exceeds limit")
    optical_from_board = np.eye(4)
    optical_from_board[:3, :3] = rotation
    optical_from_board[:3, 3] = tvec.reshape(3)
    return optical_from_board, len(ids), rms


def transform_matrix(msg):
    tr = msg.transform
    q = np.array([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = q / norm
    m = np.eye(4)
    m[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    m[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return rigid_matrix(m)


def transform_message(parent, child, matrix, stamp):
    m = rigid_matrix(matrix)
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id, msg.header.stamp = parent, child, stamp
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, m[:3, 3])
    q = matrix_quaternion(m)
    msg.transform.rotation.x, msg.transform.rotation.y = float(q[0]), float(q[1])
    msg.transform.rotation.z, msg.transform.rotation.w = float(q[2]), float(q[3])
    return msg


class CharucoWorld(Node):
    def __init__(self):
        cv2.setNumThreads(1)
        super().__init__("so101_charuco_world")
        defaults = {
            "color_topic": "/camera/color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "optical_frame": "camera_color_optical_frame",
            "camera_link_frame": "camera_link",
            "board_frame": "charuco_board",
            "max_pair_delta_s": 0.04,
            "max_age_s": 0.5,
            "min_corners": 6,
            "max_reprojection_error_px": 2.5,
            "max_processing_hz": 5.0,
            "initial_base_from_camera_file": "",
            "initial_camera_from_board_file": "",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.board_frame = str(self.get_parameter("board_frame").value)
        self.optical_frame = str(self.get_parameter("optical_frame").value)
        self.camera_link_frame = str(self.get_parameter("camera_link_frame").value)
        self.max_age = float(self.get_parameter("max_age_s").value)
        self.min_corners = int(self.get_parameter("min_corners").value)
        self.max_error = float(self.get_parameter("max_reprojection_error_px").value)
        self.period = 1.0 / min(5.0, max(0.1, float(self.get_parameter("max_processing_hz").value)))
        self.last_process_mono = 0.0
        self.last_stamp = None
        self.last_valid_stamp = None
        self.bridge = CvBridge()
        self.board, self.detector = board_definition()
        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.status_pub = self.create_publisher(String, "/so101_perception/charuco_world/status", 10)
        image = message_filters.Subscriber(self, Image, self.get_parameter("color_topic").value,
                                           qos_profile=qos_profile_sensor_data)
        info = message_filters.Subscriber(self, CameraInfo, self.get_parameter("camera_info_topic").value,
                                          qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image, info], queue_size=6, slop=float(self.get_parameter("max_pair_delta_s").value))
        self.sync.registerCallback(self._pair)
        self.timer = self.create_timer(0.2, self._stale_status)
        self._publish_provisional_base_seed()

    def _status(self, state, stamp=None, **extra):
        payload = {"state": state, "observation_valid": state == "tracking", "base_calibration_qualified": False, "stamp": stamp, **extra}
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _pair(self, image_msg, info_msg):
        stamp = stamp_seconds(image_msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if not stamp_is_fresh(now, stamp, self.max_age):
            self._status("stale", stamp=stamp, reason="image timestamp is stale")
            return
        if image_msg.header.frame_id != self.optical_frame or info_msg.header.frame_id != self.optical_frame:
            self._status("invalid", stamp=stamp, reason="image/CameraInfo frame does not match optical_frame")
            return
        if abs(stamp - stamp_seconds(info_msg.header.stamp)) > float(self.get_parameter("max_pair_delta_s").value):
            self._status("invalid", stamp=stamp, reason="image and CameraInfo timestamps differ")
            return
        if self.last_stamp == stamp or time.monotonic() - self.last_process_mono < self.period:
            return
        self.last_stamp, self.last_process_mono = stamp, time.monotonic()
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            optical_from_board, count, rms = estimate_board_pose(
                image, info_msg, self.board, self.detector, self.min_corners, self.max_error)
            # lookup gives optical_T_camera_link; compose board_T_camera_link
            # so the vendor camera_link -> optical edge remains the sole parent.
            optical_from_link_msg = self.buffer.lookup_transform(
                self.optical_frame, self.camera_link_frame, Time())
            optical_from_link = transform_matrix(optical_from_link_msg)
            board_from_link = world_from_opencv_board() @ np.linalg.inv(optical_from_board) @ optical_from_link
            tf = transform_message(self.board_frame, self.camera_link_frame, board_from_link,
                                   image_msg.header.stamp)
            self.broadcaster.sendTransform(tf)
            self.last_valid_stamp = stamp
            self._status("tracking", stamp=stamp, charuco_corners=count, reprojection_rms_px=rms,
                         parent=self.board_frame, child=self.camera_link_frame)
        except (ValueError, cv2.error, TransformException, np.linalg.LinAlgError) as exc:
            self._status("occluded", stamp=stamp, reason=str(exc))

    def _stale_status(self):
        if self.last_valid_stamp is None:
            self._status("occluded", reason="no board observation yet")
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not stamp_is_fresh(now, self.last_valid_stamp, self.max_age):
            self._status("stale", stamp=self.last_valid_stamp, reason="no fresh board observation")

    def _publish_provisional_base_seed(self):
        base_path = str(self.get_parameter("initial_base_from_camera_file").value)
        board_path = str(self.get_parameter("initial_camera_from_board_file").value)
        if not base_path and not board_path:
            return
        if not base_path or not board_path:
            raise ValueError("both provisional transform files must be provided")
        # Files contain a 4x4 matrix under the named transform key. This seed
        # only places the robot in the board world; it is never qualified.
        with open(base_path, encoding="utf-8") as f:
            base_data = json.load(f)
        with open(board_path, encoding="utf-8") as f:
            board_data = json.load(f)
        base_from_camera = rigid_matrix(base_data["base_from_camera"], "base_from_camera")
        board_matrix = board_data.get("camera_from_board")
        if board_matrix is None and board_data.get("transform_direction") == "camera_from_target":
            board_matrix = board_data["camera_from_target"]
        camera_from_board = rigid_matrix(board_matrix, "camera_from_board")
        board_from_base = world_from_opencv_board() @ np.linalg.inv(base_from_camera @ camera_from_board)
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        static = StaticTransformBroadcaster(self)
        self._base_seed_broadcaster = static
        seed = transform_message(self.board_frame, "base_link", board_from_base,
                                 self.get_clock().now().to_msg())
        static.sendTransform(seed)
        self.get_logger().warning("Publishing board->base_link INITIAL ESTIMATE (provisional; not qualified)")


def main(args=None):
    rclpy.init(args=args)
    node = CharucoWorld()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
