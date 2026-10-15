"""Publish depth-backed AprilTag center observations without adding TF frames."""

import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray


def ensure_aruco_detector_api(aruco):
    """Bridge OpenCV 4.6's function API to the existing tracker API."""
    if not hasattr(aruco, "DetectorParameters") and hasattr(aruco, "DetectorParameters_create"):
        aruco.DetectorParameters = aruco.DetectorParameters_create
    if not hasattr(aruco, "ArucoDetector") and hasattr(aruco, "detectMarkers"):
        class ArucoDetectorCompat:
            def __init__(self, dictionary, parameters):
                self.dictionary = dictionary
                self.parameters = parameters

            def detectMarkers(self, image):
                return aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)

        aruco.ArucoDetector = ArucoDetectorCompat


ensure_aruco_detector_api(cv2.aruco)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def timestamps_paired(color_stamp, depth_stamp, max_delta_s=0.04):
    delta = float(depth_stamp) - float(color_stamp)
    return abs(delta) <= max_delta_s


def stamp_is_fresh(now_seconds, stamp_seconds_value, max_age_s=0.5):
    return 0 <= float(now_seconds) - float(stamp_seconds_value) <= float(max_age_s)


def camera_intrinsics(info, image_shape):
    """Validate CameraInfo against the image and return finite pinhole values."""
    height, width = image_shape
    if (int(info.width), int(info.height)) != (int(width), int(height)):
        raise ValueError("CameraInfo dimensions differ from image dimensions")
    k = np.asarray(info.k, dtype=float)
    distortion = np.asarray(info.d, dtype=float)
    if k.size != 9 or not np.isfinite(k).all() or not np.isfinite(distortion).all():
        raise ValueError("CameraInfo intrinsics and distortion must be finite")
    values = {"fx": float(k[0]), "fy": float(k[4]),
              "ppx": float(k[2]), "ppy": float(k[5])}
    if values["fx"] <= 0 or values["fy"] <= 0:
        raise ValueError("CameraInfo focal lengths must be positive")
    return values


def depth_scale_for_encoding(encoding, depth, configured_scale=0.001):
    """Return metres per sample for supported ROS depth encodings."""
    if encoding == "16UC1" and depth.dtype == np.uint16:
        scale = float(configured_scale)
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("16UC1 depth_scale_m must be positive and finite")
        return scale
    if encoding == "32FC1" and depth.dtype == np.float32:
        return 1.0
    raise ValueError(f"Depth encoding {encoding!r} does not match decoded array dtype")


def marker_grayscale(bridge, color_message):
    """Decode directly to mono8 so the detector avoids a full RGB array."""
    gray = bridge.imgmsg_to_cv2(color_message, desired_encoding="mono8")
    gray = np.asarray(gray)
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise ValueError("color bridge did not produce a mono8 image")
    return gray


def select_region_detection(detections, previous_centers=(), max_distance_px=60.0):
    """Fail closed on multiple candidates or a temporal jump outside one ROI."""
    if len(detections) != 1:
        return None
    detection = detections[0]
    center = np.mean(detection["corners_px"], axis=0)
    if previous_centers and min(np.linalg.norm(np.asarray(previous) - center)
                                for previous in previous_centers) > max_distance_px:
        return None
    return detection


def depth_center_point(center_px, depth, intrinsics, depth_scale_m):
    u, v = np.rint(center_px).astype(int)
    if not (1 <= u < depth.shape[1] - 1 and 1 <= v < depth.shape[0] - 1):
        return None
    patch = np.asarray(depth[v-1:v+2, u-1:u+2], dtype=float)
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size < 5:
        return None
    z = float(np.median(valid)) * float(depth_scale_m)
    point = ((float(u) - intrinsics["ppx"]) * z / intrinsics["fx"],
             (float(v) - intrinsics["ppy"]) * z / intrinsics["fy"], z)
    return point if np.isfinite(point).all() else None


def unassociated_tag_points(detections, depth, intrinsics, depth_scale_m):
    """Return only depth-backed ID0 points without claiming physical identity."""
    observations = []
    for detection in detections:
        if int(detection.get("code_id", -1)) != 0:
            continue
        corners = np.asarray(detection["corners_px"], dtype=float).reshape(4, 2)
        if not np.isfinite(corners).all():
            continue
        point = depth_center_point(corners.mean(axis=0), depth, intrinsics, depth_scale_m)
        if point is not None:
            observations.append({"tag_id": 0, "association": "unassociated", "xyz": point})
    return observations


class TagObservations(Node):
    def __init__(self, *, subscribe_images=True, use_global_arguments=True,
                 parameter_overrides=None):
        cv2.setNumThreads(1)  # Small 640x480 callbacks do not need a host-wide thread pool.
        from so101_station_perception_tag_tracking.tag_tracking import TagTracker

        super().__init__("so101_tag_observations", use_global_arguments=use_global_arguments,
                         parameter_overrides=parameter_overrides)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("diagnostic_frame", "")
        self.declare_parameter("display_hold_s", 1.5)
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("max_pair_delta_s", 0.04)
        self.declare_parameter("camera_info_max_age_s", 0.5)
        self.declare_parameter("max_processing_hz", 5.0)
        self.bridge = CvBridge()
        self.depth_scale = float(self.get_parameter("depth_scale_m").value)
        if not np.isfinite(self.depth_scale) or self.depth_scale <= 0:
            raise ValueError("depth_scale_m must be positive and finite")
        self.max_pair_delta = float(self.get_parameter("max_pair_delta_s").value)
        self.camera_info_max_age = float(self.get_parameter("camera_info_max_age_s").value)
        self.min_processing_period = 1.0 / min(5.0, max(0.1, float(
            self.get_parameter("max_processing_hz").value)))
        self.last_processing = 0.0
        self.diagnostic_frame = self.get_parameter("diagnostic_frame").value
        self.camera_info = None
        # This detector only returns code IDs and image corners. It does not
        # apply an unverified tag size or assign physical arm-link identities.
        self.detector = TagTracker(detector_scale=2)
        self.publisher = self.create_publisher(MarkerArray, "/so101_perception/tag_observations", 10)
        self.info_sub = self.color_sub = self.depth_sub = self.sync = None
        if subscribe_images:
            self.info_sub = self.create_subscription(
                CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_info,
                qos_profile_sensor_data)
            self.color_sub = message_filters.Subscriber(
                self, Image, self.get_parameter("color_topic").value,
                qos_profile=qos_profile_sensor_data)
            self.depth_sub = message_filters.Subscriber(
                self, Image, self.get_parameter("depth_topic").value,
                qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub], queue_size=5, slop=self.max_pair_delta)
            self.sync.registerCallback(self._images)

    def _camera_info(self, msg):
        self.camera_info = msg

    def _empty(self, stamp, frame_id):
        output = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.diagnostic_frame or frame_id
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        self.publisher.publish(output)

    def _images(self, color_msg, depth_msg, prepared_gray=None):
        now = self.get_clock().now().nanoseconds * 1e-9
        color_stamp = stamp_seconds(color_msg.header.stamp)
        if not stamp_is_fresh(now, color_stamp):
            return
        if prepared_gray is None and time.monotonic() - self.last_processing < self.min_processing_period:
            return
        self.last_processing = time.monotonic()
        delta = stamp_seconds(depth_msg.header.stamp) - stamp_seconds(color_msg.header.stamp)
        info = self.camera_info
        if (abs(delta) > self.max_pair_delta or info is None
                or color_msg.header.frame_id != depth_msg.header.frame_id
                or info.header.frame_id != color_msg.header.frame_id
                or abs(color_stamp - stamp_seconds(info.header.stamp)) > self.camera_info_max_age):
            self._empty(color_msg.header.stamp, color_msg.header.frame_id)
            return
        try:
            gray = (marker_grayscale(self.bridge, color_msg)
                    if prepared_gray is None else prepared_gray)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if gray.shape != depth.shape or depth.ndim != 2:
                raise ValueError("RGB and aligned depth dimensions differ")
            base_intrinsics = camera_intrinsics(info, depth.shape)
            depth_scale = depth_scale_for_encoding(depth_msg.encoding, depth, self.depth_scale)
            output = MarkerArray()
            clear = Marker()
            clear.header.stamp = color_msg.header.stamp
            clear.header.frame_id = self.diagnostic_frame or color_msg.header.frame_id
            clear.action = Marker.DELETEALL
            output.markers.append(clear)
            detections = self.detector.detect(gray)
            points = unassociated_tag_points(detections, depth, base_intrinsics, depth_scale)
            for marker_id, observation in enumerate(points):
                point = observation["xyz"]
                marker = Marker()
                marker.header.stamp = color_msg.header.stamp
                marker.header.frame_id = self.diagnostic_frame or color_msg.header.frame_id
                marker.ns = "apriltag_id0_unassociated"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.012
                marker.color.r, marker.color.g, marker.color.b = (0.2, 0.75, 0.95)
                marker.color.a = 0.95
                marker.lifetime = Duration(seconds=float(self.get_parameter("display_hold_s").value)).to_msg()
                output.markers.append(marker)
                text = Marker()
                text.header = marker.header
                text.ns = marker.ns + "_label"
                text.id = marker.id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x, text.pose.position.y, text.pose.position.z = point[0], point[1], point[2] + 0.025
                text.pose.orientation.w = 1.0
                text.scale.z = 0.018
                text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                text.text = "tag0-unassociated"
                text.lifetime = Duration(seconds=float(self.get_parameter("display_hold_s").value)).to_msg()
                output.markers.append(text)
            self.publisher.publish(output)
        except (ValueError, TypeError, IndexError, cv2.error) as error:
            self.get_logger().warning(f"Tag observation unavailable: {error}")
            self._empty(color_msg.header.stamp, color_msg.header.frame_id)


def main(args=None):
    rclpy.init(args=args)
    node = TagObservations()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
