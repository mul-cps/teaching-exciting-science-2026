"""Publish depth-backed AprilTag center observations without adding TF frames."""

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
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
    return ((float(u) - intrinsics["ppx"]) * z / intrinsics["fx"],
            (float(v) - intrinsics["ppy"]) * z / intrinsics["fy"], z)


class TagObservations(Node):
    # Source-space image ROIs keep the two repeated ID-0 codes distinct.
    DEFAULT_REGIONS = {
        "wrist_mount": (330, 190, 460, 320),
        "upperarm_mount": (345, 35, 465, 155),
    }

    def __init__(self):
        from so101_station_perception_tag_tracking.tag_tracking import TagTracker

        super().__init__("so101_tag_observations")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("diagnostic_frame", "")
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("max_pair_delta_s", 0.04)
        self.declare_parameter("camera_info_max_age_s", 0.5)
        self.bridge = CvBridge()
        self.depth_scale = float(self.get_parameter("depth_scale_m").value)
        self.max_pair_delta = float(self.get_parameter("max_pair_delta_s").value)
        self.camera_info_max_age = float(self.get_parameter("camera_info_max_age_s").value)
        self.diagnostic_frame = self.get_parameter("diagnostic_frame").value
        self.camera_info = None
        self.trackers = {
            name: TagTracker(max_distance_px=60.0, max_missed=3, detector_scale=2,
                             size_candidates=(0.0194,))
            for name in self.DEFAULT_REGIONS
        }
        self.publisher = self.create_publisher(MarkerArray, "/so101_perception/tag_observations", 10)
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_info, 5)
        self.color_sub = message_filters.Subscriber(
            self, Image, self.get_parameter("color_topic").value)
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter("depth_topic").value)
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

    def _images(self, color_msg, depth_msg):
        delta = stamp_seconds(depth_msg.header.stamp) - stamp_seconds(color_msg.header.stamp)
        info = self.camera_info
        if (abs(delta) > self.max_pair_delta or info is None
                or abs(stamp_seconds(color_msg.header.stamp) - stamp_seconds(info.header.stamp))
                > self.camera_info_max_age):
            self._empty(color_msg.header.stamp, color_msg.header.frame_id)
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if rgb.shape[:2] != depth.shape[:2] or depth.ndim != 2:
                raise ValueError("RGB and aligned depth dimensions differ")
            k = info.k
            base_intrinsics = {"fx": float(k[0]), "fy": float(k[4]),
                               "ppx": float(k[2]), "ppy": float(k[5])}
            if min(base_intrinsics["fx"], base_intrinsics["fy"]) <= 0:
                raise ValueError("camera_info has invalid pinhole intrinsics")
            height, width = depth.shape
            scale_x, scale_y = width / 640.0, height / 480.0
            output = MarkerArray()
            clear = Marker()
            clear.header.stamp = color_msg.header.stamp
            clear.header.frame_id = self.diagnostic_frame or color_msg.header.frame_id
            clear.action = Marker.DELETEALL
            output.markers.append(clear)
            marker_id = 1
            for region_index, (name, bounds) in enumerate(self.DEFAULT_REGIONS.items()):
                x0, y0, x1, y1 = [int(round(v * (scale_x if i % 2 == 0 else scale_y)))
                                  for i, v in enumerate(bounds)]
                crop = rgb[max(y0, 0):min(y1, height), max(x0, 0):min(x1, width)]
                if crop.size == 0:
                    continue
                detections = [d for d in self.trackers[name].detect(crop) if d["code_id"] == 0]
                tracker = self.trackers[name]
                active = [state for state in tracker.tracks.values() if state["missed"] <= 3]
                selected = select_region_detection(
                    detections, [state["center"] for state in active])
                if selected is None:
                    # An ROI with multiple same-code candidates is ambiguous;
                    # a distant return cannot steal another region's identity.
                    tracker.update([], base_intrinsics, depth=None)
                    continue
                detection = selected
                center = np.mean(detection["corners_px"], axis=0) + [x0, y0]
                crop_depth = depth[y0:y1, x0:x1]
                crop_intrinsics = dict(base_intrinsics)
                crop_intrinsics["ppx"] -= x0
                crop_intrinsics["ppy"] -= y0
                observations = tracker.update([detection], crop_intrinsics, depth=crop_depth,
                                              depth_scale_m=self.depth_scale,
                                              distortion=np.asarray(info.d))
                point = depth_center_point(center, depth, base_intrinsics, self.depth_scale)
                if not observations or point is None:
                    continue
                marker = Marker()
                marker.header.stamp = color_msg.header.stamp
                marker.header.frame_id = self.diagnostic_frame or color_msg.header.frame_id
                marker.ns = f"{name}_id0_unqualified"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.012
                marker.color.r, marker.color.g, marker.color.b = ((0.15, 0.85, 0.3)
                    if region_index == 0 else (0.2, 0.55, 0.95))
                marker.color.a = 0.95
                marker.lifetime = Duration(seconds=0.4).to_msg()
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
                text.text = f"{name} · tag 0 · diagnostic"
                text.lifetime = Duration(seconds=0.4).to_msg()
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
