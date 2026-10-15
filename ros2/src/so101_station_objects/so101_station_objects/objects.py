"""RGB-D colored-object observations and unqualified geometric grasp proposals."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def _repository_root():
    configured = os.environ.get("SO101_STATION_ROOT")
    candidates = ([Path(configured)] if configured else []) + [
        Path(__file__).resolve().parents[4], Path.cwd(),
    ]
    for candidate in candidates:
        if (candidate / "workshop/digital_twin/grasp_geometry.py").is_file() and (
                candidate / "tools/propose_rgbd_grasp.py").is_file():
            return candidate.resolve()
    raise RuntimeError("cannot locate workshop grasp geometry; set SO101_STATION_ROOT")


_ROOT = _repository_root()
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.propose_rgbd_grasp import color_masks  # noqa: E402
from workshop.digital_twin.grasp_geometry import fit_support_plane, footprint_grasps  # noqa: E402

COLOR_NAMES = ("red", "green", "blue", "yellow")
MAX_SUPPORT_POINTS = 2000


def camera_points(pixels, depth_m, intrinsics, distortion=None):
    """Back-project pixel centers with calibrated distortion and metric depth."""
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    depth_m = np.asarray(depth_m, dtype=np.float64).reshape(-1)
    if len(pixels) != len(depth_m) or not np.isfinite(pixels).all():
        raise ValueError("pixel and depth samples must be finite and aligned")
    valid = np.isfinite(depth_m) & (depth_m > 0)
    pixels, depth_m = pixels[valid], depth_m[valid]
    k = np.array([[intrinsics["fx"], 0, intrinsics["ppx"]],
                  [0, intrinsics["fy"], intrinsics["ppy"]], [0, 0, 1]], dtype=float)
    d = np.zeros(5) if distortion is None else np.asarray(distortion, dtype=float).reshape(-1)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("camera intrinsics and distortion must be finite with positive focal lengths")
    normalized = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    return np.column_stack((normalized * depth_m[:, None], depth_m))


def transform_points(points, target_from_source):
    points = np.asarray(points, dtype=np.float64)
    transform = np.asarray(target_from_source, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or transform.shape != (4, 4)
            or not np.isfinite(points).all() or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)):
        raise ValueError("expected finite Nx3 points and a homogeneous transform")
    return points @ transform[:3, :3].T + transform[:3, 3]


def _orient_plane_up(plane):
    normal = np.asarray(plane["normal"], dtype=float)
    offset = float(plane["d"])
    if normal[2] < 0:
        normal, offset = -normal, -offset
    if normal[2] < 0.85:
        raise ValueError("measured support plane is not tabletop-like in charuco_board")
    result = dict(plane, normal=normal, d=offset)
    # Board z=0 is only a prior after the observed plane independently agrees.
    if normal[2] >= math.cos(math.radians(5)) and abs(offset) <= 0.015:
        result.update(normal=np.array([0.0, 0.0, 1.0]), d=0.0,
                      plane_source="measured_table_consistent_with_board_z0")
    else:
        result["plane_source"] = "measured_table_plane"
    return result


def _fit_world_support(depth, valid_depth, rgb, intrinsics, distortion,
                       board_from_camera, world_roi, *, stride=8, max_points=MAX_SUPPORT_POINTS):
    height, width = depth.shape
    vivid = np.logical_or.reduce(tuple(color_masks(rgb).values()))
    excluded = cv2.dilate(vivid.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    yy, xx = np.mgrid[0:height:stride, 0:width:stride]
    sampled = valid_depth[yy, xx] & ~excluded[yy, xx]
    pixels = np.column_stack((xx[sampled], yy[sampled]))
    depths = depth[yy[sampled], xx[sampled]]
    camera = camera_points(pixels, depths, intrinsics, distortion)
    board = transform_points(camera, board_from_camera)
    bounds = world_roi
    inside = ((board[:, 0] >= bounds[0]) & (board[:, 0] <= bounds[1])
              & (board[:, 1] >= bounds[2]) & (board[:, 1] <= bounds[3])
              & (board[:, 2] >= bounds[4]) & (board[:, 2] <= bounds[5]))
    board = board[inside]
    if len(board) > max_points:
        indices = np.random.default_rng(17).choice(len(board), max_points, replace=False)
        board = board[indices]
    if len(board) < 30:
        raise ValueError("fewer than 30 observed table points inside bounded world ROI")
    plane = _orient_plane_up(fit_support_plane(board, threshold_m=0.003))
    inliers = int(plane["inlier_mask"].sum())
    if inliers < 30 or inliers / len(board) < 0.35:
        raise ValueError("no dominant measured support plane in table ROI")
    plane["support_points"] = len(board)
    plane["support_inliers"] = inliers
    return plane


def _plane_world_to_camera(plane, board_from_camera):
    rotation = board_from_camera[:3, :3]
    translation = board_from_camera[:3, 3]
    normal_world = np.asarray(plane["normal"], dtype=float)
    normal_camera = rotation.T @ normal_world
    d_camera = float(normal_world @ translation + plane["d"])
    return {"normal": normal_camera, "d": d_camera}


def _component_geometry(contour, pixel_points, depth_values, intrinsics, distortion,
                        board_from_camera, plane_world, color, max_opening_m):
    camera = camera_points(pixel_points, depth_values, intrinsics, distortion)
    board = transform_points(camera, board_from_camera)
    normal = np.asarray(plane_world["normal"])
    above = board @ normal + plane_world["d"]
    valid = np.isfinite(above) & (above >= 0.002) & (above <= 0.25)
    if int(valid.sum()) < 8:
        return None
    board = board[valid]
    above = above[valid]
    height_m = float(np.percentile(above, 90))
    if not 0.004 <= height_m <= 0.25:
        return None

    reference = np.array([1.0, 0.0, 0.0])
    axis_x = reference - normal * float(reference @ normal)
    if np.linalg.norm(axis_x) < 0.1:
        reference = np.array([0.0, 1.0, 0.0])
        axis_x = reference - normal * float(reference @ normal)
    axis_x /= np.linalg.norm(axis_x)
    axis_y = np.cross(normal, axis_x)
    tangent = np.column_stack((board @ axis_x, board @ axis_y))
    centered = tangent - np.median(tangent, axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    long_axis = vt[0, 0] * axis_x + vt[0, 1] * axis_y
    if long_axis @ axis_x < 0:
        long_axis = -long_axis
    short_axis = np.cross(normal, long_axis)
    coords_long, coords_short = board @ long_axis, board @ short_axis
    lo_long, hi_long = np.percentile(coords_long, [5, 95])
    lo_short, hi_short = np.percentile(coords_short, [5, 95])
    size_long, size_short = max(0.005, float(hi_long - lo_long)), max(0.005, float(hi_short - lo_short))
    center = (long_axis * ((lo_long + hi_long) / 2)
              + short_axis * ((lo_short + hi_short) / 2)
              - plane_world["d"] * normal + normal * height_m / 2)
    basis = np.column_stack((long_axis, short_axis, normal))
    plane_camera = _plane_world_to_camera(plane_world, board_from_camera)
    # footprint_grasps requires the silhouette plane to be strictly within its
    # estimated object height; keep a micron of numerical headroom.
    contour_height = max(0.0, height_m - 1e-6)
    candidates = footprint_grasps(contour, plane_camera, intrinsics, height_m,
                                  max_opening_m, contour_height_m=contour_height)
    grasps = []
    for candidate in candidates:
        closing = board_from_camera[:3, :3] @ np.asarray(candidate["closing_axis"])
        closing /= np.linalg.norm(closing)
        # Pick across the short footprint when both orientations fit.
        opening = float(candidate["opening_m"])
        center_camera = np.asarray(candidate["center"])
        grasp_world = board_from_camera[:3, :3] @ center_camera + board_from_camera[:3, 3]
        approach_world = -normal
        grasps.append({"position": grasp_world, "closing_axis": closing,
                       "approach_direction": approach_world, "opening_m": opening,
                       "contacts_camera": np.asarray(candidate["contacts"]),
                       "axis_rank": opening})
    if grasps:
        grasps = [min(grasps, key=lambda item: item["axis_rank"])]
    observed_heights = np.percentile(above, [50, 90, 95])
    return {
        "color": color,
        "position": center,
        "orientation_matrix": basis,
        "dimensions_m": np.array([size_long, size_short, height_m]),
        "dimension_uncertainty_m": np.array([
            max(0.001, float(np.std(coords_long))),
            max(0.001, float(np.std(coords_short))),
            max(0.001, float(observed_heights[2] - observed_heights[0])),
        ]),
        "height_m": height_m,
        "depth_points": int(valid.sum()),
        "contour": np.asarray(contour, dtype=np.float32),
        "grasps": grasps,
        "geometric_only": True,
        "ik_validated": False,
        "collision_checked": False,
    }


def analyze_rgbd(rgb, depth, intrinsics, distortion, depth_scale_m,
                 board_from_camera, world_roi=(-0.35, 0.35, -0.35, 0.35, -0.05, 0.35),
                 max_opening_m=0.055, min_component_pixels=30):
    """Produce measured tabletop objects and geometric-only grasp candidates."""
    rgb = np.asarray(rgb)
    depth = np.asarray(depth)
    transform = np.asarray(board_from_camera, dtype=float)
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8
            or depth.ndim != 2 or depth.shape != rgb.shape[:2]
            or depth.dtype not in (np.uint16, np.float32)
            or not np.isfinite(transform).all() or transform.shape != (4, 4)):
        raise ValueError("expected matching uint8 RGB, uint16/float32 aligned depth and finite transform")
    if not np.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError("depth scale must be positive and finite")
    depth_m = depth.astype(np.float32) * float(depth_scale_m)
    valid_depth = np.isfinite(depth_m) & (depth_m >= 0.10) & (depth_m <= 2.0)
    if int(valid_depth.sum()) < 100:
        raise ValueError("insufficient valid depth in camera frame")
    plane = _fit_world_support(depth_m, valid_depth, rgb, intrinsics, distortion,
                              transform, world_roi)
    masks = color_masks(rgb)
    objects = []
    for color in COLOR_NAMES:
        mask = masks[color].astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_component_pixels:
                continue
            component = (labels == label).astype(np.uint8)
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
            yy, xx = np.nonzero(labels == label)
            samples = depth_m[yy, xx]
            valid = valid_depth[yy, xx]
            if int(valid.sum()) < 8:
                continue
            item = _component_geometry(contour, np.column_stack((xx[valid], yy[valid])),
                                       samples[valid], intrinsics, distortion, transform,
                                       plane, color, max_opening_m)
            if item is not None:
                item["pixel_area"] = area
                objects.append(item)
    return {"support_plane": plane, "objects": objects,
            "qualified": False, "frame_id": "charuco_board"}


class StableObjectTracker:
    """Greedy color-constrained spatial association in the stable world frame."""
    def __init__(self, max_distance_m=0.06, max_age_s=1.0):
        self.max_distance = float(max_distance_m)
        self.max_age = float(max_age_s)
        self.next_id = 1
        self.tracks = {}

    def update(self, objects, stamp_s):
        stamp_s = float(stamp_s)
        self.tracks = {key: track for key, track in self.tracks.items()
                       if 0 <= stamp_s - track["stamp"] <= self.max_age}
        pairs = []
        for obj_index, obj in enumerate(objects):
            position = np.asarray(obj["position"], dtype=float)
            for track_id, track in self.tracks.items():
                if obj["color"] == track["color"]:
                    distance = float(np.linalg.norm(position - track["position"]))
                    if distance <= self.max_distance:
                        pairs.append((distance, obj_index, track_id))
        assigned_objects, assigned_tracks = set(), set()
        assignments = {}
        for _, obj_index, track_id in sorted(pairs):
            if obj_index not in assigned_objects and track_id not in assigned_tracks:
                assignments[obj_index] = track_id
                assigned_objects.add(obj_index)
                assigned_tracks.add(track_id)
        output = []
        for index, obj in enumerate(objects):
            position = np.asarray(obj["position"], dtype=float)
            track_id = assignments.get(index)
            previous = self.tracks.get(track_id) if track_id is not None else None
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
                velocity = np.zeros(3)
                first_stamp = stamp_s
            else:
                dt = stamp_s - previous["stamp"]
                velocity = ((position - previous["position"]) / dt
                            if dt > 1e-6 else previous["velocity"])
                first_stamp = previous["first_stamp"]
            record = dict(obj, track_id=track_id, velocity=velocity,
                          age_s=max(0.0, stamp_s - first_stamp))
            self.tracks[track_id] = {"color": obj["color"], "position": position,
                                     "velocity": velocity, "stamp": stamp_s,
                                     "first_stamp": first_stamp}
            output.append(record)
        return output


def matrix_quaternion(rotation):
    """Convert a right-handed 3x3 rotation matrix to xyzw."""
    m = np.asarray(rotation, dtype=float)
    if m.shape != (3, 3) or not np.isfinite(m).all():
        raise ValueError("rotation must be finite 3x3")
    if not np.allclose(m.T @ m, np.eye(3), atol=1e-5) or np.linalg.det(m) < 0.999:
        raise ValueError("rotation must be proper and orthonormal")
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [ (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
              (m[1, 0] - m[0, 1]) / s, 0.25 * s ]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
        vector = [0.0, 0.0, 0.0]
        vector[i] = 0.25 * s
        vector[j] = (m[j, i] + m[i, j]) / s
        vector[k] = (m[k, i] + m[i, k]) / s
        q = [*vector, (m[k, j] - m[j, k]) / s]
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def grasp_pose_rotation(closing_axis, approach_direction):
    x_axis = np.asarray(closing_axis, dtype=float)
    z_axis = np.asarray(approach_direction, dtype=float)
    z_axis /= np.linalg.norm(z_axis)
    x_axis -= z_axis * float(x_axis @ z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def transform_from_msg(message):
    t = message.transform
    q = np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w], dtype=float)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("invalid TF quaternion")
    x, y, z, w = q / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return matrix


class ColoredObjectsNode:
    """ROS adapter; imports ROS only at instantiation for offline helper tests."""
    def __new__(cls, *args, **kwargs):
        import rclpy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import Pose, PoseArray
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformException, TransformListener
        from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
        from visualization_msgs.msg import Marker, MarkerArray
        import message_filters

        class _Node(Node):
            def __init__(self):
                cv2.setNumThreads(1)
                super().__init__("so101_colored_objects")
                defaults = {
                    "color_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "board_frame": "charuco_board",
                    "depth_scale_m": 0.001,
                    "world_roi_x_min": -0.35, "world_roi_x_max": 0.35,
                    "world_roi_y_min": -0.35, "world_roi_y_max": 0.35,
                    "world_roi_z_min": -0.05, "world_roi_z_max": 0.35,
                    "max_gripper_opening_m": 0.055,
                    "max_processing_hz": 5.0,
                }
                for key, value in defaults.items():
                    self.declare_parameter(key, value)
                self.board_frame = str(self.get_parameter("board_frame").value)
                self.depth_scale = float(self.get_parameter("depth_scale_m").value)
                self.roi = tuple(float(self.get_parameter(name).value) for name in (
                    "world_roi_x_min", "world_roi_x_max", "world_roi_y_min",
                    "world_roi_y_max", "world_roi_z_min", "world_roi_z_max"))
                self.max_opening = float(self.get_parameter("max_gripper_opening_m").value)
                requested_hz = float(self.get_parameter("max_processing_hz").value)
                if not np.isfinite([self.depth_scale, self.max_opening, requested_hz, *self.roi]).all():
                    raise ValueError("object node parameters must be finite")
                if self.depth_scale <= 0 or self.max_opening <= 0 or requested_hz <= 0:
                    raise ValueError("depth scale, opening, and processing frequency must be positive")
                if any(self.roi[i] >= self.roi[i+1] for i in (0, 2, 4)):
                    raise ValueError("world ROI minima must be below maxima")
                self.period = 1.0 / min(5.0, requested_hz)
                self.last_process = 0.0
                self.bridge = CvBridge()
                self.tracker = StableObjectTracker()
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.detections_pub = self.create_publisher(Detection3DArray, "/so101/objects", 10)
                self.markers_pub = self.create_publisher(MarkerArray, "/so101/object_markers", 10)
                self.grasps_pub = self.create_publisher(PoseArray, "/so101/grasp_candidates", 10)
                self.status_pub = self.create_publisher(String, "/so101/grasp_status", 10)
                color = message_filters.Subscriber(
                    self, Image, self.get_parameter("color_topic").value,
                    qos_profile=qos_profile_sensor_data)
                depth = message_filters.Subscriber(
                    self, Image, self.get_parameter("depth_topic").value,
                    qos_profile=qos_profile_sensor_data)
                info = message_filters.Subscriber(
                    self, CameraInfo, self.get_parameter("camera_info_topic").value,
                    qos_profile=qos_profile_sensor_data)
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    [color, depth, info], queue_size=6, slop=0.04)
                self.sync.registerCallback(self._process)

            def _publish_empty(self, stamp):
                array = Detection3DArray()
                array.header.stamp, array.header.frame_id = stamp, self.board_frame
                self.detections_pub.publish(array)
                poses = PoseArray()
                poses.header = array.header
                self.grasps_pub.publish(poses)
                markers = MarkerArray()
                clear = Marker()
                clear.header = array.header
                clear.action = Marker.DELETEALL
                markers.markers.append(clear)
                self.markers_pub.publish(markers)

            def _process(self, color_msg, depth_msg, info_msg):
                from so101_station_perception.tag_observations import (
                    camera_intrinsics, depth_scale_for_encoding, stamp_is_fresh, stamp_seconds)
                now = self.get_clock().now().nanoseconds * 1e-9
                stamp = stamp_seconds(color_msg.header.stamp)
                if not stamp_is_fresh(now, stamp, 0.5):
                    self._publish_empty(color_msg.header.stamp)
                    return
                if time.monotonic() - self.last_process < self.period:
                    return
                self.last_process = time.monotonic()
                if (color_msg.header.frame_id != depth_msg.header.frame_id
                        or color_msg.header.frame_id != info_msg.header.frame_id
                        or abs(stamp - stamp_seconds(depth_msg.header.stamp)) > 0.04
                        or abs(stamp - stamp_seconds(info_msg.header.stamp)) > 0.04):
                    self._publish_empty(color_msg.header.stamp)
                    return
                try:
                    rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
                    depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                    intrinsics = camera_intrinsics(info_msg, depth.shape)
                    scale = depth_scale_for_encoding(depth_msg.encoding, depth, self.depth_scale)
                    tf = self.tf_buffer.lookup_transform(
                        self.board_frame, color_msg.header.frame_id,
                        Time.from_msg(color_msg.header.stamp), timeout=Duration(seconds=0.08))
                    board_from_camera = transform_from_msg(tf)
                    report = analyze_rgbd(rgb, depth, intrinsics, info_msg.d, scale,
                                          board_from_camera, self.roi, self.max_opening)
                    objects = self.tracker.update(report["objects"], stamp)
                    report["objects"] = objects
                    report["stamp"] = stamp
                    report["support_plane"] = {
                        key: (value.tolist() if isinstance(value, np.ndarray) else value)
                        for key, value in report["support_plane"].items()
                        if key != "inlier_mask" and key != "residuals_m"}
                    self._publish_report(report, color_msg.header.stamp)
                except (ValueError, TypeError, cv2.error, TransformException) as error:
                    self.get_logger().warning(f"Colored object observation unavailable: {error}")
                    self._publish_empty(color_msg.header.stamp)

            def _publish_report(self, report, stamp):
                detections = Detection3DArray()
                detections.header.stamp, detections.header.frame_id = stamp, self.board_frame
                markers = MarkerArray()
                clear = Marker()
                clear.header = detections.header
                clear.action = Marker.DELETEALL
                markers.markers.append(clear)
                grasps = PoseArray()
                grasps.header = detections.header
                grasp_records = []
                for index, obj in enumerate(report["objects"]):
                    rotation = obj["orientation_matrix"]
                    quaternion = matrix_quaternion(rotation)
                    position = obj["position"]
                    dimensions = obj["dimensions_m"]
                    det = Detection3D()
                    det.header = detections.header
                    det.id = f"{obj['color']}-{obj['track_id']:04d}"
                    det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = map(float, position)
                    det.bbox.center.orientation.x, det.bbox.center.orientation.y = map(float, quaternion[:2])
                    det.bbox.center.orientation.z, det.bbox.center.orientation.w = map(float, quaternion[2:])
                    det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = map(float, dimensions)
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = obj["color"]
                    # This is a geometric color classification, not a calibrated
                    # detector probability. Leave the message default score.
                    hypothesis.pose.pose = det.bbox.center
                    det.results.append(hypothesis)
                    detections.detections.append(det)
                    cube = Marker()
                    cube.header = detections.header
                    cube.ns, cube.id, cube.type, cube.action = "colored_objects", index * 2, Marker.CUBE, Marker.ADD
                    cube.pose = det.bbox.center
                    cube.scale.x, cube.scale.y, cube.scale.z = map(float, dimensions)
                    colors = {"red": (1.0, .1, .1), "green": (.1, 1.0, .1),
                              "blue": (.1, .3, 1.0), "yellow": (1.0, .8, .05)}
                    cube.color.r, cube.color.g, cube.color.b = colors[obj["color"]]
                    cube.color.a = .35
                    markers.markers.append(cube)
                    label = Marker()
                    label.header = detections.header
                    label.ns, label.id, label.type, label.action = "colored_object_labels", index * 2 + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
                    label.pose = det.bbox.center
                    label.pose.position.z += float(dimensions[2]) / 2 + .015
                    label.scale.z = .015
                    label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                    label.text = f"{obj['color']} {det.id}"
                    markers.markers.append(label)
                    for grasp_index, grasp in enumerate(obj["grasps"]):
                        rotation_grasp = grasp_pose_rotation(grasp["closing_axis"], grasp["approach_direction"])
                        q_grasp = matrix_quaternion(rotation_grasp)
                        pregrasp = Pose()
                        preposition = grasp["position"] - grasp["approach_direction"] * .06
                        pregrasp.position.x, pregrasp.position.y, pregrasp.position.z = map(float, preposition)
                        pregrasp.orientation.x, pregrasp.orientation.y = map(float, q_grasp[:2])
                        pregrasp.orientation.z, pregrasp.orientation.w = map(float, q_grasp[2:])
                        grasps.poses.append(pregrasp)
                        grasp_records.append({
                            "candidate_id": f"{det.id}-g{grasp_index}", "track_id": obj["track_id"],
                            "color": obj["color"], "frame_id": self.board_frame,
                            "geometric_only": True, "ik_validated": False, "collision_checked": False,
                            "closing_axis_world": grasp["closing_axis"].tolist(),
                            "approach_direction_world": grasp["approach_direction"].tolist(),
                            "opening_m": grasp["opening_m"],
                            "contact_position_world": grasp["position"].tolist(),
                            "pregrasp_position_world": preposition.tolist(),
                            "pregrasp_offset_m": .06,
                        })
                self.detections_pub.publish(detections)
                self.markers_pub.publish(markers)
                self.grasps_pub.publish(grasps)
                status = {
                    "schema": 1, "state": "tracking", "frame_id": self.board_frame,
                    "stamp": report["stamp"], "qualified": False,
                    "support_plane": report["support_plane"],
                    "objects": [{
                        "track_id": obj["track_id"], "color": obj["color"],
                        "position_world": obj["position"].tolist(),
                        "velocity_world_m_s": obj["velocity"].tolist(), "age_s": obj["age_s"],
                        "dimensions_observed_m": obj["dimensions_m"].tolist(),
                        "dimension_uncertainty_m": obj["dimension_uncertainty_m"].tolist(),
                        "dimension_source": "robust visible RGB-D point extent; not CAD or full-object measurement",
                        "depth_points": obj["depth_points"], "geometric_only": True,
                        "ik_validated": False, "collision_checked": False,
                    } for obj in report["objects"]],
                    "grasp_candidates": grasp_records,
                }
                message = String()
                message.data = json.dumps(status, sort_keys=True, allow_nan=False)
                self.status_pub.publish(message)

        return _Node(*args, **kwargs)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = ColoredObjectsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
