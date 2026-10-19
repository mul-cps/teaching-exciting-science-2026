"""Read-only ROS 2 adapter that snapshots the inputs also consumed by RViz."""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import os
import tempfile
import time
from pathlib import Path
import uuid

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
import message_filters
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from .ros_state import RosTwinState, resolve_package_mesh_uris, transform_matrix
from .point_cloud_samples import sampled_cloud, encode_rgb8
from .placement_region import (PlacementRegionTracker, SparseRGBDAnchor,
    pose_from_rectangle, project_rectangle_to_plane, placement_camera_frame_summary,
    waiting_for_anchor_status)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RosStatePublisher(Node):
    """Serialize authoritative state to an atomic local file; never publishes ROS."""

    def __init__(self, output, root_frame="charuco_board", max_age_s=0.5,
                 placement_directory=None, color_topic="/camera/color/image_raw",
                 depth_topic="/camera/aligned_depth_to_color/image_raw",
                 camera_info_topic="/camera/color/camera_info",
                 placement_selection_timeout_s=30.):
        super().__init__("so101_web_twin_state")
        self.output = output
        self.root_frame = root_frame.lstrip("/")
        self.state = RosTwinState(max_age_s=float(max_age_s))
        cv2.setNumThreads(1)
        self.cv_bridge = CvBridge()
        self.placement_directory = Path(placement_directory or
                                        (Path(output).parent / "placement-camera"))
        self.placement_directory.mkdir(parents=True, exist_ok=True)
        self.placement_request_path = Path(output).with_name("placement-region-request.json")
        self.placement_frames = deque(maxlen=16)
        # Human selection can take several seconds. Keep only the already-encoded
        # color JPEGs needed to prove adjacent LK continuity; depth is never
        # historied and remains bound to the latest synchronized RGB-D frame.
        self.placement_gray_history = deque(maxlen=500)
        self.placement_history_seconds = 30.
        self.placement_replay_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="placement-history")
        self.placement_replay_future = None
        self.placement_replay_target = None
        self.placement_replay_candidate = None
        self.placement_replay_base = None
        self.placement_replay_active = False
        self.placement_selection_timeout_s = float(placement_selection_timeout_s)
        if not np.isfinite(self.placement_selection_timeout_s) or self.placement_selection_timeout_s <= 0:
            raise ValueError("placement selection timeout must be positive and finite")
        self.placement_tracker = PlacementRegionTracker(
            max_selection_age_s=self.placement_selection_timeout_s)
        self.placement_frontend = SparseRGBDAnchor(self.placement_tracker)
        self.placement_last_request_id = None
        self.placement_pinned_frame = None
        self.placement_pin_expires = None
        self.placement_selection_request_id = None
        self.placement_selection_mode = None
        self.placement_selection_failure_reason = None
        self.placement_status = {"state": "waiting_for_rgbd", "valid": False,
            "reason": "waiting_for_rgbd", "placement_motion_qualified": False}
        self.placement_frame = None
        self.placement_plane_metrics = None
        self.placement_last_tf_warning = 0.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latched_tf = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/follower/joint_states", self._joints, 10)
        self.create_subscription(String, "/follower/robot_description", self._description, latched)
        self.create_subscription(TFMessage, "/tf", self._dynamic_tf, 100)
        self.create_subscription(TFMessage, "/tf_static", self._static_tf, latched_tf)
        self.create_subscription(MarkerArray, "/so101/object_markers", self._markers, 10)
        self.create_subscription(PoseArray, "/so101/grasp_contacts", self._contacts, 10)
        self.create_subscription(PoseArray, "/so101/grasp_candidates", self._candidates, 10)
        self.create_subscription(PointCloud2, "/camera/depth/color/points", self._point_cloud, 2)
        self.placement_pub = self.create_publisher(String,
            "/station/placement_goal_observation", 10)
        self.placement_selection_service = None
        try:
            from so101_station_workflow.srv import SelectPlacementRegion
            self.placement_selection_service = self.create_service(
                SelectPlacementRegion, "/station/placement/select",
                self._placement_selection_service)
        except ImportError as error:
            self.get_logger().warning(
                f"Placement ROS selection service unavailable until so101_station_workflow is built: {error}")
        color = message_filters.Subscriber(self, Image, color_topic,
                                           qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(self, Image, depth_topic,
                                           qos_profile=qos_profile_sensor_data)
        info = message_filters.Subscriber(self, CameraInfo, camera_info_topic,
                                          qos_profile=qos_profile_sensor_data)
        self.camera_sync = message_filters.ApproximateTimeSynchronizer(
            [color, depth, info], queue_size=6, slop=0.04)
        self.camera_sync.registerCallback(self._placement_rgbd)
        self.create_timer(0.05, self._write)
        self.get_logger().info(
            "Passive ROS twin adapter: subscribing only; no camera, serial, or command interface")

    def _lookup_camera_pose(self, target, source, stamp_ns):
        stamp = Time(nanoseconds=int(stamp_ns), clock_type=self.get_clock().clock_type)
        try:
            if not self.tf_buffer.can_transform(target, source, stamp, timeout=Duration(seconds=0.0)):
                return None
            value = self.tf_buffer.lookup_transform(target, source, stamp,
                                                    timeout=Duration(seconds=0.0))
        except Exception:
            return None
        t, q = value.transform.translation, value.transform.rotation
        return transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def _placement_rgbd(self, color_msg, depth_msg, info_msg):
        now = time.monotonic()
        color_stamp, depth_stamp, info_stamp = map(_stamp_ns,
            (color_msg.header.stamp, depth_msg.header.stamp, info_msg.header.stamp))
        stamp_ns = color_stamp
        if (stamp_ns <= 0 or max(color_stamp, depth_stamp, info_stamp)
                - min(color_stamp, depth_stamp, info_stamp) > 40_000_000):
            self.placement_status = {"state": "invalid", "valid": False,
                "reason": "rgbd_camera_info_timestamp_mismatch",
                "stamp_ns": stamp_ns, "placement_motion_qualified": False}
            return
        frame_id = color_msg.header.frame_id.lstrip("/")
        if not frame_id or depth_msg.header.frame_id.lstrip("/") not in ("", frame_id):
            self.placement_status = {"state": "invalid", "valid": False,
                "reason": "rgbd_frame_mismatch", "stamp_ns": stamp_ns,
                "placement_motion_qualified": False}
            return
        try:
            rgb = self.cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
            depth_raw = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            rgb = np.asarray(rgb, dtype=np.uint8)
            depth_raw = np.asarray(depth_raw)
            if rgb.ndim != 3 or rgb.shape[2] != 3 or depth_raw.shape != rgb.shape[:2]:
                raise ValueError("color and aligned depth must have matching HxW")
            k = np.asarray(info_msg.k, dtype=float).reshape(3, 3)
            distortion = np.asarray(info_msg.d, dtype=float)
            h, w = rgb.shape[:2]
            if (info_msg.width != w or info_msg.height != h or
                    not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0):
                raise ValueError("camera intrinsics must match RGB dimensions")
            if len(distortion) and np.any(np.abs(distortion) > 1e-12):
                new_k, _ = cv2.getOptimalNewCameraMatrix(k, distortion, (w, h), 0., (w, h))
                map_x, map_y = cv2.initUndistortRectifyMap(k, distortion, None, new_k,
                                                           (w, h), cv2.CV_32FC1)
                rgb = cv2.remap(rgb, map_x, map_y, cv2.INTER_LINEAR)
                depth_raw = cv2.remap(depth_raw, map_x, map_y, cv2.INTER_NEAREST)
                k = new_k
            if depth_raw.dtype == np.uint16:
                depth_m = depth_raw.astype(np.float32) * .001
            elif np.issubdtype(depth_raw.dtype, np.floating):
                depth_m = depth_raw.astype(np.float32)
            else:
                raise ValueError(f"unsupported depth encoding {depth_msg.encoding}")
            root_from_camera = self._lookup_camera_pose(self.root_frame, frame_id, stamp_ns)
            base_from_camera = self._lookup_camera_pose("base_link", frame_id, stamp_ns)
            encoded_ok, encoded = cv2.imencode(".jpg",
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not encoded_ok:
                raise ValueError("could not encode synchronized camera image")
            encoded_bytes = encoded.tobytes()
            generation = f"{stamp_ns}-{uuid.uuid4().hex[:8]}"
            image_path = self.placement_directory / f"{generation}.jpg"
            temporary = image_path.with_suffix(".tmp.jpg")
            temporary.write_bytes(encoded_bytes)
            os.replace(temporary, image_path)
            record = {"generation": generation, "stamp_ns": stamp_ns,
                "received_monotonic": now, "frame_id": frame_id,
                "root_frame": self.root_frame,
                "root_from_camera": (None if root_from_camera is None else root_from_camera),
                "base_from_camera": base_from_camera,
                "camera_matrix": k, "rgb": rgb.copy(), "depth_m": depth_m.copy(),
                "image_path": str(image_path), "image_shape": [h, w]}
            self.placement_frames.append(record)
            if (self.placement_pinned_frame is not None
                    and self.placement_pin_expires is not None
                    and now > self.placement_pin_expires):
                self.placement_pinned_frame = None
                self.placement_pin_expires = None
                self.placement_selection_request_id = None
                self.placement_selection_mode = None
                self.placement_gray_history.clear()
            if self.placement_pinned_frame is not None or self.placement_replay_active:
                self._append_placement_gray_frame(record, rgb)
            self.placement_frame = placement_camera_frame_summary(record)
            if (self.placement_replay_active
                    and self.placement_replay_candidate is not None):
                # The isolated worker owns its candidate frontend. Keep receiving
                # synchronized RGB-D and extending the bounded gray tail; the main
                # thread will replay that tail and publish only a fresh result.
                pass
            elif self.placement_frontend.tracker.anchor_points is not None:
                observation = self.placement_frontend.update(rgb, depth_m, k,
                    np.eye(4), stamp_ns=stamp_ns, frame_id=frame_id,
                    generation=generation, received_monotonic=now)
                self.placement_status = self._placement_snapshot_dict(observation)
                self.placement_status["received_monotonic"] = now
                self.placement_status["camera_frame"] = frame_id
                self.placement_status["identity_reacquired_after_gap"] = (
                    self.placement_frontend.identity_reacquired_after_gap)
                self.placement_status["plane_metrics"] = self.placement_plane_metrics
                self.placement_status["camera_from_goal"] = None
                if observation.goal_corners_root is not None:
                    camera_from_goal, geometry = pose_from_rectangle(observation.goal_corners_root)
                    self.placement_status["camera_from_goal"] = camera_from_goal.tolist()
                    self.placement_status["camera_from_goal_stamp_ns"] = stamp_ns
                    self.placement_status["selection_id"] = observation.selection_id
                    self.placement_status["goal_corners_camera"] = observation.goal_corners_root.tolist()
                    self.placement_status["goal_geometry"] = geometry
                    self.placement_status["root_from_camera_at_stamp"] = (
                        None if root_from_camera is None else root_from_camera.tolist())
                    self.placement_status["goal_corners_root"] = (
                        None if root_from_camera is None else
                        (observation.goal_corners_root @ root_from_camera[:3, :3].T
                         + root_from_camera[:3, 3]).tolist())
                self.placement_status["base_tf_available"] = base_from_camera is not None
                self.placement_status["base_from_camera_at_stamp"] = (
                    None if base_from_camera is None else base_from_camera.tolist())
                if base_from_camera is not None and observation.goal_corners_root is not None:
                    self.placement_status["goal_corners_base"] = (
                        (observation.goal_corners_root @ base_from_camera[:3, :3].T
                         + base_from_camera[:3, 3]).tolist())
            else:
                if self.placement_selection_failure_reason is not None:
                    self.placement_status.update({"state": "invalid", "valid": False,
                        "anchor_tracking_valid": False,
                        "reason": self.placement_selection_failure_reason,
                        "generation": generation, "stamp_ns": stamp_ns,
                        "frame_id": frame_id})
                else:
                    self.placement_status = waiting_for_anchor_status(
                        self.placement_status, generation=generation, stamp_ns=stamp_ns,
                        frame_id=frame_id)
            self.placement_status.update({
                "camera_frame": frame_id, "stamp_ns": stamp_ns,
                "received_monotonic": now, "root_frame": self.root_frame,
                "root_from_camera_at_stamp": (None if root_from_camera is None else
                                               root_from_camera.tolist()),
                "base_from_camera_at_stamp": (None if base_from_camera is None else
                                              base_from_camera.tolist()),
                "camera_base_registration_qualified": False,
                "physical_placement_admission": False,
                "placement_motion_qualified": False,
            })
            keep = {entry["image_path"] for entry in self.placement_frames}
            for old in self.placement_directory.glob("*.jpg"):
                if str(old) not in keep:
                    old.unlink(missing_ok=True)
        except Exception as error:
            self.placement_tracker.invalidate("camera_observation_invalid", generation=generation,
                stamp_ns=stamp_ns, received_monotonic=now)
            self.placement_status = {"state": "invalid", "valid": False,
                "reason": f"camera_observation_invalid:{error}", "stamp_ns": stamp_ns,
                "placement_motion_qualified": False}

    @staticmethod
    def _placement_snapshot_dict(snapshot):
        anchor_tracking_valid = bool(snapshot.reason == "tracked"
                                      and snapshot.root_from_anchor is not None
                                      and snapshot.age_s is not None
                                      and snapshot.age_s <= .25)
        state = ("tracked" if snapshot.valid else
                 "anchor_tracked" if anchor_tracking_valid else "invalid")
        return {"state": state, "anchor_tracking_valid": anchor_tracking_valid,
            "valid": snapshot.valid, "reason": snapshot.reason,
            "generation": snapshot.generation, "stamp_ns": snapshot.stamp_ns,
            "selection_id": snapshot.selection_id,
            "frame_id": snapshot.frame_id, "age_s": snapshot.age_s,
            "camera_from_anchor": (None if snapshot.root_from_anchor is None else
                                    snapshot.root_from_anchor.tolist()),
            "goal_corners_camera": (None if snapshot.goal_corners_root is None else
                                    snapshot.goal_corners_root.tolist()),
            "last_valid_goal_corners_camera": (
                None if snapshot.last_valid_goal_corners_root is None else
                snapshot.last_valid_goal_corners_root.tolist()),
            "feature_count": snapshot.feature_count, "inlier_count": snapshot.inlier_count,
            "fit_rms_m": snapshot.fit_rms_m,
            "placement_motion_qualified": False,
            "physical_placement_admission": False,
            "camera_from_anchor_is_measurement": True,
            "camera_base_registration_qualified": False}

    def _consume_placement_request(self):
        try:
            request = json.loads(self.placement_request_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        request_id = request.get("request_id") if isinstance(request, dict) else None
        if not request_id or request_id == self.placement_last_request_id:
            return
        self.placement_last_request_id = request_id
        self._process_placement_request(request)

    def _process_placement_request(self, request):
        if request.get("action") == "cancel_pin":
            pin_request_id = str(request.get("pin_request_id", ""))
            if pin_request_id and pin_request_id == self.placement_selection_request_id:
                self.placement_pinned_frame = None
                self.placement_pin_expires = None
                self.placement_selection_request_id = None
                self.placement_selection_mode = None
                self.placement_gray_history.clear()
            self.placement_status = {"state": "selection_cancelled", "valid": False,
                "reason": "selection_cancelled", "placement_motion_qualified": False}
            return
        if request.get("action") == "pin_latest_frame":
            mode = str(request.get("mode", ""))
            request_id = str(request.get("request_id", ""))
            if self.placement_replay_active:
                self.placement_status = {"state": "invalid", "valid": False,
                    "reason": "selection_replay_in_progress",
                    "placement_motion_qualified": False}
                return
            if mode not in {"select_feeder_anchor", "select_placement_area"} or not request_id:
                self.placement_status = {"state": "invalid", "valid": False,
                    "reason": "selection_pin_request_invalid",
                    "placement_motion_qualified": False}
                return
            if (mode == "select_placement_area"
                    and not (self.placement_status.get("anchor_tracking_valid")
                             or self.placement_status.get("valid"))):
                self.placement_status = {"state": "invalid", "valid": False,
                    "reason": "no_feeder_anchor_selected",
                    "placement_motion_qualified": False}
                return
            now = time.monotonic()
            fresh_frames = [item for item in self.placement_frames
                if 0. <= now - float(item.get("received_monotonic", 0.)) <= .5]
            if not fresh_frames:
                self.placement_status = {"state": "invalid", "valid": False,
                    "reason": "no_fresh_rgbd_frame_to_pin",
                    "placement_motion_qualified": False}
                return
            frame = max(fresh_frames, key=lambda item: int(item["stamp_ns"]))
            if mode == "select_placement_area":
                pose_record = self.placement_tracker._history.get(frame["generation"])
                if pose_record is None or int(pose_record[1]) != int(frame["stamp_ns"]):
                    self.placement_status = {"state": "invalid", "valid": False,
                        "reason": "anchor_pose_unavailable_at_goal_pin",
                        "placement_motion_qualified": False}
                    return
                frame = dict(frame, anchor_pose_at_pin=pose_record[0].copy())
            self.placement_pinned_frame = frame
            self.placement_pin_expires = now + self.placement_selection_timeout_s
            self.placement_selection_request_id = request_id
            self.placement_selection_mode = mode
            self.placement_gray_history.clear()
            self._append_placement_gray_frame(frame, frame["rgb"])
            self.placement_status = {"state": "selection_frame_pinned", "valid": False,
                "reason": "selection_frame_pinned", "generation": frame["generation"],
                "stamp_ns": int(frame["stamp_ns"]), "frame_id": frame["frame_id"],
                "selection_request_id": request_id, "selection_mode": mode,
                "selection_expires_in_s": self.placement_selection_timeout_s,
                "placement_motion_qualified": False}
            return
        generation = str(request.get("generation", ""))
        if request.get("action") == "clear_selection":
            self._cancel_placement_replay()
            self.placement_tracker = PlacementRegionTracker(
                max_selection_age_s=self.placement_selection_timeout_s)
            self.placement_frontend = SparseRGBDAnchor(self.placement_tracker)
            self.placement_plane_metrics = None
            self.placement_pinned_frame = None
            self.placement_pin_expires = None
            self.placement_selection_request_id = None
            self.placement_selection_mode = None
            self.placement_gray_history.clear()
            self.placement_selection_failure_reason = None
            self.placement_status = {"state": "selection_cleared", "valid": False,
                "reason": "selection_cleared", "generation": generation,
                "placement_motion_qualified": False,
                "physical_placement_admission": False}
            return
        frame = None
        if (self.placement_pinned_frame is not None
                and self.placement_pinned_frame["generation"] == generation
                and self.placement_pin_expires is not None
                and time.monotonic() <= self.placement_pin_expires):
            frame = self.placement_pinned_frame
        else:
            frame = next((item for item in self.placement_frames
                          if item["generation"] == generation), None)
        if request.get("action") == "pin_frame":
            if self.placement_replay_active:
                self.placement_status = {"state": "invalid", "valid": False,
                    "reason": "selection_replay_in_progress",
                    "placement_motion_qualified": False}
                return
            if frame is None or time.monotonic() - frame["received_monotonic"] > .4:
                self.placement_status = {"state": "invalid", "valid": False,
                    "reason": "selection_frame_stale_before_pin", "generation": generation,
                    "placement_motion_qualified": False}
                return
            if request.get("mode") == "select_placement_area":
                pose_record = self.placement_tracker._history.get(frame["generation"])
                if pose_record is None or int(pose_record[1]) != int(frame["stamp_ns"]):
                    self.placement_status = {"state": "invalid", "valid": False,
                        "reason": "anchor_pose_unavailable_at_goal_pin",
                        "placement_motion_qualified": False}
                    return
                frame = dict(frame, anchor_pose_at_pin=pose_record[0].copy())
            self.placement_pinned_frame = frame
            self.placement_pin_expires = time.monotonic() + self.placement_selection_timeout_s
            self.placement_selection_request_id = str(request.get("request_id", ""))
            self.placement_selection_mode = str(request.get("mode", ""))
            self.placement_gray_history.clear()
            self._append_placement_gray_frame(frame, frame["rgb"])
            self.placement_status = {"state": "selection_frame_pinned", "valid": False,
                "reason": "selection_frame_pinned", "generation": generation,
                "selection_request_id": self.placement_selection_request_id,
                "selection_mode": self.placement_selection_mode,
                "selection_expires_in_s": self.placement_selection_timeout_s,
                "placement_motion_qualified": False}
            return
        if request.get("action") == "unpin_frame":
            if self.placement_pinned_frame is not None and self.placement_pinned_frame["generation"] == generation:
                self.placement_pinned_frame = None
                self.placement_pin_expires = None
                self.placement_selection_request_id = None
                self.placement_selection_mode = None
                self.placement_gray_history.clear()
            self.placement_status = {"state": "selection_cancelled", "valid": False,
                "reason": "selection_cancelled", "generation": generation,
                "placement_motion_qualified": False}
            return
        if (frame is None and self.placement_pinned_frame is not None
                and self.placement_pinned_frame["generation"] == generation
                and self.placement_pin_expires is not None
                and time.monotonic() <= self.placement_pin_expires):
            frame = self.placement_pinned_frame
        if frame is None:
            self.placement_gray_history.clear()
            self.placement_status = {"state": "invalid", "valid": False,
                "reason": ("selection_frame_pin_expired" if self.placement_pinned_frame is not None
                           else "selection_frame_expired"), "generation": generation,
                "placement_motion_qualified": False}
            return
        now = time.monotonic()
        pin_valid = (frame is self.placement_pinned_frame
                     and self.placement_pin_expires is not None
                     and now <= self.placement_pin_expires)
        if not pin_valid and now - frame["received_monotonic"] > .4:
            self.placement_gray_history.clear()
            self.placement_status = {"state": "invalid", "valid": False,
                "reason": "selection_frame_stale", "generation": generation,
                "placement_motion_qualified": False}
            return
        if pin_valid:
            self.placement_pinned_frame = None
            self.placement_pin_expires = None
            self.placement_selection_request_id = None
            self.placement_selection_mode = None
        try:
            rectangle = request["rect_xyxy"]
            if request.get("action") == "select_feeder_anchor":
                if self.placement_replay_active:
                    raise ValueError("placement history replay already active")
                old = self.placement_tracker
                # Accepting a replacement retires the previous UUID and area
                # before any feature extraction or history replay can fail.
                retired_tracker = PlacementRegionTracker(
                    max_selection_age_s=self.placement_selection_timeout_s)
                self.placement_tracker = retired_tracker
                self.placement_frontend = SparseRGBDAnchor(retired_tracker)
                self.placement_plane_metrics = None
                self.placement_selection_failure_reason = None
                candidate_tracker = PlacementRegionTracker(
                    max_age_s=old.max_age_s, max_frame_gap_s=old.max_frame_gap_s,
                    inlier_threshold_m=old.inlier_threshold_m,
                    minimum_features=old.minimum_features,
                    minimum_fraction=old.minimum_fraction,
                    history_size=old.history_size,
                    max_selection_age_s=self.placement_selection_timeout_s)
                candidate = SparseRGBDAnchor(candidate_tracker,
                    maximum_features=self.placement_frontend.maximum_features,
                    minimum_depth_m=self.placement_frontend.minimum_depth_m,
                    maximum_depth_m=self.placement_frontend.maximum_depth_m,
                    forward_backward_limit_px=self.placement_frontend.forward_backward_limit_px)
                okay = candidate.select_anchor(frame["rgb"], frame["depth_m"],
                    frame["camera_matrix"], np.eye(4), rectangle,
                    stamp_ns=frame["stamp_ns"], frame_id=frame["frame_id"],
                    generation=generation, received_monotonic=frame["received_monotonic"])
                if not okay:
                    raise ValueError(candidate_tracker.reason)
                self.placement_plane_metrics = None
                latest = max(self.placement_frames,
                             key=lambda item: int(item["stamp_ns"])) if self.placement_frames else frame
                if int(latest["stamp_ns"]) > int(frame["stamp_ns"]):
                    history = self._placement_replay_frames(frame, latest)
                    if history is None:
                        self.placement_selection_failure_reason = "selection_history_unavailable"
                        self.placement_status = {"state": "invalid", "valid": False,
                            "reason": "selection_history_unavailable",
                            "generation": latest["generation"],
                            "placement_motion_qualified": False}
                        return
                    self.placement_status = {"state": "selection_replay_pending",
                        "valid": False, "anchor_tracking_valid": False,
                        "reason": "replaying_continuous_frame_history",
                        "selection_id": candidate_tracker.selection_id,
                        "placement_motion_qualified": False,
                        "physical_placement_admission": False}
                    self.placement_replay_candidate = candidate
                    self.placement_replay_base = {
                        "generation": latest["generation"],
                        "stamp_ns": int(latest["stamp_ns"])}
                    self.placement_replay_active = True
                    self.placement_replay_future = self.placement_replay_executor.submit(
                        candidate.replay_to_current, history, latest["depth_m"],
                        latest["camera_matrix"], np.eye(4), frame_id=latest["frame_id"],
                        now_monotonic=time.monotonic(),
                        max_history_s=self.placement_history_seconds)
                    self.placement_replay_target = latest
                    self.placement_status = {"state": "selection_replay_pending",
                        "valid": False, "anchor_tracking_valid": False,
                        "reason": "replaying_continuous_frame_history",
                        "generation": latest["generation"], "stamp_ns": latest["stamp_ns"],
                        "frame_id": latest["frame_id"],
                        "selection_id": candidate_tracker.selection_id,
                        "feature_count": len(candidate_tracker.anchor_points),
                        "placement_motion_qualified": False,
                        "physical_placement_admission": False}
                else:
                    self.placement_frontend = candidate
                    self.placement_tracker = candidate_tracker
                    self.placement_status = {"state": "anchor_selected", "valid": False,
                        "reason": "placement_area_not_selected", "generation": generation,
                        "stamp_ns": frame["stamp_ns"], "frame_id": frame["frame_id"],
                        "feature_count": len(candidate_tracker.anchor_points),
                        "selection_id": candidate_tracker.selection_id,
                        "placement_motion_qualified": False}
            elif request.get("action") == "select_placement_area":
                polygon, metrics = project_rectangle_to_plane(rectangle, frame["depth_m"],
                    frame["camera_matrix"], np.eye(4))
                if not self.placement_tracker.set_goal(frame["stamp_ns"], polygon,
                        generation=generation, received_monotonic=frame["received_monotonic"],
                        now_monotonic=now,
                        pose_override=frame.get("anchor_pose_at_pin")):
                    raise ValueError(self.placement_tracker.reason)
                self.placement_status = {"state": "goal_selected", "valid": False,
                    "reason": "awaiting_fresh_anchor_update", "generation": generation,
                    "stamp_ns": frame["stamp_ns"], "frame_id": frame["frame_id"],
                    "goal_corners_root": None, "goal_corners_camera": polygon.tolist(),
                    "camera_frame": frame["frame_id"], "plane_metrics": metrics,
                    "camera_from_goal": pose_from_rectangle(polygon)[0].tolist(),
                    "camera_from_goal_stamp_ns": frame["stamp_ns"],
                    "selection_id": self.placement_tracker.selection_id,
                    "received_monotonic": now, "fit_rms_m": 0.0,
                    "placement_motion_qualified": False,
                    "physical_placement_admission": False}
                self.placement_plane_metrics = metrics
            else:
                raise ValueError("unknown placement selection action")
        except (KeyError, TypeError, ValueError) as error:
            if request.get("action") == "select_feeder_anchor":
                self.placement_selection_failure_reason = f"selection_rejected:{error}"
            self.placement_status = {"state": "invalid", "valid": False,
                "reason": f"selection_rejected:{error}", "generation": generation,
                "placement_motion_qualified": False}
        finally:
            if not self.placement_replay_active:
                self.placement_gray_history.clear()

    def _placement_replay_frames(self, pinned_frame, latest_frame):
        """Decode bounded compressed frames strictly between pin and latest RGB-D."""
        import cv2
        history = list(self.placement_gray_history)
        generations = [item["generation"] for item in history]
        try:
            start = generations.index(str(pinned_frame["generation"]))
        except ValueError:
            return None
        selected = [item for item in history[start + 1:]
                    if int(item["stamp_ns"]) <= int(latest_frame["stamp_ns"])]
        expected_frame = str(pinned_frame["frame_id"])
        if (not selected or selected[-1]["generation"] != latest_frame["generation"]
                or str(latest_frame["frame_id"]) != expected_frame
                or any(str(item["frame_id"]) != expected_frame for item in selected)):
            return None
        def decoded_frames():
            for item in selected:
                image = cv2.imdecode(np.frombuffer(item["jpeg"], dtype=np.uint8),
                                     cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise ValueError("invalid compressed placement history frame")
                yield (item["stamp_ns"], item["generation"],
                       item["received_monotonic"], image)
        return decoded_frames()

    def _append_placement_gray_frame(self, record, rgb):
        """Append one bounded grayscale JPEG only during a pinned selection."""
        gray = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2GRAY)
        okay, encoded = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if not okay:
            raise ValueError("could not encode placement tracking frame")
        self.placement_gray_history.append({
            "stamp_ns": int(record["stamp_ns"]),
            "generation": str(record["generation"]),
            "frame_id": str(record["frame_id"]),
            "received_monotonic": float(record["received_monotonic"]),
            "jpeg": encoded.tobytes()})
        cutoff = float(record["received_monotonic"]) - self.placement_history_seconds
        while (self.placement_gray_history
               and float(self.placement_gray_history[0]["received_monotonic"]) < cutoff):
            self.placement_gray_history.popleft()

    def _finish_placement_replay(self):
        """Poll isolated LK replay; all potentially long catch-up stays off-thread."""
        if not self.placement_replay_active:
            return
        if self.placement_replay_future is not None:
            if not self.placement_replay_future.done():
                return
            try:
                result = self.placement_replay_future.result()
            except Exception as error:
                self._fail_placement_replay(f"history_replay_error:{error}")
                return
            self.placement_replay_future = None
            if result.reason not in {"tracked", "stale_frame"}:
                self._fail_placement_replay(f"history_replay_{result.reason}")
                return
            if self.placement_replay_target is not None:
                self.placement_replay_base = {
                    "generation": self.placement_replay_target["generation"],
                    "stamp_ns": int(self.placement_replay_target["stamp_ns"])}
                self.placement_replay_target = None

        candidate = self.placement_replay_candidate
        if candidate is None or self.placement_replay_base is None:
            self._fail_placement_replay("history_replay_state_missing")
            return
        latest = (max(self.placement_frames, key=lambda item: int(item["stamp_ns"]))
                  if self.placement_frames else None)
        if latest is None:
            self._fail_placement_replay("latest_rgbd_frame_unavailable")
            return
        base = self.placement_replay_base
        if int(latest["stamp_ns"]) > int(base["stamp_ns"]):
            history = self._placement_replay_frames({
                "generation": base["generation"], "frame_id": latest["frame_id"]}, latest)
            if history is None:
                self._fail_placement_replay("history_tail_unavailable")
                return
            target = latest
            self.placement_replay_target = target
            self.placement_replay_future = self.placement_replay_executor.submit(
                candidate.replay_to_current, history, target["depth_m"],
                target["camera_matrix"], np.eye(4), frame_id=target["frame_id"],
                now_monotonic=time.monotonic(),
                max_history_s=self.placement_history_seconds)
            return
        snapshot = candidate.tracker.snapshot(time.monotonic())
        if (snapshot.reason == "tracked" and snapshot.age_s is not None
                and snapshot.age_s <= candidate.tracker.max_age_s
                and candidate._reference_stamp_ns == int(latest["stamp_ns"])):
            self.placement_frontend = candidate
            self.placement_tracker = candidate.tracker
            self.placement_status = self._placement_snapshot_dict(snapshot)
            self.placement_status.update({
                "camera_frame": latest["frame_id"],
                "received_monotonic": latest["received_monotonic"],
                "anchor_tracking_valid": True,
                "camera_from_goal": None,
                "plane_metrics": None,
                "placement_motion_qualified": False,
                "physical_placement_admission": False})
            self.placement_replay_active = False
            self.placement_replay_candidate = None
            self.placement_replay_base = None
            self.placement_replay_target = None
            self.placement_gray_history.clear()
        else:
            self.placement_status = {
                "state": "selection_replay_pending", "valid": False,
                "anchor_tracking_valid": False,
                "reason": "waiting_for_fresh_rgbd_after_history_replay",
                "generation": latest["generation"], "stamp_ns": latest["stamp_ns"],
                "frame_id": latest["frame_id"],
                "selection_id": candidate.tracker.selection_id,
                "feature_count": (0 if candidate.tracker.anchor_points is None
                                  else len(candidate.tracker.anchor_points)),
                "placement_motion_qualified": False,
                "physical_placement_admission": False}

    def _cancel_placement_replay(self):
        if self.placement_replay_future is not None:
            self.placement_replay_future.cancel()
        self.placement_replay_future = None
        self.placement_replay_candidate = None
        self.placement_replay_base = None
        self.placement_replay_target = None
        self.placement_replay_active = False
        self.placement_gray_history.clear()

    def _fail_placement_replay(self, reason):
        candidate = self.placement_replay_candidate
        selection_id = (None if candidate is None
                        else candidate.tracker.selection_id)
        self.placement_status = {"state": "invalid", "valid": False,
            "anchor_tracking_valid": False, "reason": str(reason),
            "selection_id": selection_id,
            "placement_motion_qualified": False,
            "physical_placement_admission": False}
        self.placement_selection_failure_reason = str(reason)
        # Do not leave a retired frontend installed: a later RGB-D callback could
        # otherwise make an older selection valid again after B failed.
        retired_tracker = PlacementRegionTracker(
            max_selection_age_s=self.placement_selection_timeout_s)
        self.placement_tracker = retired_tracker
        self.placement_frontend = SparseRGBDAnchor(retired_tracker)
        self.placement_plane_metrics = None
        self.placement_replay_future = None
        self.placement_replay_candidate = None
        self.placement_replay_base = None
        self.placement_replay_target = None
        self.placement_replay_active = False
        self.placement_gray_history.clear()

    def destroy_node(self):
        self.placement_replay_executor.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

    def _placement_selection_service(self, request, response):
        modes = {"feeder": "select_feeder_anchor", "goal": "select_placement_area",
                 "clear": "clear_selection"}
        mode = str(request.mode)
        request_id = uuid.uuid4().hex
        if mode in {"pin_feeder", "pin_goal"}:
            selection_mode = ("select_feeder_anchor" if mode == "pin_feeder"
                              else "select_placement_area")
            generation = str(request.generation)
            command = {"request_id": request_id,
                       "mode": selection_mode,
                       "generation": generation}
            if generation:
                command["action"] = "pin_frame"
            else:
                command["action"] = "pin_latest_frame"
            self._process_placement_request(command)
            pinned = self.placement_status.get("state") == "selection_frame_pinned"
            payload = {
                "state": self.placement_status.get("state", "invalid"),
                "reason": self.placement_status.get("reason", ""),
                "mode": mode,
                "generation": self.placement_status.get("generation", ""),
                "request_id": request_id if pinned else "",
                "expires_in_s": (self.placement_status.get("selection_expires_in_s", 0.)
                                 if pinned else 0.),
            }
            response.success = bool(pinned)
            response.message = json.dumps(payload, sort_keys=True)
            response.selection_id = str(self.placement_status.get("selection_id", ""))
            return response
        if mode == "cancel_pin":
            generation = str(request.generation)
            active = self.placement_pinned_frame
            matches = (active is not None and
                       (not generation or active.get("generation") == generation))
            if matches:
                self._process_placement_request({
                    "request_id": request_id, "action": "cancel_pin",
                    "pin_request_id": self.placement_selection_request_id})
            payload = {
                "state": self.placement_status.get("state", "invalid"),
                "reason": self.placement_status.get("reason", ""),
                "generation": generation,
            }
            response.success = bool(matches)
            response.message = json.dumps(payload, sort_keys=True)
            response.selection_id = str(self.placement_status.get("selection_id", ""))
            return response
        if mode not in modes:
            response.success = False
            response.message = "mode must be feeder, goal, clear, pin_feeder, pin_goal, or cancel_pin"
            response.selection_id = ""
            return response
        command = {"request_id": uuid.uuid4().hex, "action": modes[mode],
                   "generation": str(request.generation)}
        if mode != "clear":
            command["rect_xyxy"] = list(request.pixel_rectangle)
        self._process_placement_request(command)
        response.success = (self.placement_status.get("state") in {
            "anchor_selected", "goal_selected", "selection_cleared",
            "selection_replay_pending"}
            or (mode == "feeder" and
                self.placement_status.get("anchor_tracking_valid") is True))
        response.message = str(self.placement_status.get("reason", ""))
        response.selection_id = str(self.placement_status.get("selection_id", ""))
        return response

    def _joints(self, message):
        if not message.name or len(message.name) != len(message.position):
            return
        self.state.update_joints(_stamp_ns(message.header.stamp), time.monotonic(),
                                 list(message.name), list(message.position))

    def _description(self, message):
        if message.data:
            description = message.data
            try:
                from ament_index_python.packages import get_package_share_directory
                description = resolve_package_mesh_uris(
                    description, get_package_share_directory)
            except Exception as error:
                self.get_logger().warning(f"Could not resolve URDF package meshes: {error}")
            self.state.robot_description = description

    def _point_cloud(self, message):
        try:
            points, colors, color_source = sampled_cloud(message, max_points=12000)
            self.state.point_cloud = {
                "frame_id": message.header.frame_id.lstrip("/"),
                "stamp_ns": _stamp_ns(message.header.stamp),
                "points_m": points.astype(float).tolist(),
                "color_source": color_source,
                "rgb8_b64": None if colors is None else encode_rgb8(colors),
                "source": "/camera/depth/color/points",
                "received_monotonic": time.monotonic(),
            }
        except Exception as error:
            self.get_logger().warning(f"Ignoring malformed point cloud: {error}")

    def _tf(self, message, static):
        received = time.monotonic()
        for transform in message.transforms:
            t, q = transform.transform.translation, transform.transform.rotation
            matrix = transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            self.state.update_transform(transform.header.frame_id, transform.child_frame_id,
                                        _stamp_ns(transform.header.stamp), received, matrix,
                                        static=static)

    def _dynamic_tf(self, message):
        self._tf(message, False)

    def _static_tf(self, message):
        self._tf(message, True)

    def _markers(self, message):
        received = time.monotonic()
        for marker in message.markers:
            name = f"{marker.ns}/{marker.id}"
            if marker.action == marker.DELETEALL or marker.action == marker.DELETE:
                self.state.objects.pop(name, None)
                continue
            pose = marker.pose
            matrix = transform_matrix((pose.position.x, pose.position.y, pose.position.z),
                                      (pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w))
            self.state.update_object(name, _stamp_ns(marker.header.stamp), received, {
                "frame_id": marker.header.frame_id.lstrip("/"),
                "stamp_ns": _stamp_ns(marker.header.stamp), "matrix": matrix.tolist(),
                "scale_m": [marker.scale.x, marker.scale.y, marker.scale.z],
                "color_rgba": [marker.color.r, marker.color.g, marker.color.b, marker.color.a],
                "kind": int(marker.type), "text": marker.text,
                "source": "/so101/object_markers", "received_monotonic": received,
            })

    def _poses(self, message, topic):
        stamp = _stamp_ns(message.header.stamp)
        frame = message.header.frame_id.lstrip("/")
        received = time.monotonic()
        for index, pose in enumerate(message.poses):
            name = f"{topic}/{index}"
            matrix = transform_matrix((pose.position.x, pose.position.y, pose.position.z),
                                      (pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w))
            self.state.update_object(name, stamp, received, {"frame_id": frame, "stamp_ns": stamp,
                "matrix": matrix.tolist(), "source": topic,
                "received_monotonic": received})

    def _contacts(self, message):
        self._poses(message, "/so101/grasp_contacts")

    def _candidates(self, message):
        self._poses(message, "/so101/grasp_candidates")

    def _write(self):
        self._consume_placement_request()
        self._finish_placement_replay()
        now = time.monotonic()
        if self.placement_tracker.anchor_points is not None:
            tracker_snapshot = self.placement_tracker.snapshot(now)
            if self.placement_status.get("valid") and not tracker_snapshot.valid:
                self.placement_status = self._placement_snapshot_dict(tracker_snapshot)
                self.placement_status["valid"] = False
                self.placement_status["goal_corners_camera"] = None
                self.placement_status["camera_from_goal"] = None
        if (self.placement_pin_expires is not None and now > self.placement_pin_expires):
            self.placement_pinned_frame = None
            self.placement_pin_expires = None
            self.placement_selection_request_id = None
            self.placement_selection_mode = None
            self.placement_gray_history.clear()
        elif self.placement_status.get("state") in {"tracked", "anchor_tracked"}:
            age = now - float(self.placement_status.get("received_monotonic", now))
            if age > self.placement_tracker.max_age_s:
                self.placement_status = dict(self.placement_status,
                    state="invalid", valid=False, anchor_tracking_valid=False,
                    reason="stale_frame", received_monotonic=now,
                    goal_corners_camera=None, camera_from_goal=None,
                    goal_corners_root=None, goal_corners_base=None)
        snapshot = self.state.snapshot(now)
        snapshot["root_frame"] = self.root_frame
        snapshot["placement_camera_frame"] = self.placement_frame
        snapshot["placement_goal"] = self.placement_status
        pinned = self.placement_pinned_frame
        if pinned is None or self.placement_pin_expires is None or now > self.placement_pin_expires:
            snapshot["placement_selection_frame"] = None
        else:
            snapshot["placement_selection_frame"] = dict(
                placement_camera_frame_summary(pinned),
                request_id=self.placement_selection_request_id,
                mode=self.placement_selection_mode,
                expires_in_s=max(0., self.placement_pin_expires - now))
        parent = os.path.dirname(self.output) or "."
        os.makedirs(parent, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=".ros-twin-", dir=parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(snapshot, stream, separators=(",", ":"))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.output)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        goal = String()
        goal.data = json.dumps(self.placement_status, separators=(",", ":"))
        self.placement_pub.publish(goal)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--root-frame", default="charuco_board")
    parser.add_argument("--max-age", type=float, default=0.5)
    parser.add_argument("--placement-directory", type=Path)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--placement-selection-timeout", type=float, default=30.)
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = RosStatePublisher(args.output, args.root_frame, args.max_age,
        args.placement_directory, args.color_topic, args.depth_topic, args.camera_info_topic,
        args.placement_selection_timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
