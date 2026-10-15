#!/usr/bin/env python3
"""Save one settled, timestamp-paired RGB-D and ROS TF registration bundle."""

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import re
import time

import numpy as np


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
          "wrist_roll", "gripper")
GRIPPER_FRAME = "gripper_frame_link"


def timeout_message(last_rejection_reason):
    detail = last_rejection_reason or "no complete candidate frame arrived"
    return f"timed out waiting for a valid settled ROS registration frame; last rejection: {detail}"


def validate_frame_triplet(stamps, frame_ids, *, now, expected_frame,
                           max_age_s=0.5, max_pair_delta_s=0.04):
    """Validate one exact color/depth/CameraInfo tuple and return color stamp."""
    if set(stamps) != {"color", "depth", "camera_info"}:
        raise ValueError("RGB-D frame stamps are incomplete")
    if set(frame_ids) != {"color", "depth", "camera_info"}:
        raise ValueError("RGB-D frame ids are incomplete")
    stamp = float(stamps["color"])
    values = [float(stamps[name]) for name in ("color", "depth", "camera_info")]
    if not all(math.isfinite(value) and value > 0 for value in values):
        raise ValueError("RGB-D frame stamps must be finite and positive")
    if not math.isfinite(now) or stamp > now + 0.05 or now - stamp > max_age_s:
        raise ValueError("color input is stale or from the future")
    if any(abs(value - stamp) > max_pair_delta_s for value in values[1:]):
        raise ValueError("RGB-D and CameraInfo stamps differ by more than 40 ms")
    if any(frame_ids[name] != expected_frame for name in frame_ids):
        raise ValueError("RGB-D frame id does not match the configured optical frame")
    return stamp


def validate_tracking_status(status, *, now, max_age_s=0.5):
    """Require a current, valid ChArUco world observation."""
    if not isinstance(status, dict) or status.get("state") != "tracking" or status.get("observation_valid") is not True:
        raise ValueError("fresh board tracking is required")
    try:
        stamp = float(status["stamp"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("fresh board tracking status has no valid stamp") from None
    if not math.isfinite(stamp) or stamp <= 0 or stamp > now + 0.05 or now - stamp > max_age_s:
        raise ValueError("board tracking status is not fresh")
    return stamp


def closest_settled_joint_state(samples, image_stamp, joint_names=JOINTS, *,
                                max_age_s=0.08, window_s=0.25,
                                max_span_rad=0.004):
    """Return the closest six-joint state after confirming a quiet prior window.

    Samples are ``(stamp_seconds, names, positions)`` tuples. The settled
    window must cover the full interval immediately before the image stamp.
    """
    expected = tuple(joint_names)
    valid = []
    for stamp, names, positions in samples:
        stamp = float(stamp)
        names = tuple(names)
        positions = tuple(float(value) for value in positions)
        if (not math.isfinite(stamp) or len(names) != len(positions)
                or len(set(names)) != len(names) or set(names) != set(expected)
                or not all(math.isfinite(value) for value in positions)):
            continue
        valid.append((stamp, dict(zip(names, positions))))
    if not valid:
        raise ValueError("no valid six-joint samples are available")
    nearest_stamp, nearest = min(valid, key=lambda item: abs(item[0] - image_stamp))
    if abs(nearest_stamp - image_stamp) > max_age_s:
        raise ValueError("no joint state is nearby the image timestamp")
    boundary = image_stamp - window_s
    before_boundary = [(stamp, pose) for stamp, pose in valid if stamp <= boundary]
    if not before_boundary:
        raise ValueError("joint samples do not cover the settled window")
    bracket = max(before_boundary, key=lambda item: item[0])
    if boundary - bracket[0] > max_age_s:
        raise ValueError("joint samples do not bracket the settled window boundary")
    window = [bracket] + [(stamp, pose) for stamp, pose in valid
                          if boundary < stamp <= image_stamp]
    if (len(window) < 2 or image_stamp - window[-1][0] > max_age_s):
        raise ValueError("joint samples do not cover the settled window")
    for joint in expected:
        values = [pose[joint] for _, pose in window]
        if max(values) - min(values) > max_span_rad:
            raise ValueError("joint state is moving; settled window span exceeds 0.004 rad")
    return nearest


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _transform_matrix(message):
    from scipy.spatial.transform import Rotation

    transform = message.transform
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([
        transform.rotation.x, transform.rotation.y,
        transform.rotation.z, transform.rotation.w]).as_matrix()
    result[:3, 3] = [transform.translation.x, transform.translation.y,
                     transform.translation.z]
    return result


def _mount_roll_from_config(path):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(r"^\s*gripper_mount_roll_deg\s*:\s*([-+0-9.eE]+)\s*$", text, re.M)
    if not match:
        raise ValueError(f"model offset config has no gripper_mount_roll_deg: {path}")
    return float(match.group(1))


def _station_identity(config_path):
    if not config_path:
        return {}
    config_path = Path(config_path)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    arm = config.get("arm", {})
    result = {"arm_id": arm.get("id"),
              "station_config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest()}
    calibration = arm.get("calibration", {})
    calibration_path = calibration.get("path")
    if calibration_path and Path(calibration_path).is_file():
        result["calibration_sha256"] = hashlib.sha256(Path(calibration_path).read_bytes()).hexdigest()
    if calibration.get("sha256"):
        result["configured_calibration_sha256"] = calibration["sha256"]
    return result


class RegistrationCaptureNode:
    """ROS wrapper; importing this file does not require ROS to be installed."""

    def __new__(cls, *args, **kwargs):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener

        class _Node(Node):
            def __init__(self, output_directory, *, timeout_s, station_config="",
                         color_topic="/camera/color/image_raw",
                         depth_topic="/camera/aligned_depth_to_color/image_raw",
                         camera_info_topic="/camera/color/camera_info",
                         joint_states_topic="/follower/joint_states",
                         tracking_topic="/so101_perception/charuco_world/status",
                         optical_frame="camera_color_optical_frame",
                         world_frame="charuco_board", base_frame="base_link",
                         gripper_frame=GRIPPER_FRAME):
                super().__init__("capture_ros_robot_registration")
                self.output_directory = Path(output_directory)
                self.timeout_s = timeout_s
                self.station_config = station_config
                self.optical_frame, self.world_frame = optical_frame, world_frame
                self.base_frame, self.gripper_frame = base_frame, gripper_frame
                self.bridge = CvBridge()
                self.color = deque(maxlen=8)
                self.depth = deque(maxlen=8)
                self.camera_info = deque(maxlen=8)
                self.joints = deque(maxlen=128)
                self.tracking = None
                self.started = time.monotonic()
                self.done = False
                self.failure = None
                self.last_rejection_reason = "waiting for matching color/depth/CameraInfo frames"
                self.pending_since = {}
                self.robot_description = None
                self.description_client = AsyncParameterClient(
                    self, "/follower/robot_state_publisher")
                self.description_future = self.description_client.get_parameters(["robot_description"])
                self.buffer = Buffer()
                self.listener = TransformListener(self.buffer, self)
                self.create_subscription(Image, color_topic, self._color, qos_profile_sensor_data)
                self.create_subscription(Image, depth_topic, self._depth, qos_profile_sensor_data)
                self.create_subscription(CameraInfo, camera_info_topic, self._info, qos_profile_sensor_data)
                self.create_subscription(JointState, joint_states_topic, self._joint, qos_profile_sensor_data)
                self.create_subscription(String, tracking_topic, self._tracking, 10)
                self.timer = self.create_timer(0.02, self._capture_pending)

            def _color(self, message):
                self.color.append(message)

            def _depth(self, message):
                self.depth.append(message)

            def _info(self, message):
                self.camera_info.append(message)

            def _joint(self, message):
                self.joints.append((_stamp_seconds(message.header.stamp),
                                    tuple(message.name), tuple(message.position)))

            def _tracking(self, message):
                try:
                    self.tracking = json.loads(message.data)
                except (TypeError, json.JSONDecodeError):
                    self.tracking = None

            def _closest(self, queue, stamp):
                return min(queue, key=lambda item: abs(_stamp_seconds(item.header.stamp) - stamp)) if queue else None

            def _capture_pending(self):
                if self.done:
                    return
                if time.monotonic() - self.started > self.timeout_s:
                    self.failure = RuntimeError(timeout_message(self.last_rejection_reason))
                    self.done = True
                    return
                if self.robot_description is None:
                    if not self.description_future.done():
                        return
                    try:
                        response = self.description_future.result()
                        value = response.values[0]
                        if not value.string_value:
                            raise RuntimeError("robot_state_publisher returned no robot_description")
                        self.robot_description = value.string_value
                    except Exception as error:
                        self.failure = RuntimeError(f"could not read actual robot_description: {error}")
                        self.done = True
                        return
                for color in reversed(tuple(self.color)):
                    stamp = _stamp_seconds(color.header.stamp)
                    if stamp in self.pending_since:
                        since = self.pending_since[stamp]
                    else:
                        self.pending_since[stamp] = time.monotonic()
                        since = self.pending_since[stamp]
                    depth, info = self._closest(self.depth, stamp), self._closest(self.camera_info, stamp)
                    if depth is None or info is None:
                        continue
                    if time.monotonic() - since > 0.25:
                        self.pending_since.pop(stamp, None)
                        try:
                            self.color.remove(color)
                        except ValueError:
                            pass
                        continue
                    now = self.get_clock().now().nanoseconds * 1e-9
                    try:
                        frame_stamp = validate_frame_triplet(
                            {"color": stamp, "depth": _stamp_seconds(depth.header.stamp),
                             "camera_info": _stamp_seconds(info.header.stamp)},
                            {"color": color.header.frame_id, "depth": depth.header.frame_id,
                             "camera_info": info.header.frame_id}, now=now,
                            expected_frame=self.optical_frame)
                        validate_tracking_status(self.tracking, now=now)
                        joint_positions = closest_settled_joint_state(
                            self.joints, frame_stamp, JOINTS)
                        transforms = {
                            "world_from_camera": self._lookup(self.world_frame, self.optical_frame, frame_stamp),
                            "world_from_base": self._lookup(self.world_frame, self.base_frame, frame_stamp),
                            "base_from_gripper_frame": self._lookup(self.base_frame, self.gripper_frame, frame_stamp),
                        }
                    except ValueError as error:
                        self.last_rejection_reason = str(error)
                        continue
                    except Exception as error:
                        self.last_rejection_reason = (
                            f"exact TF lookup at image stamp {frame_stamp:.9f} failed: {error}")
                        continue
                    self._write_capture(color, depth, info, frame_stamp,
                                        joint_positions, transforms, now)
                    self.done = True
                    self.timer.cancel()
                    return

            def _lookup(self, target, source, stamp):
                request_time = Time(seconds=stamp)
                return _transform_matrix(self.buffer.lookup_transform(target, source, request_time))

            def _write_capture(self, color_msg, depth_msg, info, stamp,
                               joint_positions, transforms, now):
                try:
                    import cv2
                    from ament_index_python.packages import get_package_share_directory
                    color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
                    depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                    if depth_msg.encoding != "16UC1" or depth.dtype != np.uint16:
                        raise ValueError("aligned depth must use raw 16UC1 encoding")
                    if color.shape[:2] != depth.shape:
                        raise ValueError("color and aligned depth dimensions differ")
                    share = Path(get_package_share_directory("so101_description"))
                    offsets_share = Path(get_package_share_directory("so101_station_control"))
                    package_paths = {"so101_description": str(share)}
                    roll = _mount_roll_from_config(offsets_share / "config" / "model_offsets.yaml")
                    if not cv2.imwrite(str(self.output_directory / "color.png"), color):
                        raise RuntimeError("could not save color.png")
                    np.save(self.output_directory / "depth.npy", depth, allow_pickle=False)
                    urdf_path.write_text(self.robot_description, encoding="utf-8")
                    capture = {
                        "schema": 1,
                        "stamp": stamp,
                        "color_stamp": stamp,
                        "depth_stamp": _stamp_seconds(depth_msg.header.stamp),
                        "camera_info_stamp": _stamp_seconds(info.header.stamp),
                        "captured_at_stamp": now,
                        "frames": {"world": self.world_frame, "camera": self.optical_frame,
                                   "base": self.base_frame, "gripper": self.gripper_frame},
                        "world_from_camera": transforms["world_from_camera"].tolist(),
                        "world_from_base": transforms["world_from_base"].tolist(),
                        "base_from_gripper_frame": transforms["base_from_gripper_frame"].tolist(),
                        "camera": {"k": list(info.k), "d": list(info.d),
                                   "width": int(info.width), "height": int(info.height)},
                        "depth_encoding": "16UC1", "depth_scale_m": 0.001,
                        "joint_positions": joint_positions,
                        "tracking_status": self.tracking,
                        "mesh_package_paths": package_paths,
                        "gripper_mount_roll_deg": roll,
                        "urdf_sha256": hashlib.sha256(self.robot_description.encode()).hexdigest(),
                        **_station_identity(self.station_config),
                    }
                    (self.output_directory / "capture.json").write_text(
                        json.dumps(capture, indent=2, sort_keys=True, allow_nan=False) + "\n",
                        encoding="utf-8")
                except Exception as error:
                    self.failure = error
                    self.done = True

        return _Node(*args, **kwargs)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--station-config", default="")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--joint-states-topic", default="/follower/joint_states")
    parser.add_argument("--tracking-topic", default="/so101_perception/charuco_world/status")
    parser.add_argument("--optical-frame", default="camera_color_optical_frame")
    parser.add_argument("--world-frame", default="charuco_board")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--gripper-frame", default=GRIPPER_FRAME)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    args.output_directory = args.output_directory.resolve()
    args.output_directory.mkdir(parents=True, exist_ok=False)
    import rclpy
    rclpy.init(args=None)
    node = RegistrationCaptureNode(
        args.output_directory, timeout_s=args.timeout, station_config=args.station_config,
        color_topic=args.color_topic, depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic, joint_states_topic=args.joint_states_topic,
        tracking_topic=args.tracking_topic, optical_frame=args.optical_frame,
        world_frame=args.world_frame, base_frame=args.base_frame,
        gripper_frame=args.gripper_frame)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.failure:
            raise node.failure
        print(f"Saved ROS registration capture to {args.output_directory}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
