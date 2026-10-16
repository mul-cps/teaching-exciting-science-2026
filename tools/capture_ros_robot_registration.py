#!/usr/bin/env python3
"""Save one settled, timestamp-paired RGB-D and ROS TF registration bundle."""

import argparse
from collections import deque
from datetime import datetime, timezone
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


def timeout_message(last_rejection_reason, *, received_counts=None,
                    rejection_counts=None, parameter_state=None):
    detail = last_rejection_reason or "no complete candidate frame arrived"
    message = f"timed out waiting for a valid settled ROS registration frame; last rejection: {detail}"
    if parameter_state:
        message += f"; robot_description={parameter_state}"
    if received_counts:
        message += "; received=" + ", ".join(
            f"{key}={received_counts[key]}" for key in sorted(received_counts))
    if rejection_counts:
        message += "; rejections=" + ", ".join(
            f"{key}={rejection_counts[key]}" for key in sorted(rejection_counts))
    return message


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
                                max_span_rad=0.004, return_stamp=False):
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
    return (nearest_stamp, nearest) if return_stamp else nearest


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
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    arm = config.get("arm", {})
    result = {"arm_id": arm.get("id"),
              "station_config_sha256": hashlib.sha256(config_bytes).hexdigest()}
    calibration = arm.get("calibration", {})
    calibration_path = calibration.get("path")
    if calibration_path and Path(calibration_path).is_file():
        result["calibration_sha256"] = hashlib.sha256(Path(calibration_path).read_bytes()).hexdigest()
    if calibration.get("sha256"):
        result["configured_calibration_sha256"] = calibration["sha256"]
    return result


def validate_station_identity_snapshot(initial, current):
    """Reject station/calibration source changes between bind and bundle write."""
    keys = ("arm_id", "station_config_sha256", "calibration_sha256",
            "configured_calibration_sha256")
    changed = [key for key in keys if initial.get(key) != current.get(key)]
    if changed:
        raise ValueError("station identity source changed before capture write: " +
                         ", ".join(changed))


def _normalized_serial(value):
    if not isinstance(value, str):
        return None
    # librealsense launch accepts the serial with an optional leading underscore.
    return value.strip().lstrip("_") or None


def camera_identity_provenance(*, expected_serial, device_info, parameter_serial,
                               require_bound):
    """Bind the requested serial to both the driver parameter and DeviceInfo."""
    expected = _normalized_serial(expected_serial)
    observed = _normalized_serial(
        device_info.get("serial_number") if isinstance(device_info, dict) else None)
    configured = _normalized_serial(parameter_serial)
    firmware = (device_info.get("firmware_version")
                if isinstance(device_info, dict) else None)
    reasons = []
    if not expected:
        reasons.append("expected camera serial was not supplied")
    if not observed:
        reasons.append("camera DeviceInfo serial is unavailable")
    if not configured:
        reasons.append("camera serial_no parameter is unavailable")
    if expected and observed and expected != observed:
        reasons.append("camera DeviceInfo serial does not match the pinned serial")
    if expected and configured and expected != configured:
        reasons.append("camera serial_no parameter does not match the pinned serial")
    if not isinstance(firmware, str) or not firmware.strip():
        reasons.append("camera DeviceInfo firmware version is unavailable")
        firmware = None
    result = {"bound": not reasons, "expected_serial": expected,
              "serial_number": observed, "serial_no_parameter": configured,
              "firmware_version": firmware, "reason": "; ".join(reasons) or None}
    if require_bound and reasons:
        raise ValueError(result["reason"])
    return result


def _owner_timestamp(state):
    try:
        value = datetime.fromisoformat(str(state["timestamp_utc"]).replace("Z", "+00:00"))
        if value.tzinfo is None:
            return None
        return value.astimezone(timezone.utc).timestamp()
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def owner_state_provenance(samples, *, image_stamp, joint_positions,
                           expected_arm_id, expected_calibration_sha256,
                           now_utc, require_bound, max_pair_delta_s=0.04,
                           max_age_s=0.08, max_arm_joint_error_rad=0.01):
    """Select and validate an ArmSession acquisition paired to the color stamp.

    The five arm angles are compared using the actual published bridge mapping
    (owner degrees to ROS radians). The gripper value is deliberately excluded:
    the bridge's gripper joint is a visual proxy, not a qualified jaw encoder.
    """
    now = now_utc.astimezone(timezone.utc).timestamp()
    stamped = [(stamp, state) for state in samples
               if isinstance(state, dict) and (stamp := _owner_timestamp(state)) is not None]
    selected = min(stamped, key=lambda item: abs(item[0] - float(image_stamp))) if stamped else None
    reasons = []
    if selected is None:
        reasons.append("no timestamped ArmSession sample is available")
        state, acquisition_stamp = None, None
    else:
        acquisition_stamp, state = selected
        pair_delta = abs(acquisition_stamp - float(image_stamp))
        age = now - acquisition_stamp
        if pair_delta > max_pair_delta_s:
            reasons.append(f"ArmSession acquisition is not paired to image ({pair_delta:.3f}s)")
        if age < -0.05 or age > max_age_s:
            reasons.append(f"ArmSession acquisition is stale at capture ({age:.3f}s old)")
        if state.get("schema") != 1 or state.get("event") != "sample":
            reasons.append("ArmSession state is not a measured sample")
        if (state.get("arm_id") != expected_arm_id or
                state.get("calibration_sha256") != expected_calibration_sha256):
            reasons.append("ArmSession identity does not match station arm/calibration")
        raw = state.get("raw")
        if (not isinstance(raw, dict) or set(raw) != set(JOINTS) or
                any(type(value) is not int or not 0 <= value <= 4095
                    for value in raw.values())):
            reasons.append("ArmSession sample has no valid raw six-joint counts")
        pose = state.get("pose")
        if (not isinstance(pose, dict) or set(pose) != set(JOINTS) or
                any(type(value) not in (int, float) or not math.isfinite(value)
                    for value in pose.values())):
            reasons.append("ArmSession sample has no valid six-joint pose")
            arm_error = None
        elif (not isinstance(joint_positions, dict) or
              not set(JOINTS).issubset(joint_positions)):
            reasons.append("paired ROS joint state is incomplete")
            arm_error = None
        else:
            arm_error = max(abs(math.radians(float(pose[joint])) -
                                float(joint_positions[joint])) for joint in JOINTS[:-1])
            if arm_error > max_arm_joint_error_rad:
                reasons.append(f"ArmSession and ROS arm joints disagree ({arm_error:.4f} rad)")
        try:
            monotonic = float(state["monotonic"])
            if not math.isfinite(monotonic) or monotonic < 0:
                raise ValueError
        except (KeyError, TypeError, ValueError):
            monotonic = None
            reasons.append("ArmSession monotonic acquisition timestamp is invalid")
        pair_delta = abs(acquisition_stamp - float(image_stamp))
        age = now - acquisition_stamp
    result = {
        "bound": not reasons,
        "reason": "; ".join(reasons) or None,
        "arm_id": state.get("arm_id") if state else None,
        "calibration_sha256": state.get("calibration_sha256") if state else None,
        "timestamp_utc": state.get("timestamp_utc") if state else None,
        "monotonic": state.get("monotonic") if state else None,
        "raw": state.get("raw") if state else None,
        "pose": state.get("pose") if state else None,
        "acquisition_stamp": acquisition_stamp,
        "pair_delta_s": pair_delta if selected else None,
        "age_at_capture_s": age if selected else None,
        "arm_joint_max_error_rad": arm_error if selected else None,
        "physical_gripper_mapping_qualified": False,
    }
    if require_bound and reasons:
        raise ValueError(result["reason"])
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
        try:
            from realsense2_camera_msgs.srv import DeviceInfo
        except ImportError:
            DeviceInfo = None

        class _Node(Node):
            def __init__(self, output_directory, *, timeout_s, station_config="",
                         state_file=None, expected_camera_serial=None,
                         require_bound_provenance=False, camera_node="/camera",
                         device_info_service="", max_owner_age_s=0.08,
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
                self.station_identity = _station_identity(station_config)
                self.state_file = Path(state_file) if state_file else None
                self.expected_camera_serial = expected_camera_serial
                self.require_bound_provenance = bool(require_bound_provenance)
                self.max_owner_age_s = float(max_owner_age_s)
                self.owner_states = deque(maxlen=256)
                self._last_owner_monotonic = None
                self.owner_state_error = None
                self.device_info = None
                self.device_info_observed_at = None
                self.camera_serial_parameter = None
                self.camera_parameter_observed_at = None
                self.camera_identity_error = None
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
                self.last_rejection_reason = None
                self.received_counts = {"color": 0, "depth": 0, "camera_info": 0,
                                        "joint_states": 0, "tracking_status": 0}
                self.rejection_counts = {}
                self._counted_rejections = {}
                self.pending_since = {}
                self.robot_description = None
                self.description_client = AsyncParameterClient(
                    self, "/follower/robot_state_publisher")
                self.description_future = None
                self.camera_node = camera_node.rstrip("/") or "/camera"
                self.device_info_service = device_info_service or f"{self.camera_node}/device_info"
                self.device_info_type = DeviceInfo
                self.device_info_client = (self.create_client(DeviceInfo, self.device_info_service)
                                           if DeviceInfo is not None else None)
                self.device_info_future = None
                self.camera_parameter_client = AsyncParameterClient(self, self.camera_node)
                self.camera_parameter_future = None
                self._last_device_info_query = 0.0
                self._last_camera_parameter_query = 0.0
                self.buffer = Buffer()
                self.listener = TransformListener(self.buffer, self)
                self.create_subscription(Image, color_topic, self._color, qos_profile_sensor_data)
                self.create_subscription(Image, depth_topic, self._depth, qos_profile_sensor_data)
                self.create_subscription(CameraInfo, camera_info_topic, self._info, qos_profile_sensor_data)
                self.create_subscription(JointState, joint_states_topic, self._joint, qos_profile_sensor_data)
                self.create_subscription(String, tracking_topic, self._tracking, 10)
                self.timer = self.create_timer(0.02, self._capture_pending)

            def _color(self, message):
                self.received_counts["color"] += 1
                self.color.append(message)

            def _depth(self, message):
                self.received_counts["depth"] += 1
                self.depth.append(message)

            def _info(self, message):
                self.received_counts["camera_info"] += 1
                self.camera_info.append(message)

            def _joint(self, message):
                self.received_counts["joint_states"] += 1
                self.joints.append((_stamp_seconds(message.header.stamp),
                                    tuple(message.name), tuple(message.position)))

            def _tracking(self, message):
                self.received_counts["tracking_status"] += 1
                try:
                    self.tracking = json.loads(message.data)
                except (TypeError, json.JSONDecodeError):
                    self.tracking = None

            def _reject(self, reason, stamp=None, *, concrete=True):
                if stamp is not None:
                    reasons = self._counted_rejections.setdefault(stamp, set())
                    if reason in reasons:
                        return
                    reasons.add(reason)
                    if len(self._counted_rejections) > 128:
                        self._counted_rejections.pop(next(iter(self._counted_rejections)))
                self.rejection_counts[reason] = self.rejection_counts.get(reason, 0) + 1
                if concrete or self.last_rejection_reason is None:
                    self.last_rejection_reason = reason

            def _parameter_state(self):
                if self.robot_description is not None:
                    return "received"
                if self.description_future is None:
                    return ("service ready, request not sent" if
                            self.description_client.services_are_ready() else "service unavailable")
                return "response pending" if not self.description_future.done() else "response failed"

            def _closest(self, queue, stamp):
                return min(queue, key=lambda item: abs(_stamp_seconds(item.header.stamp) - stamp)) if queue else None

            def _capture_pending(self):
                if self.done:
                    return
                self._sample_owner_state()
                self._sample_camera_identity()
                if time.monotonic() - self.started > self.timeout_s:
                    self.failure = RuntimeError(timeout_message(
                        self.last_rejection_reason, received_counts=self.received_counts,
                        rejection_counts=self.rejection_counts,
                        parameter_state=self._parameter_state()))
                    self.done = True
                    return
                if self.robot_description is None:
                    if self.description_future is None:
                        if self.description_client.services_are_ready():
                            self.description_future = self.description_client.get_parameters(
                                ["robot_description"])
                        return
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
                    if depth is None:
                        self._reject("no matching aligned depth frame", stamp, concrete=False)
                        continue
                    if info is None:
                        self._reject("no matching CameraInfo frame", stamp, concrete=False)
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
                        joint_stamp, joint_positions = closest_settled_joint_state(
                            self.joints, frame_stamp, JOINTS, return_stamp=True)
                        now_utc = datetime.now(timezone.utc)
                        identity = camera_identity_provenance(
                            expected_serial=self.expected_camera_serial,
                            device_info=self.device_info,
                            parameter_serial=self.camera_serial_parameter,
                            require_bound=self.require_bound_provenance)
                        owner = owner_state_provenance(
                            self.owner_states, image_stamp=frame_stamp,
                            joint_positions=joint_positions,
                            expected_arm_id=self.station_identity.get("arm_id"),
                            expected_calibration_sha256=(
                                self.station_identity.get("configured_calibration_sha256") or
                                self.station_identity.get("calibration_sha256")),
                            now_utc=now_utc, require_bound=self.require_bound_provenance,
                            max_age_s=self.max_owner_age_s)
                        owner["ros_joint_state_stamp"] = joint_stamp
                        owner["ros_joint_state_delta_s"] = abs(joint_stamp - frame_stamp)
                        owner["state_file"] = str(self.state_file) if self.state_file else None
                        owner["state_read_error"] = self.owner_state_error
                        identity["device_info_service"] = self.device_info_service
                        identity["device_info_error"] = getattr(self, "device_info_error", None)
                        identity["camera_parameter_error"] = getattr(
                            self, "camera_parameter_error", None)
                        identity["device_info_observed_at_utc"] = (
                            self.device_info_observed_at.isoformat()
                            if self.device_info_observed_at else None)
                        identity["camera_parameter_observed_at_utc"] = (
                            self.camera_parameter_observed_at.isoformat()
                            if self.camera_parameter_observed_at else None)
                        device_age = (now_utc - self.device_info_observed_at).total_seconds() \
                            if self.device_info_observed_at else None
                        parameter_age = (now_utc - self.camera_parameter_observed_at).total_seconds() \
                            if self.camera_parameter_observed_at else None
                        identity["device_info_age_at_capture_s"] = device_age
                        identity["camera_parameter_age_at_capture_s"] = parameter_age
                        if self.require_bound_provenance and (
                                device_age is None or not 0 <= device_age <= 2.0 or
                                parameter_age is None or not 0 <= parameter_age <= 2.0):
                            raise ValueError("camera identity query is missing, future-dated, or stale")
                        station_hashes_match = (
                            bool(self.station_identity.get("station_config_sha256")) and
                            bool(self.station_identity.get("arm_id")) and
                            bool(self.station_identity.get("calibration_sha256")) and
                            self.station_identity.get("calibration_sha256") ==
                            self.station_identity.get("configured_calibration_sha256"))
                        if self.require_bound_provenance:
                            self._validate_station_hashes()
                        transforms = {
                            "world_from_camera": self._lookup(self.world_frame, self.optical_frame, frame_stamp),
                            "world_from_base": self._lookup(self.world_frame, self.base_frame, frame_stamp),
                            "base_from_gripper_frame": self._lookup(self.base_frame, self.gripper_frame, frame_stamp),
                        }
                    except ValueError as error:
                        self._reject(str(error), stamp)
                        continue
                    except Exception as error:
                        self._reject(
                            f"exact TF lookup at image stamp {frame_stamp:.9f} failed: {error}", stamp)
                        continue
                    self._write_capture(color, depth, info, frame_stamp,
                                        joint_positions, transforms, now,
                                        provenance={"camera": identity, "owner": owner,
                                                    "station_hashes_match": station_hashes_match,
                                                    "qualification_inputs_bound":
                                                    identity["bound"] and owner["bound"] and
                                                    station_hashes_match,
                                                    "physical_gripper_mapping_qualified": False})
                    self.done = True
                    self.timer.cancel()
                    return

            def _lookup(self, target, source, stamp):
                request_time = Time(seconds=stamp)
                return _transform_matrix(self.buffer.lookup_transform(target, source, request_time))

            def _sample_owner_state(self):
                if self.state_file is None:
                    return
                try:
                    state = json.loads(self.state_file.read_text(encoding="utf-8"))
                    monotonic = float(state["monotonic"])
                    if (state.get("event") == "sample" and math.isfinite(monotonic) and
                            (self._last_owner_monotonic is None or
                             monotonic > self._last_owner_monotonic)):
                        self.owner_states.append(state)
                        self._last_owner_monotonic = monotonic
                    self.owner_state_error = None
                except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
                    self.owner_state_error = str(error)

            def _sample_camera_identity(self):
                if self.device_info_client is not None:
                    elapsed = time.monotonic() - self._last_device_info_query
                    if (self.device_info_future is None and elapsed >= 1.0 and
                            self.device_info_client.service_is_ready()):
                        self.device_info_future = self.device_info_client.call_async(
                            self.device_info_type.Request())
                        self._last_device_info_query = time.monotonic()
                    if self.device_info_future is not None and self.device_info_future.done():
                        try:
                            response = self.device_info_future.result()
                            self.device_info = {
                                "serial_number": response.serial_number,
                                "firmware_version": response.firmware_version,
                                "device_name": response.device_name,
                                "usb_type_descriptor": response.usb_type_descriptor,
                                "physical_port": response.physical_port,
                            }
                            self.device_info_observed_at = datetime.now(timezone.utc)
                            self.device_info_error = None
                        except Exception as error:
                            self.device_info_error = str(error)
                        self.device_info_future = None
                else:
                    self.device_info_error = "realsense2_camera_msgs/DeviceInfo is unavailable"
                elapsed = time.monotonic() - self._last_camera_parameter_query
                if (self.camera_parameter_future is None and elapsed >= 1.0 and
                        self.camera_parameter_client.services_are_ready()):
                    self.camera_parameter_future = self.camera_parameter_client.get_parameters(
                        ["serial_no"])
                    self._last_camera_parameter_query = time.monotonic()
                if (self.camera_parameter_future is not None and
                        self.camera_parameter_future.done()):
                    try:
                        values = self.camera_parameter_future.result().values
                        if values and values[0].string_value:
                            self.camera_serial_parameter = values[0].string_value
                            self.camera_parameter_observed_at = datetime.now(timezone.utc)
                        self.camera_parameter_error = None
                    except Exception as error:
                        self.camera_parameter_error = str(error)
                    self.camera_parameter_future = None

            def _validate_station_hashes(self):
                identity = self.station_identity
                if not identity.get("station_config_sha256") or not identity.get("arm_id"):
                    raise ValueError("strict provenance requires station-config arm identity and hash")
                calibration = identity.get("calibration_sha256")
                configured = identity.get("configured_calibration_sha256")
                if not calibration or not configured:
                    raise ValueError("strict provenance requires both calibration file and configured hashes")
                if calibration != configured:
                    raise ValueError("station calibration file hash does not match configured hash")

            def _write_capture(self, color_msg, depth_msg, info, stamp,
                               joint_positions, transforms, now, provenance=None):
                try:
                    station_identity_now = _station_identity(self.station_config)
                    identity_unchanged = True
                    try:
                        validate_station_identity_snapshot(
                            self.station_identity, station_identity_now)
                    except ValueError:
                        identity_unchanged = False
                        if self.require_bound_provenance:
                            raise
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
                    capture_provenance = provenance or {
                        "camera": camera_identity_provenance(
                            expected_serial=self.expected_camera_serial,
                            device_info=self.device_info,
                            parameter_serial=self.camera_serial_parameter,
                            require_bound=False),
                        "owner": {"bound": False,
                                  "reason": self.owner_state_error or
                                  "no timestamp-paired ArmSession sample selected",
                                  "physical_gripper_mapping_qualified": False},
                        "station_hashes_match": False,
                        "qualification_inputs_bound": False,
                        "physical_gripper_mapping_qualified": False,
                    }
                    if not identity_unchanged:
                        capture_provenance["qualification_inputs_bound"] = False
                        capture_provenance["station_identity_changed_during_capture"] = True
                    if not cv2.imwrite(str(self.output_directory / "color.png"), color):
                        raise RuntimeError("could not save color.png")
                    np.save(self.output_directory / "depth.npy", depth, allow_pickle=False)
                    (self.output_directory / "robot.urdf").write_text(self.robot_description, encoding="utf-8")
                    capture = {
                        "schema": 2,
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
                        **self.station_identity,
                        "provenance": capture_provenance,
                        "qualified": False,
                        "station_identity_unchanged_at_write": identity_unchanged,
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
    parser.add_argument("--state-file", type=Path,
                        help="ArmSession state file to sample alongside the ROS joint states")
    parser.add_argument("--expected-camera-serial",
                        help="pinned RealSense serial required for bound provenance")
    parser.add_argument("--camera-node", default="/camera",
                        help="RealSense ROS node namespace used for DeviceInfo and serial_no")
    parser.add_argument("--device-info-service", default="",
                        help="DeviceInfo service (default: <camera-node>/device_info)")
    parser.add_argument("--max-owner-age", type=float, default=0.08)
    parser.add_argument("--require-bound-provenance", action="store_true",
                        help="reject captures lacking pinned camera, arm/calibration and time-paired owner identity")
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
    if not math.isfinite(args.max_owner_age) or not 0 < args.max_owner_age <= 0.08:
        parser.error("--max-owner-age must be positive and no greater than 0.08 seconds")
    if args.require_bound_provenance and not (
            args.station_config and args.state_file and args.expected_camera_serial):
        parser.error("--require-bound-provenance requires --station-config, --state-file, and --expected-camera-serial")
    args.output_directory = args.output_directory.resolve()
    args.output_directory.mkdir(parents=True, exist_ok=False)
    import rclpy
    rclpy.init(args=None)
    node = RegistrationCaptureNode(
        args.output_directory, timeout_s=args.timeout, station_config=args.station_config,
        state_file=args.state_file, expected_camera_serial=args.expected_camera_serial,
        require_bound_provenance=args.require_bound_provenance,
        camera_node=args.camera_node, device_info_service=args.device_info_service,
        max_owner_age_s=args.max_owner_age,
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
