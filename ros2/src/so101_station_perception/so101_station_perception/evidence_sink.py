"""Bounded rolling RGB-D evidence sink; does not open the camera device."""

import json
import math
import os
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def utc_iso(seconds):
    return datetime.fromtimestamp(float(seconds), timezone.utc).isoformat()


def atomic_bytes(path, content):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_npy(path, array):
    with tempfile.NamedTemporaryFile(prefix=Path(path).name + ".", suffix=".npy",
                                     dir=Path(path).parent, delete=False) as stream:
        temporary = stream.name
        np.save(stream, array, allow_pickle=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def bounded_write_period(requested_hz):
    """Return a write interval with a hard five-sample-per-second ceiling."""
    requested_hz = float(requested_hz)
    if not math.isfinite(requested_hz) or requested_hz <= 0:
        raise ValueError("max_write_hz must be positive finite")
    return 1.0 / min(5.0, requested_hz)


def validated_camera_metadata(info, image_shape):
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


def validate_depth_encoding(encoding, depth, configured_scale=0.001):
    if encoding == "16UC1" and depth.dtype == np.uint16:
        scale = float(configured_scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("16UC1 depth_scale_m must be positive and finite")
        return scale
    if encoding == "32FC1" and depth.dtype == np.float32:
        return 1.0
    raise ValueError(f"Depth encoding {encoding!r} does not match decoded array dtype")


def read_resident_state(path, max_bytes=8192):
    if not path:
        return None
    try:
        with open(path, "rb") as stream:
            payload = stream.read(max_bytes + 1)
        if len(payload) > max_bytes:
            return None
        state = json.loads(payload)
        if (state.get("schema") != 1 or state.get("event") != "sample"
                or state.get("arm_id") != "hs_rig01_follower"
                or not isinstance(state.get("raw"), dict)
                or not math.isfinite(float(state.get("monotonic")))):
            return None
        return state
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None


def arm_pair_record(state, frame_wall, frame_monotonic, now_monotonic):
    if state is None:
        return {"qualified_sync": False, "reason": "resident_state_unavailable"}
    try:
        arm_mono = float(state["monotonic"])
        arm_wall = datetime.fromisoformat(state["timestamp_utc"]).timestamp()
        if state.get("arm_id") != "hs_rig01_follower":
            raise ValueError("resident sample is not the Rig01 follower")
        raw = state["raw"]
        frame_delta = float(frame_monotonic) - arm_mono
        age = float(now_monotonic) - arm_mono
        valid_raw = all(name in raw for name in (
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"))
        clock_consistent = abs(arm_wall - arm_mono - (frame_wall - frame_monotonic)) < 2.0
        paired = valid_raw and clock_consistent and age >= -0.05 and age <= 0.5 and abs(frame_delta) <= 0.16
        return {"qualified_sync": bool(paired), "raw": raw,
                "monotonic": arm_mono, "delta_s": frame_delta,
                "timestamp_utc": state.get("timestamp_utc"),
                "reason": "paired_within_160ms" if paired else "pose_not_temporally_qualified"}
    except (KeyError, ValueError, TypeError, OverflowError):
        return {"qualified_sync": False, "reason": "resident_state_invalid"}


class EvidenceSink(Node):
    def __init__(self):
        super().__init__("so101_rgbd_evidence_sink")
        defaults = (
            ("output_folder", ""), ("resident_state_file", ""),
            ("color_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
            ("camera_info_topic", "/camera/color/camera_info"),
            ("camera_serial", "243322071783"), ("depth_scale_m", 0.001),
            ("max_pair_delta_s", 0.04), ("max_write_hz", 5.0),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        output_folder = self.get_parameter("output_folder").value
        if not output_folder:
            raise ValueError("output_folder is required")
        self.folder = Path(output_folder)
        if self.folder.exists() and any(self.folder.iterdir()):
            raise ValueError("Choose an empty output folder; evidence is never overwritten")
        self.folder.mkdir(parents=True, exist_ok=True)
        self.state_file = self.get_parameter("resident_state_file").value
        self.serial = self.get_parameter("camera_serial").value
        self.depth_scale = float(self.get_parameter("depth_scale_m").value)
        if not math.isfinite(self.depth_scale) or self.depth_scale <= 0:
            raise ValueError("depth_scale_m must be positive and finite")
        self.max_pair_delta = float(self.get_parameter("max_pair_delta_s").value)
        self.min_period = bounded_write_period(self.get_parameter("max_write_hz").value)
        self.last_write = 0.0
        self.index = 0
        self.bridge = CvBridge()
        self.info = None
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
        self.get_logger().info(f"Rolling RGB-D output enabled in empty folder {self.folder}")

    def _camera_info(self, msg):
        self.info = msg

    def _images(self, color_msg, depth_msg):
        now_mono = time.monotonic()
        if now_mono - self.last_write < self.min_period:
            return
        color_stamp = stamp_seconds(color_msg.header.stamp)
        depth_stamp = stamp_seconds(depth_msg.header.stamp)
        delta = depth_stamp - color_stamp
        info = self.info
        if (abs(delta) > self.max_pair_delta or info is None
                or abs(time.time() - color_stamp) > 0.5
                or color_msg.header.frame_id != depth_msg.header.frame_id
                or info.header.frame_id != color_msg.header.frame_id
                or abs(color_stamp - stamp_seconds(info.header.stamp)) > 0.5):
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if rgb.shape[:2] != depth.shape or depth.ndim != 2:
                raise ValueError("RGB and aligned depth dimensions differ")
            depth_scale = validate_depth_encoding(depth_msg.encoding, depth, self.depth_scale)
            intrinsics = validated_camera_metadata(info, rgb.shape[:2])
            local_wall, local_mono = time.time(), time.monotonic()
            clock_offset = local_mono - local_wall
            frame_mono = color_stamp + clock_offset
            state = read_resident_state(self.state_file)
            arm = arm_pair_record(state, color_stamp, frame_mono, local_mono)
            frame_metadata = {
                "schema": 1, "index": self.index,
                "utc": utc_iso(color_stamp), "timestamp_utc": utc_iso(color_stamp),
                "monotonic": frame_mono,
                "received_monotonic": local_mono,
                "color_stamp_ros_s": color_stamp, "depth_stamp_ros_s": depth_stamp,
                "rgb_depth_delta_s": delta,
                "paths": {"latest_color": "latest.png", "latest_depth": "latest-depth.npy"},
                "camera": {"serial": self.serial, "width": int(info.width),
                           "height": int(info.height), "frame_id": color_msg.header.frame_id,
                           "intrinsics": {"width": int(info.width), "height": int(info.height),
                                          **intrinsics,
                                          "model": info.distortion_model,
                                          "coeffs": [float(x) for x in info.d]},
                           "depth_scale_m": depth_scale},
                "aligned_to": "color", "depth_encoding": depth_msg.encoding,
                "intrinsics": {"width": int(info.width), "height": int(info.height),
                               **intrinsics,
                               "coeffs": [float(x) for x in info.d]},
                "depth_scale_m": depth_scale,
                "arm": arm,
            }
            encoded = json.dumps(frame_metadata, separators=(",", ":")).encode("utf-8")
            if len(encoded) > 8192:
                raise ValueError("latest metadata exceeds fixed 8 KiB bound")
            # Removing the manifest first makes concurrent readers fail closed
            # until both atomic file replacements and metadata publish complete.
            latest_json = self.folder / "latest.json"
            latest_json.unlink(missing_ok=True)
            if self.index == 0:
                capture = {"schema": 1, "device": {"serial_number": self.serial},
                           "camera": frame_metadata["camera"],
                           "intrinsics": frame_metadata["intrinsics"],
                           "depth_scale_m": depth_scale, "aligned_to": "color",
                           "depth_encoding": depth_msg.encoding, "source": "ROS 2 RealSense topics",
                           "host_clock": "system UTC + monotonic mapped from camera image stamp"}
                atomic_bytes(self.folder / "capture.json",
                             (json.dumps(capture, indent=2) + "\n").encode("utf-8"))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, png = cv2.imencode(".png", bgr)
            if not ok:
                raise RuntimeError("OpenCV could not encode latest RGB frame")
            atomic_bytes(self.folder / "latest.png", png.tobytes())
            atomic_npy(self.folder / "latest-depth.npy", depth)
            self._write_overview(rgb, depth, depth_scale)
            atomic_bytes(latest_json, encoded)
            self.index += 1
            self.last_write = local_mono
        except (ValueError, TypeError, cv2.error, OSError, RuntimeError) as error:
            self.get_logger().warning(f"RGB-D evidence sample rejected: {error}")

    def _write_overview(self, rgb, depth, depth_scale):
        color = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (320, 240),
                           interpolation=cv2.INTER_AREA)
        depth_m = depth.astype(np.float32) * depth_scale
        depth_u8 = np.clip((depth_m - 0.3) / (0.9 - 0.3) * 255.0, 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        depth_color[depth == 0] = 0
        depth_small = cv2.resize(depth_color, (320, 240), interpolation=cv2.INTER_NEAREST)
        overview = np.concatenate((color, depth_small), axis=1)
        ok, jpeg = cv2.imencode(".jpg", overview, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            atomic_bytes(self.folder / "overview.jpg", jpeg.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = EvidenceSink()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
