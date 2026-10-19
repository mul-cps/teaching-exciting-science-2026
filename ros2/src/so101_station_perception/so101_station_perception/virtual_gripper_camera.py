"""Optional synthetic ROS camera from observed depth points; no hardware writes."""
import json
import os
import math
import time
from pathlib import Path
import hashlib

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from so101_station_perception_tag_tracking.virtual_camera import (
    optical_projection, tool_from_virtual_optical, virtual_intrinsics,
    validate_pinch_overlay, project_pinch_overlay, JointOpeningHistory)
from so101_station_perception_tag_tracking.point_cloud_samples import sampled_cloud
from so101_station_perception_tag_tracking.gripper_mesh import load_gripper_visuals, LINKS
from ament_index_python.packages import get_package_share_directory
from .diagnostic_cloud import quaternion_matrix
from .virtual_camera_gpu import GpuPointCamera


def sampled_xyz(message, max_points=20000):
    """Compatibility XYZ accessor; production decodes XYZ/RGB together once."""
    return sampled_cloud(message, max_points)[0]


class VirtualGripperCamera(Node):
    """Single-threaded EGL/ROS owner; depth stamp is the acquisition time."""

    def __init__(self):
        super().__init__("virtual_gripper_camera")
        defaults = {"enabled": False, "input_topic": "/camera/depth/color/points",
            "output_namespace": "/so101/synthetic_gripper_camera", "width": 320,
            "height": 240, "max_hz": 3., "max_points": 20000,
            "vertical_fov_deg": 60., "forward_axis": "+Z", "backoff_m": .04,
            "near_m": .01, "far_m": 2., "max_age_s": .5, "pinch_artifact": "",
            "publish_annotation": False, "joint_states_topic": "/follower/joint_states",
            "render_gripper_mesh": True, "robot_description_topic": "/follower/robot_description"}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.renderer = None
        self.status_publisher = self.create_publisher(String, p("output_namespace") + "/status", 2)
        if not p("enabled"):
            self._status("disabled")
            return
        self.width, self.height, self.max_points = int(p("width")), int(p("height")), int(p("max_points"))
        rate, self.max_age = float(p("max_hz")), float(p("max_age_s"))
        if not math.isfinite(rate) or not 0 < rate <= 3 or not 0 < self.max_age <= .5:
            raise ValueError("synthetic rate must be <=3 Hz, acquisition age <=0.5 s")
        self.period, self.last_attempt, self.last_stamp = 1 / rate, -float("inf"), -1
        self.mount = tool_from_virtual_optical(str(p("forward_axis")), float(p("backoff_m")))
        self.optical_from_tool = np.linalg.inv(self.mount)
        self.mount_quat = Rotation.from_matrix(self.mount[:3, :3]).as_quat()
        self.tool_frame, self.optical_frame = "gripper_frame_link", "synthetic_gripper_camera_optical"
        self.k = virtual_intrinsics(self.width, self.height, float(p("vertical_fov_deg")))
        self.near, self.far = float(p("near_m")), float(p("far_m"))
        self.projection = optical_projection(self.k, self.width, self.height, self.near, self.far)
        self.pinch, self.pinch_sha = None, None
        self.opening_history = JointOpeningHistory()
        if p("pinch_artifact"):
            raw = Path(p("pinch_artifact")).read_bytes()
            self.pinch = validate_pinch_overlay(json.loads(raw))
            self.pinch_sha = hashlib.sha256(raw).hexdigest()
        self.publish_annotation = bool(p("publish_annotation"))
        self.renderer = GpuPointCamera(self.width, self.height, self.max_points)
        self.render_gripper_mesh = bool(p("render_gripper_mesh"))
        self.mesh_description, self.meshes, self.mesh_provenance = None, [], {"state": "waiting_for_description"}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_publisher = TransformBroadcaster(self)
        namespace = p("output_namespace")
        self.images = self.create_publisher(Image, namespace + "/image_raw", qos_profile_sensor_data)
        self.infos = self.create_publisher(CameraInfo, namespace + "/camera_info", qos_profile_sensor_data)
        self.annotated = self.create_publisher(Image, namespace + "/image_annotated", qos_profile_sensor_data)
        self.annotation_masks = self.create_publisher(Image, namespace + "/annotation_mask", qos_profile_sensor_data)
        self.scenes = self.create_publisher(Image, namespace + "/image_scene", qos_profile_sensor_data)
        self.provenance_masks = self.create_publisher(Image, namespace + "/provenance_mask", qos_profile_sensor_data)
        self.description_subscription = self.create_subscription(String, p("robot_description_topic"),
            self._description, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.joints = self.create_subscription(JointState, p("joint_states_topic"),
            self._joints, qos_profile_sensor_data)
        self.subscription = self.create_subscription(PointCloud2, p("input_topic"),
            self._cloud, qos_profile_sensor_data)
        self.last_valid_stamp_ns = self.last_valid_tool_tf_stamp_ns = None
        self.last_valid_monotonic = None
        self.display_valid, self.invalid_frame_sent = False, False
        self.invalid_reason = "waiting_for_first_render"
        self.watchdog = self.create_timer(.2, self._watchdog)
        self._status("ready", renderer=self.renderer.renderer)

    def _description(self, message):
        if not self.render_gripper_mesh or message.data == self.mesh_description:
            return
        self.mesh_description = message.data
        try:
            meshes, provenance = load_gripper_visuals(message.data, get_package_share_directory)
            self.renderer.set_meshes(meshes)
            self.meshes, self.mesh_provenance = meshes, {"state": "loaded", **provenance}
        except (OSError, ValueError, RuntimeError) as error:
            self.renderer.set_meshes([])
            self.meshes, self.mesh_provenance = [], {"state": "rejected_description", "reason": str(error)}

    def _joints(self, message):
        if "gripper" not in message.name:
            return
        index = message.name.index("gripper")
        if index >= len(message.position) or not math.isfinite(message.position[index]):
            return
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.opening_history.add(stamp_ns, float(message.position[index]),
                                 self.get_clock().now().nanoseconds)

    def _annotation(self, image, rgba, stamp_ns):
        import cv2
        uv, reason = None, "pinch_unconfigured"
        now_ns = self.get_clock().now().nanoseconds
        joint_stamp, angle = (self.opening_history.nearest(stamp_ns, now_ns)
                              or (None, None))
        if self.pinch is not None:
            uv, reason = project_pinch_overlay(self.pinch, self.mount, self.k,
                self.width, self.height, opening_angle_rad=angle, joint_stamp_ns=joint_stamp,
                acquisition_stamp_ns=stamp_ns, now_ns=now_ns,
                near=self.near, far=self.far)
        rgb = rgba[:, :, :3].copy()
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        if uv is not None:
            x, y = np.rint(uv).astype(int)
            x, y = min(self.width - 1, x), min(self.height - 1, y)
            cv2.drawMarker(mask, (int(x), int(y)), 255, cv2.MARKER_CROSS, 13, 1)
        label = "NOMINAL / UNCALIBRATED" if uv is not None else "NOMINAL CENTER UNAVAILABLE"
        cv2.putText(mask, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, .35, 255, 1)
        rgb[mask != 0] = [255, 190, 30]
        annotated, annotation_mask = Image(), Image()
        for output in (annotated, annotation_mask):
            output.header = image.header
            output.width, output.height = self.width, self.height
        annotated.encoding, annotated.step, annotated.data = "rgb8", self.width * 3, rgb.tobytes()
        annotation_mask.encoding, annotation_mask.step, annotation_mask.data = "mono8", self.width, mask.tobytes()
        self.annotated.publish(annotated)
        self.annotation_masks.publish(annotation_mask)
        return {"state": reason, "pixel_xy": None if uv is None else uv.tolist(),
                "qualified": False, "artifact_sha256": self.pinch_sha,
                "annotation_is_depth": False, "joint_stamp_ns": joint_stamp,
                "joint_cloud_delta_ns": None if joint_stamp is None else joint_stamp - stamp_ns}

    def _mark_valid(self, stamp_ns, tool_tf_stamp_ns, received_monotonic):
        self.last_valid_stamp_ns = stamp_ns
        self.last_valid_tool_tf_stamp_ns = tool_tf_stamp_ns
        self.last_valid_monotonic = received_monotonic
        self.display_valid, self.invalid_frame_sent = True, False
        self.invalid_reason = None

    def _invalidate(self, reason):
        """Replace the visible scene once; never renew acquisition/TF/CameraInfo."""
        self.display_valid, self.invalid_reason = False, reason
        if not self.invalid_frame_sent:
            import cv2
            stamp_ns = self.last_valid_stamp_ns or 0
            rgba = bytes(self.width * self.height * 4)
            mask = bytes(self.width * self.height)
            rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            cv2.putText(rgb, "STALE / INVALID", (8, min(32, self.height - 1)),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 170, 80), 1, cv2.LINE_AA)
            for publisher, encoding, step, pixels in (
                    (self.images, "rgba8", self.width * 4, rgba),
                    (self.scenes, "rgba8", self.width * 4, rgba),
                    (self.annotated, "rgb8", self.width * 3, rgb.tobytes()),
                    (self.annotation_masks, "mono8", self.width, mask),
                    (self.provenance_masks, "mono8", self.width, mask)):
                image = Image()
                image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                image.header.frame_id = self.optical_frame
                image.width, image.height = self.width, self.height
                image.encoding, image.is_bigendian, image.step, image.data = encoding, 0, step, pixels
                publisher.publish(image)
            self.invalid_frame_sent = True
        self._status("invalid", reason=reason)

    def _watchdog(self):
        now_ns, now = self.get_clock().now().nanoseconds, time.monotonic()
        if self.last_valid_stamp_ns is None:
            self._invalidate("waiting_for_first_render")
            return
        age = (now_ns - self.last_valid_stamp_ns) / 1e9
        receipt_age = now - self.last_valid_monotonic
        if (not math.isfinite(receipt_age) or not 0 <= age <= self.max_age
                or not 0 <= receipt_age <= self.max_age):
            self._invalidate("last_render_expired_or_clock_invalid")
        elif not self.display_valid:
            self._status("invalid", reason=self.invalid_reason)

    def _status(self, state, **extra):
        message = String()
        stamp_ns = getattr(self, "last_valid_stamp_ns", None)
        now_ns = self.get_clock().now().nanoseconds
        message.data = json.dumps({"state": state, "synthetic": True,
            "visual_valid": getattr(self, "display_valid", False),
            "checked_ros_ns": now_ns,
            "last_valid_acquisition_stamp_ns": stamp_ns,
            "last_valid_tool_tf_stamp_ns": getattr(self, "last_valid_tool_tf_stamp_ns", None),
            "last_valid_acquisition_age_s": None if stamp_ns is None else (now_ns - stamp_ns) / 1e9,
            "calibration_qualified": False, "unknown_pixels": "image_raw alpha=0; image_scene uses separate provenance_mask",
            "observed_samples": "one-pixel measured RGB when available; no hole filling", **extra})
        self.status_publisher.publish(message)

    def _cloud(self, message):
        start = time.monotonic()
        if start - self.last_attempt < self.period:
            return
        self.last_attempt = start
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= self.last_stamp or stamp_ns <= 0 or not 0 <= age <= self.max_age:
            self._invalidate("rejected_stamp")
            return
        try:
            # Exact acquisition-time TF, never latest/time-zero. Interpolated TF
            # must exist at this stamp; timeout bounded to 20 ms.
            transform = self.buffer.lookup_transform(self.tool_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp), timeout=Duration(seconds=.02))
            t, q = transform.transform.translation, transform.transform.rotation
            tool_from_cloud = np.eye(4)
            tool_from_cloud[:3, :3] = quaternion_matrix(q.x, q.y, q.z, q.w)
            tool_from_cloud[:3, 3] = [t.x, t.y, t.z]
            points, colors, color_source = sampled_cloud(message, self.max_points)
            mesh_matrices = {}
            mesh_state = dict(self.mesh_provenance) if self.render_gripper_mesh else {"state": "disabled"}
            if self.render_gripper_mesh and self.meshes:
                try:
                    link_poses = {}
                    for link in LINKS:
                        link_tf = self.buffer.lookup_transform(self.tool_frame, link,
                            Time.from_msg(message.header.stamp), timeout=Duration(seconds=.02))
                        lt, lq = link_tf.transform.translation, link_tf.transform.rotation
                        pose = np.eye(4)
                        pose[:3, :3] = quaternion_matrix(lq.x, lq.y, lq.z, lq.w)
                        pose[:3, 3] = [lt.x, lt.y, lt.z]
                        link_poses[link] = pose
                    mesh_matrices = {mesh["name"]: self.projection @ self.optical_from_tool @ link_poses[mesh["link"]]
                                     for mesh in self.meshes}
                except TransformException as error:
                    self._invalidate("missing_mesh_acquisition_tf:" + str(error)[:512])
                    return
            optical_from_cloud = self.optical_from_tool @ tool_from_cloud
            render_start = time.monotonic()
            layers = self.renderer.render_layers(points, self.projection @ optical_from_cloud,
                                                  colors=colors, mesh_matrices=mesh_matrices)
            rgba, scene, provenance = layers["raw"], layers["scene"], layers["provenance"]
            readback_ms = (time.monotonic() - render_start) * 1000
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if not 0 <= age <= self.max_age:
                self._invalidate("rejected_after_render")
                return
            image = Image()
            image.header.stamp = message.header.stamp
            image.header.frame_id = self.optical_frame
            image.width, image.height = self.width, self.height
            image.encoding, image.is_bigendian, image.step = "rgba8", 0, self.width * 4
            image.data = rgba.tobytes()
            info = CameraInfo()
            info.header = image.header
            info.width, info.height = self.width, self.height
            info.distortion_model, info.d = "plumb_bob", [0.] * 5
            info.k = self.k.reshape(-1).tolist()
            info.r = np.eye(3).reshape(-1).tolist()
            info.p = np.column_stack((self.k, np.zeros(3))).reshape(-1).tolist()
            mount_tf = TransformStamped()
            mount_tf.header.stamp, mount_tf.header.frame_id = message.header.stamp, self.tool_frame
            mount_tf.child_frame_id = self.optical_frame
            mt = mount_tf.transform.translation
            mt.x, mt.y, mt.z = map(float, self.mount[:3, 3])
            mq = mount_tf.transform.rotation
            mq.x, mq.y, mq.z, mq.w = map(float, self.mount_quat)
            self.tf_publisher.sendTransform(mount_tf)
            self.infos.publish(info)
            self.images.publish(image)
            scene_image, provenance_image = Image(), Image()
            for output in (scene_image, provenance_image):
                output.header = image.header
                output.width, output.height = self.width, self.height
            scene_image.encoding, scene_image.step, scene_image.data = "rgba8", self.width * 4, scene.tobytes()
            provenance_image.encoding, provenance_image.step, provenance_image.data = "mono8", self.width, provenance.tobytes()
            self.scenes.publish(scene_image)
            self.provenance_masks.publish(provenance_image)
            self.last_stamp = stamp_ns
            self._mark_valid(stamp_ns, transform.header.stamp.sec * 10**9 +
                             transform.header.stamp.nanosec, start)
            if self.publish_annotation:
                annotation = self._annotation(image, scene, stamp_ns)
            else:
                # Keep the diagnostic display restorable without enabling pinch annotations.
                display = Image()
                display.header = image.header
                display.width, display.height = self.width, self.height
                display.encoding, display.is_bigendian, display.step = "rgb8", 0, self.width * 3
                display.data = scene[:, :, :3].tobytes()
                self.annotated.publish(display)
                annotation = {"state": "disabled"}
            self._status("published", annotation=annotation, color_source=color_source,
                virtual_camera={"width": self.width, "height": self.height,
                    "vertical_fov_deg": math.degrees(2 * math.atan(self.height / (2 * self.k[1, 1]))),
                    "horizontal_fov_deg": math.degrees(2 * math.atan(self.width / (2 * self.k[0, 0]))),
                    "tool_from_optical": self.mount.tolist()},
                model=mesh_state, model_visuals_rendered=len(mesh_matrices),
                provenance_values={"unknown": 0, "observed": 1, "nominal_model": 2}, stamp_ns=stamp_ns, age_s=age, points=len(points),
                gpu_render_readback_ms=readback_ms, callback_ms=(time.monotonic()-start)*1000,
                readback_bytes=self.renderer.readback_bytes, renderer=self.renderer.renderer)
        except (TransformException, ValueError, RuntimeError) as error:
            self._invalidate("render_or_acquisition_tf_failed:" + str(error)[:512])

    def destroy_node(self):
        if hasattr(self, "watchdog"):
            self.watchdog.cancel()
        if self.renderer is not None:
            self.renderer.close()
        return super().destroy_node()


def main(args=None):
    if int(os.environ.get("ROS_DOMAIN_ID", "76")) == 0:
        raise ValueError("synthetic camera requires a nonzero ROS_DOMAIN_ID")
    os.environ.setdefault("ROS_DOMAIN_ID", "76")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualGripperCamera()
        rclpy.spin(node)
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
