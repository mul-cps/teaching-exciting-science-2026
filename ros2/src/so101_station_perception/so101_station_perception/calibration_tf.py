"""Publish directed camera calibration without replacing vendor camera TF."""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .calibration import load_calibration_frames, matrix_quaternion, rigid_matrix


def _transform_matrix(transform):
    t = transform.translation
    q = np.asarray([transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("RealSense TF quaternion must be finite and nonzero")
    x, y, z, w = q / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return rigid_matrix(matrix, "camera_link_from_optical")


def compose_base_from_camera_link(base_from_optical, camera_link_from_optical):
    """Attach calibrated optical pose at camera_link using vendor optical TF."""
    return rigid_matrix(
        rigid_matrix(base_from_optical, "base_from_camera_optical")
        @ np.linalg.inv(rigid_matrix(camera_link_from_optical, "camera_link_from_optical")),
        "base_from_camera_link")


def _message(parent, child, matrix):
    matrix = rigid_matrix(matrix)
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(matrix[0, 3])
    msg.transform.translation.y = float(matrix[1, 3])
    msg.transform.translation.z = float(matrix[2, 3])
    qx, qy, qz, qw = matrix_quaternion(matrix)
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


class CalibrationTfPublisher(Node):
    def __init__(self):
        super().__init__("so101_calibration_tf")
        for name, default in (
            ("camera_target_calibration", ""), ("camera_arm_calibration", ""),
            ("qualified_mode", False), ("base_frame", "follower/base_link"),
            ("optical_frame", "camera_color_optical_frame"),
            ("camera_link_frame", "camera_link"),
        ):
            self.declare_parameter(name, default)
        self.base_frame = self.get_parameter("base_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.camera_link_frame = self.get_parameter("camera_link_frame").value
        transforms, _ = load_calibration_frames(
            self.get_parameter("camera_target_calibration").value,
            self.get_parameter("camera_arm_calibration").value,
            qualified_mode=self.get_parameter("qualified_mode").value,
            base_frame=self.base_frame,
            optical_frame=self.optical_frame,
        )
        self.broadcaster = StaticTransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending_arm = []
        immediate = []
        for item in transforms:
            if item.get("qualified") is True and item.get("kind") == "arm":
                self.pending_arm.append(item)
                continue
            message = _message(item["parent"], item["child"], item["matrix"])
            message.header.stamp = self.get_clock().now().to_msg()
            immediate.append(message)
            label = "qualified" if item["qualified"] else "UNQUALIFIED DIAGNOSTIC"
            if item.get("kind") == "anchor":
                label = "VISUAL-ONLY identity anchor"
            self.get_logger().warning(
                f"Publishing {label} transform {item['parent']} -> {item['child']} "
                f"from {item['source']}"
            )
        if immediate:
            self.broadcaster.sendTransform(immediate)
        self.pending_timer = None
        self.last_missing_log = 0.0
        if self.pending_arm:
            self.pending_timer = self.create_timer(0.1, self._publish_qualified_camera_link)

    def _publish_qualified_camera_link(self):
        remaining = []
        for item in self.pending_arm:
            try:
                # Calibration is base_from_color_optical. Compose the inverse
                # official camera_link_from_optical edge to preserve the full
                # vendor tree and avoid assigning optical frame a second parent.
                camera_link_from_optical = _transform_matrix(self.buffer.lookup_transform(
                    self.camera_link_frame, self.optical_frame, Time()))
                base_from_link = compose_base_from_camera_link(
                    item["matrix"], camera_link_from_optical)
                message = _message(self.base_frame, self.camera_link_frame, base_from_link)
                message.header.stamp = self.get_clock().now().to_msg()
                self.broadcaster.sendTransform(message)
                self.get_logger().warning(
                    f"Publishing qualified transform {self.base_frame} -> {self.camera_link_frame} "
                    f"composed from official RealSense TF and {item['source']}"
                )
            except (TransformException, ValueError, np.linalg.LinAlgError) as error:
                remaining.append(item)
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self.last_missing_log >= 2.0:
                    self.get_logger().warning(
                        f"Waiting for official {self.camera_link_frame} -> {self.optical_frame} TF: {error}")
                    self.last_missing_log = now
        self.pending_arm = remaining
        if not remaining and self.pending_timer is not None:
            self.destroy_timer(self.pending_timer)
            self.pending_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
