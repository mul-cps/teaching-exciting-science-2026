"""Rate-limited camera cloud alias for explicitly unqualified visualization."""

import copy
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


MAX_POINTS = 640 * 480


def quaternion_matrix(x, y, z, w):
    q = np.asarray([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("TF quaternion must be finite and nonzero")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def transform_xyz_data(data, *, height, width, point_step, row_step, fields,
                       is_bigendian, rotation, translation):
    """Transform XYZ fields in a bounded PointCloud2 byte buffer, vectorized."""
    count = int(height) * int(width)
    if count <= 0 or count > MAX_POINTS:
        raise ValueError("PointCloud2 dimensions exceed the fixed 640x480 bound")
    # The pinned RealSense unordered cloud retains its allocation row_step
    # after compacting valid points. A single tightly packed row is unambiguous.
    if height == 1 and len(data) == width * point_step:
        row_step = width * point_step
    if (point_step <= 0 or row_step < width * point_step
            or len(data) < height * row_step):
        raise ValueError("Malformed PointCloud2 layout")
    output_data = bytearray(data)
    by_name = {field.name: field for field in fields}
    if any(name not in by_name for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain x, y, and z fields")
    endian = ">" if is_bigendian else "<"
    coordinates = []
    for name in ("x", "y", "z"):
        field = by_name[name]
        if int(field.datatype) != 7 or int(field.count) != 1 or field.offset + 4 > point_step:
            raise ValueError("PointCloud2 XYZ fields must be scalar float32 values")
        coordinates.append(np.ndarray(
            (height, width), dtype=np.dtype(endian + "f4"), buffer=output_data,
            offset=int(field.offset), strides=(int(row_step), int(point_step))))
    xyz = np.stack(coordinates, axis=-1)
    flat = xyz.reshape(count, 3)
    finite = np.isfinite(flat).all(axis=1)
    matrix = np.asarray(rotation, dtype=float)
    offset = np.asarray(translation, dtype=float)
    if matrix.shape != (3, 3) or offset.shape != (3,) or not np.isfinite(matrix).all() or not np.isfinite(offset).all():
        raise ValueError("PointCloud2 transform must be finite SE(3) components")
    flat[finite] = flat[finite] @ matrix.T + offset
    for axis, coordinate in enumerate(coordinates):
        coordinate[...] = xyz[..., axis]
    return bytes(output_data)


class DiagnosticCloud(Node):
    """Publish points in the calibration camera alias, never as planning TF."""

    def __init__(self):
        super().__init__("so101_diagnostic_pointcloud")
        self.declare_parameter("input_topic", "/camera/depth/color/points")
        self.declare_parameter("output_topic", "/so101_perception/diagnostic/points")
        self.declare_parameter("color_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("diagnostic_frame", "so101_diag_camera_optical")
        self.declare_parameter("max_publish_hz", 5.0)
        requested_hz = float(self.get_parameter("max_publish_hz").value)
        if not math.isfinite(requested_hz) or requested_hz <= 0:
            raise ValueError("max_publish_hz must be positive finite")
        self.min_period = 1.0 / min(5.0, requested_hz)
        self.color_frame = self.get_parameter("color_optical_frame").value
        self.alias_frame = self.get_parameter("diagnostic_frame").value
        self.last_publish = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, 2)
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value, self._cloud, 2)

    def _cloud(self, message):
        now = time.monotonic()
        if now - self.last_publish < self.min_period:
            return
        try:
            if message.header.frame_id == self.color_frame:
                rotation, translation = np.eye(3), np.zeros(3)
            else:
                transform = self.buffer.lookup_transform(
                    self.color_frame, message.header.frame_id,
                    Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.02))
                t, q = transform.transform.translation, transform.transform.rotation
                rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
                translation = np.asarray([t.x, t.y, t.z], dtype=float)
            output = copy.copy(message)
            output.header = copy.copy(message.header)
            output.header.frame_id = self.alias_frame
            output.data = transform_xyz_data(
                message.data, height=message.height, width=message.width,
                point_step=message.point_step, row_step=message.row_step,
                fields=message.fields, is_bigendian=message.is_bigendian,
                rotation=rotation, translation=translation)
            if message.height == 1 and len(message.data) == message.width * message.point_step:
                output.row_step = message.width * message.point_step
            self.publisher.publish(output)
            self.last_publish = now
        except (TransformException, ValueError, TypeError) as error:
            self.get_logger().warning(f"Diagnostic cloud omitted: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
