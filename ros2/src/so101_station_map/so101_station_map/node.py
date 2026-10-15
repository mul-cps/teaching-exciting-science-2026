"""ROS 2 node publishing an expiring occupied RGB-D surface as one RViz marker."""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformException, TransformListener

from .surface_map import SurfaceVoxelMap, cloud_xyz_chunks


def transform_matrix(transform):
    tr = transform.transform
    q = np.array([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("TF quaternion is invalid")
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    translation = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
    if not np.isfinite(translation).all():
        raise ValueError("TF translation is invalid")
    return rotation, translation


class SurfaceVoxelMapNode(Node):
    def __init__(self):
        super().__init__("so101_surface_voxel_map")
        defaults = {
            "input_topic": "/camera/depth/color/points",
            "map_topic": "/so101_map/occupied_surface",
            "status_topic": "/so101_map/status",
            "target_frame": "charuco_board",
            "max_range_m": 1.2,
            "resolution_m": 0.01,
            "ttl_s": 0.8,
            "max_voxels": 60000,
            "publish_hz": 2.0,
            "max_cloud_age_s": 0.5,
            "workspace_x_min": -0.6,
            "workspace_x_max": 0.6,
            "workspace_y_min": -0.6,
            "workspace_y_max": 0.6,
            "workspace_z_min": -0.05,
            "workspace_z_max": 0.8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.target_frame = str(p("target_frame"))
        self.max_range = float(p("max_range_m"))
        self.max_age = float(p("max_cloud_age_s"))
        self.publish_period = 1.0 / float(p("publish_hz"))
        if not math.isfinite(self.max_range) or self.max_range <= 0:
            raise ValueError("max_range_m must be positive and finite")
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError("max_cloud_age_s must be positive and finite")
        if not math.isfinite(self.publish_period) or self.publish_period < 0.5:
            raise ValueError("publish_hz must be in (0, 2]")
        bounds = tuple((p(f"workspace_{axis}_min"), p(f"workspace_{axis}_max"))
                       for axis in "xyz")
        self.map = SurfaceVoxelMap(p("resolution_m"), p("ttl_s"), p("max_voxels"), bounds)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.marker_pub = self.create_publisher(Marker, str(p("map_topic")), 1)
        self.status_pub = self.create_publisher(String, str(p("status_topic")), 10)
        self.subscription = self.create_subscription(
            PointCloud2, str(p("input_topic")), self._cloud, qos_profile_sensor_data)
        self.last_cloud_mono = 0.0
        self.last_output_stamp = None
        self.timer = self.create_timer(self.publish_period, self._publish)

    def _cloud(self, message):
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = now_ros - stamp
        if (not message.header.frame_id or stamp <= 0 or age < -0.05
                or age > self.max_age):
            self._status("rejected_stale_or_invalid_stamp", stamp=stamp, age_s=age)
            return
        mono = time.monotonic()
        if mono - self.last_cloud_mono < self.publish_period:
            return
        self.last_cloud_mono = mono
        try:
            transform = self.buffer.lookup_transform(
                self.target_frame, message.header.frame_id, Time.from_msg(message.header.stamp))
            rotation, translation = transform_matrix(transform)
            self.map.last_input_points = 0
            self.map.last_accepted_points = 0
            total_points = 0
            for camera_points in cloud_xyz_chunks(message):
                total_points += len(camera_points)
                finite = np.isfinite(camera_points).all(axis=1)
                in_range = np.einsum("ij,ij->i", camera_points, camera_points) <= self.max_range ** 2
                candidates = camera_points[finite & in_range]
                if len(candidates):
                    target_points = candidates @ rotation.T + translation
                    self.map.integrate(target_points, now_ros)
            self.map.last_input_points = total_points
            self.map.last_status = "tracking"
            self.last_output_stamp = message.header.stamp
            self._status("tracking", stamp=stamp, age_s=age)
        except (TransformException, ValueError, TypeError, OverflowError) as error:
            self.map.last_status = "rejected_cloud"
            self._status("rejected_cloud", stamp=stamp, reason=str(error))

    def _status(self, state, **details):
        msg = String()
        msg.data = json.dumps({
            "state": state,
            "target_frame": self.target_frame,
            "occupied_surface_voxels": len(self.map.cells),
            "input_points": self.map.last_input_points,
            "cropped_points": self.map.last_accepted_points,
            "max_voxels": self.map.max_voxels,
            "ttl_s": self.map.ttl_s,
            **details,
        }, sort_keys=True)
        self.status_pub.publish(msg)

    def _publish(self):
        now = self.get_clock().now()
        self.map.clear_expired(now.nanoseconds * 1e-9)
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = now.to_msg()
        marker.ns = "observed_rgbd_surface"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self.map.resolution
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.18, 0.72, 0.95, 0.9
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in self.map.centers()]
        self.marker_pub.publish(marker)
        self._status(self.map.last_status, stamp=(
            None if self.last_output_stamp is None else
            self.last_output_stamp.sec + self.last_output_stamp.nanosec * 1e-9))


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceVoxelMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
