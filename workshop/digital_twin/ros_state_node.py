"""Read-only ROS 2 adapter that snapshots the inputs also consumed by RViz."""
from __future__ import annotations

import argparse
import json
import os
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from .ros_state import RosTwinState, resolve_package_mesh_uris, transform_matrix


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RosStatePublisher(Node):
    """Serialize authoritative state to an atomic local file; never publishes ROS."""

    def __init__(self, output, root_frame="charuco_board", max_age_s=0.5):
        super().__init__("so101_web_twin_state")
        self.output = output
        self.root_frame = root_frame.lstrip("/")
        self.state = RosTwinState(max_age_s=float(max_age_s))
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
        self.create_timer(0.05, self._write)
        self.get_logger().info(
            "Passive ROS twin adapter: subscribing only; no camera, serial, or command interface")

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
            from sensor_msgs_py import point_cloud2
            points = point_cloud2.read_points_numpy(message, field_names=("x", "y", "z"))
            points = points.reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) > 12000:
                points = points[::max(1, len(points) // 12000)][:12000]
            self.state.point_cloud = {
                "frame_id": message.header.frame_id.lstrip("/"),
                "stamp_ns": _stamp_ns(message.header.stamp),
                "points_m": points.astype(float).tolist(),
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
        snapshot = self.state.snapshot(time.monotonic())
        snapshot["root_frame"] = self.root_frame
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--root-frame", default="charuco_board")
    parser.add_argument("--max-age", type=float, default=0.5)
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = RosStatePublisher(args.output, args.root_frame, args.max_age)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
