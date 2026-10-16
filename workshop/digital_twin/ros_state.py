"""Pure state rules shared by the ROS snapshot bridge and the web twin.

ROS timestamps are nanoseconds. Positions stay in metres and rotations are
xyzw quaternions; no optical-frame or viewer-axis conversion happens here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
import re
from typing import Any

import numpy as np


_PACKAGE_MESH_URI = re.compile(r"package://([^/\s\"'<>]+)/([^\s\"'<>]+)")


def resolve_package_mesh_uris(robot_description: str, package_share_resolver) -> str:
    """Resolve package mesh URIs without allowing lexical traversal from share."""
    def replace(match):
        package, relative_text = match.groups()
        relative = PurePosixPath(relative_text)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"package mesh URI escapes package share: {match.group(0)}")
        # Keep the install-prefix path: colcon commonly places legitimate
        # package resources behind symlinks to the source/install tree.
        share = Path(package_share_resolver(package)).absolute()
        asset = share.joinpath(*relative.parts)
        try:
            asset.relative_to(share)
        except ValueError as error:
            raise ValueError(f"package mesh URI escapes package share: {match.group(0)}") from error
        return str(asset)

    return _PACKAGE_MESH_URI.sub(replace, str(robot_description))


@dataclass
class StampedValue:
    stamp_ns: int
    received_monotonic: float
    value: Any


@dataclass
class RosTwinState:
    """Reject reordered inputs and expose an explicit stale/synchrony verdict."""

    max_age_s: float = 0.5
    max_joint_tf_delta_s: float = 0.1
    transforms: dict[str, StampedValue] = field(default_factory=dict)
    joints: StampedValue | None = None
    robot_description: str | None = None
    point_cloud: dict[str, Any] | None = None
    objects: dict[str, StampedValue] = field(default_factory=dict)

    def update_transform(self, parent: str, child: str, stamp_ns: int,
                         received_monotonic: float, matrix: Any, static=False) -> bool:
        key = child.lstrip("/")
        stamp_ns = int(stamp_ns)
        previous = self.transforms.get(key)
        if previous is not None and not static and stamp_ns < previous.stamp_ns:
            return False
        self.transforms[key] = StampedValue(stamp_ns, float(received_monotonic), {
            "parent": parent.lstrip("/"), "child": key,
            "matrix": np.asarray(matrix, dtype=float).reshape(4, 4).tolist(),
            "static": bool(static), "stamp_ns": stamp_ns,
            "received_monotonic": float(received_monotonic),
        })
        return True

    def update_joints(self, stamp_ns: int, received_monotonic: float,
                      names: list[str], positions: list[float]) -> bool:
        stamp_ns = int(stamp_ns)
        if self.joints is not None and stamp_ns < self.joints.stamp_ns:
            return False
        if len(names) != len(positions) or not np.all(np.isfinite(positions)):
            return False
        self.joints = StampedValue(stamp_ns, float(received_monotonic),
                                   dict(zip(names, map(float, positions))))
        return True

    def update_object(self, name: str, stamp_ns: int, received_monotonic: float,
                      value: dict[str, Any]) -> bool:
        """Store one timestamped marker/grasp observation in snapshot form."""
        name = str(name)
        stamp_ns = int(stamp_ns)
        received_monotonic = float(received_monotonic)
        if not name or not np.isfinite(received_monotonic) or not isinstance(value, dict):
            return False
        previous = self.objects.get(name)
        if previous is not None and stamp_ns < previous.stamp_ns:
            return False
        self.objects[name] = StampedValue(stamp_ns, received_monotonic, value)
        return True

    def snapshot(self, now_monotonic: float) -> dict[str, Any]:
        now = float(now_monotonic)
        dynamic = {name: item for name, item in self.transforms.items()
                   if not item.value["static"]}
        fresh_tf = {name: item for name, item in dynamic.items()
                    if now - item.received_monotonic <= self.max_age_s}
        joints_fresh = (self.joints is not None
                        and now - self.joints.received_monotonic <= self.max_age_s)
        base_tf = self.transforms.get("base_link")
        base_tf_fresh = bool(base_tf is not None and (
            base_tf.value["static"] or now - base_tf.received_monotonic <= self.max_age_s))
        def is_arm_descendant(name):
            current = name
            seen = set()
            while current in self.transforms and current not in seen:
                seen.add(current)
                parent = self.transforms[current].value["parent"]
                if parent == "base_link":
                    return True
                current = parent
            return name == "base_link"

        arm_tfs = {name: item for name, item in dynamic.items()
                   if is_arm_descendant(name)}
        arm_deltas = ([] if self.joints is None else [
            abs(self.joints.stamp_ns - item.stamp_ns) / 1e9
            for item in arm_tfs.values()])
        delta_s = max(arm_deltas) if arm_deltas else None
        sync_basis = ("dynamic_arm_tf_stamps_with_static_base"
                      if base_tf is not None and base_tf.value["static"]
                      else "dynamic_arm_tf_stamps")
        tf_synchronized = bool(delta_s is not None
                               and delta_s <= self.max_joint_tf_delta_s)
        synchronized = bool(joints_fresh and base_tf_fresh and tf_synchronized)
        object_ages = {name: max(0., now - item.received_monotonic)
                       for name, item in self.objects.items()}
        return {
            "mode": "ros_passive_twin",
            "source": "ROS 2 /follower/robot_description, /follower/joint_states, /tf, /tf_static",
            "units": {"translation": "m", "joint_position": "rad", "quaternion": "xyzw"},
            "robot_description": self.robot_description,
            "joint_state": (None if self.joints is None else {
                "stamp_ns": self.joints.stamp_ns, "positions_rad": self.joints.value,
                "fresh": joints_fresh,
            }),
            "transforms": {name: item.value for name, item in self.transforms.items()},
            "transform_fresh": {name: name in fresh_tf or item.value["static"]
                                for name, item in self.transforms.items()},
            "objects": {name: item.value for name, item in self.objects.items()},
            "object_age_s": object_ages,
            "object_fresh": {name: age <= self.max_age_s
                             for name, age in object_ages.items()},
            "point_cloud": self.point_cloud,
            "diagnostics": {
                "freshness": "fresh" if synchronized else "stale_or_unsynchronized",
                "joint_tf_delta_s": delta_s,
                "joint_tf_sync_basis": sync_basis,
                "joint_state_age_s": (None if self.joints is None else
                                      max(0., now - self.joints.received_monotonic)),
                "dynamic_tf_fresh_count": len(fresh_tf),
                "dynamic_tf_count": len(dynamic),
                "arm_tf_fresh_count": sum(name in fresh_tf for name in arm_tfs),
                "arm_tf_count": len(arm_tfs),
                "base_link_tf_fresh": base_tf_fresh,
                "held": not synchronized,
            },
        }


def transform_matrix(translation, quaternion_xyzw):
    """Return a ROS parent_from_child SE(3) matrix, translation in metres."""
    from scipy.spatial.transform import Rotation

    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix
