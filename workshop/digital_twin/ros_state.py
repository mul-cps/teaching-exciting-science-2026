"""Pure state rules shared by the ROS snapshot bridge and the web twin.

ROS timestamps are nanoseconds. Positions stay in metres and rotations are
xyzw quaternions; no optical-frame or viewer-axis conversion happens here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


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
            "static": bool(static),
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
        delta_s = (None if self.joints is None or base_tf is None else
                   abs(self.joints.stamp_ns - base_tf.stamp_ns) / 1e9)
        synchronized = bool(joints_fresh and base_tf_fresh and delta_s is not None
                            and delta_s <= self.max_joint_tf_delta_s)
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
            "point_cloud": self.point_cloud,
            "diagnostics": {
                "freshness": "fresh" if synchronized else "stale_or_unsynchronized",
                "joint_tf_delta_s": delta_s,
                "joint_state_age_s": (None if self.joints is None else
                                      max(0., now - self.joints.received_monotonic)),
                "dynamic_tf_fresh_count": len(fresh_tf),
                "dynamic_tf_count": len(dynamic),
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
