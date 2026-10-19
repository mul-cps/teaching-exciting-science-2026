"""Rig-specific mechanical model offsets; never modifies encoder readings."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def apply_gripper_mount_roll(description, degrees):
    """Rotate the entire tool assembly about the wrist-roll axis once at load."""
    angle = float(degrees)
    if not math.isfinite(angle):
        raise ValueError("gripper mount offset must be finite")
    if angle == 0:
        return description
    root = ET.fromstring(description)
    joint = root.find("joint[@name='wrist_roll']")
    if joint is None or joint.find("child").get("link") != "gripper_link":
        raise ValueError("expected wrist_roll -> gripper_link model joint")
    axis = joint.find("axis")
    if axis is None or not np.allclose([float(x) for x in axis.get("xyz").split()], [0, 0, 1]):
        raise ValueError("wrist_roll model axis must be local +Z")
    origin = joint.find("origin")
    if origin is None:
        raise ValueError("wrist_roll model joint has no origin")
    original = Rotation.from_euler("xyz", [float(x) for x in origin.get("rpy", "0 0 0").split()])
    corrected = original * Rotation.from_euler("z", angle, degrees=True)
    origin.set("rpy", " ".join(format(x, ".17g") for x in corrected.as_euler("xyz")))
    return ET.tostring(root, encoding="unicode")


JOINT_ZERO_NAMES = frozenset(("shoulder_lift", "elbow_flex", "wrist_flex"))


def apply_joint_zero_offsets(description, offsets_degrees):
    """Encode provisional geometry at load; q and limits retain encoder meaning.

    URDF uses origin * axis_rotation(q). Rotating origin about that same local
    axis therefore gives FK(new, q) == FK(old, q + offset), without changing q.
    """
    from collections.abc import Mapping
    from numbers import Real

    if not isinstance(offsets_degrees, Mapping) or set(offsets_degrees) - JOINT_ZERO_NAMES:
        raise ValueError("model zero offsets allow only lift, elbow and wrist flex")
    for value in offsets_degrees.values():
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or abs(value) > 12.:
            raise ValueError("model zero offsets must be finite numbers within ±12 degrees")
    if not any(offsets_degrees.values()):
        return description
    root = ET.fromstring(description)
    for name, degrees in offsets_degrees.items():
        matches = root.findall(f"joint[@name='{name}']")
        if len(matches) != 1 or matches[0].get("type") != "revolute":
            raise ValueError(f"expected one revolute model joint: {name}")
        joint = matches[0]
        origin, axis = joint.find("origin"), joint.find("axis")
        if origin is None or axis is None:
            raise ValueError(f"model joint requires origin and axis: {name}")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        direction = np.fromstring(axis.get("xyz", ""), sep=" ")
        if (rpy.shape != (3,) or direction.shape != (3,) or
                not np.isfinite(rpy).all() or not np.isfinite(direction).all() or
                np.linalg.norm(direction) < 1e-12):
            raise ValueError(f"finite model origin and nonzero axis required: {name}")
        if degrees == 0:
            continue
        corrected = Rotation.from_euler("xyz", rpy) * Rotation.from_rotvec(
            direction / np.linalg.norm(direction) * math.radians(degrees))
        origin.set("rpy", " ".join(format(value, ".17g") for value in corrected.as_euler("xyz")))
    return ET.tostring(root, encoding="unicode")


def apply_model_offsets(description, configuration):
    """One shared once-at-load pipeline for the physical station description."""
    return apply_joint_zero_offsets(
        apply_gripper_mount_roll(description, configuration["gripper_mount_roll_deg"]),
        configuration.get("joint_zero_offsets_deg", {}))
