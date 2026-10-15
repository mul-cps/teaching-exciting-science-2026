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
