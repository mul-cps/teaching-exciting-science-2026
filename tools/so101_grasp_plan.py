"""ROS-independent helpers for staged SO-ARM101 grasp pose planning."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseTarget:
    frame_id: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        if not self.frame_id:
            raise ValueError("pose target frame_id is required")
        if len(self.position) != 3 or len(self.orientation_xyzw) != 4:
            raise ValueError("pose target needs XYZ and an XYZW quaternion")
        values = (*self.position, *self.orientation_xyzw)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("pose target must contain finite values")
        if sum(float(value) ** 2 for value in self.orientation_xyzw) < 1e-12:
            raise ValueError("pose target quaternion must be nonzero")


def staged_targets(contact: PoseTarget, *, approach_m=0.06, lift_m=0.08,
                   place: PoseTarget | None = None):
    """Return plan targets; tool +Z is assumed to point along approach direction."""
    for value, label in ((approach_m, "approach"), (lift_m, "lift")):
        if not math.isfinite(value) or value <= 0 or value > 0.30:
            raise ValueError(f"{label} distance must be in (0, 0.30] m")
    qx, qy, qz, qw = contact.orientation_xyzw
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
    # Third column of the normalized quaternion rotation matrix.
    approach = (2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y))

    def shifted(distance):
        return PoseTarget(contact.frame_id,
                          tuple(p + distance * a for p, a in zip(contact.position, approach)),
                          (x, y, z, w))

    targets = [("pregrasp", shifted(approach_m)), ("contact", shifted(0.0)),
               ("lift", shifted(lift_m))]
    if place is not None:
        targets.append(("place", place))
    return targets


def candidate_for_track(status, track_id):
    """Select a proposal by numeric track ID or visible detection ID (red-0003)."""
    key = str(track_id)
    matches = [item for item in status.get("grasp_candidates", [])
               if (str(item.get("track_id")) == key or
                   str(item.get("candidate_id", "")).rsplit("-g", 1)[0] == key)]
    if not matches:
        raise ValueError(f"no geometric grasp candidate for track {track_id!r}")
    item = matches[0]
    if item.get("ik_validated") or item.get("collision_checked"):
        # Tracking reports are source geometry, not planner results.
        raise ValueError("tracking source must remain marked geometric_only")
    return PoseTarget(str(item["frame_id"]),
                      tuple(float(v) for v in item["contact_position_world"]),
                      tuple(float(v) for v in _quaternion_from_axes(
                          item["closing_axis_world"], item["approach_direction_world"])))


def _quaternion_from_axes(closing, approach):
    """Build a right-handed tool basis: X=closing, Z=approach."""
    import numpy as np
    x = np.asarray(closing, dtype=float)
    z = np.asarray(approach, dtype=float)
    if x.shape != (3,) or z.shape != (3,) or not np.isfinite(x).all() or not np.isfinite(z).all():
        raise ValueError("candidate axes must be finite 3-vectors")
    x /= np.linalg.norm(x)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    if np.linalg.norm(y) < 1e-8:
        raise ValueError("candidate closing and approach axes are parallel")
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    m = np.column_stack((x, y, z))
    # Stable matrix to quaternion conversion without ROS dependencies.
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = ((m[2, 1]-m[1, 2])/s, (m[0, 2]-m[2, 0])/s,
             (m[1, 0]-m[0, 1])/s, 0.25*s)
    else:
        i = max(range(3), key=lambda k: m[k, k])
        j, k = (i+1) % 3, (i+2) % 3
        s = math.sqrt(max(1e-12, 1 + m[i, i] - m[j, j] - m[k, k])) * 2
        qv = [0.0, 0.0, 0.0]
        qv[i] = 0.25*s
        qv[j] = (m[j, i] + m[i, j]) / s
        qv[k] = (m[k, i] + m[i, k]) / s
        q = (*qv, (m[k, j] - m[j, k]) / s)
    return q
