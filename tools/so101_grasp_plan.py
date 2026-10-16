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
                   place: PoseTarget | None = None, tip_from_grasp=None):
    """Retreat opposite geometric grasp +Z, then solve the calibrated tip pose.

    The grasp convention is X=closing and Z=approach toward the object. Place
    is already an explicit tool-link pose and is preserved unchanged.
    """
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
                          tuple(p - distance * a for p, a in zip(contact.position, approach)),
                          (x, y, z, w))

    targets = [("pregrasp", shifted(approach_m)), ("contact", shifted(0.0)),
               ("lift", shifted(lift_m))]
    if tip_from_grasp is not None:
        import numpy as np
        # Validate the declared direction before inversion, including rigidity.
        transform_pose(contact, tip_from_grasp)
        grasp_from_tip = np.linalg.inv(np.asarray(tip_from_grasp, dtype=float))
        targets = [(name, transform_pose(target, grasp_from_tip)) for name, target in targets]
    if place is not None:
        targets.append(("place", place))
    return targets


def transform_pose(target: PoseTarget, right_transform):
    """Compose a pose with a declared rigid child-frame transform on the right."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    matrix = np.asarray(right_transform, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0., atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), rtol=0., atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., rtol=0., atol=1e-6)):
        raise ValueError("pose transform must be a proper finite SE(3) matrix")
    world = np.eye(4)
    world[:3, :3] = Rotation.from_quat(target.orientation_xyzw).as_matrix()
    world[:3, 3] = target.position
    composed = world @ matrix
    return PoseTarget(target.frame_id, tuple(composed[:3, 3]),
                      tuple(Rotation.from_matrix(composed[:3, :3]).as_quat()))


def candidate_for_track(status, track_id, *, require_pickup_eligible=False):
    """Select a proposal by numeric track ID or visible detection ID (red-0003)."""
    key = str(track_id)
    matches = [item for item in status.get("grasp_candidates", [])
               if (str(item.get("track_id")) == key or
                   str(item.get("candidate_id", "")).rsplit("-g", 1)[0] == key)]
    if not matches:
        raise ValueError(f"no geometric grasp candidate for track {track_id!r}")
    item = matches[0]
    if require_pickup_eligible and item.get("pickup_eligible") is not True:
        raise ValueError("geometric preview is not explicitly admitted to the pickup region")
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
