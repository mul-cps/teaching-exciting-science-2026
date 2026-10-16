"""Resolve a pinned tracked destination into its commissioned tool pose."""
import math
from dataclasses import dataclass
from workshop.digital_twin.placement_region import resolve_placement_goal


@dataclass(frozen=True)
class DestinationPose:
    frame_id: str
    position: tuple
    orientation_xyzw: tuple
    selection_id: str
    registration_sha256: str
    area_half_extents_m: tuple
    observation_stamp_ns: int = 0


def require_observed_footprint(size, footprint):
    if (not isinstance(size, (list, tuple)) or len(size) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in size)
            or not isinstance(footprint, (list, tuple)) or len(footprint) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in footprint)
            or math.hypot(size[0], size[1]) > min(footprint)):
        raise ValueError('actual carried brick exceeds commissioned goal-axis footprint')


def require_area_contains(status, target, registration, *, footprint_m,
                          tool_footprint_m, clearance_m, registration_uncertainty_m,
                          goal_from_tip):
    import numpy as np
    values = [clearance_m, registration_uncertainty_m]
    if any(type(v) not in (float, int) or not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('positive commissioned placement clearance and registration uncertainty required')
    envelopes = []
    for dimensions in (footprint_m, tool_footprint_m):
        if (not isinstance(dimensions, (list, tuple)) or len(dimensions) != 2
                or any(type(v) not in (float, int) or not math.isfinite(v) or v <= 0 for v in dimensions)):
            raise ValueError('commissioned carried-brick and tool goal-axis footprints required')
        envelopes.append(np.asarray(dimensions, dtype=float)/2.)
    corners = np.asarray(status.get('goal_corners_camera'), dtype=float)
    if corners.shape != (4, 3) or not np.isfinite(corners).all():
        raise ValueError('current four-corner placement area is required')
    base = corners @ registration.target_from_camera[:3, :3].T + registration.target_from_camera[:3, 3]
    local = (base - target.pose_matrix[:3, 3]) @ target.pose_matrix[:3, :3]
    tolerance = .001
    half = (local[:, :2].max(axis=0)-local[:, :2].min(axis=0))/2
    if (np.max(np.abs(local[:, 2])) > tolerance
            or np.max(np.abs(local.mean(axis=0))) > tolerance
            or np.min(half) <= tolerance
            or not np.allclose(np.abs(local[:, :2]), half, atol=tolerance)):
        raise ValueError('placement corners are inconsistent with a centered planar rectangle')
    if not np.allclose(2*half, [target.area_width_m, target.area_height_m], atol=tolerance):
        raise ValueError("placement area extent disagrees with observed corners")
    signs = {tuple(np.sign(row)) for row in local[:, :2]}
    edges = np.roll(local[:, :2], -1, axis=0)-local[:, :2]
    cross = edges[:, 0]*np.roll(edges[:, 1], -1)-edges[:, 1]*np.roll(edges[:, 0], -1)
    if len(signs) != 4 or not (np.all(cross > 0) or np.all(cross < 0)):
        raise ValueError('placement area corner ordering is malformed')
    margin = clearance_m + registration_uncertainty_m + 3*float(target.fit_uncertainty_m)
    # The carried envelope is commissioned about the selected goal origin;
    # the tool envelope is commissioned in goal axes about the tool origin.
    needed = np.maximum(envelopes[0], envelopes[1]+np.abs(goal_from_tip[:2, 3])) + margin
    if np.any(half < needed):
        raise ValueError('carried brick/tool footprint exceeds placement area after uncertainty clearance')
    return tuple(float(v) for v in half)


def resolve_tool_destination(status, *, selection_id, registration, received_monotonic,
                             now_monotonic, now_ros_ns, goal_from_tip, footprint_m=None, tool_footprint_m=None,
                             clearance_m=None, registration_uncertainty_m=None):
    import numpy as np
    from scipy.spatial.transform import Rotation
    target = resolve_placement_goal(status, expected_selection_id=selection_id,
        registration=registration, received_monotonic=received_monotonic,
        now_monotonic=now_monotonic, now_ros_ns=now_ros_ns)
    transform = np.asarray(goal_from_tip, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(transform[:3, :3].T@transform[:3, :3], np.eye(3), atol=1e-6)
            or not math.isclose(float(np.linalg.det(transform[:3, :3])), 1., abs_tol=1e-6)):
        raise ValueError('commissioned placement_goal_from_tip must be proper SE(3)')
    area = require_area_contains(status, target, registration, footprint_m=footprint_m,
        tool_footprint_m=tool_footprint_m, clearance_m=clearance_m,
        registration_uncertainty_m=registration_uncertainty_m, goal_from_tip=transform)
    matrix = target.pose_matrix @ transform
    return target, tuple(matrix[:3, 3]), tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()), area


def same_destination(first, second, *, position_tolerance_m=.001, quaternion_dot=.99999):
    return (first.selection_id == second.selection_id and
            first.registration_sha256 == second.registration_sha256 and
            max(abs(a-b) for a,b in zip(first.area_half_extents_m, second.area_half_extents_m)) <= position_tolerance_m and
            first.frame_id == second.frame_id and
            math.dist(first.position, second.position) <= position_tolerance_m and
            abs(sum(a*b for a,b in zip(first.orientation_xyzw, second.orientation_xyzw))) >= quaternion_dot)
