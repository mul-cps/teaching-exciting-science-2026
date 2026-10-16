"""Sparse RGB-D tracking for a placement patch attached to a moving feeder.

This module is deliberately independent of robot command interfaces.  Its
``placement_motion_qualified`` result is always false: it is an observation
for the station UI and downstream review, not an actuator admission token.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from pathlib import Path
import hashlib
import json
import uuid
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class PoseFit:
    reason: str
    inlier_indices: tuple[int, ...] = ()
    rms_m: float | None = None
    inlier_fraction: float = 0.


@dataclass(frozen=True)
class PlacementRegionSnapshot:
    valid: bool
    reason: str
    generation: str | None
    stamp_ns: int | None
    frame_id: str | None
    age_s: float | None
    root_from_anchor: np.ndarray | None
    goal_corners_root: np.ndarray | None
    last_valid_goal_corners_root: np.ndarray | None
    feature_count: int
    inlier_count: int
    fit_rms_m: float | None
    placement_motion_qualified: bool = False
    selection_id: str | None = None


@dataclass(frozen=True)
class QualifiedFrameRegistration:
    """Server-loaded, hash-pinned camera-to-base calibration artifact."""
    camera_frame: str
    target_frame: str
    target_from_camera: np.ndarray
    sha256: str


@dataclass(frozen=True)
class PlacementGoalTarget:
    """Fresh placement target rebuilt in a qualified planning frame."""
    frame_id: str
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    pose_matrix: np.ndarray
    selection_id: str
    observation_stamp_ns: int
    registration_sha256: str
    ready_for_planning: bool = True
    area_width_m: float = 0.
    area_height_m: float = 0.
    normal_xyz: tuple[float, float, float] = (0., 0., 1.)
    fit_uncertainty_m: float = 0.
    plane_rms_m: float = 0.


def _matrix4(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("finite 4x4 transform required")
    if not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-8):
        raise ValueError("homogeneous transform bottom row must be [0,0,0,1]")
    return matrix



def load_qualified_frame_registration(path):
    """Load trusted station calibration bytes; never read qualification from UI state."""
    raw = Path(path).read_bytes()
    try:
        data = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid frame-registration JSON") from error
    if (not isinstance(data, dict) or data.get("schema") != "camera-arm-calibration-v1"
            or data.get("transform_direction") != "base_from_camera"
            or data.get("qualified") is not True):
        raise ValueError("qualified camera-arm frame registration is required")
    camera_frame = data.get("camera_frame")
    if not isinstance(camera_frame, str) or not camera_frame.strip():
        raise ValueError("trusted registration must bind a camera_frame")
    transform = _matrix4(data.get("base_from_camera"))
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4)
            or not np.isclose(np.linalg.det(rotation), 1., atol=2e-4)):
        raise ValueError("base_from_camera must be a proper SE(3) transform")
    transform.setflags(write=False)
    return QualifiedFrameRegistration(camera_frame.strip().lstrip("/"), "base_link",
        transform, hashlib.sha256(raw).hexdigest())


def _quaternion_xyzw(rotation):
    """Convert a proper 3x3 rotation to normalized XYZW quaternion."""
    r = np.asarray(rotation, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError("finite rotation matrix required")
    if not np.allclose(r.T @ r, np.eye(3), atol=2e-5) or not np.isclose(np.linalg.det(r), 1., atol=2e-5):
        raise ValueError("proper rotation matrix required")
    trace = float(np.trace(r))
    if trace > 0.:
        scale = np.sqrt(trace + 1.) * 2.
        q = np.array([(r[2, 1] - r[1, 2]) / scale,
                      (r[0, 2] - r[2, 0]) / scale,
                      (r[1, 0] - r[0, 1]) / scale, scale / 4.])
    else:
        axis = int(np.argmax(np.diag(r)))
        if axis == 0:
            scale = np.sqrt(max(0., 1. + r[0, 0] - r[1, 1] - r[2, 2])) * 2.
            q = np.array([scale / 4., (r[0, 1] + r[1, 0]) / scale,
                          (r[0, 2] + r[2, 0]) / scale, (r[2, 1] - r[1, 2]) / scale])
        elif axis == 1:
            scale = np.sqrt(max(0., 1. + r[1, 1] - r[0, 0] - r[2, 2])) * 2.
            q = np.array([(r[0, 1] + r[1, 0]) / scale, scale / 4.,
                          (r[1, 2] + r[2, 1]) / scale, (r[0, 2] - r[2, 0]) / scale])
        else:
            scale = np.sqrt(max(0., 1. + r[2, 2] - r[0, 0] - r[1, 1])) * 2.
            q = np.array([(r[0, 2] + r[2, 0]) / scale,
                          (r[1, 2] + r[2, 1]) / scale, scale / 4.,
                          (r[1, 0] - r[0, 1]) / scale])
    q /= np.linalg.norm(q)
    return q


def resolve_placement_goal(status, *, expected_selection_id, registration,
                           received_monotonic, now_monotonic, now_ros_ns,
                           max_age_s=.25, max_ros_age_s=.5, max_fit_rms_m=.008,
                           max_plane_rms_m=.008):
    """Resolve a live selected area into a current base-frame planning target.

    ``registration`` must come from :func:`load_qualified_frame_registration` on
    a station-owned commissioning file. Status fields such as ``qualified`` or
    ``physical_placement_admission`` are intentionally ignored. The returned
    target is ready for the caller's existing TCP, IK, collision and motion gates.
    The subscriber supplies ``received_monotonic`` locally; status JSON monotonic
    values are not trusted across processes. ``now_ros_ns`` checks acquisition-time
    freshness. Call again immediately before placement; a changed stamp produces
    a new target and requires the caller to replan.
    """
    if not isinstance(registration, QualifiedFrameRegistration):
        raise ValueError("qualified station registration is required")
    if (not np.isfinite(max_age_s) or max_age_s <= 0
            or not np.isfinite(max_ros_age_s) or max_ros_age_s <= 0
            or not np.isfinite(max_fit_rms_m) or max_fit_rms_m < 0
            or not np.isfinite(max_plane_rms_m) or max_plane_rms_m < 0):
        raise ValueError("resolver age and residual limits must be finite and valid")
    if not isinstance(status, dict) or status.get("valid") is not True or status.get("state") != "tracked":
        raise ValueError("selected placement area is not currently tracked")
    selection_id = status.get("selection_id")
    if not expected_selection_id or selection_id != expected_selection_id:
        raise ValueError("placement selection identity changed")
    frame_id = str(status.get("frame_id", "")).strip().lstrip("/")
    if frame_id != registration.camera_frame:
        raise ValueError("placement observation camera frame does not match trusted registration")
    stamp_ns = status.get("stamp_ns")
    goal_stamp_ns = status.get("camera_from_goal_stamp_ns")
    if (not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns <= 0
            or goal_stamp_ns != stamp_ns):
        raise ValueError("placement goal pose is not paired to its observation stamp")
    received = float(received_monotonic)
    now = float(now_monotonic)
    ros_now = int(now_ros_ns)
    if (not np.isfinite(received) or not np.isfinite(now) or now < received
            or now - received > max_age_s or ros_now <= 0
            or ros_now < stamp_ns or (ros_now - stamp_ns) > max_ros_age_s * 1e9):
        raise ValueError("placement observation is stale")
    rms = status.get("fit_rms_m")
    if (not isinstance(rms, (int, float)) or not np.isfinite(rms)
            or rms < 0 or rms > max_fit_rms_m):
        raise ValueError("placement feature fit uncertainty exceeds limit")
    geometry = status.get("goal_geometry")
    metrics = status.get("plane_metrics")
    if not isinstance(geometry, dict) or not isinstance(metrics, dict):
        raise ValueError("placement area extent and depth-plane uncertainty are required")
    width, height = geometry.get("width_m"), geometry.get("height_m")
    plane_rms = metrics.get("plane_rms_m")
    if (not isinstance(width, (int, float)) or not np.isfinite(width) or width <= 0
            or not isinstance(height, (int, float)) or not np.isfinite(height) or height <= 0):
        raise ValueError("placement area dimensions must be finite and positive")
    if (not isinstance(plane_rms, (int, float)) or not np.isfinite(plane_rms)
            or plane_rms < 0 or plane_rms > max_plane_rms_m):
        raise ValueError("placement depth-plane uncertainty exceeds limit")
    camera_from_goal = _matrix4(status.get("camera_from_goal"))
    rotation = camera_from_goal[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4)
            or not np.isclose(np.linalg.det(rotation), 1., atol=2e-4)):
        raise ValueError("camera_from_goal must be a proper SE(3) pose")
    target_pose = registration.target_from_camera @ camera_from_goal
    quaternion = _quaternion_xyzw(target_pose[:3, :3])
    normal = target_pose[:3, 2]
    return PlacementGoalTarget(registration.target_frame,
        tuple(float(v) for v in target_pose[:3, 3]),
        tuple(float(v) for v in quaternion), target_pose, str(selection_id),
        stamp_ns, registration.sha256, True, float(width), float(height),
        tuple(float(v) for v in normal),
        float(np.hypot(float(rms), float(plane_rms))), float(plane_rms))



def waiting_for_anchor_status(previous, *, generation, stamp_ns, frame_id):
    """Build a truthful no-anchor state without erasing a selection error."""
    previous = previous if isinstance(previous, dict) else {}
    reason = str(previous.get("reason", ""))
    if reason.startswith(("selection_rejected:", "selection_frame_")):
        return dict(previous)
    return {"state": "waiting_for_anchor", "valid": False,
        "reason": "no_feeder_anchor_selected", "generation": str(generation),
        "stamp_ns": int(stamp_ns), "frame_id": str(frame_id),
        "placement_motion_qualified": False}

def placement_camera_frame_summary(record):
    """Return JSON-safe metadata for one cached RGB-D acquisition."""
    summary = {key: record[key] for key in (
        "generation", "stamp_ns", "received_monotonic", "frame_id",
        "root_frame", "image_path", "image_shape")}
    for key in ("root_from_camera", "base_from_camera", "camera_matrix"):
        value = record.get(key)
        summary[key] = None if value is None else np.asarray(value, dtype=float).tolist()
    return summary

def deproject_pixels(pixels_xy, depth_m, intrinsics):
    """Deproject ``(u,v)`` pixels to optical-frame XYZ using rectified K."""
    pixels = np.asarray(pixels_xy, dtype=float)
    depths = np.asarray(depth_m, dtype=float).reshape(-1)
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) != len(depths):
        raise ValueError("pixel Nx2 and matching depth N arrays required")
    if (not np.isfinite(pixels).all() or not np.isfinite(depths).all()
            or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or np.any(depths <= 0)):
        raise ValueError("finite pixels, positive depth, and valid intrinsics required")
    xyz = np.empty((len(pixels), 3), dtype=float)
    xyz[:, 0] = (pixels[:, 0] - k[0, 2]) * depths / k[0, 0]
    xyz[:, 1] = (pixels[:, 1] - k[1, 2]) * depths / k[1, 1]
    xyz[:, 2] = depths
    return xyz


def transform_points(transform, points):
    matrix = _matrix4(transform)
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or not np.isfinite(pts).all():
        raise ValueError("finite Nx3 points required")
    return pts @ matrix[:3, :3].T + matrix[:3, 3]


def pose_from_rectangle(corners):
    """Construct camera_from_goal from ordered TL, TR, BR, BL 3D corners."""
    points = np.asarray(corners, dtype=float)
    if points.shape != (4, 3) or not np.isfinite(points).all():
        raise ValueError("four finite ordered rectangle corners required")
    x_axis = points[1] - points[0]
    y_axis = points[3] - points[0]
    width, height = np.linalg.norm(x_axis), np.linalg.norm(y_axis)
    if width <= 1e-5 or height <= 1e-5:
        raise ValueError("placement rectangle must have nonzero width and height")
    x_axis /= width
    y_axis -= x_axis * float(x_axis @ y_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-5:
        raise ValueError("placement rectangle axes are degenerate")
    y_axis /= y_norm
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    pose[:3, 3] = points.mean(axis=0)
    return pose, {"width_m": float(width), "height_m": float(height),
                  "normal_camera": z_axis.tolist(),
                  "geometry": "observed_depth_surface_rectangle"}


def _rigid_fit(source, target):
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    a, b = source - source_center, target - target_center
    singular = np.linalg.svd(a, compute_uv=False)
    if len(singular) < 2 or singular[1] <= 1e-5:
        return None
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = target_center - rotation @ source_center
    return pose


def fit_rigid_pose(source_points, target_points, *, inlier_threshold_m=.012,
                   minimum_inliers=6, minimum_fraction=.55, max_hypotheses=512):
    """Estimate current-root-from-anchor with bounded 3D RANSAC.

    Rows are matched feature correspondences. Duplicate source rows are
    permitted and represent descriptor reacquisition alternatives. Two
    similarly supported, spatially distinct poses are reported as ambiguous.
    """
    source = np.asarray(source_points, dtype=float)
    target = np.asarray(target_points, dtype=float)
    invalid = PoseFit("insufficient_inliers")
    if (source.ndim != 2 or target.shape != source.shape or source.shape[1:] != (3,)
            or not np.isfinite(source).all() or not np.isfinite(target).all()
            or len(source) < minimum_inliers):
        return None, invalid
    if not np.isfinite(inlier_threshold_m) or inlier_threshold_m <= 0:
        raise ValueError("positive finite inlier threshold required")

    n = len(source)
    total = n * (n - 1) * (n - 2) // 6
    if total <= max_hypotheses:
        triples = combinations(range(n), 3)
    else:
        rng = np.random.default_rng(0)
        sampled = set()
        while len(sampled) < max_hypotheses:
            sampled.add(tuple(sorted(rng.choice(n, 3, replace=False).tolist())))
        triples = iter(sorted(sampled))
    hypotheses = []
    for indices in triples:
        pose = _rigid_fit(source[list(indices)], target[list(indices)])
        if pose is None:
            continue
        errors = np.linalg.norm(transform_points(pose, source) - target, axis=1)
        inliers = np.flatnonzero(errors <= inlier_threshold_m)
        if len(inliers) < minimum_inliers:
            continue
        refined = _rigid_fit(source[inliers], target[inliers])
        if refined is None:
            continue
        errors = np.linalg.norm(transform_points(refined, source) - target, axis=1)
        inliers = np.flatnonzero(errors <= inlier_threshold_m)
        if len(inliers) < minimum_inliers:
            continue
        refined = _rigid_fit(source[inliers], target[inliers])
        errors = np.linalg.norm(transform_points(refined, source) - target, axis=1)
        inliers = np.flatnonzero(errors <= inlier_threshold_m)
        rms = float(np.sqrt(np.mean(errors[inliers] ** 2)))
        if len(inliers) >= minimum_inliers:
            hypotheses.append((len(inliers), rms, refined, tuple(map(int, inliers))))
    if not hypotheses:
        return None, invalid
    hypotheses.sort(key=lambda row: (-row[0], row[1]))
    best = hypotheses[0]
    distinct = []
    for candidate in hypotheses[1:]:
        if candidate[0] < best[0] - 1:
            break
        translation = np.linalg.norm(candidate[2][:3, 3] - best[2][:3, 3])
        delta = best[2][:3, :3].T @ candidate[2][:3, :3]
        cosine = np.clip((np.trace(delta) - 1.) * .5, -1., 1.)
        rotation_deg = float(np.rad2deg(np.arccos(cosine)))
        if translation > inlier_threshold_m * 1.5 or rotation_deg > 4.:
            distinct.append(candidate)
            break
    if distinct:
        return None, PoseFit("ambiguous_motion", best[3], best[1], best[0] / n)
    if len(best[3]) < minimum_inliers or len(best[3]) / n < minimum_fraction:
        return None, PoseFit("insufficient_inliers", best[3], best[1], best[0] / n)
    return best[2], PoseFit("tracked", best[3], best[1], best[0] / n)


def project_rectangle_to_plane(rect_xyxy, depth_m, intrinsics, root_from_camera,
                               *, min_samples=24, max_plane_rms_m=.012):
    """Project a pixel rectangle onto the robust local depth plane in root."""
    depth = np.asarray(depth_m, dtype=float)
    if depth.ndim != 2 or not np.isfinite(depth).any():
        raise ValueError("finite 2D depth image required")
    x0, y0, x1, y1 = map(int, rect_xyxy)
    h, w = depth.shape
    if x0 < 0 or y0 < 0 or x1 > w or y1 > h or x1 - x0 < 3 or y1 - y0 < 3:
        raise ValueError("placement rectangle must be a nonempty in-image area")
    roi = depth[y0:y1, x0:x1]
    yy, xx = np.mgrid[y0:y1, x0:x1]
    valid = np.isfinite(roi) & (roi > .05) & (roi < 4.)
    if int(valid.sum()) < min_samples:
        raise ValueError("insufficient valid depth in placement area")
    pixels = np.column_stack((xx[valid], yy[valid])).astype(float)
    optical = deproject_pixels(pixels, roi[valid], intrinsics)
    root_points = transform_points(root_from_camera, optical)
    center = np.median(root_points, axis=0)
    _, singular, vt = np.linalg.svd(root_points - center, full_matrices=False)
    if len(singular) < 3 or singular[1] <= 1e-5:
        raise ValueError("placement area depth does not support a plane")
    normal = vt[-1]
    residual = (root_points - center) @ normal
    rms = float(np.sqrt(np.mean(residual ** 2)))
    if rms > max_plane_rms_m:
        raise ValueError(f"placement area plane residual {rms:.4f} m exceeds limit")
    corners_px = np.array([[x0, y0], [x1 - 1, y0], [x1 - 1, y1 - 1],
                           [x0, y1 - 1]], dtype=float)
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    rays_cam = np.column_stack(((corners_px[:, 0] - k[0, 2]) / k[0, 0],
                                (corners_px[:, 1] - k[1, 2]) / k[1, 1],
                                np.ones(4)))
    ray_origin = _matrix4(root_from_camera)[:3, 3]
    ray_directions = rays_cam @ _matrix4(root_from_camera)[:3, :3].T
    denom = ray_directions @ normal
    if np.any(np.abs(denom) < 1e-8):
        raise ValueError("placement boundary rays are parallel to measured plane")
    scales = ((center - ray_origin) @ normal) / denom
    if np.any(scales <= 0):
        raise ValueError("placement plane is behind image boundary")
    polygon = ray_origin + ray_directions * scales[:, None]
    return polygon, {"plane_rms_m": rms, "depth_samples": int(valid.sum()),
                    "visible_surface_only": True}


def intersect_ray_with_image_plane(ray_origin, ray_direction, root_from_camera,
                                   intrinsics, image_shape, *, distance_m=.35):
    """Map a Viser click ray to pixels on its matching optical image plane."""
    origin = np.asarray(ray_origin, dtype=float).reshape(3)
    direction = np.asarray(ray_direction, dtype=float).reshape(3)
    matrix = _matrix4(root_from_camera)
    direction /= np.linalg.norm(direction)
    plane_center = transform_points(matrix, np.array([[0., 0., distance_m]]))[0]
    normal = matrix[:3, 2]
    denominator = float(direction @ normal)
    if abs(denominator) < 1e-8:
        return None
    distance = float((plane_center - origin) @ normal / denominator)
    if distance <= 0:
        return None
    point_root = origin + direction * distance
    point_camera = transform_points(np.linalg.inv(matrix), point_root[None, :])[0]
    x, y = point_camera[:2]
    if abs(point_camera[2] - distance_m) > 1e-5:
        return None
    height, width = map(int, image_shape)
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    u = k[0, 0] * x / distance_m + k[0, 2]
    v = k[1, 1] * y / distance_m + k[1, 2]
    if not (0 <= u < width and 0 <= v < height):
        return None
    return float(u), float(v)


class PlacementRegionTracker:
    """Maintain a feeder-local placement polygon from sparse 3D matches."""
    def __init__(self, *, max_age_s=.25, max_frame_gap_s=.5,
                 inlier_threshold_m=.012, minimum_features=6,
                 minimum_fraction=.55, history_size=256,
                 max_selection_age_s=10.):
        for name, value in (("max_age_s", max_age_s),
                            ("max_frame_gap_s", max_frame_gap_s),
                            ("inlier_threshold_m", inlier_threshold_m)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if minimum_features < 4 or history_size < 2 or not 0 < minimum_fraction <= 1:
            raise ValueError("invalid tracker limits")
        self.max_age_s = float(max_age_s)
        self.max_frame_gap_s = float(max_frame_gap_s)
        self.inlier_threshold_m = float(inlier_threshold_m)
        self.minimum_features = int(minimum_features)
        self.minimum_fraction = float(minimum_fraction)
        self.history_size = int(history_size)
        if not np.isfinite(max_selection_age_s) or max_selection_age_s <= 0:
            raise ValueError("max_selection_age_s must be positive and finite")
        self.max_selection_age_s = float(max_selection_age_s)
        self.selection_id = None
        self.anchor_points = None
        self.feature_ids = None
        self.goal_corners_local = None
        self.anchor_frame_id = None
        self.anchor_generation = None
        self.anchor_stamp_ns = None
        self.reason = "no_anchor"
        self._last_stamp_ns = None
        self._last_received = None
        self._last_generation = None
        self._last_pose = None
        self._last_valid_goal = None
        self._history = {}
        self._fit = PoseFit("no_anchor")
        self._match_count = 0

    def set_anchor(self, stamp_ns, anchor_points_root, *, frame_id, generation,
                   received_monotonic, feature_ids=None):
        points = np.asarray(anchor_points_root, dtype=float)
        ids = list(range(len(points))) if feature_ids is None else list(feature_ids)
        if (points.ndim != 2 or points.shape[1:] != (3,) or not np.isfinite(points).all()
                or len(points) < self.minimum_features or len(ids) != len(points)
                or len(set(ids)) != len(ids)):
            self.reason = "insufficient_features" if len(points) < self.minimum_features else "invalid_anchor"
            return False
        if not frame_id or not generation or int(stamp_ns) <= 0:
            self.reason = "invalid_anchor_metadata"
            return False
        centered = points - points.mean(axis=0)
        singular = np.linalg.svd(centered, compute_uv=False)
        if len(singular) < 2 or singular[1] < 1e-5:
            self.reason = "degenerate_anchor_geometry"
            return False
        self.selection_id = uuid.uuid4().hex
        self.anchor_points = points.copy()
        self.feature_ids = ids
        self.anchor_frame_id = str(frame_id).lstrip("/")
        self.anchor_generation = str(generation)
        self.anchor_stamp_ns = int(stamp_ns)
        self._last_stamp_ns = int(stamp_ns)
        self._last_received = float(received_monotonic)
        self._last_generation = str(generation)
        self._last_pose = np.eye(4)
        self._history = {str(generation): (np.eye(4), int(stamp_ns),
                                          float(received_monotonic))}
        self._last_valid_goal = None
        self.goal_corners_local = None
        self._fit = PoseFit("anchor_selected", tuple(range(len(points))), 0., 1.)
        self.reason = "anchor_selected"
        return True

    def set_goal(self, stamp_ns, goal_corners_root, *, generation, received_monotonic,
                 now_monotonic=None):
        corners = np.asarray(goal_corners_root, dtype=float)
        generation = str(generation)
        if (corners.shape != (4, 3) or not np.isfinite(corners).all()
                or generation not in self._history):
            self.reason = "goal_frame_unavailable"
            return False
        now = float(received_monotonic if now_monotonic is None else now_monotonic)
        history = self._history.get(generation)
        if (history is None or history[1] != int(stamp_ns)
                or now - float(received_monotonic) < 0
                or now - float(received_monotonic) > self.max_selection_age_s
                or self._last_pose is None):
            self.reason = "stale_frame"
            return False
        pose = history[0]
        self.goal_corners_local = transform_points(np.linalg.inv(pose), corners)
        self._last_valid_goal = corners.copy()
        self.reason = "goal_selected"
        return True

    def update(self, stamp_ns, feature_ids: Iterable, observed_points_root, *,
               generation, received_monotonic, now_monotonic=None):
        now = float(received_monotonic if now_monotonic is None else now_monotonic)
        stamp_ns = int(stamp_ns)
        generation = str(generation)
        points = np.asarray(observed_points_root, dtype=float)
        ids = list(feature_ids)
        if (points.ndim != 2 or points.shape[1:] != (3,) or len(points) != len(ids)
                or not np.isfinite(points).all()):
            self.reason = "invalid_observations"
            return self.snapshot(now)
        if self.anchor_points is None:
            self.reason = "no_anchor"
            return self.snapshot(now)
        if stamp_ns <= self._last_stamp_ns:
            self.reason = "nonmonotonic_frame"
            return self.snapshot(now)
        age = now - float(received_monotonic)
        gap = (stamp_ns - self._last_stamp_ns) / 1e9
        if age < 0 or age > self.max_age_s or gap > self.max_frame_gap_s:
            self.reason = "stale_frame"
            self._last_stamp_ns = stamp_ns
            self._last_received = float(received_monotonic)
            self._last_generation = generation
            self._last_pose = None
            self._fit = PoseFit("stale_frame")
            self._match_count = 0
            return self.snapshot(now)
        index = {feature_id: idx for idx, feature_id in enumerate(self.feature_ids)}
        source_rows, target_rows, retained_ids = [], [], []
        for feature_id, point in zip(ids, points):
            if feature_id in index:
                source_rows.append(self.anchor_points[index[feature_id]])
                target_rows.append(point)
                retained_ids.append(feature_id)
        self._last_stamp_ns = stamp_ns
        self._last_received = float(received_monotonic)
        self._last_generation = generation
        self._match_count = len(retained_ids)
        if len(source_rows) < self.minimum_features:
            self.reason = "insufficient_features"
            self._last_pose = None
            self._fit = PoseFit(self.reason)
            return self.snapshot(now)
        pose, fit = fit_rigid_pose(source_rows, target_rows,
            inlier_threshold_m=self.inlier_threshold_m,
            minimum_inliers=self.minimum_features, minimum_fraction=self.minimum_fraction)
        self._fit = fit
        if pose is None:
            self.reason = fit.reason
            self._last_pose = None
            return self.snapshot(now)
        self._last_pose = pose
        self._history[generation] = (pose, stamp_ns, float(received_monotonic))
        while len(self._history) > self.history_size:
            self._history.pop(next(iter(self._history)))
        self.reason = "tracked"
        if self.goal_corners_local is not None:
            self._last_valid_goal = transform_points(pose, self.goal_corners_local)
        return self.snapshot(now)

    def snapshot(self, now_monotonic):
        age = None if self._last_received is None else max(0., float(now_monotonic) - self._last_received)
        reason = self.reason
        if age is not None and age > self.max_age_s:
            reason = "stale_frame"
        valid = bool(self._last_pose is not None and age is not None and age <= self.max_age_s
                     and self.goal_corners_local is not None and reason == "tracked")
        corners = (transform_points(self._last_pose, self.goal_corners_local)
                   if valid else None)
        return PlacementRegionSnapshot(valid, reason, self._last_generation,
            self._last_stamp_ns, self.anchor_frame_id, age,
            None if self._last_pose is None else self._last_pose.copy(),
            None if corners is None else corners, 
            None if self._last_valid_goal is None else self._last_valid_goal.copy(),
            self._match_count, len(self._fit.inlier_indices), self._fit.rms_m, False,
            self.selection_id)

    def invalidate(self, reason, *, generation=None, stamp_ns=None,
                   received_monotonic=None):
        """Invalidate current admission while retaining only an explicitly
        labeled last-known polygon for UI diagnosis."""
        self.reason = str(reason)
        self._last_pose = None
        self._fit = PoseFit(self.reason)
        self._match_count = 0
        if generation is not None:
            self._last_generation = str(generation)
        if stamp_ns is not None and self._last_stamp_ns is not None:
            self._last_stamp_ns = max(self._last_stamp_ns, int(stamp_ns))
        if received_monotonic is not None:
            self._last_received = float(received_monotonic)
        return self.snapshot(float(received_monotonic or 0.))


class SparseRGBDAnchor:
    """Bounded Shi-Tomasi/LK front-end that feeds the rigid region tracker."""
    def __init__(self, tracker=None, *, maximum_features=80, minimum_depth_m=.08,
                 maximum_depth_m=1.2, forward_backward_limit_px=1.5):
        if maximum_features < 6 or not 0 < minimum_depth_m < maximum_depth_m:
            raise ValueError("invalid sparse tracker limits")
        self.tracker = tracker or PlacementRegionTracker()
        self.maximum_features = int(maximum_features)
        self.minimum_depth_m = float(minimum_depth_m)
        self.maximum_depth_m = float(maximum_depth_m)
        self.forward_backward_limit_px = float(forward_backward_limit_px)
        self._gray = None
        self._pixels = None
        self._ids = None
        self._camera_matrix = None
        self._root_from_camera = None
        self._needs_reselect = False
        self._reference_stamp_ns = None
        self._anchor_descriptors = None
        self._anchor_descriptor_ids = None
        self.identity_reacquired_after_gap = False

    @staticmethod
    def _gray8(image):
        import cv2
        pixels = np.asarray(image)
        if pixels.ndim != 3 or pixels.shape[2] != 3 or pixels.dtype != np.uint8:
            raise ValueError("RGB uint8 image with shape HxWx3 required")
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)

    def _sample_depths(self, depth, pixels):
        depth = np.asarray(depth, dtype=float)
        if depth.ndim != 2:
            raise ValueError("depth must be a 2D metric image")
        h, w = depth.shape
        samples, kept, kept_indices = [], [], []
        for index, (u, v) in enumerate(np.asarray(pixels, dtype=float)):
            x, y = int(round(u)), int(round(v))
            if x < 2 or y < 2 or x >= w - 2 or y >= h - 2:
                continue
            patch = depth[y - 2:y + 3, x - 2:x + 3]
            valid = patch[np.isfinite(patch) & (patch >= self.minimum_depth_m)
                          & (patch <= self.maximum_depth_m)]
            if len(valid) < 5:
                continue
            samples.append(float(np.median(valid)))
            kept.append((u, v))
            kept_indices.append(index)
        return (np.asarray(kept, dtype=float).reshape(-1, 2),
                np.asarray(samples, dtype=float), np.asarray(kept_indices, dtype=int))

    def select_anchor(self, image_rgb, depth_m, intrinsics, root_from_camera, rect_xyxy,
                      *, stamp_ns, frame_id, generation, received_monotonic):
        import cv2
        gray = self._gray8(image_rgb)
        h, w = gray.shape
        x0, y0, x1, y1 = map(int, rect_xyxy)
        if x0 < 0 or y0 < 0 or x1 > w or y1 > h or x1 <= x0 or y1 <= y0:
            self.tracker.reason = "anchor_rectangle_out_of_bounds"
            return False
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y0:y1, x0:x1] = 255
        orb = cv2.ORB_create(nfeatures=self.maximum_features, fastThreshold=8)
        keypoints, descriptors = orb.detectAndCompute(gray, mask)
        if not keypoints or descriptors is None:
            self.tracker.reason = "insufficient_features"
            return False
        keypoint_pixels = np.asarray([point.pt for point in keypoints], dtype=float)
        pixels, depths, kept = self._sample_depths(depth_m, keypoint_pixels)
        if len(pixels) < self.tracker.minimum_features:
            self.tracker.reason = "insufficient_depth_features"
            return False
        points_camera = deproject_pixels(pixels, depths, intrinsics)
        points_root = transform_points(root_from_camera, points_camera)
        ids = kept.tolist()
        if not self.tracker.set_anchor(stamp_ns, points_root, frame_id=frame_id,
                generation=generation, received_monotonic=received_monotonic,
                feature_ids=ids):
            return False
        self._gray = gray
        self._pixels = pixels.astype(np.float32)
        self._ids = ids
        self._anchor_descriptors = descriptors[kept].copy()
        self._anchor_descriptor_ids = ids.copy()
        self._reference_stamp_ns = int(stamp_ns)
        self._camera_matrix = np.asarray(intrinsics, dtype=float).reshape(3, 3).copy()
        self._root_from_camera = _matrix4(root_from_camera).copy()
        self._needs_reselect = False
        return True

    def _descriptor_observations(self, gray, depth_m):
        """Match the pinned anchor descriptors into one current image."""
        import cv2
        if self._anchor_descriptors is None or len(self._anchor_descriptors) < self.tracker.minimum_features:
            return [], np.empty((0, 3)), np.empty((0, 2), dtype=np.float32)
        orb = cv2.ORB_create(nfeatures=max(self.maximum_features * 3, 120), fastThreshold=8)
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        if not keypoints or descriptors is None:
            return [], np.empty((0, 3)), np.empty((0, 2), dtype=np.float32)
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        pairs = matcher.knnMatch(self._anchor_descriptors, descriptors, k=2)
        matches = []
        for pair in pairs:
            if len(pair) == 2 and pair[0].distance < .72 * pair[1].distance:
                matches.append(pair[0])
        matches.sort(key=lambda match: match.distance)
        used_current, ids, pixels = set(), [], []
        for match in matches:
            if match.trainIdx in used_current:
                continue
            used_current.add(match.trainIdx)
            ids.append(self._anchor_descriptor_ids[match.queryIdx])
            pixels.append(keypoints[match.trainIdx].pt)
        pixels, depths, kept = self._sample_depths(depth_m, pixels)
        ids = [ids[index] for index in kept]
        points_camera = deproject_pixels(pixels, depths, self._camera_matrix) if len(pixels) else np.empty((0, 3))
        points_root = transform_points(self._root_from_camera, points_camera) if len(points_camera) else points_camera
        return ids, points_root, pixels.astype(np.float32)

    def update(self, image_rgb, depth_m, intrinsics, root_from_camera, *, stamp_ns,
               frame_id, generation, received_monotonic, now_monotonic=None):
        import cv2
        gray = self._gray8(image_rgb)
        self.identity_reacquired_after_gap = False
        if self._needs_reselect:
            return self.tracker.invalidate("anchor_reselect_required",
                generation=generation, stamp_ns=stamp_ns,
                received_monotonic=received_monotonic)
        if self._gray is None or self._pixels is None:
            self.tracker.reason = "no_anchor"
            return self.tracker.snapshot(received_monotonic if now_monotonic is None else now_monotonic)
        reference_gap = ((int(stamp_ns) - int(self._reference_stamp_ns)) / 1e9
                         if self._reference_stamp_ns is not None else float("inf"))
        if reference_gap > self.tracker.max_frame_gap_s:
            # The intervening images were not linked to the optical-flow reference.
            # Reacquire from descriptors pinned to the selected anchor identity.
            ids, points_root, pixels = self._descriptor_observations(gray, depth_m)
            result = self.tracker.update(stamp_ns, ids, points_root,
                generation=generation, received_monotonic=received_monotonic,
                now_monotonic=now_monotonic)
            if result.reason == "tracked":
                self._gray, self._pixels, self._ids = gray, pixels, ids
                self._reference_stamp_ns = int(stamp_ns)
                self._camera_matrix = np.asarray(intrinsics, dtype=float).reshape(3, 3).copy()
                self._root_from_camera = _matrix4(root_from_camera).copy()
                self.identity_reacquired_after_gap = True
            return result
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(self._gray, gray, self._pixels,
            None, winSize=(21, 21), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .03))
        if tracked is None or status is None:
            ids, points_root, pixels = self._descriptor_observations(gray, depth_m)
            result = self.tracker.update(stamp_ns, ids, points_root, generation=generation,
                received_monotonic=received_monotonic, now_monotonic=now_monotonic)
            if result.reason == "tracked":
                self._gray, self._pixels, self._ids = gray, pixels, ids
                self._reference_stamp_ns = int(stamp_ns)
            return result
        backwards, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self._gray, tracked,
            None, winSize=(21, 21), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .03))
        h, w = gray.shape
        p0, p1 = self._pixels.reshape(-1, 2), tracked.reshape(-1, 2)
        good = status.reshape(-1).astype(bool) & back_status.reshape(-1).astype(bool)
        good &= np.linalg.norm(backwards.reshape(-1, 2) - p0, axis=1) <= self.forward_backward_limit_px
        good &= ((p1[:, 0] >= 2) & (p1[:, 1] >= 2)
                 & (p1[:, 0] < w - 2) & (p1[:, 1] < h - 2))
        candidate = p1[good]
        candidate_ids = np.asarray(self._ids, dtype=int)[good]
        pixels, depths, kept_indices = self._sample_depths(depth_m, candidate)
        candidate_ids = candidate_ids[kept_indices]
        points_camera = deproject_pixels(pixels, depths, intrinsics) if len(pixels) else np.empty((0, 3))
        points_root = transform_points(root_from_camera, points_camera) if len(points_camera) else points_camera
        if len(pixels) < self.tracker.minimum_features:
            candidate_ids_list, descriptor_points, descriptor_pixels = self._descriptor_observations(gray, depth_m)
            if len(candidate_ids_list) >= self.tracker.minimum_features:
                candidate_ids, points_root, pixels = (np.asarray(candidate_ids_list, dtype=int),
                    descriptor_points, descriptor_pixels)
            else:
                candidate_ids, points_root, pixels = (np.empty((0,), dtype=int),
                    np.empty((0, 3)), np.empty((0, 2)))
        result = self.tracker.update(stamp_ns, candidate_ids.tolist(), points_root,
            generation=generation, received_monotonic=received_monotonic,
            now_monotonic=now_monotonic)
        if result.reason == "tracked":
            inliers = np.asarray(self.tracker._fit.inlier_indices, dtype=int)
            self._pixels = pixels[inliers].astype(np.float32)
            self._ids = candidate_ids[inliers].tolist()
            self._gray = gray
            self._reference_stamp_ns = int(stamp_ns)
            self._camera_matrix = np.asarray(intrinsics, dtype=float).reshape(3, 3).copy()
            self._root_from_camera = _matrix4(root_from_camera).copy()
        return result
