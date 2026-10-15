"""ROS-independent geometry and ownership rules for live scene updates."""

from dataclasses import dataclass
import math
from urllib.parse import quote

import numpy as np


@dataclass(frozen=True)
class TrackedBox:
    collision_id: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    last_seen_mono_s: float


def object_collision_id(track_id: str) -> str:
    """Map a stable tracker ID into this node's collision-object namespace."""
    if not isinstance(track_id, str) or not track_id or len(track_id) > 96:
        raise ValueError("detection must have a stable track id")
    encoded = quote(track_id, safe="-_.~")
    if not encoded or encoded in (".", ".."):
        raise ValueError("detection must have a stable track id")
    return f"colorbrick_{encoded}"


def validate_box(size_m, *, min_extent_m: float, max_extent_m: float):
    values = tuple(float(v) for v in size_m)
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("box dimensions must contain three finite extents")
    if min_extent_m <= 0 or max_extent_m < min_extent_m:
        raise ValueError("box extent limits are invalid")
    if any(v < min_extent_m or v > max_extent_m for v in values):
        raise ValueError("box extent lies outside configured limits")
    return values


def quaternion_matrix_xyzw(quaternion_xyzw):
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("quaternion has zero length")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def matrix_quaternion_xyzw(rotation):
    r = np.asarray(rotation, dtype=np.float64)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or np.linalg.det(r) < 0.999:
        raise ValueError("rotation must be a proper orthonormal matrix")
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = ((r[2, 1] - r[1, 2]) / s,
             (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = (0.25 * s, (r[0, 1] + r[1, 0]) / s,
             (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = ((r[0, 1] + r[1, 0]) / s, 0.25 * s,
             (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    else:
        s = math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
             0.25 * s, (r[1, 0] - r[0, 1]) / s)
    q = np.asarray(q, dtype=np.float64)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q *= -1
    return tuple(float(v) for v in q)


def transform_box_pose(target_from_source, center_m, orientation_xyzw):
    """Compose an SE(3) frame transform with one box pose."""
    transform = np.asarray(target_from_source, dtype=np.float64)
    center = np.asarray(center_m, dtype=np.float64)
    if transform.shape != (4, 4) or center.shape != (3,) or not np.isfinite(transform).all() or not np.isfinite(center).all():
        raise ValueError("box transform and center must be finite SE(3)/3-vectors")
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or np.linalg.det(transform[:3, :3]) < 0.999):
        raise ValueError("box transform must be a proper rigid SE(3)")
    source_rotation = quaternion_matrix_xyzw(orientation_xyzw)
    target_center = transform[:3, :3] @ center + transform[:3, 3]
    target_rotation = transform[:3, :3] @ source_rotation
    return (tuple(float(v) for v in target_center),
            matrix_quaternion_xyzw(target_rotation))


def stamp_is_fresh(*, now_s: float, stamp_s: float, max_age_s: float,
                   future_tolerance_s: float = 0.05) -> bool:
    if not all(math.isfinite(v) for v in (now_s, stamp_s, max_age_s, future_tolerance_s)):
        return False
    if stamp_s <= 0 or max_age_s <= 0 or future_tolerance_s < 0:
        return False
    age = now_s - stamp_s
    return -future_tolerance_s <= age <= max_age_s


def table_patches_around_keepout(center_xy_m, size_xy_m, keepout_center_xy_m,
                                 keepout_size_xy_m, *, min_patch_m=0.01):
    """Subtract a rectangular robot-base keepout from a rectangular table ROI.

    Returns non-overlapping (center_x, center_y, width, height) patches in the
    board frame. The result is at most four boxes and has no area inside the
    keepout footprint.
    """
    values = tuple(float(v) for v in (*center_xy_m, *size_xy_m,
                                      *keepout_center_xy_m, *keepout_size_xy_m,
                                      min_patch_m))
    if len(values) != 9 or not all(math.isfinite(v) for v in values):
        raise ValueError("table and keepout rectangles must be finite 2D values")
    cx, cy, width, height, kx, ky, kw, kh, minimum = values
    if min(width, height, kw, kh, minimum) <= 0:
        raise ValueError("table and keepout extents must be positive")
    x0, x1 = cx - width / 2, cx + width / 2
    y0, y1 = cy - height / 2, cy + height / 2
    kx0, kx1 = kx - kw / 2, kx + kw / 2
    ky0, ky1 = ky - kh / 2, ky + kh / 2
    ix0, ix1 = max(x0, kx0), min(x1, kx1)
    iy0, iy1 = max(y0, ky0), min(y1, ky1)
    if ix1 <= ix0 or iy1 <= iy0:
        return [(cx, cy, width, height)]
    candidates = [
        (cx, (y0 + iy0) / 2, width, iy0 - y0),
        (cx, (iy1 + y1) / 2, width, y1 - iy1),
        ((x0 + ix0) / 2, (iy0 + iy1) / 2, ix0 - x0, iy1 - iy0),
        ((ix1 + x1) / 2, (iy0 + iy1) / 2, x1 - ix1, iy1 - iy0),
    ]
    return [patch for patch in candidates
            if patch[2] >= minimum and patch[3] >= minimum]


class LiveObjectRegistry:
    """Bounded TTL registry; it can only expire its own colorbrick IDs."""

    def __init__(self, *, ttl_s: float, min_extent_m: float, max_extent_m: float,
                 padding_m: float = 0.0, max_objects: int = 32):
        if not math.isfinite(ttl_s) or ttl_s <= 0:
            raise ValueError("track TTL must be finite and positive")
        if not math.isfinite(padding_m) or padding_m < 0:
            raise ValueError("box padding must be finite and nonnegative")
        if max_objects <= 0:
            raise ValueError("max_objects must be positive")
        self.ttl_s = float(ttl_s)
        self.min_extent_m = float(min_extent_m)
        self.max_extent_m = float(max_extent_m)
        self.padding_m = float(padding_m)
        self.max_objects = int(max_objects)
        self._objects: dict[str, TrackedBox] = {}

    def update(self, track_id, center_m, size_m, orientation_xyzw, *, now_mono_s):
        key = object_collision_id(track_id)
        center = tuple(float(v) for v in center_m)
        if len(center) != 3 or not all(math.isfinite(v) for v in center):
            raise ValueError("box center must contain three finite coordinates")
        size = validate_box(size_m, min_extent_m=self.min_extent_m,
                            max_extent_m=self.max_extent_m)
        if not math.isfinite(now_mono_s):
            raise ValueError("monotonic receipt time must be finite")
        quat = tuple(float(v) for v in orientation_xyzw)
        quaternion_matrix_xyzw(quat)
        if key not in self._objects and len(self._objects) >= self.max_objects:
            raise ValueError("tracked-object limit reached")
        padded_size = tuple(v + 2 * self.padding_m for v in size)
        item = TrackedBox(key, center, padded_size, quat, float(now_mono_s))
        self._objects[key] = item
        return item

    def expire(self, *, now_mono_s):
        if not math.isfinite(now_mono_s):
            raise ValueError("monotonic time must be finite")
        expired = [key for key, item in self._objects.items()
                   if now_mono_s - item.last_seen_mono_s >= self.ttl_s]
        for key in expired:
            del self._objects[key]
        return sorted(expired)

