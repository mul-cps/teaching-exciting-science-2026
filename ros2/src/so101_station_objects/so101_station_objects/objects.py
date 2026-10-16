"""RGB-D colored-object observations and unqualified geometric grasp proposals."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import time
from copy import deepcopy

import cv2
import numpy as np


def _repository_root():
    configured = os.environ.get("SO101_STATION_ROOT")
    candidates = ([Path(configured)] if configured else []) + [
        Path(__file__).resolve().parents[4], Path.cwd(),
    ]
    for candidate in candidates:
        if (candidate / "workshop/digital_twin/grasp_geometry.py").is_file() and (
                candidate / "tools/propose_rgbd_grasp.py").is_file():
            return candidate.resolve()
    raise RuntimeError("cannot locate workshop grasp geometry; set SO101_STATION_ROOT")


_ROOT = _repository_root()
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tools.propose_rgbd_grasp import color_masks  # noqa: E402
from workshop.digital_twin.grasp_geometry import fit_support_plane, footprint_grasps  # noqa: E402

COLOR_NAMES = ("red", "green", "blue", "yellow")
MAX_SUPPORT_POINTS = 2000
DEFAULT_TABLE_ROI = (-0.03, 0.21, -0.03, 0.16, -0.05, 0.06)
DEFAULT_MAX_OBJECT_HEIGHT_M = 0.025
MAX_OBJECT_EXTENT_M = 0.065
MIN_OBJECT_EXTENT_M = 0.010
MIN_OBJECT_SURFACE_CLEARANCE_M = 0.0005
MIN_OBJECT_HEIGHT_M = 0.002
MAX_OBJECT_ASPECT_RATIO = 2.5
MARKER_HOLD_SECONDS = 1.0


class SupportPlaneUnavailable(ValueError):
    """The current depth frame has too few stable support-plane samples."""


def cached_marker_should_clear(last_update_mono, now_mono, clear_already_sent,
                               hold_seconds=MARKER_HOLD_SECONDS):
    """Clear a retained visualization once, after its bounded display hold."""
    return (last_update_mono is not None and not clear_already_sent
            and float(now_mono) - float(last_update_mono) > float(hold_seconds))


def camera_points(pixels, depth_m, intrinsics, distortion=None):
    """Back-project pixel centers with calibrated distortion and metric depth."""
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    depth_m = np.asarray(depth_m, dtype=np.float64).reshape(-1)
    if len(pixels) != len(depth_m) or not np.isfinite(pixels).all():
        raise ValueError("pixel and depth samples must be finite and aligned")
    valid = np.isfinite(depth_m) & (depth_m > 0)
    pixels, depth_m = pixels[valid], depth_m[valid]
    k = np.array([[intrinsics["fx"], 0, intrinsics["ppx"]],
                  [0, intrinsics["fy"], intrinsics["ppy"]], [0, 0, 1]], dtype=float)
    d = np.zeros(5) if distortion is None else np.asarray(distortion, dtype=float).reshape(-1)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("camera intrinsics and distortion must be finite with positive focal lengths")
    normalized = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    return np.column_stack((normalized * depth_m[:, None], depth_m))


def transform_points(points, target_from_source):
    points = np.asarray(points, dtype=np.float64)
    transform = np.asarray(target_from_source, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or transform.shape != (4, 4)
            or not np.isfinite(points).all() or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)):
        raise ValueError("expected finite Nx3 points and a homogeneous transform")
    return points @ transform[:3, :3].T + transform[:3, 3]


def _orient_plane_up(plane):
    normal = np.asarray(plane["normal"], dtype=float)
    offset = float(plane["d"])
    if normal[2] < 0:
        normal, offset = -normal, -offset
    if normal[2] < 0.85:
        raise ValueError("measured support plane is not tabletop-like in charuco_board")
    result = dict(plane, normal=normal, d=offset)
    # Board z=0 is only a prior after the observed plane independently agrees.
    if normal[2] >= math.cos(math.radians(5)) and abs(offset) <= 0.015:
        result.update(normal=np.array([0.0, 0.0, 1.0]), d=0.0,
                      plane_source="measured_table_consistent_with_board_z0")
    else:
        result["plane_source"] = "measured_table_plane"
    return result


def _fit_world_support(depth, valid_depth, rgb, intrinsics, distortion,
                       board_from_camera, support_roi, *, stride=8, max_points=MAX_SUPPORT_POINTS):
    height, width = depth.shape
    vivid = np.logical_or.reduce(tuple(color_masks(rgb).values()))
    excluded = cv2.dilate(vivid.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    yy, xx = np.mgrid[0:height:stride, 0:width:stride]
    sampled = valid_depth[yy, xx] & ~excluded[yy, xx]
    pixels = np.column_stack((xx[sampled], yy[sampled]))
    depths = depth[yy[sampled], xx[sampled]]
    camera = camera_points(pixels, depths, intrinsics, distortion)
    board = transform_points(camera, board_from_camera)
    bounds = support_roi
    inside = ((board[:, 0] >= bounds[0]) & (board[:, 0] <= bounds[1])
              & (board[:, 1] >= bounds[2]) & (board[:, 1] <= bounds[3])
              & (board[:, 2] >= bounds[4]) & (board[:, 2] <= bounds[5]))
    board = board[inside]
    if len(board) > max_points:
        indices = np.random.default_rng(17).choice(len(board), max_points, replace=False)
        board = board[indices]
    if len(board) < 30:
        raise SupportPlaneUnavailable("fewer than 30 observed table points inside bounded world ROI")
    plane = _orient_plane_up(fit_support_plane(board, threshold_m=0.003))
    inliers = int(plane["inlier_mask"].sum())
    if inliers < 30 or inliers / len(board) < 0.35:
        raise SupportPlaneUnavailable("no dominant measured support plane in table ROI")
    plane["support_points"] = len(board)
    plane["support_inliers"] = inliers
    return plane


def _plane_world_to_camera(plane, board_from_camera):
    rotation = board_from_camera[:3, :3]
    translation = board_from_camera[:3, 3]
    normal_world = np.asarray(plane["normal"], dtype=float)
    normal_camera = rotation.T @ normal_world
    d_camera = float(normal_world @ translation + plane["d"])
    return {"normal": normal_camera, "d": d_camera}


def _split_elongated_contours(contours):
    """Split touching same-color footprints only when their silhouette has a clear neck."""
    result = []
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        if min(width, height) == 0 or max(width, height) < 1.9 * min(width, height):
            result.append(contour)
            continue
        local = np.zeros((height, width), dtype=np.uint8)
        shifted = contour.copy()
        shifted[:, 0, 0] -= x
        shifted[:, 0, 1] -= y
        cv2.drawContours(local, [shifted], -1, 1, thickness=cv2.FILLED)
        horizontal = width >= height
        view = local if horizontal else local.T
        projection = view.sum(axis=0).astype(float)
        if len(projection) < 10:
            result.append(contour)
            continue
        smooth = np.convolve(projection, np.ones(3) / 3, mode="same")
        lo, hi = int(.22 * len(smooth)), int(.78 * len(smooth))
        if hi <= lo:
            result.append(contour)
            continue
        valley = lo + int(np.argmin(smooth[lo:hi]))
        left_peak = float(np.max(smooth[:valley]))
        right_peak = float(np.max(smooth[valley + 1:]))
        if (min(left_peak, right_peak) <= 0
                or smooth[valley] > .72 * min(left_peak, right_peak)):
            result.append(contour)
            continue
        pieces = []
        for first, last in ((0, valley), (valley + 1, view.shape[1])):
            piece = np.zeros_like(view)
            piece[:, first:last] = view[:, first:last]
            if not horizontal:
                piece = piece.T
            found, _ = cv2.findContours(piece, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for item in found:
                if cv2.contourArea(item) < .18 * cv2.contourArea(contour):
                    continue
                item = item.copy()
                item[:, 0, 0] += x
                item[:, 0, 1] += y
                pieces.append(item)
        if len(pieces) == 2:
            result.extend(pieces)
        else:
            result.append(contour)
    return result


def _component_geometry(contour, pixel_points, depth_values, intrinsics, distortion,
                        board_from_camera, plane_world, color, max_opening_m,
                        pickup_roi, max_object_height_m):
    camera = camera_points(pixel_points, depth_values, intrinsics, distortion)
    board = transform_points(camera, board_from_camera)
    normal = np.asarray(plane_world["normal"])
    above = board @ normal + plane_world["d"]
    bounds = pickup_roi
    valid = (np.isfinite(above) & (above >= MIN_OBJECT_SURFACE_CLEARANCE_M)
             & (above <= max_object_height_m)
             & _inside_roi(board, bounds))
    # Do not fit a partial in-ROI slice of a larger observed component. The
    # complete depth-supported footprint and height must pass pickup admission.
    if int(valid.sum()) < 8 or not np.all(valid):
        return None
    board = board[valid]
    above = above[valid]
    height_m = float(np.percentile(above, 90))
    if not MIN_OBJECT_HEIGHT_M <= height_m <= 0.25:
        return None

    reference = np.array([1.0, 0.0, 0.0])
    axis_x = reference - normal * float(reference @ normal)
    if np.linalg.norm(axis_x) < 0.1:
        reference = np.array([0.0, 1.0, 0.0])
        axis_x = reference - normal * float(reference @ normal)
    axis_x /= np.linalg.norm(axis_x)
    axis_y = np.cross(normal, axis_x)
    tangent = np.column_stack((board @ axis_x, board @ axis_y))
    centered = tangent - np.median(tangent, axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    long_axis = vt[0, 0] * axis_x + vt[0, 1] * axis_y
    if long_axis @ axis_x < 0:
        long_axis = -long_axis
    short_axis = np.cross(normal, long_axis)
    coords_long, coords_short = board @ long_axis, board @ short_axis
    lo_long, hi_long = np.percentile(coords_long, [5, 95])
    lo_short, hi_short = np.percentile(coords_short, [5, 95])
    size_long, size_short = max(0.005, float(hi_long - lo_long)), max(0.005, float(hi_short - lo_short))
    center = (long_axis * ((lo_long + hi_long) / 2)
              + short_axis * ((lo_short + hi_short) / 2)
              - plane_world["d"] * normal + normal * height_m / 2)
    basis = np.column_stack((long_axis, short_axis, normal))
    plane_camera = _plane_world_to_camera(plane_world, board_from_camera)
    # footprint_grasps requires the silhouette plane to be strictly within its
    # estimated object height; keep a micron of numerical headroom.
    contour_height = max(0.0, height_m - 1e-6)
    candidates = footprint_grasps(contour, plane_camera, intrinsics, height_m,
                                  max_opening_m, contour_height_m=contour_height)
    grasps = []
    for candidate in candidates:
        closing = board_from_camera[:3, :3] @ np.asarray(candidate["closing_axis"])
        closing /= np.linalg.norm(closing)
        # Pick across the short footprint when both orientations fit.
        opening = float(candidate["opening_m"])
        center_camera = np.asarray(candidate["center"])
        grasp_world = board_from_camera[:3, :3] @ center_camera + board_from_camera[:3, 3]
        approach_world = -normal
        grasps.append({"position": grasp_world, "closing_axis": closing,
                       "approach_direction": approach_world, "opening_m": opening,
                       "contacts_camera": np.asarray(candidate["contacts"]),
                       "axis_rank": opening})
    if grasps:
        grasps = [min(grasps, key=lambda item: item["axis_rank"])]
    observed_heights = np.percentile(above, [50, 90, 95])
    surface_centroid = np.median(transform_points(
        camera_points(pixel_points, depth_values, intrinsics, distortion),
        board_from_camera), axis=0)
    return {
        "color": color,
        "position": center,
        "surface_centroid": surface_centroid,
        "surface_centroid_spread_m": np.maximum(
            0.001, np.percentile(np.abs(board - surface_centroid), 90, axis=0)),
        "orientation_matrix": basis,
        "dimensions_m": np.array([size_long, size_short, height_m]),
        "dimension_uncertainty_m": np.array([
            max(0.001, float(np.std(coords_long))),
            max(0.001, float(np.std(coords_short))),
            max(0.001, float(observed_heights[2] - observed_heights[0])),
        ]),
        "height_m": height_m,
        "depth_points": int(valid.sum()),
        "contour": np.asarray(contour, dtype=np.float32),
        "grasps": grasps,
        "geometric_only": True,
        "pickup_eligible": True,
        "observation_geometry": "support_plane_box_estimate",
        "object_center_qualified": False,
        "dimensions_semantics": "support_plane_box_extent_estimate",
        "ik_validated": False,
        "collision_checked": False,
    }


def _validate_roi(roi, name):
    if (roi is not None and
            (len(roi) != 6 or not np.isfinite(roi).all()
             or any(roi[i] >= roi[i + 1] for i in (0, 2, 4)))):
        raise ValueError(f"{name} must be absent or contain finite ordered XYZ bounds")


def _inside_roi(points, roi):
    if roi is None:
        return np.ones(len(points), dtype=bool)
    return ((points[:, 0] >= roi[0]) & (points[:, 0] <= roi[1])
            & (points[:, 1] >= roi[2]) & (points[:, 1] <= roi[3])
            & (points[:, 2] >= roi[4]) & (points[:, 2] <= roi[5]))


def _visible_component_geometry(pixel_points, depth_values, intrinsics, distortion,
                                 board_from_camera, plane_world, color):
    """Describe the visible colored surface without inferring brick thickness."""
    camera = camera_points(pixel_points, depth_values, intrinsics, distortion)
    board = transform_points(camera, board_from_camera)
    normal = np.asarray(plane_world["normal"], dtype=float)
    reference = np.array([1.0, 0.0, 0.0])
    axis_x = reference - normal * float(reference @ normal)
    if np.linalg.norm(axis_x) < 0.1:
        reference = np.array([0.0, 1.0, 0.0])
        axis_x = reference - normal * float(reference @ normal)
    axis_x /= np.linalg.norm(axis_x)
    axis_y = np.cross(normal, axis_x)
    plane_coords = np.column_stack((board @ axis_x, board @ axis_y))
    _, _, vt = np.linalg.svd(plane_coords - np.median(plane_coords, axis=0),
                             full_matrices=False)
    long_axis = vt[0, 0] * axis_x + vt[0, 1] * axis_y
    if long_axis @ axis_x < 0:
        long_axis = -long_axis
    short_axis = np.cross(normal, long_axis)
    long_values, short_values = board @ long_axis, board @ short_axis
    lo_long, hi_long = np.percentile(long_values, [5, 95])
    lo_short, hi_short = np.percentile(short_values, [5, 95])
    dimensions = np.array([max(0.001, hi_long - lo_long),
                           max(0.001, hi_short - lo_short),
                           max(0.001, float(np.percentile(board @ normal, 95)
                                            - np.percentile(board @ normal, 5)))])
    centroid = np.median(board, axis=0)
    spread = np.maximum(0.001, np.percentile(np.abs(board - centroid), 90, axis=0))
    return {
        "color": color,
        "position": centroid.copy(),
        "surface_centroid": centroid,
        "surface_centroid_spread_m": spread,
        "orientation_matrix": np.column_stack((long_axis, short_axis, normal)),
        "dimensions_m": dimensions,
        "dimension_uncertainty_m": np.maximum(0.001, np.std(board, axis=0)),
        "height_m": None,
        "depth_points": len(board),
        "grasps": [],
        "geometric_only": True,
        "pickup_eligible": False,
        "observation_geometry": "visible_surface_centroid",
        "object_center_qualified": False,
        "dimensions_semantics": "visible_surface_extent_only",
        "ik_validated": False,
        "collision_checked": False,
    }


def analyze_rgbd(rgb, depth, intrinsics, distortion, depth_scale_m,
                 board_from_camera, world_roi=DEFAULT_TABLE_ROI,
                 max_opening_m=0.055, min_component_pixels=30,
                 max_object_height_m=DEFAULT_MAX_OBJECT_HEIGHT_M,
                 support_plane=None, *, observation_roi=None, pickup_roi=None,
                 support_roi=None):
    """Track visible colored surfaces; propose grasps only in the pickup ROI.

    `world_roi` remains the compatibility default for support fitting and
    pickup admission. `support_roi` and `pickup_roi` split those policies;
    `observation_roi=None` deliberately observes the full valid RGB-D image.
    """
    rgb = np.asarray(rgb)
    depth = np.asarray(depth)
    transform = np.asarray(board_from_camera, dtype=float)
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8
            or depth.ndim != 2 or depth.shape != rgb.shape[:2]
            or depth.dtype not in (np.uint16, np.float32)
            or not np.isfinite(transform).all() or transform.shape != (4, 4)):
        raise ValueError("expected matching uint8 RGB, uint16/float32 aligned depth and finite transform")
    if not np.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError("depth scale must be positive and finite")
    support_roi = world_roi if support_roi is None else support_roi
    pickup_roi = world_roi if pickup_roi is None else pickup_roi
    _validate_roi(support_roi, "support ROI")
    _validate_roi(pickup_roi, "pickup ROI")
    _validate_roi(observation_roi, "observation ROI")
    if not np.isfinite(max_object_height_m) or max_object_height_m <= 0:
        raise ValueError("pickup maximum object height must be positive and finite")
    depth_m = depth.astype(np.float32) * float(depth_scale_m)
    valid_depth = np.isfinite(depth_m) & (depth_m >= 0.10) & (depth_m <= 2.0)
    if int(valid_depth.sum()) < 100:
        raise ValueError("insufficient valid depth in camera frame")
    if support_plane is None:
        plane = _fit_world_support(depth_m, valid_depth, rgb, intrinsics, distortion,
                                   transform, support_roi)
    else:
        normal = np.asarray(support_plane.get("normal"), dtype=float).reshape(3)
        offset = float(support_plane.get("d"))
        if (not np.isfinite(normal).all() or not np.isfinite(offset)
                or not np.isclose(np.linalg.norm(normal), 1.0, atol=1e-3)
                or normal[2] < 0.85):
            raise ValueError("cached support plane must be finite, normalized, and upward-facing")
        plane = dict(support_plane, normal=normal, d=offset)
    masks = color_masks(rgb)
    objects = []
    for color in COLOR_NAMES:
        mask = masks[color].astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_component_pixels:
                continue
            x0 = int(stats[label, cv2.CC_STAT_LEFT])
            y0 = int(stats[label, cv2.CC_STAT_TOP])
            box_width = int(stats[label, cv2.CC_STAT_WIDTH])
            box_height = int(stats[label, cv2.CC_STAT_HEIGHT])
            label_roi = labels[y0:y0 + box_height, x0:x0 + box_width]
            yy_local, xx_local = np.nonzero(label_roi == label)
            yy, xx = yy_local + y0, xx_local + x0
            samples = depth_m[yy, xx]
            valid = valid_depth[yy, xx]
            if int(valid.sum()) < 8:
                continue
            pixels = np.column_stack((xx[valid], yy[valid]))
            camera = camera_points(pixels, samples[valid], intrinsics, distortion)
            board = transform_points(camera, transform)
            inside = _inside_roi(board, observation_roi)
            if int(inside.sum()) < 8:
                continue
            # Tracking uses every valid visible surface point. Pickup geometry
            # is computed separately below from support-plane-compatible points.
            component = np.zeros((box_height, box_width), dtype=np.uint8)
            component[pixels[inside, 1] - y0, pixels[inside, 0] - x0] = 1
            component = cv2.morphologyEx(component, cv2.MORPH_CLOSE,
                                         np.ones((3, 3), dtype=np.uint8))
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            # Split two same-color bricks joined by a narrow seam when the
            # one-dimensional silhouette has a clear internal valley.
            contours = _split_elongated_contours(contours)
            for contour in contours:
                contour_mask = np.zeros((box_height, box_width), dtype=np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 1, thickness=cv2.FILLED)
                selected = inside & (contour_mask[pixels[:, 1] - y0,
                                                   pixels[:, 0] - x0] != 0)
                if int(selected.sum()) < 8:
                    continue
                contour_global = contour.reshape(-1, 2).copy()
                contour_global[:, 0] += x0
                contour_global[:, 1] += y0
                visible = _visible_component_geometry(
                    pixels[selected], samples[valid][selected], intrinsics, distortion,
                    transform, plane, color)
                footprint = visible["dimensions_m"][:2]
                if (float(np.max(footprint)) > MAX_OBJECT_EXTENT_M
                        or float(np.min(footprint)) < MIN_OBJECT_EXTENT_M
                        or float(np.max(footprint) / np.min(footprint))
                        > MAX_OBJECT_ASPECT_RATIO):
                    continue
                # A supported estimate is available only when the component
                # independently passes the historic pickup/support filters.
                pickup = _component_geometry(
                    contour_global, pixels[selected], samples[valid][selected],
                    intrinsics, distortion, transform, plane, color, max_opening_m,
                    pickup_roi, max_object_height_m)
                if pickup is not None and _inside_roi(
                        np.asarray(pickup["position"]).reshape(1, 3), pickup_roi)[0]:
                    pickup["surface_centroid"] = visible["surface_centroid"]
                    pickup["surface_centroid_spread_m"] = visible[
                        "surface_centroid_spread_m"]
                    item = pickup
                else:
                    item = visible
                item["pixel_area"] = int(selected.sum())
                objects.append(item)
    return {"support_plane": plane, "objects": objects,
            "qualified": False, "frame_id": "charuco_board"}


class StableObjectTracker:
    """Velocity-predicted color-constrained association in the stable world frame."""
    def __init__(self, max_distance_m=0.06, max_age_s=1.0,
                 max_speed_m_s=0.5, prediction_distance_cap_m=0.35,
                 ambiguity_margin_m=0.015):
        self.max_distance = float(max_distance_m)
        self.max_age = float(max_age_s)
        self.max_speed = float(max_speed_m_s)
        self.prediction_distance_cap = float(prediction_distance_cap_m)
        self.ambiguity_margin = float(ambiguity_margin_m)
        if (not np.isfinite([self.max_distance, self.max_age, self.max_speed,
                             self.prediction_distance_cap, self.ambiguity_margin]).all()
                or min(self.max_distance, self.max_age, self.max_speed,
                       self.prediction_distance_cap) <= 0
                or self.ambiguity_margin < 0):
            raise ValueError("tracker gates must be finite and positive; ambiguity margin may be zero")
        self.next_id = 1
        self.tracks = {}
        self.owned = set()
        self.attachment_records = {}
        self.expired_tracks = []

    def ownership_status(self, payload):
        """Only acknowledged world replacement releases a reserved identity."""
        track_id = payload.get("track_id", "")
        for key, track in self.tracks.items():
            if track_id != f"{track['color']}-{key:04d}":
                continue
            if (payload.get("state") in {"locked", "reserved", "attached"} or
                    payload.get("state") == "fault" and payload.get("ownership_reserved") is True):
                self.owned.add(key)
                record = payload.get("attachment")
                if isinstance(record, dict):
                    self.attachment_records[key] = record
            elif payload.get("state") in {"detached", "unlocked"}:
                self.owned.discard(key)
                self.attachment_records.pop(key, None)
            return

    def update(self, objects, stamp_s, *, expected_owned_positions=None):
        stamp_s = float(stamp_s)
        self.expired_tracks = []
        retained = {}
        for key, track in self.tracks.items():
            age = stamp_s - track["stamp"]
            if key in self.owned or 0 <= age <= self.max_age:
                retained[key] = track
            elif age > self.max_age:
                self.expired_tracks.append({
                    "track_id": f"{track['color']}-{key:04d}",
                    "color": track["color"],
                    "last_position_world": track["position"].tolist(),
                    "last_position_semantics": track.get("position_semantics"),
                    "last_surface_centroid_world": np.asarray(
                        track.get("surface_centroid", []), dtype=float).tolist(),
                    "last_observation_stamp": track["stamp"],
                    "expired_at_stamp": stamp_s,
                    "reason": "not_observed_past_max_age",
                })
        self.tracks = retained
        pairs = []
        expected_owned_positions = expected_owned_positions or {}
        owned_candidates = {}
        for key in self.owned:
            expected = expected_owned_positions.get(key)
            if expected is None:
                # A stale tabletop position cannot identify a lifted brick.
                owned_candidates[key] = []
                continue
            expected = np.asarray(expected, dtype=float)
            if expected.shape != (3,) or not np.isfinite(expected).all():
                raise ValueError("expected held-object position must be finite XYZ")
            owned_candidates[key] = [index for index, obj in enumerate(objects)
                if obj["color"] == self.tracks[key]["color"] and
                np.linalg.norm(np.asarray(obj["position"]) - expected) <= self.max_distance]
        for obj_index, obj in enumerate(objects):
            position = np.asarray(obj["position"], dtype=float)
            for track_id, track in self.tracks.items():
                if track_id in self.owned:
                    if owned_candidates[track_id] != [obj_index]:
                        continue
                    reference = expected_owned_positions[track_id]
                else:
                    dt = max(0.0, stamp_s - track["stamp"])
                    predicted = track["position"] + track["velocity"] * dt
                    reference = track["position"]
                if obj["color"] == track["color"]:
                    distance = float(np.linalg.norm(position - reference))
                    if track_id in self.owned:
                        gate = self.max_distance
                    else:
                        dt = max(0.0, stamp_s - track["stamp"])
                        gate = min(self.prediction_distance_cap,
                                   self.max_distance + self.max_speed * dt)
                        distance = min(distance,
                                       float(np.linalg.norm(position - predicted)))
                    if distance <= gate:
                        pairs.append((distance, obj_index, track_id))
        # Do not guess through near-ties. Wait for separation and leave the
        # involved detections unassigned rather than transferring a color ID.
        ambiguous_objects, ambiguous_tracks = set(), set()
        for obj_index in range(len(objects)):
            ranked = sorted((distance, track_id) for distance, index, track_id in pairs
                            if index == obj_index)
            if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.ambiguity_margin:
                ambiguous_objects.add(obj_index)
                ambiguous_tracks.update(track_id for _, track_id in ranked[:2])
        for track_id in self.tracks:
            ranked = sorted((distance, index) for distance, index, candidate_track in pairs
                            if candidate_track == track_id)
            if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.ambiguity_margin:
                ambiguous_tracks.add(track_id)
                ambiguous_objects.update(index for _, index in ranked[:2])
        assigned_objects, assigned_tracks = set(), set()
        assignments = {}
        for _, obj_index, track_id in sorted(pairs):
            if obj_index in ambiguous_objects or track_id in ambiguous_tracks:
                continue
            if obj_index not in assigned_objects and track_id not in assigned_tracks:
                assignments[obj_index] = track_id
                assigned_objects.add(obj_index)
                assigned_tracks.add(track_id)
        # Withhold unmatched same-color observations while owned identity is
        # unresolved. Only a unique observation near timestamped gripper TF is
        # pickup evidence; a different tabletop brick cannot become that proof.
        ambiguous = set(ambiguous_objects)
        for track_id in self.owned - assigned_tracks:
            ambiguous.update(index for index, obj in enumerate(objects)
                             if index not in assigned_objects and
                             obj["color"] == self.tracks[track_id]["color"])
        for track_id in ambiguous_tracks:
            ambiguous.update(index for distance, index, candidate_track in pairs
                             if candidate_track == track_id)
        output = []
        for index, obj in enumerate(objects):
            if index in ambiguous:
                continue
            position = np.asarray(obj["position"], dtype=float)
            track_id = assignments.get(index)
            previous = self.tracks.get(track_id) if track_id is not None else None
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
                velocity = np.zeros(3)
                first_stamp = stamp_s
            else:
                dt = stamp_s - previous["stamp"]
                velocity = ((position - previous["position"]) / dt
                            if dt > 1e-6 else previous["velocity"])
                first_stamp = previous["first_stamp"]
            record = dict(obj, track_id=track_id, velocity=velocity,
                          age_s=max(0.0, stamp_s - first_stamp))
            self.tracks[track_id] = {"color": obj["color"], "position": position,
                                     "velocity": velocity, "stamp": stamp_s,
                                     "first_stamp": first_stamp,
                                     "position_semantics": obj.get("observation_geometry"),
                                     "surface_centroid": np.asarray(
                                         obj.get("surface_centroid", position), dtype=float).copy()}
            output.append(record)
        return output


def matrix_quaternion(rotation):
    """Convert a right-handed 3x3 rotation matrix to xyzw."""
    m = np.asarray(rotation, dtype=float)
    if m.shape != (3, 3) or not np.isfinite(m).all():
        raise ValueError("rotation must be finite 3x3")
    if not np.allclose(m.T @ m, np.eye(3), atol=1e-5) or np.linalg.det(m) < 0.999:
        raise ValueError("rotation must be proper and orthonormal")
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [ (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
              (m[1, 0] - m[0, 1]) / s, 0.25 * s ]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
        vector = [0.0, 0.0, 0.0]
        vector[i] = 0.25 * s
        vector[j] = (m[j, i] + m[i, j]) / s
        vector[k] = (m[k, i] + m[i, k]) / s
        q = [*vector, (m[k, j] - m[j, k]) / s]
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def grasp_pose_rotation(closing_axis, approach_direction):
    x_axis = np.asarray(closing_axis, dtype=float)
    z_axis = np.asarray(approach_direction, dtype=float)
    z_axis /= np.linalg.norm(z_axis)
    x_axis -= z_axis * float(x_axis @ z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def transform_from_msg(message):
    t = message.transform
    q = np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w], dtype=float)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("invalid TF quaternion")
    x, y, z, w = q / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return matrix


def elevated_pose_copy(pose, offset_m):
    """Return a vertically offset pose without mutating its source message."""
    result = deepcopy(pose)
    result.position.z += float(offset_m)
    return result


def pregrasp_position(contact_position, approach_direction, offset_m=0.06):
    """Offset a geometric contact point opposite its approach direction."""
    contact = np.asarray(contact_position, dtype=float).reshape(3)
    approach = np.asarray(approach_direction, dtype=float).reshape(3)
    norm = float(np.linalg.norm(approach))
    if (not np.isfinite(contact).all() or not np.isfinite(approach).all()
            or not np.isfinite(offset_m) or offset_m <= 0 or norm < 1e-9):
        raise ValueError("contact, approach, and pregrasp offset must be finite and valid")
    return contact - approach / norm * float(offset_m)


class ColoredObjectsNode:
    """ROS adapter; imports ROS only at instantiation for offline helper tests."""
    def __new__(cls, *args, **kwargs):
        import rclpy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import Pose, PoseArray
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformException, TransformListener
        from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
        from visualization_msgs.msg import Marker, MarkerArray
        import message_filters

        class _Node(Node):
            def __init__(self):
                cv2.setNumThreads(1)
                super().__init__("so101_colored_objects")
                defaults = {
                    "color_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "board_frame": "charuco_board",
                    "attachment_status_topic": "/so101_scene/attachment_status",
                    "registration_status_topic": "/so101_perception/charuco_world/status",
                    "require_registration_tracking": True,
                    "depth_scale_m": 0.001,
                    "world_roi_x_min": DEFAULT_TABLE_ROI[0], "world_roi_x_max": DEFAULT_TABLE_ROI[1],
                    "world_roi_y_min": DEFAULT_TABLE_ROI[2], "world_roi_y_max": DEFAULT_TABLE_ROI[3],
                    "world_roi_z_min": DEFAULT_TABLE_ROI[4], "world_roi_z_max": DEFAULT_TABLE_ROI[5],
                    "pickup_roi_x_min": DEFAULT_TABLE_ROI[0], "pickup_roi_x_max": DEFAULT_TABLE_ROI[1],
                    "pickup_roi_y_min": DEFAULT_TABLE_ROI[2], "pickup_roi_y_max": DEFAULT_TABLE_ROI[3],
                    "pickup_roi_z_min": DEFAULT_TABLE_ROI[4], "pickup_roi_z_max": DEFAULT_TABLE_ROI[5],
                    "observation_roi_enabled": False,
                    "observation_roi_x_min": -1.0, "observation_roi_x_max": 1.0,
                    "observation_roi_y_min": -1.0, "observation_roi_y_max": 1.0,
                    "observation_roi_z_min": -1.0, "observation_roi_z_max": 1.0,
                    "max_object_height_m": DEFAULT_MAX_OBJECT_HEIGHT_M,
                    "max_gripper_opening_m": 0.055,
                    "max_processing_hz": 5.0,
                }
                for key, value in defaults.items():
                    self.declare_parameter(key, value)
                self.board_frame = str(self.get_parameter("board_frame").value)
                self.require_registration_tracking = bool(
                    self.get_parameter("require_registration_tracking").value)
                self.registration_last_tracking_stamp = None
                self.depth_scale = float(self.get_parameter("depth_scale_m").value)
                self.support_roi = tuple(float(self.get_parameter(name).value) for name in (
                    "world_roi_x_min", "world_roi_x_max", "world_roi_y_min",
                    "world_roi_y_max", "world_roi_z_min", "world_roi_z_max"))
                self.pickup_roi = tuple(float(self.get_parameter(name).value) for name in (
                    "pickup_roi_x_min", "pickup_roi_x_max", "pickup_roi_y_min",
                    "pickup_roi_y_max", "pickup_roi_z_min", "pickup_roi_z_max"))
                self.observation_roi = None
                if bool(self.get_parameter("observation_roi_enabled").value):
                    self.observation_roi = tuple(float(self.get_parameter(name).value) for name in (
                        "observation_roi_x_min", "observation_roi_x_max",
                        "observation_roi_y_min", "observation_roi_y_max",
                        "observation_roi_z_min", "observation_roi_z_max"))
                self.max_opening = float(self.get_parameter("max_gripper_opening_m").value)
                self.max_object_height = float(self.get_parameter("max_object_height_m").value)
                requested_hz = float(self.get_parameter("max_processing_hz").value)
                if not np.isfinite([self.depth_scale, self.max_opening,
                                    self.max_object_height, requested_hz,
                                    *self.support_roi, *self.pickup_roi,
                                    *(self.observation_roi or ())]).all():
                    raise ValueError("object node parameters must be finite")
                if (self.depth_scale <= 0 or self.max_opening <= 0
                        or self.max_object_height <= 0 or requested_hz <= 0):
                    raise ValueError("depth scale, opening, object height, and processing frequency must be positive")
                _validate_roi(self.support_roi, "support ROI")
                _validate_roi(self.pickup_roi, "pickup ROI")
                _validate_roi(self.observation_roi, "observation ROI")
                self.period = 1.0 / min(5.0, requested_hz)
                self.last_process = 0.0
                self.last_warning = 0.0
                self.last_support_plane = None
                self.last_marker_update_mono = None
                self.last_marker_update_stamp = None
                self.marker_clear_sent = True
                self.bridge = CvBridge()
                self.tracker = StableObjectTracker()
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.detections_pub = self.create_publisher(Detection3DArray, "/so101/objects", 10)
                self.markers_pub = self.create_publisher(MarkerArray, "/so101/object_markers", 10)
                self.grasps_pub = self.create_publisher(PoseArray, "/so101/grasp_candidates", 10)
                self.contacts_pub = self.create_publisher(PoseArray, "/so101/grasp_contacts", 10)
                self.status_pub = self.create_publisher(String, "/so101/grasp_status", 10)
                self.attachment_sub = self.create_subscription(
                    String, self.get_parameter("attachment_status_topic").value,
                    self._attachment_status, 10)
                self.registration_sub = self.create_subscription(
                    String, self.get_parameter("registration_status_topic").value,
                    self._registration_status, 10)
                color = message_filters.Subscriber(
                    self, Image, self.get_parameter("color_topic").value,
                    qos_profile=qos_profile_sensor_data)
                depth = message_filters.Subscriber(
                    self, Image, self.get_parameter("depth_topic").value,
                    qos_profile=qos_profile_sensor_data)
                info = message_filters.Subscriber(
                    self, CameraInfo, self.get_parameter("camera_info_topic").value,
                    qos_profile=qos_profile_sensor_data)
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    [color, depth, info], queue_size=6, slop=0.04)
                self.sync.registerCallback(self._process)

            def _attachment_status(self, message):
                try:
                    payload = json.loads(message.data)
                    if isinstance(payload, dict):
                        self.tracker.ownership_status(payload)
                except (TypeError, ValueError, json.JSONDecodeError):
                    return

            def _registration_status(self, message):
                try:
                    payload = json.loads(message.data)
                    stamp = payload.get("stamp")
                    if (payload.get("state") == "tracking"
                            and payload.get("observation_valid") is True
                            and stamp is not None and math.isfinite(float(stamp))):
                        stamp = float(stamp)
                        if (self.registration_last_tracking_stamp is None
                                or stamp >= self.registration_last_tracking_stamp):
                            self.registration_last_tracking_stamp = stamp
                except (TypeError, ValueError, json.JSONDecodeError):
                    return

            def _publish_empty(self, stamp, *, clear_markers=True):
                array = Detection3DArray()
                array.header.stamp, array.header.frame_id = stamp, self.board_frame
                self.detections_pub.publish(array)
                poses = PoseArray()
                poses.header = array.header
                self.grasps_pub.publish(poses)
                contacts = PoseArray()
                contacts.header = array.header
                self.contacts_pub.publish(contacts)
                if clear_markers:
                    markers = MarkerArray()
                    clear = Marker()
                    clear.header = array.header
                    clear.action = Marker.DELETEALL
                    markers.markers.append(clear)
                    self.markers_pub.publish(markers)

            def _publish_unavailable(self, stamp, state, reason, **extra):
                from so101_station_perception.tag_observations import stamp_seconds
                now_mono = time.monotonic()
                clear_markers = cached_marker_should_clear(
                    self.last_marker_update_mono, now_mono, self.marker_clear_sent)
                if clear_markers:
                    # Do not publish empty arrays on each unavailable image:
                    # silence preserves the last observation stamp so
                    # consumers can reject it by age. Clear once at expiry.
                    clear_stamp = self.last_marker_update_stamp or stamp
                    self._publish_empty(clear_stamp, clear_markers=True)
                    self.marker_clear_sent = True
                message = String()
                message.data = json.dumps({
                    "schema": 1, "state": state, "frame_id": self.board_frame,
                    "stamp": stamp_seconds(stamp), "qualified": False,
                    "reason": reason, **extra,
                }, sort_keys=True, allow_nan=False)
                self.status_pub.publish(message)

            def _process(self, color_msg, depth_msg, info_msg):
                from so101_station_perception.tag_observations import (
                    camera_intrinsics, depth_scale_for_encoding, stamp_is_fresh, stamp_seconds)
                now = self.get_clock().now().nanoseconds * 1e-9
                stamp = stamp_seconds(color_msg.header.stamp)
                if not stamp_is_fresh(now, stamp, 0.5):
                    self._publish_unavailable(color_msg.header.stamp, "stale", "RGB-D image timestamp is stale")
                    return
                if self.require_registration_tracking:
                    tracking_stamp = self.registration_last_tracking_stamp
                    age = None if tracking_stamp is None else now - tracking_stamp
                    if age is None or age < 0 or age > 0.5:
                        self._publish_unavailable(
                            color_msg.header.stamp, "registration_unavailable",
                            "ChArUco world localization has no recent valid image observation",
                            registration_observation_age_s=age,
                            registration_status_topic=self.get_parameter("registration_status_topic").value)
                        return
                if time.monotonic() - self.last_process < self.period:
                    return
                self.last_process = time.monotonic()
                if (color_msg.header.frame_id != depth_msg.header.frame_id
                        or color_msg.header.frame_id != info_msg.header.frame_id
                        or abs(stamp - stamp_seconds(depth_msg.header.stamp)) > 0.04
                        or abs(stamp - stamp_seconds(info_msg.header.stamp)) > 0.04):
                    self._publish_unavailable(color_msg.header.stamp, "invalid_pair",
                                              "RGB-D and CameraInfo frame IDs or timestamps differ")
                    return
                try:
                    rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
                    depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                    intrinsics = camera_intrinsics(info_msg, depth.shape)
                    scale = depth_scale_for_encoding(depth_msg.encoding, depth, self.depth_scale)
                    tf = self.tf_buffer.lookup_transform(
                        self.board_frame, color_msg.header.frame_id,
                        Time.from_msg(color_msg.header.stamp), timeout=Duration(seconds=0.35))
                    board_from_camera = transform_from_msg(tf)
                    try:
                        report = analyze_rgbd(rgb, depth, intrinsics, info_msg.d, scale,
                                              board_from_camera, self.support_roi, self.max_opening,
                                              max_object_height_m=self.max_object_height,
                                              observation_roi=self.observation_roi,
                                              pickup_roi=self.pickup_roi)
                        plane = report["support_plane"]
                        self.last_support_plane = {
                            "normal": np.asarray(plane["normal"], dtype=float).copy(),
                            "d": float(plane["d"]),
                            "plane_source": plane["plane_source"],
                            "support_points": int(plane["support_points"]),
                            "support_inliers": int(plane["support_inliers"]),
                            "stamp": stamp,
                        }
                    except SupportPlaneUnavailable as error:
                        cache = self.last_support_plane
                        cache_age = None if cache is None else stamp - cache["stamp"]
                        if (cache is None or cache_age is None or cache_age < 0
                                or cache_age > 2.0):
                            raise
                        report = analyze_rgbd(rgb, depth, intrinsics, info_msg.d, scale,
                                              board_from_camera, self.support_roi, self.max_opening,
                                              max_object_height_m=self.max_object_height,
                                              observation_roi=self.observation_roi,
                                              pickup_roi=self.pickup_roi,
                                              support_plane={key: value for key, value in cache.items()
                                                             if key != "stamp"})
                        report["support_plane"]["plane_source"] = "cached_measured_support_plane"
                        report["support_plane"]["support_plane_age_s"] = float(cache_age)
                    if not stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9, stamp, 0.5):
                        self._publish_unavailable(color_msg.header.stamp, "stale",
                                                  "RGB-D pair expired while waiting for timestamped TF")
                        return
                    if self.require_registration_tracking:
                        tracking_stamp = self.registration_last_tracking_stamp
                        age = (self.get_clock().now().nanoseconds * 1e-9 - tracking_stamp
                               if tracking_stamp is not None else None)
                        if age is None or age < 0 or age > 0.5:
                            self._publish_unavailable(color_msg.header.stamp,
                                                      "registration_unavailable",
                                                      "ChArUco localization expired while waiting for timestamped TF",
                                                      registration_observation_age_s=age)
                            return
                    expected_owned_positions = {}
                    for key, record in self.tracker.attachment_records.items():
                        try:
                            source_frame = (record["world_frame"] if record.get("phase") == "locked"
                                            else record["link_name"])
                            held_tf = self.tf_buffer.lookup_transform(
                                self.board_frame, source_frame, Time.from_msg(color_msg.header.stamp))
                            board_from_link = transform_from_msg(held_tf)
                            local = np.asarray(record["world_position"] if record.get("phase") == "locked"
                                               else record["link_pose"]["position"], dtype=float)
                            expected_owned_positions[key] = board_from_link[:3, :3] @ local + board_from_link[:3, 3]
                        except (KeyError, TypeError, ValueError, TransformException):
                            continue
                    objects = self.tracker.update(report["objects"], stamp,
                                                  expected_owned_positions=expected_owned_positions)
                    report["objects"] = objects
                    report["expired_tracks"] = self.tracker.expired_tracks
                    report["stamp"] = stamp
                    report["support_plane"] = {
                        key: (value.tolist() if isinstance(value, np.ndarray) else value)
                        for key, value in report["support_plane"].items()
                        if key != "inlier_mask" and key != "residuals_m"}
                    self._publish_report(report, color_msg.header.stamp)
                except (ValueError, TypeError, cv2.error, TransformException) as error:
                    if time.monotonic() - self.last_warning >= 2.0:
                        self.get_logger().warning(f"Colored object observation unavailable: {error}")
                        self.last_warning = time.monotonic()
                    self._publish_unavailable(color_msg.header.stamp, "unavailable", str(error))

            def _publish_report(self, report, stamp):
                detections = Detection3DArray()
                detections.header.stamp, detections.header.frame_id = stamp, self.board_frame
                markers = MarkerArray()
                clear = Marker()
                clear.header = detections.header
                clear.action = Marker.DELETEALL
                markers.markers.append(clear)
                grasps = PoseArray()
                grasps.header = detections.header
                contacts = PoseArray()
                contacts.header = detections.header
                grasp_records = []
                for index, obj in enumerate(report["objects"]):
                    rotation = obj["orientation_matrix"]
                    quaternion = matrix_quaternion(rotation)
                    position = obj["position"]
                    dimensions = obj["dimensions_m"]
                    det = Detection3D()
                    det.header = detections.header
                    det.id = f"{obj['color']}-{obj['track_id']:04d}"
                    det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = map(float, position)
                    det.bbox.center.orientation.x, det.bbox.center.orientation.y = map(float, quaternion[:2])
                    det.bbox.center.orientation.z, det.bbox.center.orientation.w = map(float, quaternion[2:])
                    det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = map(float, dimensions)
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = obj["color"]
                    # This is a geometric color classification, not a calibrated
                    # detector probability. Leave the message default score.
                    hypothesis.pose.pose = det.bbox.center
                    det.results.append(hypothesis)
                    detections.detections.append(det)
                    cube = Marker()
                    cube.header = detections.header
                    cube.ns, cube.id, cube.type, cube.action = "colored_objects", index * 2, Marker.CUBE, Marker.ADD
                    cube.pose = det.bbox.center
                    cube.scale.x, cube.scale.y, cube.scale.z = map(float, dimensions)
                    colors = {"red": (1.0, .1, .1), "green": (.1, 1.0, .1),
                              "blue": (.1, .3, 1.0), "yellow": (1.0, .8, .05)}
                    cube.color.r, cube.color.g, cube.color.b = colors[obj["color"]]
                    cube.color.a = .35
                    cube.lifetime.sec = int(MARKER_HOLD_SECONDS)
                    markers.markers.append(cube)
                    label = Marker()
                    label.header = detections.header
                    label.ns, label.id, label.type, label.action = "colored_object_labels", index * 2 + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
                    label.pose = elevated_pose_copy(det.bbox.center,
                                                    float(dimensions[2]) / 2 + .015)
                    label.scale.z = .015
                    label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                    # RViz text can split on spaces; keep the visible label as
                    # the stable identifier so it stays next to the object.
                    label.text = det.id
                    label.lifetime.sec = int(MARKER_HOLD_SECONDS)
                    markers.markers.append(label)
                    for grasp_index, grasp in enumerate(obj["grasps"]):
                        rotation_grasp = grasp_pose_rotation(grasp["closing_axis"], grasp["approach_direction"])
                        q_grasp = matrix_quaternion(rotation_grasp)
                        contact = Pose()
                        contact.position.x, contact.position.y, contact.position.z = map(
                            float, grasp["position"])
                        contact.orientation.x, contact.orientation.y = map(float, q_grasp[:2])
                        contact.orientation.z, contact.orientation.w = map(float, q_grasp[2:])
                        contacts.poses.append(contact)
                        pregrasp = Pose()
                        preposition = pregrasp_position(grasp["position"],
                                                        grasp["approach_direction"], .06)
                        pregrasp.position.x, pregrasp.position.y, pregrasp.position.z = map(float, preposition)
                        pregrasp.orientation.x, pregrasp.orientation.y = map(float, q_grasp[:2])
                        pregrasp.orientation.z, pregrasp.orientation.w = map(float, q_grasp[2:])
                        grasps.poses.append(pregrasp)
                        grasp_records.append({
                            "candidate_id": f"{det.id}-g{grasp_index}", "track_id": obj["track_id"],
                            "color": obj["color"], "frame_id": self.board_frame,
                            "geometric_only": True, "ik_validated": False, "collision_checked": False,
                            "closing_axis_world": grasp["closing_axis"].tolist(),
                            "approach_direction_world": grasp["approach_direction"].tolist(),
                            "opening_m": grasp["opening_m"],
                            "contact_position_world": grasp["position"].tolist(),
                            "pregrasp_position_world": preposition.tolist(),
                            "pregrasp_offset_m": .06,
                        })
                self.detections_pub.publish(detections)
                self.markers_pub.publish(markers)
                self.grasps_pub.publish(grasps)
                self.contacts_pub.publish(contacts)
                self.last_marker_update_mono = time.monotonic()
                self.last_marker_update_stamp = detections.header.stamp
                self.marker_clear_sent = not bool(report["objects"])
                status = {
                    "schema": 1, "state": "tracking", "frame_id": self.board_frame,
                    "stamp": report["stamp"], "qualified": False,
                    "support_plane": report["support_plane"],
                    "objects": [{
                        "track_id": obj["track_id"], "color": obj["color"],
                        "position_world": obj["position"].tolist(),
                        "position_semantics": obj["observation_geometry"],
                        "surface_centroid_world": obj["surface_centroid"].tolist(),
                        "surface_centroid_spread_m": obj[
                            "surface_centroid_spread_m"].tolist(),
                        "object_center_qualified": obj["object_center_qualified"],
                        "pickup_eligible": obj["pickup_eligible"],
                        "velocity_world_m_s": obj["velocity"].tolist(), "age_s": obj["age_s"],
                        "dimensions_observed_m": obj["dimensions_m"].tolist(),
                        "dimensions_semantics": obj["dimensions_semantics"],
                        "dimension_uncertainty_m": obj["dimension_uncertainty_m"].tolist(),
                        "dimension_source": "robust visible RGB-D point extent; not CAD or full-object measurement",
                        "depth_points": obj["depth_points"], "geometric_only": True,
                        "ik_validated": False, "collision_checked": False,
                    } for obj in report["objects"]],
                    "grasp_candidates": grasp_records,
                    "expired_tracks": report.get("expired_tracks", []),
                }
                message = String()
                message.data = json.dumps(status, sort_keys=True, allow_nan=False)
                self.status_pub.publish(message)

        return _Node(*args, **kwargs)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ColoredObjectsNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
