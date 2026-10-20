"""RGB-D colored-object observations and unqualified geometric grasp proposals."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import time
from copy import deepcopy
from threading import Lock

import cv2
import numpy as np

from so101_station_objects.timing_diagnostic import (TimingRecorder, begin_node, site as timing_site,
                                finish_node, worker_timing, count_analyze)


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
from so101_station_objects.colored_object_validation import (  # noqa: E402
    DEFAULT_MAX_OBSERVATION_DEPTH_M,
    MIN_OBSERVATION_DEPTH_M,
    validate_observation_depth_limit,
    split_depth_coherent_components,
    validate_brick_silhouette,
    validate_colored_component,
    validate_child_visibility,
)
from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker  # noqa: E402
from so101_station_objects.percentile import linear_percentile  # noqa: E402
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
                       board_from_camera, support_roi, *, stride=8, max_points=MAX_SUPPORT_POINTS, masks=None):
    height, width = depth.shape
    # Reuse the identical full-image masks already needed by detection. This
    # avoids a second HSV conversion without changing support/pixel semantics.
    masks = color_masks(rgb) if masks is None else masks
    vivid = np.logical_or.reduce(tuple(masks.values()))
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
    valid = (np.isfinite(above) & (above >= MIN_OBJECT_SURFACE_CLEARANCE_M)
             & (above <= max_object_height_m))
    # Fit the complete observed component. Never crop its geometry to the
    # pickup ROI; admission is a separate full-component predicate below.
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
    pickup_eligible = bool(_inside_roi(board, pickup_roi).all())
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
        "pickup_eligible": pickup_eligible,
        "execution_eligible": False,
        "eligibility_reason": ("inside_pickup_roi" if pickup_eligible
                               else "outside_pickup_roi"),
        "execution_eligibility_reason": "not_ik_or_collision_checked",
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
    """Describe visible geometry in its measured 3D principal surface axes."""
    camera = camera_points(pixel_points, depth_values, intrinsics, distortion)
    board = transform_points(camera, board_from_camera)
    centroid = np.median(board, axis=0)
    centered = board - centroid
    covariance = centered.T @ centered / max(1, len(centered))
    _, eigenvectors = np.linalg.eigh(covariance)
    axes = eigenvectors[:, ::-1]
    # PCA eigenvector signs are arbitrary. Stabilize the first two signs, then
    # form the third as a cross product so the reported basis is right-handed.
    for index in (0, 1):
        dominant = int(np.argmax(np.abs(axes[:, index])))
        if axes[dominant, index] < 0:
            axes[:, index] *= -1
    axes[:, 2] = np.cross(axes[:, 0], axes[:, 1])
    axes[:, 2] /= np.linalg.norm(axes[:, 2])
    principal = centered @ axes
    limits = linear_percentile(principal, [5, 95], axis=0)
    extents = np.maximum(0.001, limits[1]-limits[0])
    long_axis, short_axis, surface_normal = axes.T
    spread = np.maximum(0.001, linear_percentile(np.abs(board - centroid), 90, axis=0))
    return {
        "color": color,
        "position": centroid.copy(),
        "surface_centroid": centroid,
        "surface_centroid_spread_m": spread,
        "orientation_matrix": axes,
        "dimensions_m": extents,
        "dimension_uncertainty_m": np.maximum(0.001, np.std(board, axis=0)),
        "height_m": None,
        "depth_points": len(board),
        "grasps": [],
        "geometric_only": True,
        "pickup_eligible": False,
        "execution_eligible": False,
        "eligibility_reason": "not_support_plane_qualified",
        "execution_eligibility_reason": "not_ik_or_collision_checked",
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
                 support_roi=None,
                 max_observation_depth_m=DEFAULT_MAX_OBSERVATION_DEPTH_M):
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
    max_observation_depth_m = validate_observation_depth_limit(max_observation_depth_m)
    depth_m = depth.astype(np.float32) * float(depth_scale_m)
    valid_depth = (np.isfinite(depth_m) & (depth_m >= MIN_OBSERVATION_DEPTH_M)
                   & (depth_m <= max_observation_depth_m))
    if int(valid_depth.sum()) < 100:
        raise ValueError("insufficient valid depth in camera frame")
    masks = color_masks(rgb)
    if support_plane is None:
        plane = _fit_world_support(depth_m, valid_depth, rgb, intrinsics, distortion,
                                   transform, support_roi, masks=masks)
    else:
        normal = np.asarray(support_plane.get("normal"), dtype=float).reshape(3)
        offset = float(support_plane.get("d"))
        if (not np.isfinite(normal).all() or not np.isfinite(offset)
                or not np.isclose(np.linalg.norm(normal), 1.0, atol=1e-3)
                or normal[2] < 0.85):
            raise ValueError("cached support plane must be finite, normalized, and upward-facing")
        plane = dict(support_plane, normal=normal, d=offset)
    objects = []
    validation_diagnostics = {
        "color_parent_components": 0, "depth_components": 0,
        "depth_accepted_components": 0, "rejected_components": 0,
        "silhouette_checks": 0, "silhouette_accepted": 0,
        "rejection_reasons": {}, "rejected_samples": [],
    }

    def reject_component(color, reason, parent_pixels, component_pixels=0, metrics=None):
        validation_diagnostics["rejected_components"] += 1
        reasons = validation_diagnostics["rejection_reasons"]
        reasons[reason] = reasons.get(reason, 0) + 1
        samples = validation_diagnostics["rejected_samples"]
        if len(samples) < 5:
            sample = {"color": color, "reason": reason,
                      "parent_pixels": int(parent_pixels),
                      "component_pixels": int(component_pixels)}
            if metrics:
                sample["metrics"] = metrics
            samples.append(sample)

    for color in COLOR_NAMES:
        mask = masks[color].astype(np.uint8)
        # Same8-connected parent pixels/stats/labels; explicit block-based
        # GRANA avoids OpenCV5 default Spaghetti full-image scan overhead.
        count, labels, stats, _ = cv2.connectedComponentsWithStatsWithAlgorithm(mask, 8, cv2.CV_32S, cv2.CCL_GRANA)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < min_component_pixels:
                continue
            validation_diagnostics["color_parent_components"] += 1
            # Reject wholly out-of-range parents from small existing 2D stats
            # before full masks, connected-depth splitting or 3D geometry.
            x, y, width, height = (int(stats[label, key]) for key in (
                cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            crop = (slice(y, y+height), slice(x, x+width))
            if not np.any(valid_depth[crop] & (labels[crop] == label)):
                reject_component(color, "outside_observation_depth_range", area)
                continue
            # Keep original RGB support, including invalid/out-of-range holes.
            parent_mask = labels[crop] == label
            pieces = split_depth_coherent_components(
                parent_mask, depth_m[crop], min_pixels=max(24, min_component_pixels),
                max_observation_depth_m=max_observation_depth_m,
                local_output=True, pixel_origin=(x, y))
            if not pieces:
                reject_component(color, "no_depth_coherent_component", area)
                continue
            for piece in pieces:
                validation_diagnostics["depth_components"] += 1
                offset_x, offset_y = piece.origin_xy
                piece_crop = (slice(offset_y, offset_y+piece.mask.shape[0]),
                              slice(offset_x, offset_x+piece.mask.shape[1]))
                local_intrinsics = dict(intrinsics, ppx=intrinsics["ppx"]-offset_x,
                                        ppy=intrinsics["ppy"]-offset_y)
                validation = validate_colored_component(
                    rgb[piece_crop], depth_m[piece_crop], piece.mask, local_intrinsics, distortion,
                    support_mask=piece.support_mask,
                    max_observation_depth_m=max_observation_depth_m)
                component_pixels = int(piece.mask.sum())
                if not validation.accepted:
                    reject_component(color, validation.reason, piece.parent_pixels,
                                     component_pixels, validation.metrics)
                    continue
                validation_diagnostics["depth_accepted_components"] += 1
                yy, xx = np.nonzero(piece.mask)
                yy, xx = yy+offset_y, xx+offset_x
                samples = depth_m[yy, xx]
                pixels = np.column_stack((xx, yy))
                x0, x1 = int(xx.min()), int(xx.max()) + 1
                y0, y1 = int(yy.min()), int(yy.max()) + 1
                box_width, box_height = x1 - x0, y1 - y0
                camera = camera_points(pixels, samples, intrinsics, distortion)
                board = transform_points(camera, transform)
                inside = _inside_roi(board, observation_roi)
                if int(inside.sum()) < 8:
                    reject_component(color, "outside_observation_roi", piece.parent_pixels,
                                     component_pixels, validation.metrics)
                    continue
                # Keep each depth-coherent piece isolated through contour
                # extraction; morphology must never reconnect foreground and
                # background surfaces separated by a depth discontinuity.
                component = np.zeros((box_height, box_width), dtype=np.uint8)
                component[pixels[:, 1] - y0, pixels[:, 0] - x0] = 1
                observed_component = component.copy()
                component = cv2.morphologyEx(component, cv2.MORPH_CLOSE,
                                             np.ones((3, 3), dtype=np.uint8))
                contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    reject_component(color, "no_observation_contour", piece.parent_pixels,
                                     component_pixels, validation.metrics)
                    continue
                # Split two same-color bricks joined by a narrow seam, but
                # only inside this already depth-coherent component.
                contours = _split_elongated_contours(contours)
                for contour in contours:
                    contour_mask = np.zeros((box_height, box_width), dtype=np.uint8)
                    cv2.drawContours(contour_mask, [contour], -1, 1, thickness=cv2.FILLED)
                    validation_diagnostics["silhouette_checks"] += 1
                    silhouette = validate_brick_silhouette(
                        contour, observed_mask=observed_component & contour_mask,
                        min_rect_fill=0.45)
                    if not silhouette.accepted:
                        reject_component(color, silhouette.reason, piece.parent_pixels,
                                         component_pixels, silhouette.metrics)
                        continue
                    validation_diagnostics["silhouette_accepted"] += 1
                    selected = contour_mask[pixels[:, 1] - y0, pixels[:, 0] - x0] != 0
                    if int((inside & selected).sum()) < 8:
                        reject_component(color, "insufficient_roi_depth_pixels",
                                         piece.parent_pixels, int(selected.sum()),
                                         validation.metrics)
                        continue
                    # ROI controls child admission only. Splitting, visibility
                    # and geometry retain every original GLOBAL child pixel;
                    # cropping cannot manufacture a smaller complete object.
                    visibility = validate_child_visibility(pixels[selected], depth_m.shape)
                    contour_global = contour.reshape(-1, 2).copy()
                    contour_global[:, 0] += x0
                    contour_global[:, 1] += y0
                    visible = _visible_component_geometry(
                        pixels[selected], samples[selected], intrinsics, distortion,
                        transform, plane, color)
                    footprint = visible["dimensions_m"][:2]
                    if (float(np.max(footprint)) > MAX_OBJECT_EXTENT_M
                            or float(np.min(footprint)) < MIN_OBJECT_EXTENT_M
                            or float(np.max(footprint) / np.min(footprint))
                            > MAX_OBJECT_ASPECT_RATIO):
                        reject_component(color, "visible_geometry_out_of_range",
                                         piece.parent_pixels, int(selected.sum()),
                                         validation.metrics)
                        continue
                    # A supported estimate is available only when the complete
                    # coherent component passes the support-height test.
                    pickup = None if not visibility.accepted else _component_geometry(
                        contour_global, pixels[selected], samples[selected],
                        intrinsics, distortion, transform, plane, color, max_opening_m,
                        pickup_roi, max_object_height_m)
                    if pickup is not None:
                        pickup["surface_centroid"] = visible["surface_centroid"]
                        pickup["surface_centroid_spread_m"] = visible[
                            "surface_centroid_spread_m"]
                        item = pickup
                    else:
                        item = visible
                    item["acquisition_eligible"] = visibility.accepted
                    item["acquisition_eligibility_reason"] = visibility.reason
                    item["image_boundary_sides"] = visibility.metrics["image_boundary_sides"]
                    item["pixel_area"] = int(selected.sum())
                    item["depth_validation"] = {
                        "accepted": True, "reason": validation.reason,
                        "metrics": validation.metrics,
                        "depth_cut_pixels": int(piece.depth_cut_pixels),
                    }
                    item["silhouette_validation"] = {
                        "accepted": True, "reason": silhouette.reason,
                        "metrics": silhouette.metrics,
                    }
                    objects.append(item)
    return {"support_plane": plane, "objects": objects,
            "qualified": False, "frame_id": "charuco_board",
            "observation_depth_range_m": [MIN_OBSERVATION_DEPTH_M, max_observation_depth_m],
            "validation_diagnostics": validation_diagnostics}


class StableObjectTracker:
    """Velocity association with optional consecutive-frame new-track probation.

    The utility defaults to immediate acquisition for offline callers. The ROS
    adapter explicitly requires three independent accepted acquisitions. Once
    confirmed, tracks keep the existing motion and ownership behavior.
    """
    def __init__(self, max_distance_m=0.06, max_age_s=1.0,
                 max_speed_m_s=0.5, prediction_distance_cap_m=0.35,
                 ambiguity_margin_m=0.015, acquisition_min_frames=1,
                 max_pending_tracks=64):
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
        for name, value in (("acquisition_min_frames", acquisition_min_frames),
                            ("max_pending_tracks", max_pending_tracks)):
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        self.acquisition_min_frames = int(acquisition_min_frames)
        self.max_pending_tracks = int(max_pending_tracks)
        self.last_acquisition_stamp = None
        self.next_id = 1
        self.tracks = {}
        self.owned = set()
        self.attachment_records = {}
        self.expired_tracks = []
        self.ineligible_new_observations = 0

    def invalidate_pending_acquisition(self, stamp_s, *, timestamp_is_trusted=False):
        """Break probation without trusting rejected exposures as a clock.

        Only a caller that checked a failure's timestamp against its current
        acquisition clock may advance the barrier. Future/stale failures must
        never prevent later valid exposures from updating confirmed tracks.
        """
        stamp_s = float(stamp_s)
        if not np.isfinite(stamp_s):
            raise ValueError("acquisition timestamp must be finite")
        if self.last_acquisition_stamp is not None and stamp_s <= self.last_acquisition_stamp:
            return
        if timestamp_is_trusted:
            self.last_acquisition_stamp = stamp_s
        self.tracks = {key: track for key, track in self.tracks.items() if track["confirmed"]}

    def acquisition_diagnostics(self):
        pending = sum(not track["confirmed"] for track in self.tracks.values())
        return {"minimum_frames": self.acquisition_min_frames,
                "pending_count": pending,
                "confirmed_count": len(self.tracks) - pending,
                "max_pending_tracks": self.max_pending_tracks,
                "ineligible_new_observations": self.ineligible_new_observations,
                "evidence": "consecutive_unambiguous_independent_acquisition_frames"}

    def ownership_status(self, payload):
        """Only acknowledged world replacement releases a reserved identity."""
        track_id = payload.get("track_id", "")
        for key, track in self.tracks.items():
            if not track["confirmed"] or track_id != f"{track['color']}-{key:04d}":
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
        if not np.isfinite(stamp_s):
            raise ValueError("acquisition timestamp must be finite")
        self.expired_tracks = []
        if self.last_acquisition_stamp is not None and stamp_s <= self.last_acquisition_stamp:
            # Republishing or reprocessing one exposure is not more evidence.
            return []
        self.last_acquisition_stamp = stamp_s
        self.ineligible_new_observations = 0
        retained = {}
        for key, track in self.tracks.items():
            age = stamp_s - track["stamp"]
            if key in self.owned or 0 <= age <= self.max_age:
                retained[key] = track
            elif age > self.max_age and track["confirmed"]:
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
        # Lock clear mutual-nearest pairs before checking ambiguity. A distant
        # unmatched track can be nearly equidistant from two already-resolved
        # detections; it must not veto their independent near-zero matches.
        def nearest_is_clear(ranked):
            return (len(ranked) <= 1 or
                    (ranked[1][0] - ranked[0][0] >= self.ambiguity_margin
                     and ranked[1][0] - ranked[0][0] > 1e-9))

        object_rankings = {
            obj_index: sorted((distance, track_id) for distance, index, track_id in pairs
                              if index == obj_index)
            for obj_index in range(len(objects))
        }
        track_rankings = {
            track_id: sorted((distance, obj_index) for distance, obj_index, candidate_track in pairs
                             if candidate_track == track_id)
            for track_id in self.tracks
        }
        clear_objects = {index for index, ranked in object_rankings.items()
                         if ranked and nearest_is_clear(ranked)}
        clear_tracks = {track_id for track_id, ranked in track_rankings.items()
                        if ranked and nearest_is_clear(ranked)}
        assignments = {}
        for obj_index in clear_objects:
            nearest_track = object_rankings[obj_index][0][1]
            if nearest_track not in clear_tracks:
                continue
            if track_rankings[nearest_track][0][1] == obj_index:
                assignments[obj_index] = nearest_track
        assigned_objects = set(assignments)
        assigned_tracks = set(assignments.values())

        # Do not guess through remaining near-ties. Wait for separation and
        # leave the involved detections unassigned rather than transferring a
        # color ID.
        ambiguous_objects, ambiguous_tracks = set(), set()
        for obj_index in range(len(objects)):
            if obj_index in assigned_objects:
                continue
            ranked = sorted((distance, track_id) for distance, index, track_id in pairs
                            if index == obj_index and track_id not in assigned_tracks)
            if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.ambiguity_margin:
                ambiguous_objects.add(obj_index)
                ambiguous_tracks.update(track_id for _, track_id in ranked[:2])
        for track_id in self.tracks:
            if track_id in assigned_tracks:
                continue
            ranked = sorted((distance, index) for distance, index, candidate_track in pairs
                            if candidate_track == track_id and index not in assigned_objects)
            if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.ambiguity_margin:
                ambiguous_tracks.add(track_id)
                ambiguous_objects.update(index for _, index in ranked[:2])
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
        # A candidate must occur in every analyzed acquisition, including frames
        # with no valid observations. Ambiguity never contributes confirmation.
        accepted_tracks = {key for index, key in assignments.items() if index not in ambiguous
                           and (self.tracks[key]["confirmed"] or
                                objects[index].get("acquisition_eligible", True) is True)}
        for key in list(self.tracks):
            if not self.tracks[key]["confirmed"] and key not in accepted_tracks:
                del self.tracks[key]
        # Count candidates that remain pending after this frame, so an early
        # new detection is not starved by a later candidate confirming now.
        pending_count = sum(not track["confirmed"] and
                            track["acquisition_observations"] + 1 < self.acquisition_min_frames
                            for track in self.tracks.values())
        output = []
        for index, obj in enumerate(objects):
            if index in ambiguous:
                continue
            position = np.asarray(obj["position"], dtype=float)
            track_id = assignments.get(index)
            previous = self.tracks.get(track_id) if track_id is not None else None
            if (obj.get("acquisition_eligible", True) is not True and
                    (previous is None or not previous["confirmed"])):
                self.ineligible_new_observations += 1
                continue
            if track_id is None:
                if self.acquisition_min_frames > 1 and pending_count >= self.max_pending_tracks:
                    continue
                track_id = self.next_id
                self.next_id += 1
                velocity = np.zeros(3)
                first_stamp = stamp_s
                observations = 1
                confirmed = self.acquisition_min_frames == 1
                pending_count += int(not confirmed)
            else:
                dt = stamp_s - previous["stamp"]
                velocity = ((position - previous["position"]) / dt
                            if dt > 1e-6 else previous["velocity"])
                first_stamp = previous["first_stamp"]
                observations = min(self.acquisition_min_frames,
                                   previous["acquisition_observations"] + 1)
                confirmed = previous["confirmed"] or observations >= self.acquisition_min_frames
            record = dict(obj, track_id=track_id, velocity=velocity,
                          age_s=max(0.0, stamp_s - first_stamp),
                          acquisition_state="confirmed", acquisition_observations=observations)
            self.tracks[track_id] = {"color": obj["color"], "position": position,
                                     "velocity": velocity, "stamp": stamp_s,
                                     "first_stamp": first_stamp,
                                     "confirmed": confirmed,
                                     "acquisition_observations": observations,
                                     "position_semantics": obj.get("observation_geometry"),
                                     "surface_centroid": np.asarray(
                                         obj.get("surface_centroid", position), dtype=float).copy()}
            if confirmed:
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
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from rcl_interfaces.msg import ParameterDescriptor
        from tf2_ros import Buffer, TransformException, TransformListener
        from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
        from visualization_msgs.msg import Marker, MarkerArray
        import message_filters

        class _Node(Node):
            def __init__(self):
                cv2.setNumThreads(1)
                super().__init__("so101_colored_objects")
                defaults = {
                    "allow_derived_source": False,
                    "derived_source_cohort": "",
                    "color_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "board_frame": "charuco_board",
                    "attachment_status_topic": "/so101_scene/attachment_status",
                    "registration_status_topic": "/so101_perception/charuco_world/status",
                    "require_registration_tracking": True,
                    "depth_scale_m": 0.001,
                    "max_observation_depth_m": DEFAULT_MAX_OBSERVATION_DEPTH_M,
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
                    "acquisition_min_frames": 3,
                    "max_pending_tracks": 64,
                }
                for key, value in defaults.items():
                    self.declare_parameter(key, value)
                self.declare_parameter("analysis_timing_diagnostics", False, ParameterDescriptor(read_only=True))
                diagnostic_flag = self.get_parameter("analysis_timing_diagnostics").value
                if type(diagnostic_flag) is not bool:
                    raise ValueError("analysis_timing_diagnostics must be bool")
                self.timing_recorder = TimingRecorder() if diagnostic_flag else None
                self.timing_timer = None
                self.timing_last_publish = None
                self.timing_closing = False
                if diagnostic_flag:
                    self.timing_recorder.startup = {"opencv_version": str(cv2.__version__)[:64],
                                                    "opencv_threads": int(cv2.getNumThreads())}
                    self.timing_pub = self.create_publisher(String, "~/analysis_timing", 1)
                    self.timing_timer = self.create_timer(.5, self._publish_timing)
                self.board_frame = str(self.get_parameter("board_frame").value)
                self.require_registration_tracking = bool(
                    self.get_parameter("require_registration_tracking").value)
                self.registration_last_tracking_stamp = None
                self.depth_scale = float(self.get_parameter("depth_scale_m").value)
                self.max_observation_depth = validate_observation_depth_limit(
                    self.get_parameter("max_observation_depth_m").value)
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
                self.last_warning = 0.0
                self.last_support_plane = None
                self.last_marker_update_mono = None
                self.last_marker_update_stamp = None
                self.marker_clear_sent = True
                self.bridge = CvBridge()
                self.tracker = StableObjectTracker(
                    acquisition_min_frames=self.get_parameter("acquisition_min_frames").value,
                    max_pending_tracks=self.get_parameter("max_pending_tracks").value)
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
                self.derived = None
                self._fresh_lock = Lock()
                self._fresh_signal_gate = Lock()
                self._fresh_pending = None
                self._fresh_generation = 0
                self._fresh_closed = False
                self._fresh_next_attempt = 0.0
                self._fresh_guard = self.create_guard_condition(self._process_offered_derived)
                self._derived_group = MutuallyExclusiveCallbackGroup()
                if self.get_parameter("allow_derived_source").value:
                    from so101_station_perception.derived_aligned import DerivedAligned
                    self.derived = DerivedAligned(
                        self, self.tf_buffer, self._process, period_s=self.period,
                        callback_group=self._derived_group, health_callback_group=self._derived_group,
                        metadata_offer=self._offer_derived)
                else:
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
                self.analysis_ready_guard = self.create_guard_condition(self._consume_analysis)
                self.analysis_worker = LatestOnlyAnalysisWorker(
                    self._analyze_job, min_start_interval_s=self.period,
                    on_ready=self.analysis_ready_guard.trigger, diagnostics=diagnostic_flag)
                self.last_analysis_result_stamp = None
                self.last_analysis_notification_error = None
                # Keep fault/lease invalidation active even when no result exists.
                self.analysis_timer = self.create_timer(0.05, self._consume_analysis_faults)

            def _offer_derived(self, token):
                # Metadata/reference handoff only, never decode, TF or image copies.
                with self._fresh_signal_gate:
                    with self._fresh_lock:
                        if self._fresh_closed:return
                        if token is None:
                            self._fresh_generation += 1
                            self._fresh_pending = None
                            return
                        self._fresh_pending = (token, self._fresh_generation)
                    self._fresh_guard.trigger()

            def _fresh_current(self, token, generation):
                with self._fresh_lock:
                    current = not self._fresh_closed and generation == self._fresh_generation
                return current and self.derived.token_is_current(token)

            def _process_offered_derived(self):
                with self._fresh_lock:
                    pending, self._fresh_pending = self._fresh_pending, None
                worker = getattr(self, "analysis_worker", None)
                if pending is None or worker is None:return
                token, generation = pending
                readiness = worker.availability()
                ready_after = max(readiness["ready_after"], self._fresh_next_attempt)
                if (not readiness["available"] or time.monotonic() < ready_after
                        or token["lineage_receipt"] < ready_after
                        or not self._fresh_current(token, generation)):
                    return
                # Bound expensive attempts even when TF/admission fails before submit.
                # This scheduling clock never replaces a token's original receipts.
                self._fresh_next_attempt = time.monotonic() + self.period
                self._process(token["color"], token["aligned"], token["color_info"],
                              token, fresh_generation=generation)

            def _publish_timing(self):
                try:
                    recorder = self.timing_recorder
                    if recorder is None or recorder.closed or self.timing_closing:return
                    now = time.monotonic()
                    if self.timing_last_publish is not None and now-self.timing_last_publish < .5:return
                    worker = getattr(self, "analysis_worker", None)
                    if worker is not None:
                        recorder.losses(worker.pending_replacements, worker.overwritten_results,
                                        worker.diagnostic_losses["shutdown"])
                    message = String()
                    message.data = recorder.drain()
                    self.timing_pub.publish(message)
                    self.timing_last_publish = time.monotonic()
                except Exception:
                    # A diagnostic cannot become a source/admission fault.
                    if self.timing_recorder is not None:
                        self.timing_recorder._add("invalid")

            def destroy_node(self):
                with self._fresh_lock:
                    self._fresh_closed = True
                    self._fresh_generation += 1
                    self._fresh_pending = None
                # Join any metadata trigger/handoff before ROS guard destruction.
                with self._fresh_signal_gate:pass
                self.timing_closing = True
                timer = getattr(self, "timing_timer", None)
                if timer is not None:timer.cancel()
                if getattr(self, "derived", None) is not None:
                    self.derived.close()
                worker = getattr(self, "analysis_worker", None)
                if worker is not None and not worker.close():
                    self.get_logger().error("Analysis worker cleanup incomplete; guard/context retained for supervisor")
                    return False
                recorder = getattr(self, "timing_recorder", None)
                if recorder is not None:
                    recorder.losses(worker.pending_replacements, worker.overwritten_results,
                                    worker.diagnostic_losses["shutdown"])
                    recorder.close()
                return super().destroy_node()

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
                if getattr(self, "timing_recorder", None) is not None:finish_node(self, 0, reason)
                from so101_station_perception.tag_observations import stamp_is_fresh, stamp_seconds
                acquisition_stamp = stamp_seconds(stamp)
                now = self.get_clock().now().nanoseconds * 1e-9
                self.tracker.invalidate_pending_acquisition(
                    acquisition_stamp, timestamp_is_trusted=stamp_is_fresh(now, acquisition_stamp, 0.5))
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
                    "reason": reason, "acquisition": self.tracker.acquisition_diagnostics(),
                    "observation_depth_range_m": [MIN_OBSERVATION_DEPTH_M, self.max_observation_depth],
                    **extra,
                }, sort_keys=True, allow_nan=False)
                self.status_pub.publish(message)

            def _process(self, color_msg, depth_msg, info_msg, derived_token=None, *, fresh_generation=None):
                from so101_station_perception.tag_observations import (
                    camera_intrinsics, depth_scale_for_encoding, stamp_is_fresh, stamp_seconds)
                timing = begin_node(self, derived_token) if getattr(self, "timing_recorder", None) is not None else None
                if timing is not None:timing_site(timing, 1, 14)
                if derived_token is not None:
                    try:self.derived.check(derived_token)
                    except ValueError as error:
                        self._publish_unavailable(color_msg.header.stamp, "unavailable", str(error))
                        return
                TimingRecorder.clock(timing, 15) if timing is not None else None
                if timing is not None:timing_site(timing, 6)
                now = self.get_clock().now().nanoseconds * 1e-9
                stamp = stamp_seconds(color_msg.header.stamp)
                if not stamp_is_fresh(now, stamp, 0.5):
                    self._publish_unavailable(color_msg.header.stamp, "stale", "RGB-D image timestamp is stale")
                    return
                if timing is not None:timing_site(timing, 7)
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
                if timing is not None:timing_site(timing, 8)
                if (color_msg.header.frame_id != depth_msg.header.frame_id
                        or color_msg.header.frame_id != info_msg.header.frame_id
                        or abs(stamp - stamp_seconds(depth_msg.header.stamp)) > 0.04
                        or abs(stamp - stamp_seconds(info_msg.header.stamp)) > 0.04):
                    self._publish_unavailable(color_msg.header.stamp, "invalid_pair",
                                              "RGB-D and CameraInfo frame IDs or timestamps differ")
                    return
                try:
                    if timing is not None:timing_site(timing, 9)
                    rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
                    depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                    intrinsics = camera_intrinsics(info_msg, depth.shape)
                    scale = depth_scale_for_encoding(depth_msg.encoding, depth, self.depth_scale)
                    if timing is not None:timing_site(timing, 9, 1)
                    tf = self.tf_buffer.lookup_transform(
                        self.board_frame, color_msg.header.frame_id,
                        Time.from_msg(color_msg.header.stamp), timeout=Duration(seconds=0.35))
                    TimingRecorder.clock(timing, 2) if timing is not None else None
                    board_from_camera = transform_from_msg(tf)
                    now = self.get_clock().now().nanoseconds * 1e-9
                    if timing is not None:timing_site(timing, 13)
                    if not stamp_is_fresh(now, stamp, 0.5):
                        self._publish_unavailable(color_msg.header.stamp, "stale",
                                                  "RGB-D pair expired while waiting for timestamped TF")
                        return
                    if timing is not None:timing_site(timing, 14)
                    if self.require_registration_tracking:
                        tracking_stamp = self.registration_last_tracking_stamp
                        age = now - tracking_stamp if tracking_stamp is not None else None
                        if age is None or age < 0 or age > 0.5:
                            self._publish_unavailable(color_msg.header.stamp,
                                                      "registration_unavailable",
                                                      "ChArUco localization expired while waiting for timestamped TF",
                                                      registration_observation_age_s=age)
                            return
                    cache = self.last_support_plane
                    cache_snapshot = None if cache is None else {
                        **cache, "normal": np.asarray(cache["normal"], dtype=float).copy()}
                    if timing is not None:timing_site(timing, 2, 16)
                    TimingRecorder.clock(timing, 3) if timing is not None else None
                    if derived_token is not None:
                        self.derived.check(derived_token)
                    TimingRecorder.clock(timing, 17) if timing is not None else None
                    if timing is not None:timing_site(timing, 9)
                    if (fresh_generation is not None
                            and not self._fresh_current(derived_token, fresh_generation)):
                        raise ValueError("derived offer cancelled before image copy")
                    job = {
                        "derived_token": derived_token,
                        "stamp": stamp, "ros_stamp": color_msg.header.stamp,
                        "rgb": np.asarray(rgb).copy(), "depth": np.asarray(depth).copy(),
                        "intrinsics": dict(intrinsics),
                        "distortion": np.asarray(info_msg.d, dtype=float).copy(),
                        "scale": float(scale), "board_from_camera": board_from_camera.copy(),
                        "support_plane_cache": cache_snapshot,
                    }
                    if timing is not None:job["_analysis_timing"] = timing
                    if fresh_generation is None:
                        self.analysis_worker.submit(job)
                    else:
                        # Cancellation and submission have a single ownership boundary.
                        # No offer lock is held while accessing worker/adapter locks.
                        with self._fresh_signal_gate:
                            if not self._fresh_current(derived_token, fresh_generation):
                                raise ValueError("derived offer cancelled before submission")
                            self.analysis_worker.submit(job)
                except (ValueError, TypeError, cv2.error, TransformException) as error:
                    if time.monotonic() - self.last_warning >= 2.0:
                        self.get_logger().warning(f"Colored object observation unavailable: {error}")
                        self.last_warning = time.monotonic()
                    self._publish_unavailable(color_msg.header.stamp, "unavailable", str(error))

            def _analyze_job(self, job):
                """Pure CPU analysis; this worker never accesses ROS or mutable tracking state."""
                diagnostic = job.get("_analysis_timing")
                try:
                    if diagnostic is not None:count_analyze(diagnostic)
                    return analyze_rgbd(
                        job["rgb"], job["depth"], job["intrinsics"], job["distortion"],
                        job["scale"], job["board_from_camera"], self.support_roi,
                        self.max_opening, max_object_height_m=self.max_object_height,
                        observation_roi=self.observation_roi, pickup_roi=self.pickup_roi,
                        max_observation_depth_m=self.max_observation_depth)
                except SupportPlaneUnavailable:
                    cache = job["support_plane_cache"]
                    cache_age = None if cache is None else job["stamp"] - cache["stamp"]
                    if (cache is None or cache_age is None or cache_age < 0 or cache_age > 2.0):
                        raise
                    if diagnostic is not None:count_analyze(diagnostic)
                    report = analyze_rgbd(
                        job["rgb"], job["depth"], job["intrinsics"], job["distortion"],
                        job["scale"], job["board_from_camera"], self.support_roi,
                        self.max_opening, max_object_height_m=self.max_object_height,
                        observation_roi=self.observation_roi, pickup_roi=self.pickup_roi,
                        support_plane={key: value for key, value in cache.items()
                                       if key != "stamp"},
                        max_observation_depth_m=self.max_observation_depth)
                    report["support_plane"]["plane_source"] = "cached_measured_support_plane"
                    report["support_plane"]["support_plane_age_s"] = float(cache_age)
                    return report

            def _consume_analysis_faults(self):
                if getattr(self, "derived", None) is not None:
                    fault = self.derived.pop_fault()
                    if fault:
                        self._offer_derived(None)
                        timing = getattr(self, "_timing_current", None)
                        if timing is not None:timing_site(timing, 15)
                        self._publish_unavailable(self.get_clock().now().to_msg(), "unavailable", fault)
                error = self.analysis_worker.notification_error
                if error is not None:
                    self._offer_derived(None)
                    self.analysis_worker.take_result()  # Discard after failed wakeup; no timer fallback publication.
                    if error != self.last_analysis_notification_error:
                        self.last_analysis_notification_error = error
                        recorder = getattr(self, "timing_recorder", None)
                        timing = getattr(self, "_timing_current", None)
                        if recorder is not None:recorder._add("notification_lost")
                        if timing is not None:timing_site(timing, 17)
                        self._publish_unavailable(self.get_clock().now().to_msg(), "unavailable",
                                                  "analysis result notification failed: "+error)
                    return False
                return True

            def _consume_analysis(self):
                from so101_station_perception.tag_observations import stamp_is_fresh
                if not self._consume_analysis_faults():
                    return
                recorder = getattr(self, "timing_recorder", None)
                if recorder is not None:
                    packet = self.analysis_worker.take_result_with_diagnostics()
                    result = None if packet is None else packet[0]
                else:
                    result = self.analysis_worker.take_result()
                if result is None:
                    return
                job, report, error = result
                timing = job.get("_analysis_timing")
                if recorder is not None:
                    self._timing_current = timing
                    worker_timing(timing, packet[1])
                if timing is not None:timing_site(timing, 10)
                stamp = job["stamp"]
                now = self.get_clock().now().nanoseconds * 1e-9
                if (stamp <= (self.last_analysis_result_stamp or float("-inf"))
                        or not stamp_is_fresh(now, stamp, 0.5)):
                    self.tracker.invalidate_pending_acquisition(stamp)
                    if timing is not None:finish_node(self, 0, "result duplicate/stale")
                    return
                if timing is not None:timing_site(timing, 11)
                if self.require_registration_tracking:
                    tracking_stamp = self.registration_last_tracking_stamp
                    age = now - tracking_stamp if tracking_stamp is not None else None
                    if age is None or age < 0 or age > 0.5:
                        self._publish_unavailable(
                            job["ros_stamp"], "registration_unavailable",
                            "ChArUco localization expired during RGB-D analysis",
                            registration_observation_age_s=age)
                        return
                if timing is not None:timing_site(timing, 12)
                if error is not None:
                    if time.monotonic() - self.last_warning >= 2.0:
                        self.get_logger().warning(f"Colored object observation unavailable: {error}")
                        self.last_warning = time.monotonic()
                    self._publish_unavailable(job["ros_stamp"], "unavailable", str(error))
                    return
                if timing is not None:timing_site(timing, 3, 18)
                if job.get("derived_token") is not None:
                    try:self.derived.check(job["derived_token"])
                    except ValueError as error:
                        self._publish_unavailable(job["ros_stamp"], "unavailable", str(error))
                        return
                TimingRecorder.clock(timing, 19) if timing is not None else None
                self.last_analysis_result_stamp = stamp
                plane = report["support_plane"]
                if plane.get("plane_source") != "cached_measured_support_plane":
                    self.last_support_plane = {
                        "normal": np.asarray(plane["normal"], dtype=float).copy(),
                        "d": float(plane["d"]), "plane_source": plane["plane_source"],
                        "support_points": int(plane["support_points"]),
                        "support_inliers": int(plane["support_inliers"]), "stamp": stamp,
                    }
                TimingRecorder.clock(timing, 10) if timing is not None else None
                expected_owned_positions = {}
                for key, record in self.tracker.attachment_records.items():
                    try:
                        source_frame = (record["world_frame"] if record.get("phase") == "locked"
                                        else record["link_name"])
                        held_tf = self.tf_buffer.lookup_transform(
                            self.board_frame, source_frame, Time.from_msg(job["ros_stamp"]))
                        board_from_link = transform_from_msg(held_tf)
                        local = np.asarray(record["world_position"] if record.get("phase") == "locked"
                                           else record["link_pose"]["position"], dtype=float)
                        expected_owned_positions[key] = board_from_link[:3, :3] @ local + board_from_link[:3, 3]
                    except (KeyError, TypeError, ValueError, TransformException):
                        continue
                TimingRecorder.clock(timing, 11) if timing is not None else None
                if timing is not None:timing_site(timing, 4, 20)
                if job.get("derived_token") is not None:
                    try:self.derived.check(job["derived_token"])
                    except ValueError as error:
                        self._publish_unavailable(job["ros_stamp"], "unavailable", str(error))
                        return
                TimingRecorder.clock(timing, 21) if timing is not None else None
                objects = self.tracker.update(report["objects"], stamp,
                                              expected_owned_positions=expected_owned_positions)
                report["objects"] = objects
                report["expired_tracks"] = self.tracker.expired_tracks
                report["acquisition"] = self.tracker.acquisition_diagnostics()
                report["stamp"] = stamp
                report["support_plane"] = {
                    key: (value.tolist() if isinstance(value, np.ndarray) else value)
                    for key, value in report["support_plane"].items()
                    if key != "inlier_mask" and key != "residuals_m"}
                kwargs = {} if job.get("derived_token") is None else {"derived_token": job["derived_token"]}
                self._publish_report(report, job["ros_stamp"], **kwargs)

            def _publish_report(self, report, stamp, derived_token=None):
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
                            "pickup_eligible": bool(obj["pickup_eligible"]),
                            "execution_eligible": False,
                            "eligibility_reason": obj["eligibility_reason"],
                            "execution_eligibility_reason": obj[
                                "execution_eligibility_reason"],
                            "closing_axis_world": grasp["closing_axis"].tolist(),
                            "approach_direction_world": grasp["approach_direction"].tolist(),
                            "opening_m": grasp["opening_m"],
                            "contact_position_world": grasp["position"].tolist(),
                            "pregrasp_position_world": preposition.tolist(),
                            "pregrasp_offset_m": .06,
                        })
                status = {
                    "schema": 1, "state": "tracking", "frame_id": self.board_frame,
                    "stamp": report["stamp"], "qualified": False,
                    "support_plane": report["support_plane"],
                    "observation_depth_range_m": report["observation_depth_range_m"],
                    "objects": [{
                        "track_id": obj["track_id"], "color": obj["color"],
                        "acquisition_state": obj["acquisition_state"],
                        "acquisition_observations": obj["acquisition_observations"],
                        "acquisition_eligible": obj["acquisition_eligible"],
                        "acquisition_eligibility_reason": obj["acquisition_eligibility_reason"],
                        "image_boundary_sides": obj["image_boundary_sides"],
                        "position_world": obj["position"].tolist(),
                        "position_semantics": obj["observation_geometry"],
                        "surface_centroid_world": obj["surface_centroid"].tolist(),
                        "surface_centroid_spread_m": obj[
                            "surface_centroid_spread_m"].tolist(),
                        "object_center_qualified": obj["object_center_qualified"],
                        "pickup_eligible": obj["pickup_eligible"],
                        "execution_eligible": obj["execution_eligible"],
                        "eligibility_reason": obj["eligibility_reason"],
                        "velocity_world_m_s": obj["velocity"].tolist(), "age_s": obj["age_s"],
                        "dimensions_observed_m": obj["dimensions_m"].tolist(),
                        "dimensions_semantics": obj["dimensions_semantics"],
                        "dimension_uncertainty_m": obj["dimension_uncertainty_m"].tolist(),
                        "dimension_source": "robust visible RGB-D point extent; not CAD or full-object measurement",
                        "depth_validation": obj["depth_validation"],
                        "silhouette_validation": obj["silhouette_validation"],
                        "depth_points": obj["depth_points"], "geometric_only": True,
                        "ik_validated": False, "collision_checked": False,
                    } for obj in report["objects"]],
                    "grasp_candidates": grasp_records,
                    "validation_diagnostics": report.get(
                        "validation_diagnostics", {}),
                    "expired_tracks": report.get("expired_tracks", []),
                    "acquisition": report.get("acquisition", {}),
                }
                message = String()
                message.data = json.dumps(status, sort_keys=True, allow_nan=False)
                timing = getattr(self, "_timing_current", None)
                if timing is not None:timing_site(timing, 5, 12)
                if derived_token is not None:
                    try:self.derived.check(derived_token)
                    except ValueError as error:
                        self._publish_unavailable(stamp, "unavailable", str(error))
                        return
                self.detections_pub.publish(detections)
                self.markers_pub.publish(markers)
                self.grasps_pub.publish(grasps)
                self.contacts_pub.publish(contacts)
                self.last_marker_update_mono = time.monotonic()
                self.last_marker_update_stamp = detections.header.stamp
                self.marker_clear_sent = not bool(report["objects"])
                self.status_pub.publish(message)
                TimingRecorder.clock(timing, 13) if timing is not None else None
                if timing is not None:finish_node(self, 1)

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
        if node.destroy_node() is False:
            raise RuntimeError("Analysis worker not retired; ROS context retained for external supervisor")
        rclpy.shutdown()
