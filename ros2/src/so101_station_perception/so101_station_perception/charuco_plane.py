"""Bounded robust fitting of the visible ChArUco board from aligned depth."""

import math

import cv2
import numpy as np


def decode_depth(depth_image, encoding):
    """Return metric depth; ROS 16UC1 is millimetres, 32FC1 is metres."""
    values = np.asarray(depth_image)
    if encoding in ("16UC1", "mono16"):
        depth = values.astype(np.float32) * 0.001
    elif encoding in ("32FC1", "32FC"):
        depth = values.astype(np.float32)
    else:
        raise ValueError(f"unsupported aligned depth encoding {encoding!r}")
    return depth


def decode_color_pixels(color_message):
    """Expose an RGB/BGR ROS image as a NumPy view without another bridge decode."""
    encoding = str(color_message.encoding).lower()
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported color encoding for board plane {encoding!r}")
    h, w, step = int(color_message.height), int(color_message.width), int(color_message.step)
    if step < w * 3 or len(color_message.data) < h * step:
        raise ValueError("color image data or row step is invalid")
    rows = np.frombuffer(color_message.data, dtype=np.uint8, count=h * step).reshape(h, step)
    return rows[:, :w * 3].reshape(h, w, 3)


def _plane(points):
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    if norm < 1e-10 or not math.isfinite(norm):
        raise ValueError("depth points do not define a plane")
    return center, normal / norm


def fit_depth_plane(depth_m, camera_matrix, distortion, optical_from_board,
                    board_width=0.175, board_height=0.125, *,
                    color_image=None,
                    max_points=2000, residual_threshold_m=0.0025,
                    min_inliers=80, min_coverage_cells=8):
    """Fit the board interior plane, constrained by PnP and a projected board hull.

    The deterministic 32-trial RANSAC and fixed point cap keep callback work
    bounded. Returns optical-frame plane and fit metrics.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2 or not np.isfinite(depth).any():
        raise ValueError("aligned depth is empty or not a single-channel image")
    h, w = depth.shape
    vivid = np.zeros((h, w), dtype=bool)
    if color_image is not None:
        color = np.asarray(color_image)
        if color.shape != (h, w, 3) or color.dtype != np.uint8:
            raise ValueError("color image must be uint8 and match aligned depth dimensions")
        # Saturation from channel spread avoids an HSV image allocation/conversion.
        high = color.max(axis=2).astype(np.int16)
        spread = high - color.min(axis=2).astype(np.int16)
        vivid = (high >= 40) & (spread >= 35) & (spread / np.maximum(high, 1) >= 0.28)
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    d = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    pose = np.asarray(optical_from_board, dtype=np.float64).reshape(4, 4)
    outer = np.array([[0, 0, 0], [board_width, 0, 0],
                       [board_width, board_height, 0], [0, board_height, 0]], dtype=np.float64)
    projected, _ = cv2.projectPoints(outer, cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3], k, d)
    polygon = np.rint(projected.reshape(-1, 2)).astype(np.int32)
    x0, y0 = polygon.min(axis=0)
    x1, y1 = polygon.max(axis=0)
    if x1 <= x0 or y1 <= y0:
        raise ValueError("projected ChArUco board has no image extent")
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(mask, polygon, 255)
    # Keep pixels away from the board edge, where background and marker borders
    # are most likely to contaminate aligned depth.
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
    # Allocate sampling to the visible board rather than unrelated image area.
    # Otherwise a small board can lose most valid pixels before the fixed
    # inlier/coverage gates, even when its interior provides ample depth.
    sample_x0, sample_y0 = max(0, x0), max(0, y0)
    sample_x1, sample_y1 = min(w, x1+1), min(h, y1+1)
    sample_area = max(0, sample_x1-sample_x0) * max(0, sample_y1-sample_y0)
    stride = max(1, int(math.ceil(math.sqrt(sample_area / (max_points * 4)))))
    ys, xs = np.mgrid[sample_y0:sample_y1:stride, sample_x0:sample_x1:stride]
    sampled_vivid = int(np.count_nonzero((mask[ys, xs] != 0) & vivid[ys, xs]))
    valid = (mask[ys, xs] != 0) & ~vivid[ys, xs] & np.isfinite(depth[ys, xs])
    z = depth[ys, xs]
    valid &= (z > 0.08) & (z < 3.0)
    # PnP bounds the search volume, rejecting nearby colored bricks before
    # robust fitting while allowing a few centimetres of calibration error.
    candidate_x, candidate_y = xs[valid], ys[valid]
    candidate_z = z[valid]
    if not len(candidate_z):
        raise ValueError("no valid depth pixels inside projected board")
    if len(candidate_z) > max_points:
        take = np.linspace(0, len(candidate_z) - 1, max_points, dtype=np.int32)
        candidate_x, candidate_y, candidate_z = candidate_x[take], candidate_y[take], candidate_z[take]
    undistorted = cv2.undistortPoints(
        np.column_stack((candidate_x, candidate_y)).astype(np.float64).reshape(-1, 1, 2), k, d)
    points = np.column_stack((undistorted[:, 0, 0] * candidate_z,
                              undistorted[:, 0, 1] * candidate_z, candidate_z))
    seed_normal = pose[:3, 2]
    signed_seed = (points - pose[:3, 3]) @ seed_normal
    near = np.abs(signed_seed) <= 0.025
    points = points[near]
    grid_xy = np.column_stack(((candidate_x[near] - x0) / max(1, x1 - x0),
                               (candidate_y[near] - y0) / max(1, y1 - y0)))
    if len(points) < min_inliers:
        raise ValueError(f"only {len(points)} board depth samples (need {min_inliers})")

    # Fixed, reproducible spread of triples, rather than an unbounded RNG loop.
    best = np.zeros(len(points), dtype=bool)
    count = len(points)
    for trial in range(32):
        idx = np.array([(trial * 7919 + 17) % count,
                        (trial * 104729 + count // 3 + 31) % count,
                        (trial * 15485863 + 2 * count // 3 + 47) % count])
        if len(set(idx.tolist())) < 3:
            continue
        a, b, c = points[idx]
        normal = np.cross(b - a, c - a)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-8:
            continue
        distances = np.abs((points - a) @ (normal / norm))
        inliers = distances <= residual_threshold_m
        if int(inliers.sum()) > int(best.sum()):
            best = inliers
    if int(best.sum()) < min_inliers:
        raise ValueError(f"depth plane has only {int(best.sum())} inliers (need {min_inliers})")
    center, normal = _plane(points[best])
    residual = np.abs((points[best] - center) @ normal)
    # Reclassify once against the SVD-refined plane.
    refined = np.abs((points - center) @ normal) <= residual_threshold_m
    if int(refined.sum()) >= min_inliers:
        center, normal = _plane(points[refined])
        best = refined
        residual = np.abs((points[best] - center) @ normal)
    # Require broad board support. Outliers clustered on a brick or a corner
    # cannot establish the board pose.
    occupied = set((np.clip((grid_xy[best, 0] * 4).astype(int), 0, 3) * 4 +
                    np.clip((grid_xy[best, 1] * 3).astype(int), 0, 2)).tolist())
    if len(occupied) < min_coverage_cells:
        raise ValueError(f"depth plane covers only {len(occupied)} board regions")
    if np.dot(normal, center) > 0:
        normal = -normal
    metrics = {
        "plane_inliers": int(best.sum()),
        "plane_samples": int(len(points)),
        "plane_inlier_ratio": float(best.sum() / max(1, len(points))),
        "plane_median_residual_mm": float(np.median(residual) * 1000),
        "plane_p95_residual_mm": float(np.percentile(residual, 95) * 1000),
        "plane_coverage_cells": len(occupied),
        "plane_vivid_color_rejected": sampled_vivid,
        "plane_normal_optical": normal.tolist(),
    }
    return center, normal, metrics


def camera_from_world_from_plane(optical_from_board, plane_center, plane_normal):
    """Create right-handed board-world pose with camera-facing +Z on the plane."""
    pnp = np.asarray(optical_from_board, dtype=np.float64).reshape(4, 4)
    z_axis = np.asarray(plane_normal, dtype=np.float64).reshape(3)
    z_axis /= np.linalg.norm(z_axis)
    if np.dot(z_axis, plane_center) > 0:
        z_axis = -z_axis
    x_axis = pnp[:3, 0] - z_axis * float(np.dot(pnp[:3, 0], z_axis))
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    origin = pnp[:3, 3] - z_axis * float(np.dot(pnp[:3, 3] - plane_center, z_axis))
    optical_from_world = np.eye(4)
    optical_from_world[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    optical_from_world[:3, 3] = origin
    return optical_from_world


def plane_tilt_from_pnp_degrees(optical_from_board, camera_facing_normal,
                                max_tilt_degrees=None):
    """Angular disagreement between measured depth normal and PnP board face."""
    pnp_facing = -np.asarray(optical_from_board, dtype=np.float64).reshape(4, 4)[:3, 2]
    normal = np.asarray(camera_facing_normal, dtype=np.float64).reshape(3).copy()
    normal /= np.linalg.norm(normal)
    tilt = math.degrees(math.acos(float(np.clip(np.dot(pnp_facing, normal), -1, 1))))
    if max_tilt_degrees is not None and tilt > float(max_tilt_degrees):
        raise ValueError(f"depth plane tilt {tilt:.2f}deg disagrees with ChArUco PnP")
    return tilt
