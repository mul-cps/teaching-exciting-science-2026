"""Depth evidence for colored surfaces, independent of workspace and ownership.

A positive result means a locally supported coherent colored surface. RGB-D
alone does not certify that a geometrically identical colored fitting is a LEGO
brick. Semantic identity, grasp admission and reserved-track lifetime remain
separate responsibilities. No height, workspace ROI or parent-size gate exists.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthComponent:
    mask: np.ndarray
    support_mask: np.ndarray
    parent_pixels: int
    depth_cut_pixels: int


@dataclass(frozen=True)
class ComponentValidation:
    accepted: bool
    reason: str
    metrics: dict


def _depth_inputs(mask, depth_m):
    mask = np.asarray(mask)
    depth = np.asarray(depth_m)
    if mask.ndim != 2 or depth.shape != mask.shape or not np.issubdtype(depth.dtype, np.floating):
        raise ValueError('matching color mask and aligned metric floating depth required')
    mask = mask.astype(bool)
    valid = np.isfinite(depth) & (depth >= .10) & (depth <= 2.)
    return mask, depth, valid


def split_depth_coherent_components(parent_mask, depth_m, *, max_neighbor_jump_m=.012,
                                    min_pixels=24):
    """Separate depth leaks before color contour construction or silhouette fill.

    Invalid depth remains part of local support accounting. A large invalid
    background joined to a real foreground brick by a thin color bridge does
    not impose its size/invalid ratio on the foreground component. Depth jumps
    cut one pixel on each side; this deliberately withholds uncertain edges.
    """
    parent, depth, valid = _depth_inputs(parent_mask, depth_m)
    if not np.isfinite(max_neighbor_jump_m) or max_neighbor_jump_m <= 0 or type(min_pixels) is not int or min_pixels < 8:
        raise ValueError('positive depth continuity and component size required')
    candidate = parent & valid
    cut = np.zeros_like(candidate)
    for axis in (0, 1):
        left = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        right = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        bad = (candidate[left] & candidate[right] &
               (np.abs(np.where(valid[left], depth[left], 0)-np.where(valid[right], depth[right], 0)) > max_neighbor_jump_m))
        cut[left] |= bad
        cut[right] |= bad
    candidate &= ~cut
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate.astype(np.uint8), connectivity=8)
    output = []
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) < min_pixels:
            continue
        region = labels == label
        nearby = cv2.dilate(region.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        support = parent & nearby
        # Include invalid holes inside the candidate contour. They must not
        # vanish merely because only valid-depth pixels reached this function.
        contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filled = np.zeros_like(parent, dtype=np.uint8)
        cv2.drawContours(filled, contours, -1, 1, cv2.FILLED)
        support |= parent & filled.astype(bool)
        region.setflags(write=False)
        support.setflags(write=False)
        output.append(DepthComponent(region, support, int(parent.sum()), int((cut & nearby).sum())))
    return tuple(output)


def validate_colored_component(rgb, depth_m, component_mask, intrinsics, distortion=None,
                               *, support_mask=None, min_supported_fraction=.90,
                               max_neighbor_jump_m=.012, max_jump_fraction=.10,
                               max_depth_split_m=.020, min_split_fraction=.08):
    """Check local untrimmed support, discontinuities and disconnected depths.

    ``support_mask`` must contain the original color pixels local to this depth
    component, including holes/uncertain boundary pixels; use the splitter above.
    It is never the table/pickup ROI. Plane/appearance/shape metrics are reported
    without pretending they establish semantic brick identity or rigidity.
    """
    region, depth, valid = _depth_inputs(component_mask, depth_m)
    rgb = np.asarray(rgb)
    if rgb.shape != (*depth.shape, 3) or rgb.dtype != np.uint8:
        raise ValueError('matching uint8 RGB and aligned metric depth required')
    support = region if support_mask is None else np.asarray(support_mask, dtype=bool)
    if support.shape != region.shape or np.any(region & ~support):
        raise ValueError('local original-color support must contain the component')
    if not 0 < min_supported_fraction <= 1 or not 0 <= max_jump_fraction <= 1 or not 0 < min_split_fraction < .5:
        raise ValueError('validation fractions are invalid')
    if not all(np.isfinite(v) and v > 0 for v in (max_neighbor_jump_m, max_depth_split_m)):
        raise ValueError('positive finite depth continuity limits required')
    metrics = {'color_pixels': int(region.sum()), 'local_support_pixels': int(support.sum()),
               'semantic_identity': 'unqualified_colored_surface',
               'workspace_gate_applied': False, 'support_plane_gate_applied': False}
    if not support.any():
        return ComponentValidation(False, 'no_color_support', metrics)
    selected = region & valid
    supported_fraction = float(selected.sum()/support.sum())
    metrics['supported_fraction'] = supported_fraction
    metrics['invalid_local_depth_fraction'] = float((support & ~valid).sum()/support.sum())
    core = cv2.erode(support.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    metrics['core_supported_fraction'] = float((core & selected).sum()/core.sum()) if core.any() else None
    if int(selected.sum()) < 24:
        return ComponentValidation(False, 'insufficient_depth_support', metrics)
    # Calculate all geometry diagnostics even for rejected depth support so the
    # live evidence explains leaks rather than merely reporting a boolean gate.
    try:
        values = np.asarray([intrinsics[k] for k in ('fx', 'fy', 'ppx', 'ppy')], dtype=float)
    except (TypeError, KeyError) as error:
        raise ValueError('calibrated color intrinsics required') from error
    if not np.isfinite(values).all() or min(values[:2]) <= 0:
        raise ValueError('finite positive color focal lengths required')
    d = np.zeros(5) if distortion is None else np.asarray(distortion, dtype=float)
    if d.ndim != 1 or len(d) not in (4, 5, 8) or not np.isfinite(d).all():
        raise ValueError('finite explicit OpenCV distortion required')
    y, x = np.nonzero(selected)
    k = np.array([[values[0], 0, values[2]], [0, values[1], values[3]], [0, 0, 1.]])
    normalized = cv2.undistortPoints(np.column_stack((x, y)).astype(float).reshape(-1, 1, 2), k, d).reshape(-1, 2)
    points = np.column_stack((normalized*depth[y, x, None], depth[y, x]))
    center = np.median(points, axis=0)
    _, _, axes = np.linalg.svd(points-center, full_matrices=False)
    principal = (points-center) @ axes.T
    metrics['centroid_camera_m'] = center.tolist()
    metrics['visible_extents_m'] = (np.percentile(principal, 95, axis=0)-np.percentile(principal, 5, axis=0)).tolist()
    metrics['plane_residual_p95_m'] = float(np.percentile(np.abs(principal[:, 2]), 95))
    depths = np.sort(depth[y, x])
    gaps = np.diff(depths)
    split_index = int(np.argmax(gaps))
    metrics['largest_depth_gap_m'] = float(gaps[split_index])
    metrics['split_minority_fraction'] = min(split_index+1, len(depths)-split_index-1)/len(depths)
    jumps, neighbors = 0, 0
    for axis in (0, 1):
        left = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        right = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        adjacency = selected[left] & selected[right]
        neighbors += int(adjacency.sum())
        jumps += int((adjacency & (np.abs(depth[left]-depth[right]) > max_neighbor_jump_m)).sum())
    metrics['depth_neighbor_jump_fraction'] = jumps/max(1, neighbors)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    metrics['saturation_p10'] = float(np.percentile(hsv[:, :, 1][region], 10))
    contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    rectangle = cv2.minAreaRect(contour)
    rectangle_area = float(np.prod(rectangle[1]))
    metrics['silhouette_rectangularity'] = float(cv2.contourArea(contour)/max(1., rectangle_area))
    if supported_fraction < min_supported_fraction:
        return ComponentValidation(False, 'locally_unsupported_color_depth', metrics)
    if (metrics['largest_depth_gap_m'] > max_depth_split_m and
            metrics['split_minority_fraction'] >= min_split_fraction):
        return ComponentValidation(False, 'multiple_disconnected_depth_surfaces', metrics)
    if metrics['depth_neighbor_jump_fraction'] > max_jump_fraction:
        return ComponentValidation(False, 'incoherent_neighbor_depth', metrics)
    return ComponentValidation(True, 'depth_supported_colored_surface', metrics)
