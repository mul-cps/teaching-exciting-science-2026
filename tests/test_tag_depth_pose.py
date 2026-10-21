import ast
from pathlib import Path
import time

import numpy as np
import pytest

from workshop.perception.tag_depth_pose import estimate_tag_pose


INTRINSICS = {'fx': 620., 'fy': 615., 'ppx': 320., 'ppy': 240.}
BITS = np.zeros((7, 7), dtype=np.uint8)
BITS[1:6, 1:6] = np.array([
    [1, 1, 1, 1, 1], [1, 0, 1, 0, 1], [1, 1, 0, 1, 1],
    [1, 0, 1, 0, 1], [1, 1, 1, 1, 1]])


def _case(size=.018, tilts=(.18, -.12), outliers=0):
    height, width = 480, 640
    ax, ay = tilts
    rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)],
                   [0, np.sin(ax), np.cos(ax)]])
    ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0],
                   [-np.sin(ay), 0, np.cos(ay)]])
    rotation = rx @ ry @ np.diag([1., -1., -1.])
    translation = np.array([.012, -.006, .52])
    local = np.array([[-size/2, size/2, 0], [size/2, size/2, 0],
                      [size/2, -size/2, 0], [-size/2, -size/2, 0]])
    points = local @ rotation.T + translation
    corners = np.column_stack((INTRINSICS['fx']*points[:, 0]/points[:, 2] + INTRINSICS['ppx'],
                               INTRINSICS['fy']*points[:, 1]/points[:, 2] + INTRINSICS['ppy']))
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.zeros((height, width), dtype=np.uint16)
    plane_normal = rotation[:, 2]
    plane_d = -plane_normal @ translation
    cells = np.argwhere(BITS[1:6, 1:6] == 1) + 1
    # Project module centers by the exact tag plane and make only their 3x3
    # white RGB/depth patches visible to the estimator.
    for row, col in cells:
        u0 = (col + .5) / 7
        v0 = (row + .5) / 7
        p_local = np.array([(u0-.5)*size, (.5-v0)*size, 0.])
        p = rotation @ p_local + translation
        u = int(round(INTRINSICS['fx']*p[0]/p[2] + INTRINSICS['ppx']))
        v = int(round(INTRINSICS['fy']*p[1]/p[2] + INTRINSICS['ppy']))
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                y, x = v+dy, u+dx
                ray = np.array([(x-INTRINSICS['ppx'])/INTRINSICS['fx'],
                                (y-INTRINSICS['ppy'])/INTRINSICS['fy'], 1.])
                z = -plane_d/(plane_normal @ ray)
                rgb[y, x] = 255
                depth[y, x] = int(round(z*1000))
    if outliers:
        for row, col in cells[:outliers]:
            p_local = np.array([((col+.5)/7-.5)*size, (.5-(row+.5)/7)*size, 0.])
            p = rotation @ p_local + translation
            u = int(round(INTRINSICS['fx']*p[0]/p[2]+INTRINSICS['ppx']))
            v = int(round(INTRINSICS['fy']*p[1]/p[2]+INTRINSICS['ppy']))
            # Corrupt the entire 3x3 measurement patch, so the outlier cannot
            # be hidden by the within-module median.
            depth[v-1:v+2, u-1:u+2] = 900
    return corners, rgb, depth, rotation, translation


@pytest.mark.parametrize('size,tilts', [(.017, (.0, .0)), (.018, (.18, -.12)),
                                        (.020, (-.27, .21))])
def test_pose_recovers_tilted_metric_tag_frame(size, tilts):
    corners, rgb, depth, expected_r, expected_t = _case(size, tilts)
    result = estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)
    pose = np.asarray(result['camera_from_tag'])
    assert result['status'] == 'OBSERVED_DEPTH_PROVISIONAL'
    assert result['qualified'] is False
    assert pose[:3, 3] == pytest.approx(expected_t, abs=.0015)
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1., abs=1e-8)
    assert pose[:3, :3] == pytest.approx(expected_r, abs=.025)
    assert result['black_edge_side_lengths_m'] == pytest.approx([size]*4, abs=.0015)
    assert result['plane']['rms_m'] <= .001
    assert result['valid_module_count'] >= 8


def test_robust_fit_ignores_missing_black_depth_and_module_outliers():
    corners, rgb, depth, _, _ = _case(.019, (.14, -.18), outliers=5)
    # All non-module/background and black-border depth remains absent.
    result = estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)
    assert result['plane']['inlier_count'] >= 8
    assert max(result['black_edge_side_lengths_m']) - min(result['black_edge_side_lengths_m']) < .002


def test_rejects_duplicate_rounded_module_support_before_depth_counts():
    corners = np.array([[319.45, 239.45], [320.55, 239.45],
                        [320.55, 240.55], [319.45, 240.55]])
    rgb = np.full((480, 640, 3), 255, dtype=np.uint8)
    depth = np.full((480, 640), 520, dtype=np.uint16)
    with pytest.raises(ValueError, match='distinct rounded'):
        estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS, patch_radius=0)


def test_deduplicated_module_centers_keep_patch_samples_aligned():
    corners, _, _, _, _ = _case()
    center = corners.mean(axis=0)
    small_corners = center + .25 * (corners - center)
    depth = np.full((480, 640), 520, dtype=np.uint16)
    result = estimate_tag_pose(small_corners, None, depth, INTRINSICS, .001, BITS)
    unique_count = result['unique_sample_pixel_count']
    assert 8 <= unique_count < result['module_count']
    assert result['valid_module_count'] == unique_count
    assert result['plane']['rms_m'] < 1e-8


@pytest.mark.parametrize('change', ['nonzero', 'nonfinite', 'unsupported'])
def test_rejects_declared_distortion_not_supported_by_pinhole_model(change):
    corners, rgb, depth, _, _ = _case()
    intrinsics = dict(INTRINSICS)
    if change == 'nonzero':
        intrinsics.update(model='plumb_bob', distortion=[.1, 0., 0., 0., 0.])
    elif change == 'nonfinite':
        intrinsics.update(distortion={'model': 'brown_conrady', 'coeffs': [0., 0., np.nan]})
    else:
        intrinsics.update(model='equidistant', coeffs=[0., 0., 0., 0.])
    with pytest.raises(ValueError, match='distortion'):
        estimate_tag_pose(corners, rgb, depth, intrinsics, .001, BITS)


def test_accepts_explicit_zero_brown_distortion_aliases():
    corners, rgb, depth, _, _ = _case()
    intrinsics = dict(INTRINSICS, distortion_model='Brown-Conrady',
                      distortion={'coefficients': [0., 0., 0., 0., 0.]})
    result = estimate_tag_pose(corners, rgb, depth, intrinsics, .001, BITS)
    assert result['qualified'] is False


@pytest.mark.parametrize('change', ['all_invalid', 'dark', 'mixed_plane', 'too_few_cells'])
def test_rejects_invalid_or_ambiguous_depth_samples(change):
    corners, rgb, depth, _, _ = _case()
    if change == 'all_invalid':
        depth[:] = 0
    elif change == 'dark':
        rgb[:] = 0
    elif change == 'mixed_plane':
        # A substantial independent surface inside valid module patches cannot
        # be collapsed into a low-RMS plane.
        for row, col in np.argwhere(BITS[1:6, 1:6] == 1)[:12] + 1:
            p_local = np.array([((col+.5)/7-.5)*.018, (.5-(row+.5)/7)*.018, 0.])
            p = np.diag([1., -1., -1.]) @ p_local + np.array([.012, -.006, .52])
            u = int(round(INTRINSICS['fx']*p[0]/p[2]+INTRINSICS['ppx']))
            v = int(round(INTRINSICS['fy']*p[1]/p[2]+INTRINSICS['ppy']))
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    depth[v+dy, u+dx] = 900
    else:
        depth[rgb[:, :, 0] == 255] = 0
    with pytest.raises(ValueError):
        estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)


@pytest.mark.parametrize('corners', [
    [[100, 100], [200, 200], [100, 200], [200, 100]],
    [[100, 100], [200, 100], [150, 150], [100, 200]],
])
def test_rejects_self_crossing_or_degenerate_corner_quads(corners):
    _, rgb, depth, _, _ = _case()
    with pytest.raises(ValueError, match='corner quad'):
        estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)


def test_frame_axes_use_image_right_and_up_with_camera_facing_z():
    corners, rgb, depth, _, _ = _case(.018, (.08, -.05))
    rotation = np.asarray(estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)
                          ['camera_from_tag'])[:3, :3]
    assert rotation[0, 0] > .9  # +tag X projects right.
    assert rotation[1, 1] < -.9  # +tag Y projects up in optical camera axes.
    assert rotation[2, 2] < -.9  # +tag Z faces the camera.
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-8)


def test_rgb_white_patch_filter_is_optional():
    corners, rgb, depth, _, _ = _case(.018, (.08, -.05))
    rgb_result = estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)
    depth_only = estimate_tag_pose(corners, None, depth, INTRINSICS, .001, BITS)
    assert np.asarray(rgb_result['camera_from_tag']) == pytest.approx(
        np.asarray(depth_only['camera_from_tag']), abs=1e-8)


def test_tag_pattern_must_include_black_outer_border():
    corners, rgb, depth, _, _ = _case()
    invalid_bits = BITS.copy()
    invalid_bits[0, 1] = 1
    with pytest.raises(ValueError, match='black border'):
        estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, invalid_bits)


def test_ray_roundtrip_field_is_explicitly_not_a_pose_fit_score():
    corners, rgb, depth, _, _ = _case()
    result = estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)
    assert 'ray_intersection_roundtrip_error_px' in result
    assert 'reprojection_rms_px' not in result


def test_static_import_guard_and_640x480_offline_timing():
    source = Path(__file__).parents[1] / 'workshop' / 'perception' / 'tag_depth_pose.py'
    tree = ast.parse(source.read_text())
    imports = {alias.name.split('.')[0] for node in ast.walk(tree)
               if isinstance(node, (ast.Import, ast.ImportFrom))
               for alias in node.names}
    assert imports == {'numpy'}
    forbidden = {'cv2', 'mujoco', 'serial', 'pyrealsense2', 'depthai'}
    assert imports.isdisjoint(forbidden)
    corners, rgb, depth, _, _ = _case(.019, (.14, -.18))
    start = time.perf_counter()
    estimate_tag_pose(corners, rgb, depth, INTRINSICS, .001, BITS)
    elapsed = time.perf_counter() - start
    assert elapsed < .1


def test_compact_ros_tag_bridge_uses_available_metric_estimator_without_ros():
    """Replay the real compact bridge with the existing metric fixture, no node init."""
    from types import SimpleNamespace
    source = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_perception/so101_station_perception/tag_observations.py'
    method = next(n for n in ast.walk(ast.parse(source.read_text()))
                  if isinstance(n, ast.FunctionDef) and n.name == 'compact_tag_detections')
    namespace = {'np': np, 'cv2': SimpleNamespace(error=RuntimeError)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    bridge = namespace['compact_tag_detections']
    assert bridge([], None, INTRINSICS, .001, {}) == []
    corners, _, depth, _, expected_t = _case(.0194)
    result = bridge([{'code_id': 7, 'corners_px': corners}], depth, INTRINSICS, .001, {7: BITS})
    assert len(result) == 1 and result[0]['measurement_valid'] is True
    assert result[0]['measurement_source'] == 'white_module_depth_plane'
    assert result[0]['center_camera_m'] == pytest.approx(expected_t, abs=.0015)
    assert result[0]['size_m'] == .0194 and result[0]['qualified'] is False
