from dataclasses import dataclass
import ast
from pathlib import Path
import cv2
import numpy as np
import pytest
from tools.propose_rgbd_grasp import (color_masks, propose_geometry, transform_and_solve, load_episode, se3)


def scene():
    rgb = np.full((80, 100, 3), 80, np.uint8)
    depth = np.full((4, 80, 100), 500, np.uint16)
    rgb[30:42, 40:58] = [220, 20, 20]
    depth[:, 30:42, 40:58] = 493
    # A red wrist decoration above the object-height band must be rejected.
    rgb[5:18, 70:90] = [220, 20, 20]
    depth[:, 5:18, 70:90] = 450
    rgb[50:60, 10:25] = [20, 200, 20]
    depth[:, 50:60, 10:25] = 493
    meta = {'aligned_to': 'color', 'depth_scale_m': .001,
            'intrinsics': {'width': 100, 'height': 80, 'fx': 500, 'fy': 500,
                           'ppx': 50, 'ppy': 40, 'coeffs': [0]*5}}
    return rgb, depth, meta


def test_local_plane_masks_wrist_and_other_colors():
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'red', .008)
    assert report['plane_camera']['p95_m'] < 1e-8
    assert report['selected_color_pixels'] == 216
    assert report['rejected_color_pixels'] == 260
    assert len(report['candidates']) == 2
    for candidate in report['candidates']:
        assert np.allclose(candidate['center'][2], .496)
        assert not candidate['executable']
        assert candidate['contour_pixels'][:, 0].min() == 40


def test_hsv_masks_keep_yellow_front_shadows_out_of_red():
    rgb = np.array([[[255, 150, 30], [255, 212, 37], [252, 26, 8],
                     [255, 255, 255], [128, 128, 128]]], dtype=np.uint8)
    masks = color_masks(rgb)
    assert masks['yellow'][0, 0] and not masks['red'][0, 0]
    assert masks['yellow'][0, 1] and not masks['red'][0, 1]
    assert masks['red'][0, 2]
    assert not any(mask[0, 3] for mask in masks.values())
    assert not any(mask[0, 4] for mask in masks.values())


def test_saturated_hue_masks_are_mutually_exclusive_with_gap_left_unknown():
    hsv = np.zeros((180, 1, 3), dtype=np.uint8)
    hsv[:, 0, 0] = np.arange(180, dtype=np.uint8)
    hsv[:, 0, 1:] = [200, 200]
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    masks = color_masks(rgb)
    membership = np.stack(list(masks.values()), axis=0)
    assert np.all(membership.sum(axis=0) <= 1)
    # The orange transition band between red and yellow has no guessed label.
    assert not membership[:, 11:15, 0].any()


@dataclass
class Result:
    success: bool = True
    raw: dict = None
    position_error_m: float = 0.
    orientation_error_rad: float = .4
    grasp_eligible: bool = False


class FakeIK:
    instance = None
    instances = []
    def __init__(self, calibration, offset):
        self.offset = offset
        self.closed = False
        self.targets = []
        FakeIK.instance = self
        FakeIK.instances.append(self)
    def forward_raw(self, raw, opening_raw=None):
        assert opening_raw == 2200
        return np.eye(4)
    def solve_pinch(self, target, seed, orientation_weight=0):
        assert seed['gripper'] == 2200
        assert orientation_weight == 0
        self.targets.append(target)
        return Result(raw=seed)
    def close(self):
        self.closed = True


def documents():
    transform = np.eye(4)
    transform[:3, 3] = [.1, -.2, .3]
    return ({'base_frame': 'urdf:base_link', 'direction': 'base_from_camera',
             'base_from_camera': transform, 'qualified': False},
            {'parent_frame': 'urdf:gripper_frame_link',
             'gripper_frame_from_pinch': np.eye(4), 'opening_raw': 2200, 'qualified': False},
            {'gripper': {'range_min': 1800, 'range_max': 3400}}, {'gripper': 2170})


def test_transform_fixed_opening_numerical_success_not_eligible():
    FakeIK.instances.clear()
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'red', .008)
    camera_center = report['candidates'][0]['center'].copy()
    transform_and_solve(report, *documents(), 2200, ik_factory=FakeIK)
    candidate = report['candidates'][0]
    assert np.allclose(candidate['center_base_m'], camera_center + [.1, -.2, .3])
    assert candidate['ik']['success']
    assert not candidate['grasp_eligible'] and not candidate['executable']
    assert not candidate['physical_qualified']
    assert not report['provenance']['opening_calibrated']
    assert not report['provenance']['physical_qualification_verified']
    assert FakeIK.instance.closed
    assert len(FakeIK.instances) == 1
    assert np.allclose(FakeIK.instance.offset(25), np.eye(4))
    with pytest.raises(ValueError):
        FakeIK.instance.offset(26)


def test_no_ik_still_transforms_without_backend():
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'green', .008)
    transform_and_solve(report, *documents(), 2200, no_ik=True)
    assert all(c['ik'] is None and not c['executable'] and not c['physical_qualified']
               for c in report['candidates'])
    assert {'depth_fusion', 'support_plane', 'object_candidates', 'geometry_total'} <= set(report['latency_ms'])
    assert report['latency_ms']['ik_candidate_solve_total'] == 0.


@pytest.mark.parametrize('change', ['alignment', 'distortion', 'size', 'scale', 'depth', 'support'])
def test_invalid_inputs_rejected(change):
    rgb, depth, meta = scene()
    if change == 'alignment': meta['aligned_to'] = 'depth'
    if change == 'distortion': meta['intrinsics']['coeffs'][0] = .1
    if change == 'size': meta['intrinsics']['width'] = 101
    if change == 'scale': meta['depth_scale_m'] = float('nan')
    if change == 'depth': depth = depth.astype(float)
    if change == 'support': depth[:] = 0
    with pytest.raises(ValueError):
        propose_geometry(rgb, depth, meta, (0, 0, 100, 80), 'red', .008)


def test_invalid_transform_and_opening_reference():
    with pytest.raises(ValueError): se3(np.zeros((4, 4)))
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'red', .008)
    docs = documents()
    docs[1]['opening_raw'] = 2199
    with pytest.raises(ValueError): transform_and_solve(report, *docs, 2200, no_ik=True)


@pytest.mark.parametrize(('document_index', 'key', 'bad_value'), [
    (0, 'base_frame', None),
    (0, 'base_frame', 'world'),
    (0, 'base_frame', 'urdf:arm_base'),
    (0, 'direction', None),
    (0, 'direction', 'camera_from_base'),
    (1, 'parent_frame', None),
    (1, 'parent_frame', 'urdf:gripper_frame_site'),
])
def test_frame_identifiers_are_required_even_when_qualified(document_index, key, bad_value):
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'red', .008)
    docs = list(documents())
    docs[document_index][key] = bad_value
    docs[document_index]['qualified'] = True
    with pytest.raises(ValueError, match='frame|direction'):
        transform_and_solve(report, *docs, 2200, no_ik=True)


@pytest.mark.parametrize('bad_opening', [2199, 2200.5, True])
def test_pinch_opening_must_be_an_exact_integer_cli_opening(bad_opening):
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'red', .008)
    docs = list(documents())
    docs[1]['opening_raw'] = bad_opening
    with pytest.raises(ValueError, match='opening_raw'):
        transform_and_solve(report, *docs, 2200, no_ik=True)


def test_pinch_opening_must_be_declared():
    report = propose_geometry(*scene(), (0, 0, 100, 80), 'red', .008)
    docs = list(documents())
    del docs[1]['opening_raw']
    with pytest.raises(ValueError, match='opening_raw'):
        transform_and_solve(report, *docs, 2200, no_ik=True)


def test_tool_has_no_hardware_sdk_imports_or_candidate_loop_model_construction():
    source = Path(__file__).parents[1] / 'tools' / 'propose_rgbd_grasp.py'
    tree = ast.parse(source.read_text())
    forbidden = {'pybricks', 'ev3dev2', 'lerobot', 'dynamixel_sdk', 'serial', 'feetech'}
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    assert not imported & forbidden

    loops = [node for node in ast.walk(tree) if isinstance(node, ast.For)
             and isinstance(node.iter, ast.Subscript)
             and isinstance(node.iter.value, ast.Name) and node.iter.value.id == 'report'
             and isinstance(node.iter.slice, ast.Constant) and node.iter.slice.value == 'candidates']
    assert len(loops) == 1
    constructors = []
    for node in ast.walk(loops[0]):
        if isinstance(node, ast.Call):
            func = node.func.id if isinstance(node.func, ast.Name) else None
            if func in {'ik_factory', 'SO101IK'}:
                constructors.append(func)
    assert not constructors


def test_empty_episode_rejected(tmp_path):
    with pytest.raises(ValueError): load_episode(tmp_path)


def test_cli_episode_to_finite_json(tmp_path, monkeypatch):
    import json
    import sys
    from tools.propose_rgbd_grasp import main, finite_json
    rgb, depth, meta = scene()
    episode = tmp_path / 'episode'
    (episode / 'frames').mkdir(parents=True)
    import cv2
    for index, frame in enumerate(depth):
        path = episode / 'frames' / f'{index:06d}'
        np.save(path.with_suffix('.npy'), frame)
        cv2.imwrite(str(path.with_suffix('.png')), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    (episode / 'metadata.json').write_text(json.dumps(meta))
    ext, pinch, cal, pose = documents()
    ext['base_from_camera'] = ext['base_from_camera'].tolist()
    pinch['gripper_frame_from_pinch'] = pinch['gripper_frame_from_pinch'].tolist()
    inputs = {'extrinsic': ext, 'pinch-offset': pinch, 'calibration': cal, 'pose-raw': pose}
    argv = ['propose', '--episode', str(episode), '--support-roi', '0', '0', '100', '80',
            '--color', 'red', '--height-mm', '8', '--opening-raw', '2200', '--no-ik']
    for name, document in inputs.items():
        path = tmp_path / f'{name}.json'
        path.write_text(json.dumps(document))
        argv += ['--' + name, str(path)]
    output = tmp_path / 'output.json'
    monkeypatch.setattr(sys, 'argv', argv + ['--output', str(output)])
    main()
    result = json.loads(output.read_text())
    assert len(result['candidates']) == 2
    assert result['depth_frame_count'] == 4
    assert len(result['input_sha256']) == 10
    assert not result['executable']
    assert not result['provenance']['physical_qualification_verified']
    assert finite_json({'residual': float('inf'), 'array': np.array([np.nan, 1])}) == {'residual': None, 'array': [None, 1.]}
