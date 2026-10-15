"""Saved-image-only wrist AprilTag overlay contracts."""
import ast
from pathlib import Path

import numpy as np
import pytest

from workshop.digital_twin import fiducial


def _quad(cx, cy, half=10):
    return np.array([[[cx-half, cy-half], [cx+half, cy-half],
                      [cx+half, cy+half], [cx-half, cy+half]]], dtype=np.float32)


def test_bootstrap_roi_selects_wrist_tag_and_rejects_duplicate_candidates():
    selection = fiducial.track_tag_candidates(
        [(0, _quad(375, 252)), (0, _quad(374, 81))])
    assert selection['status'] == 'tracked'
    np.testing.assert_allclose(selection['center_px'], [375, 252])

    ambiguous = fiducial.track_tag_candidates(
        [(0, _quad(375, 252)), (0, _quad(380, 257))])
    assert ambiguous['status'] == 'ambiguous'
    assert ambiguous['center_px'] is None


def test_tracking_uses_nearest_candidate_inside_shift_gate_and_rejects_ties():
    previous = np.array([375., 252.])
    selection = fiducial.track_tag_candidates(
        [(0, _quad(382, 254)), (0, _quad(405, 253))], previous_center=previous)
    np.testing.assert_allclose(selection['center_px'], [382, 254])
    tied = fiducial.track_tag_candidates(
        [(0, _quad(380, 252)), (0, _quad(370, 252))], previous_center=previous)
    assert tied['status'] == 'ambiguous'
    assert fiducial.track_tag_candidates(
        [(0, _quad(420, 252))], previous_center=previous)['status'] == 'missing'


def test_native_site_pose_is_bridged_into_urdf_gripper_link_before_mount_composition():
    from workshop.digital_twin.frame_conventions import NATIVE_SITE_TO_URDF_FRAME

    world_from_native = np.eye(4)
    world_from_native[:3, :3] = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    world_from_native[:3, 3] = [0.3, -0.2, 0.5]
    gripper_from_tag = np.eye(4)
    gripper_from_tag[:3, 3] = [-.03, .004, -.068]
    expected = fiducial.expected_tag_transform(
        world_from_native, gripper_from_tag, np.eye(4))
    np.testing.assert_allclose(expected, world_from_native @ np.linalg.inv(NATIVE_SITE_TO_URDF_FRAME)
                               @ gripper_from_tag)
    # The nontrivial frame bridge changes the projected tag axes while retaining the common origin.
    assert not np.allclose(expected[:3, :3], (world_from_native @ gripper_from_tag)[:3, :3])


def test_actual_paired_pose_fk_uses_canonical_native_to_urdf_frame_bridge():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.frame_conventions import NATIVE_SITE_TO_URDF_FRAME
    from workshop.digital_twin.kinematics import set_pose
    from workshop.digital_twin.scene import make_model
    model = mujoco.MjModel.from_xml_string(make_model(
        [.2, -.1, .03], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    set_pose(model, data, {'shoulder_pan': 23, 'shoulder_lift': -71, 'elbow_flex': 84,
                           'wrist_flex': 38, 'wrist_roll': -19, 'gripper': 12})
    base_from_native = fiducial.base_from_native_site(model, data)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_frame_site')
    np.testing.assert_allclose(base_from_native[:3, :3],
        data.xmat[base_id].reshape(3, 3).T @ data.site_xmat[site_id].reshape(3, 3))
    np.testing.assert_allclose(base_from_native[:3, 3],
        data.xmat[base_id].reshape(3, 3).T @ (data.site_xpos[site_id] - data.xpos[base_id]))
    bridged = base_from_native @ np.linalg.inv(NATIVE_SITE_TO_URDF_FRAME)
    np.testing.assert_allclose(bridged @ NATIVE_SITE_TO_URDF_FRAME, base_from_native, atol=1e-9)
    np.testing.assert_allclose(bridged[:3, 3], base_from_native[:3, 3], atol=1e-9)


def test_only_fresh_paired_actual_raw_pose_can_drive_fiducial_prediction():
    pair = {'freshness': 'fresh', 'pose_raw': {'shoulder_pan': 2048},
            'pose_qualified_sync': True, 'pose_sample_delta_seconds': -.01}
    assert fiducial.paired_pose_raw(pair) == pair['pose_raw']
    assert fiducial.paired_pose_raw({**pair, 'pose_sample_delta_seconds': .161}) is None
    assert fiducial.paired_pose_raw({**pair, 'freshness': 'stale'}) is None
    assert fiducial.paired_pose_raw({**pair, 'pose_qualified_sync': False}) is None
    assert fiducial.paired_pose_raw({'freshness': 'fresh', 'pose_raw': {'shoulder_pan': 1},
                                     'pose_synchronized': True,
                                     'phase_event_delta_s': .1}) is not None


def _rotation_about_x(degrees):
    angle = np.deg2rad(degrees)
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[1., 0., 0.], [0., cosine, -sine],
                     [0., sine, cosine]])


def test_depth_plane_normal_selects_ippe_branch_over_wrong_temporal_prior():
    candidates = []
    for degrees, error in ((140., .12), (220., .13)):
        pose = np.eye(4)
        pose[:3, :3] = _rotation_about_x(degrees)
        pose[:3, 3] = [.05, 0., .47]
        candidates.append({'camera_from_tag': pose, 'reprojection_error_px': error})
    previous = candidates[0]['camera_from_tag']
    depth_normal = candidates[1]['camera_from_tag'][:3, 2].copy()

    result = fiducial.select_ippe_pose_candidate(
        candidates, previous_camera_from_tag=previous,
        depth_normal_camera=depth_normal, depth_valid_module_count=15,
        depth_plane_rms_m=.0003)

    assert result['orientation_status'] == 'depth_disambiguated'
    np.testing.assert_allclose(result['camera_from_tag'], candidates[1]['camera_from_tag'])
    assert result['branch_separation_deg'] > 15.


@pytest.mark.parametrize('valid_count,rms', [(0, None), (9, .0003), (15, .0011)])
def test_ambiguous_ippe_branch_without_qualified_depth_is_explicitly_uncertain(valid_count, rms):
    candidates = []
    for degrees, error in ((140., .12), (220., .13)):
        pose = np.eye(4)
        pose[:3, :3] = _rotation_about_x(degrees)
        pose[:3, 3] = [.05, 0., .47]
        candidates.append({'camera_from_tag': pose, 'reprojection_error_px': error})

    result = fiducial.select_ippe_pose_candidate(
        candidates, previous_camera_from_tag=candidates[1]['camera_from_tag'],
        depth_normal_camera=candidates[0]['camera_from_tag'][:3, 2],
        depth_valid_module_count=valid_count, depth_plane_rms_m=rms)

    assert result['orientation_status'] == 'orientation_uncertain'
    assert result['camera_from_tag'] is not None
    assert result['camera_from_tag'][2, 3] == pytest.approx(.47)
    assert result['rotation_deg'] is None


def test_explicit_held_calibration_extracts_rigid_fit_as_unqualified_source():
    path = Path('/home/bjoern/.cache/cps-tag-calibration-held/depth-pnp-hybrid-analysis.json')
    if not path.is_file():
        pytest.skip('held fiducial calibration is unavailable')
    import json
    calibration = fiducial.validate_fiducial_calibration(json.loads(path.read_text()))
    assert calibration['source_status'] == 'SOURCE UNQUALIFIED'
    assert calibration['tag_id'] == 0
    assert calibration['base_from_camera'].shape == (4, 4)
    assert calibration['gripper_from_tag'].shape == (4, 4)


def test_tag_pose_residual_reports_mm_pixels_and_rotation_degrees():
    expected = np.eye(4)
    expected[2, 3] = .5
    observed = expected.copy()
    observed[0, 3] += .002
    observed[:3, :3] = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]]) @ expected[:3, :3]
    residual = fiducial.tag_pose_residual(expected, observed,
        {'fx': 600., 'fy': 600., 'cx': 320., 'cy': 240.}, marker_size_m=.0194)
    assert residual['translation_mm'] == pytest.approx(2.)
    assert residual['pixel'] == pytest.approx(2.4, abs=.03)
    assert residual['rotation_deg'] == pytest.approx(90.)


def test_depth_tag_pose_helper_anchors_translation_on_synthetic_metric_plane():
    cv2 = pytest.importorskip('cv2')
    from workshop.perception.tag_depth_pose import estimate_tag_pose
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    bits = np.zeros((7, 7), dtype=np.uint8)
    bits[1:6, 1:6] = dictionary.getBitsFromByteList(dictionary.bytesList[0:1], 5)
    rgb = np.zeros((280, 280, 3), dtype=np.uint8)
    for row in range(7):
        for column in range(7):
            if bits[row, column]:
                rgb[row*40:(row+1)*40, column*40:(column+1)*40] = 255
    depth = np.full((280, 280), 500, dtype=np.uint16)
    corners = np.array([[0., 0.], [279., 0.], [279., 279.], [0., 279.]])
    estimate = estimate_tag_pose(corners, rgb, depth,
        {'fx': 500., 'fy': 500., 'ppx': 139.5, 'ppy': 139.5, 'model': 'none'},
        .001, bits)
    assert estimate['qualified'] is False
    assert estimate['camera_from_tag'][2][3] == pytest.approx(.5, abs=.002)
    assert estimate['valid_module_count'] >= 8
    assert estimate['plane']['rms_m'] < .001
    assert np.mean(estimate['black_edge_side_lengths_m']) == pytest.approx(.279, abs=.01)


def test_saved_paired_before_after_frames_report_nominal_vs_depth_translation():
    root = Path('/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/'
                'arm-tag-registration-retry')
    names = ('before', 'after')
    metadata = [root / f'relative-step-{name}-latest.json' for name in names]
    if not all(path.is_file() for path in metadata):
        pytest.skip('saved paired wrist-motion fixture is unavailable')
    import cv2
    import json
    from PIL import Image
    from workshop.perception.tag_depth_pose import estimate_tag_pose
    snapshot = json.loads(Path('/home/bjoern/.local/share/exciting-science/station/evidence/'
        '2026-10-14/realsense-station/capture.json').read_text())
    intrinsics = dict(snapshot['intrinsics'])
    intrinsics['model'] = 'none'  # saved coefficients are all zero
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    bits = np.zeros((7, 7), dtype=np.uint8)
    bits[1:6, 1:6] = dictionary.getBitsFromByteList(dictionary.bytesList[0:1], 5)
    selected_center, observed = None, None
    reports = []
    for path in metadata:
        sample = json.loads(path.read_text())
        pair = {'freshness': 'fresh', 'pose_raw': sample['arm']['raw'],
                'pose_qualified_sync': sample['arm']['qualified_sync'],
                'pose_sample_delta_seconds': sample['arm']['delta_s']}
        assert fiducial.paired_pose_raw(pair) == sample['arm']['raw']
        rgb = np.asarray(Image.open(root / sample['color']).convert('RGB'))
        depth = np.load(root / sample['depth'], allow_pickle=False)
        selection = fiducial.track_tag_candidates(fiducial.detect_apriltag_25h9(rgb),
            previous_center=selected_center)
        assert selection['status'] == 'tracked'
        selected_center = selection['center_px']
        pnp = fiducial.estimate_camera_from_tag(selection['corners'], intrinsics,
            previous_camera_from_tag=observed)
        depth_pose = estimate_tag_pose(selection['corners'].reshape(4, 2), rgb, depth,
            intrinsics, snapshot['depth_scale_m'], bits, white_rgb_min=100)
        depth_transform = np.asarray(depth_pose['camera_from_tag'])
        before_z = float(pnp[2, 3])
        after_z = float(depth_transform[2, 3])
        reports.append({'center_px': selected_center.tolist(),
            'nominal_pnp_z_m': before_z, 'depth_anchored_z_m': after_z,
            'delta_z_mm': (after_z-before_z)*1000.,
            'depth_plane_rms_m': depth_pose['plane']['rms_m'],
            'valid_modules': depth_pose['valid_module_count'],
            'observed_edge_mean_m': float(np.mean(depth_pose['black_edge_side_lengths_m']))})
        overlay = fiducial.evaluate_tag_overlay(rgb, pair, pnp, intrinsics,
            previous_center=selected_center, previous_camera_from_tag=pnp,
            depth=depth, depth_scale_m=snapshot['depth_scale_m'])
        assert overlay['depth_anchor']['status'] == 'PROVISIONAL_DEPTH_TRANSLATION'
        assert overlay['nominal_pnp_residual']['translation_mm'] < .5
        assert overlay['residual']['translation_mm'] > 10.
        failed_anchor = fiducial.evaluate_tag_overlay(rgb, pair, pnp, intrinsics,
            previous_center=selected_center, previous_camera_from_tag=pnp,
            depth=np.zeros_like(depth), depth_scale_m=snapshot['depth_scale_m'])
        assert failed_anchor['depth_anchor']['status'].startswith('UNAVAILABLE')
        assert 'ORIENTATION UNCERTAIN' in failed_anchor['status']
        assert failed_anchor['visible'] is True
        assert failed_anchor['center_px'] is not None
        assert np.isfinite(failed_anchor['observed_camera_from_tag'][:3, 3]).all()
        assert failed_anchor['residual']['rotation_deg'] is None
        observed = pnp
    assert reports[0]['valid_modules'] >= 8 and reports[1]['valid_modules'] >= 8
    assert abs(reports[0]['delta_z_mm']) > 10.
    assert abs(reports[1]['delta_z_mm']) > 10.
    assert reports[0]['depth_plane_rms_m'] < .001
    assert reports[1]['depth_plane_rms_m'] < .001
    # The pair changes actual raw arm state and image-space wrist position.
    assert np.linalg.norm(np.asarray(reports[1]['center_px']) - reports[0]['center_px']) > 15.


def test_stale_or_unqualified_pair_hides_all_tag_markers():
    image = np.full((8, 10, 3), 80, dtype=np.uint8)
    result = fiducial.evaluate_tag_overlay(image,
        {'freshness': 'stale', 'pose_raw': {'shoulder_pan': 1},
         'pose_qualified_sync': True, 'pose_sample_delta_seconds': .01},
        np.eye(4), {'fx': 600., 'fy': 600., 'cx': 5., 'cy': 4.})
    assert result['visible'] is False
    assert result['residual'] is None
    np.testing.assert_array_equal(result['image'], image)


def test_fiducial_refresh_gate_limits_saved_image_detection_to_five_hz():
    assert fiducial.detection_due(now=1., last_detection=0., min_interval_s=.2)
    assert not fiducial.detection_due(now=1.199, last_detection=1., min_interval_s=.2)
    assert fiducial.detection_due(now=1.2, last_detection=1., min_interval_s=.2)


def test_detector_helper_has_no_file_or_device_io():
    tree = ast.parse(Path(fiducial.__file__).read_text())
    for name in ('detect_apriltag_25h9', 'base_from_native_site', 'evaluate_tag_overlay'):
        helper = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                      and node.name == name)
        calls = {node.func.id for node in ast.walk(helper) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name)}
        assert not calls.intersection({'open', 'imread', 'VideoCapture', 'VideoStream'})


def test_depth_pose_hot_path_has_no_file_io_or_history_iteration():
    from workshop.perception import tag_depth_pose
    tree = ast.parse(Path(tag_depth_pose.__file__).read_text())
    helper = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                  and node.name == 'estimate_tag_pose')
    calls = {node.func.id for node in ast.walk(helper) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name)}
    assert not calls.intersection({'open', 'load', 'glob', 'rglob', 'listdir',
                                   'VideoCapture', 'imread', 'compile'})


def test_saved_bright_frame_bootstraps_wrist_amid_same_id_background_tag():
    pytest.importorskip('cv2')
    image_path = Path('/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/'
                      'gripper-direction-retry/frames/000001.png')
    if not image_path.is_file():
        pytest.skip('saved local RGB fixture is unavailable')
    from PIL import Image
    rgb = np.asarray(Image.open(image_path).convert('RGB'))
    detections = fiducial.detect_apriltag_25h9(rgb)
    id0 = [corners for marker_id, corners in detections if marker_id == 0]
    assert len(id0) >= 2
    selected = fiducial.track_tag_candidates(detections)
    assert selected['status'] == 'tracked'
    np.testing.assert_allclose(selected['center_px'], [375.2, 252.], atol=1.)
    intrinsics = {'fx': 604.8411, 'fy': 604.4875,
                  'ppx': 327.3613, 'ppy': 247.1974, 'coeffs': [0., 0., 0., 0., 0.]}
    observed_pose = fiducial.estimate_camera_from_tag(selected['corners'], intrinsics)
    assert observed_pose is not None
    paired = {'freshness': 'fresh', 'pose_raw': {'shoulder_pan': 1},
              'pose_qualified_sync': True, 'pose_sample_delta_seconds': .01}
    overlay = fiducial.evaluate_tag_overlay(rgb, paired, observed_pose, intrinsics)
    assert overlay['visible']
    assert overlay['status'].startswith('SOURCE UNQUALIFIED')
    assert overlay['residual']['translation_mm'] < .01


def test_saved_paired_actual_motion_moves_wrist_track_and_uses_each_raw_pose():
    root = Path('/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-15/'
                'arm-tag-registration-retry')
    before_meta, after_meta = root / 'relative-step-before-latest.json', root / 'relative-step-after-latest.json'
    before_image, after_image = root / 'frames/000092.png', root / 'frames/000263.png'
    if not all(path.is_file() for path in (before_meta, after_meta, before_image, after_image)):
        pytest.skip('saved paired wrist-motion fixture is unavailable')
    import json
    from PIL import Image
    snapshot = Path('/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-14/'
                    'realsense-station/capture.json')
    if not snapshot.is_file():
        pytest.skip('saved camera intrinsics are unavailable')
    intrinsics_record = json.loads(snapshot.read_text())['intrinsics']
    intrinsics = {'fx': intrinsics_record['fx'], 'fy': intrinsics_record['fy'],
                  'ppx': intrinsics_record['ppx'], 'ppy': intrinsics_record['ppy'],
                  'coeffs': intrinsics_record['coeffs']}
    from workshop.perception.tag_depth_pose import estimate_tag_pose
    cv2 = pytest.importorskip('cv2')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    tag_bits = np.zeros((7, 7), dtype=np.uint8)
    tag_bits[1:6, 1:6] = dictionary.getBitsFromByteList(dictionary.bytesList[0:1], 5)
    depth_scale = json.loads(snapshot.read_text())['depth_scale_m']
    selected = None
    observed_pose = None
    centers, poses, observed_poses = [], [], []
    for metadata_path, image_path in ((before_meta, before_image), (after_meta, after_image)):
        sample = json.loads(metadata_path.read_text())
        arm = sample['arm']
        pair = {'freshness': 'fresh', 'pose_raw': arm['raw'],
                'pose_qualified_sync': arm['qualified_sync'],
                'pose_sample_delta_seconds': arm['delta_s']}
        assert fiducial.paired_pose_raw(pair) == arm['raw']
        rgb = np.asarray(Image.open(image_path).convert('RGB'))
        selection = fiducial.track_tag_candidates(fiducial.detect_apriltag_25h9(rgb),
            previous_center=selected)
        assert selection['status'] == 'tracked'
        selected = selection['center_px']
        depth = np.load(root / sample['depth'], allow_pickle=False)
        depth_fit = estimate_tag_pose(selection['corners'].reshape(4, 2), rgb, depth,
            intrinsics, depth_scale, tag_bits, white_rgb_min=100)
        diagnostic = fiducial.estimate_camera_from_tag(selection['corners'], intrinsics,
            previous_camera_from_tag=observed_pose,
            depth_normal_camera=depth_fit['plane']['normal_camera'],
            depth_valid_module_count=depth_fit['valid_module_count'],
            depth_plane_rms_m=depth_fit['plane']['rms_m'], return_diagnostics=True)
        assert diagnostic['orientation_status'] == 'depth_disambiguated'
        observed_pose = diagnostic['camera_from_tag']
        assert observed_pose is not None
        centers.append(selected.copy())
        poses.append(arm['raw'])
        observed_poses.append(observed_pose.copy())
    assert np.linalg.norm(centers[1] - centers[0]) > 15.
    assert poses[0]['shoulder_pan'] != poses[1]['shoulder_pan']
    # The independent depth-plane normal resolves each planar IPPE mirror branch.
    relative = observed_poses[0][:3, :3].T @ observed_poses[1][:3, :3]
    angle = np.rad2deg(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    assert angle < 20.
