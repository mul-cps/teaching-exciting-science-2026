from dataclasses import dataclass
import numpy as np
import pytest

from workshop.digital_twin.frame_conventions import convert_native_pinch_offset
from tools.plan_surface_pick import _pinch_rotation, plan_surface_pick


CAL = {name: {'range_min': 1000, 'range_max': 3000} for name in (
    'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')}
SEED = {name: 2000 for name in CAL}
SEED['gripper'] = 2200


def inputs(footprint=False):
    candidate = {'center_cam': [0., 0., .51], 'closing_axis_cam': [1., 1., 2.],
                 'height_prior_m': .02}
    if footprint:
        candidate['footprint_axes_cam'] = [[0., 1., .2], [1., -1., 0.]]
    plane = {'frame': 'camera', 'normal': [0., 0., 1.], 'd': -.5}
    ext = {'base_frame': 'urdf:base_link', 'direction': 'base_from_camera',
           'base_from_camera': np.eye(4), 'qualified': False}
    offset = {'parent_frame': 'urdf:gripper_frame_link',
              'gripper_frame_from_pinch': convert_native_pinch_offset(
                  [.0059602, .0000590, .0163374]),
              'opening_raw': 2200,
              'qualified': False}
    return candidate, plane, ext, offset, CAL, SEED


@dataclass
class Result:
    success: bool
    raw: dict | None
    pose: np.ndarray | None
    position_error_m: float = 0.
    orientation_error_rad: float = 0.
    iterations: int = 2
    elapsed_s: float = .001
    reason: str = 'converged'
    provenance: dict | None = None


class FakeIK:
    instances = []
    fail_stage = None
    bad_raw = False

    def __init__(self, calibration, offset_provider):
        self.calibration = calibration
        self.offset_provider = offset_provider
        self.poses = {}
        self.seeds = []
        self.solve_count = 0
        self.closed = False
        FakeIK.instances.append(self)
        self.poses[self.key(SEED)] = np.eye(4)
        self.poses[self.key(SEED)][:3, 3] = [0., 0., .5]

    @staticmethod
    def key(raw):
        return tuple(raw[name] for name in CAL)

    def forward_raw(self, raw, opening_raw=None):
        pose = self.poses.get(self.key(raw), np.eye(4)).copy()
        if self.key(raw) not in self.poses:
            pose[:3, 3] = [0., 0., .5]
        return pose

    def solve_pinch(self, target, seed, **kwargs):
        assert kwargs['position_tolerance_m'] == .001
        assert kwargs['orientation_weight'] in (0., .01)
        self.seeds.append(dict(seed))
        self.solve_count += 1
        if FakeIK.fail_stage is not None and np.isclose(target[2, 3], .53):
            return Result(False, None, None, position_error_m=.01,
                          orientation_error_rad=.3, reason='unreachable')
        raw = dict(seed)
        raw['shoulder_pan'] += 10
        if FakeIK.bad_raw:
            raw['elbow_flex'] = CAL['elbow_flex']['range_max'] + 1
        self.poses[self.key(raw)] = np.asarray(target).copy()
        return Result(True, raw, np.asarray(target).copy())

    def close(self):
        self.closed = True


def run_plan(candidate=None):
    args = inputs() if candidate is None else (candidate, *inputs()[1:])
    return plan_surface_pick(*args, 2200, ik_factory=FakeIK)


def test_surface_rotation_is_right_handed_and_uses_projected_closing_axis():
    FakeIK.instances.clear()
    report = run_plan()
    assert len(report['attempts']) == 2
    for attempt in report['attempts']:
        rotation = np.asarray(attempt['target_rotation_base'])
        assert np.linalg.det(rotation) == pytest.approx(1.)
        assert rotation[:, 0] == pytest.approx([0., 0., -1.])
        assert abs(rotation[:, 2] @ np.array([1., 1., 0.]) / np.sqrt(2)) > .999
        assert np.isfinite(attempt['initial_seed_axis_residuals']['tool_x_down_error_rad'])
        assert attempt['usable_numerically']
        assert not attempt['executable'] and not attempt['physical_qualified']
        assert not attempt['finger_clearance_verified']
        assert attempt['pinch_midpoint_clearance_passed']
        assert all(w['axis_residuals']['tool_x_down_error_rad'] == pytest.approx(0.)
                   for w in attempt['waypoints'])
        assert [w['name'] for w in attempt['waypoints']] == ['pregrasp', 'approach', 'grasp', 'lift']
        assert [w['target_base'][2][3] for w in attempt['waypoints']] == pytest.approx(
            [.57, .53, .51, .59])
        assert all(w['interpolation_raw'] for w in attempt['waypoints'])
        seed_solve = attempt['waypoints'][0]['position_only_seed_solve']
        assert seed_solve['success'] and not seed_solve['motion_command']
        assert seed_solve['purpose'].startswith('IK seed only')
    assert report['usable_numerically'] and report['best_attempt_index'] in (0, 1)
    assert not report['executable'] and not report['grasp_eligible']
    assert not report['physical_qualified'] and not report['finger_clearance_verified']
    assert not report['execution_performed'] and len(report['planned_commands']) == 4
    assert all(c['raw']['gripper'] == 2200 for c in report['planned_commands'])
    solver = FakeIK.instances[0]
    assert solver.closed
    assert solver.seeds[1]['shoulder_pan'] == solver.seeds[0]['shoulder_pan'] + 10
    assert solver.seeds[2]['shoulder_pan'] == solver.seeds[1]['shoulder_pan'] + 10
    assert solver.seeds[3]['shoulder_pan'] == solver.seeds[2]['shoulder_pan'] + 10
    # Each axis/sign attempt restarts from the supplied current pose.
    assert solver.seeds[5] == SEED


def test_optional_footprint_axes_try_both_signs_and_choose_only_complete_paths():
    FakeIK.instances.clear()
    report = run_plan(inputs(footprint=True)[0])
    assert [(a['axis'], a['sign']) for a in report['attempts']] == [
        ('closing_axis', 1), ('closing_axis', -1),
        ('footprint_axis_0', 1), ('footprint_axis_0', -1),
        ('footprint_axis_1', 1), ('footprint_axis_1', -1)]
    assert all(attempt['usable_numerically'] for attempt in report['attempts'])


def test_pregrasp_offset_can_be_raised_to_100_mm():
    FakeIK.instances.clear()
    report = plan_surface_pick(*inputs(), 2200, ik_factory=FakeIK, pregrasp_offset_m=.1)
    assert report['parameters']['pregrasp_offset_m'] == .1
    assert all(a['waypoints'][0]['target_base'][2][3] == pytest.approx(.61)
               for a in report['attempts'])


def test_ik_failure_blocks_later_waypoints_and_never_marks_path_usable():
    FakeIK.instances.clear()
    FakeIK.fail_stage = 1
    try:
        report = run_plan()
    finally:
        FakeIK.fail_stage = None
    assert report['best_attempt_index'] is None
    assert not report['usable_numerically']
    assert report['planned_commands'] == []
    for attempt in report['attempts']:
        assert not attempt['usable_numerically']
        assert [w['name'] for w in attempt['waypoints']] == ['pregrasp', 'approach']
        assert attempt['waypoints'][0]['position_only_seed_solve']['success']
        assert not attempt['waypoints'][0]['position_only_seed_solve']['motion_command']
        assert attempt['waypoints'][1]['reason'] == 'unreachable'
        assert 'tool_x_down_error_rad' in attempt['initial_seed_axis_residuals']
        assert attempt['blocked_waypoints'] == ['grasp', 'lift']


def test_joint_bound_violation_propagates_as_failed_path():
    FakeIK.instances.clear()
    FakeIK.bad_raw = True
    try:
        report = run_plan()
    finally:
        FakeIK.bad_raw = False
    assert report['best_attempt_index'] is None
    assert all(not attempt['usable_numerically'] for attempt in report['attempts'])
    assert all('calibrated joint bounds' in attempt['waypoints'][0]['failure']
               for attempt in report['attempts'])


@pytest.mark.parametrize(('which', 'key', 'value'), [
    (2, 'base_frame', 'world'),
    (2, 'direction', 'camera_from_base'),
    (3, 'parent_frame', 'urdf:gripper_frame_site'),
    (1, 'frame', 'base'),
])
def test_wrong_frame_documents_are_rejected(which, key, value):
    docs = list(inputs())
    docs[which][key] = value
    with pytest.raises(ValueError):
        plan_surface_pick(*docs, 2200, ik_factory=FakeIK)


def test_opening_must_match_offset_and_target_frame_offsets_are_explicit():
    docs = list(inputs())
    docs[3]['opening_raw'] = 2199
    with pytest.raises(ValueError, match='opening_raw'):
        plan_surface_pick(*docs, 2200, ik_factory=FakeIK)
    report = run_plan()
    assert report['frames']['base'] == 'urdf:base_link'
    assert report['frames']['pinch_axes'].startswith('native-MuJoCo-gripper_frame_site')


def test_rejects_pinch_offset_without_native_site_axis_bridge():
    bridge = inputs()[3]['gripper_frame_from_pinch'][:3, :3]
    wrong_rotations = [np.eye(3), bridge @ np.array([
        [np.cos(.1), -np.sin(.1), 0.], [np.sin(.1), np.cos(.1), 0.], [0., 0., 1.]])]
    for rotation in wrong_rotations:
        docs = list(inputs())
        docs[3]['gripper_frame_from_pinch'] = np.eye(4)
        docs[3]['gripper_frame_from_pinch'][:3, :3] = rotation
        with pytest.raises(ValueError, match='native-site axes'):
            plan_surface_pick(*docs, 2200, ik_factory=FakeIK)


def test_native_site_target_axes_follow_official_mujoco_gripper_body_axes():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.scene import make_model
    model = mujoco.MjModel.from_xml_string(
        make_model([0, 0, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_frame_site')
    body_rotation = data.xmat[body].reshape(3, 3)
    site_rotation = data.site_xmat[site].reshape(3, 3)
    site_from_body = site_rotation.T @ body_rotation
    # Physical finger direction is gripper-body -Z; closing axis is body +X.
    np.testing.assert_allclose(site_from_body @ [0, 0, -1], [1, 0, 0], atol=1e-6)
    np.testing.assert_allclose(site_from_body @ [1, 0, 0], [0, 0, 1], atol=1e-6)
    target = _pinch_rotation(np.array([1, 0, 0]), np.array([0, 0, 1]))
    np.testing.assert_allclose(target[:, 0], [0, 0, -1])
    np.testing.assert_allclose(target[:, 2], [1, 0, 0])
    np.testing.assert_allclose(target[:, 1], [0, 1, 0])
