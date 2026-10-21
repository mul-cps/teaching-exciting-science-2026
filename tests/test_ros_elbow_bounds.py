"""Model and already-admitted limits for the held-elbow calibration path."""
import math

import pytest

from workshop.station.ros_trajectory import ARM_JOINTS, prepare_trajectory
from tools.ros_plan_current import prepare_with_retiming


def wrist_path(elbow=91.56043956043956):
    start = dict.fromkeys(ARM_JOINTS, 0.)
    start['elbow_flex'] = math.radians(elbow)
    end = dict(start, wrist_flex=-math.radians(1.))
    return start, [{'time': 0., 'positions': list(start.values())},
                   {'time': 2., 'positions': list(end.values())}]


def prepare(start, points, **kwargs):
    return prepare_trajectory(ARM_JOINTS, points, start_radians=start,
        max_velocity=.125, max_acceleration=.2, max_jerk=2., **kwargs)


def test_actual_held_elbow_wrist_only_path_and_planner_retiming_are_valid():
    start, points = wrist_path()
    result = prepare(start, points)
    factor, _, planned = prepare_with_retiming(ARM_JOINTS, points, start_radians=start)
    assert factor >= 1
    assert all(sample.positions[2] == pytest.approx(start['elbow_flex']) for sample in result.samples)
    assert planned.samples[-1].positions[3] == pytest.approx(-math.radians(1.))


@pytest.mark.parametrize('joint', ARM_JOINTS)
@pytest.mark.parametrize('sign', [-1, 1])
def test_each_joint_keeps_its_own_model_envelope(joint, sign):
    start = dict.fromkeys(ARM_JOINTS, 0.)
    limit = (1.74533 if joint == 'shoulder_lift' else
             1.69 if joint == 'elbow_flex' else
             1.65806 if joint == 'wrist_flex' else math.pi/2)
    start[joint] = sign*(limit+.0001)
    points = [{'time': 0., 'positions': list(start.values())},
              {'time': 2., 'positions': list(start.values())}]
    with pytest.raises(ValueError, match='position limit exceeded: '+joint):
        prepare(start, points)


def test_explicit_calibrated_admission_replaces_portable_model_envelope():
    start, points = wrist_path(96.84)
    wide = dict.fromkeys(ARM_JOINTS, (-2., 2.))
    assert prepare(start, points, position_limits=wide).samples[0].positions[2] == start['elbow_flex']
    start, points = wrist_path()
    narrow = dict.fromkeys(ARM_JOINTS, (-1., 1.))
    with pytest.raises(ValueError, match='position limit exceeded: elbow_flex'):
        prepare(start, points, position_limits=narrow)


@pytest.mark.parametrize('limits', [{}, {'elbow_flex': (-1., 1.)},
    dict.fromkeys(ARM_JOINTS, (1., -1.)), dict.fromkeys(ARM_JOINTS, (0., float('nan'))),
    dict.fromkeys(ARM_JOINTS, (True, 1.)), dict.fromkeys(ARM_JOINTS, (2., 3.)), dict.fromkeys(ARM_JOINTS, (0., 0.)),
    dict.fromkeys(ARM_JOINTS, (-1., float('inf')))])
def test_invalid_or_disjoint_admitted_bounds_are_rejected(limits):
    start, points = wrist_path()
    with pytest.raises(ValueError): prepare(start, points, position_limits=limits)


@pytest.mark.parametrize('joint', ARM_JOINTS)
def test_interpolated_extrema_use_admitted_bounds_between_legal_knots(joint):
    start = dict.fromkeys(ARM_JOINTS, 0.)
    points = []
    for time, value in enumerate((0., .009, .009, 0.)):
        positions = dict(start, **{joint: value})
        points.append({'time': float(time)*2, 'positions': list(positions.values())})
    limits = dict.fromkeys(ARM_JOINTS, (-1., 1.))
    limits[joint] = (-.01, .009)
    with pytest.raises(ValueError, match='interpolated position limit exceeded: '+joint):
        prepare(start, points, position_limits=limits)


def owner_at_actual_elbow(tmp_path, calibration_max=3055):
    import hashlib
    import json
    from pathlib import Path
    from test_arm_session import make_session
    session, arm, clock = make_session(tmp_path)
    arm._cal['elbow_flex'].update(range_min=848, range_max=calibration_max)
    arm.settings['limits']['elbow_flex'].update(min=-96.82986737710912, max=96.82986737710912)
    calibration = Path(arm.settings['calibration']['path'])
    calibration.write_text(json.dumps(arm._cal, sort_keys=True))
    digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
    arm.settings['calibration']['sha256'] = digest
    home_path = Path(arm.settings['home_file'])
    home = json.loads(home_path.read_text())
    home['calibration_sha256'] = digest
    home_path.write_text(json.dumps(home))
    arm.bus.data['Present_Position']['elbow_flex'] = 2993
    arm.bus.data['Goal_Position']['elbow_flex'] = 2993
    return session, arm, clock


def test_owner_accepts_actual_held_elbow_wrist_path_without_digest_change(tmp_path):
    from workshop.station.ros_trajectory import path_digest
    session, arm, _ = owner_at_actual_elbow(tmp_path)
    session.initialize()
    start = session._arm_radians(arm.bus.data['Present_Position'])
    end = dict(start, wrist_flex=start['wrist_flex']-math.radians(1.))
    points = [{'time': 0., 'positions': list(start.values())},
              {'time': 2., 'positions': list(end.values())}]
    planned = prepare(start, points)
    digest = path_digest(planned, held_gripper_raw=2000)
    session.execute_trajectory('held-elbow', ARM_JOINTS, points, expected_path_sha256=digest)
    assert arm.bus.goal_history
    assert all(goal['elbow_flex'] == 2993 for goal in arm.bus.goal_history)
    assert arm.bus.data['Present_Position']['wrist_flex'] < 2000


def test_owner_exact_admitted_limits_reject_subcount_outside_bound_before_writes(tmp_path):
    from workshop.station.ros_trajectory import path_digest
    session, arm, _ = owner_at_actual_elbow(tmp_path)
    arm.settings['limits']['wrist_flex']['max'] = 0.
    session.initialize()
    start = session._arm_radians(arm.bus.data['Present_Position'])
    end = dict(start, wrist_flex=.00001)
    points = [{'time': 0., 'positions': list(start.values())},
              {'time': 2., 'positions': list(end.values())}]
    digest = path_digest(prepare(start, points), held_gripper_raw=2000)
    with pytest.raises(ValueError, match='position limit exceeded: wrist_flex'):
        session.execute_trajectory('outside-admitted', ARM_JOINTS, points, expected_path_sha256=digest)
    assert not arm.bus.writes and not arm.bus.goal_history


@pytest.mark.parametrize('joint', ARM_JOINTS)
@pytest.mark.parametrize('sign', [-1, 1])
def test_exact_model_boundaries_remain_accepted(joint, sign):
    start = dict.fromkeys(ARM_JOINTS, 0.)
    start[joint] = sign*(1.69 if joint == 'elbow_flex' else math.pi/2)
    points = [{'time': 0., 'positions': list(start.values())},
              {'time': 2., 'positions': list(start.values())}]
    assert prepare(start, points).samples[-1].positions[ARM_JOINTS.index(joint)] == pytest.approx(start[joint])


def test_named_bounds_are_correct_with_reordered_input_joints():
    start, points = wrist_path()
    names = tuple(reversed(ARM_JOINTS))
    reordered = [dict(point, positions=list(reversed(point['positions']))) for point in points]
    result = prepare_trajectory(names, reordered, start_radians=start,
        max_velocity=.125, max_acceleration=.2, max_jerk=2.)
    assert result.samples[0].positions[2] == pytest.approx(start['elbow_flex'])


@pytest.mark.parametrize('constraint', ['calibration', 'firmware', 'configured'])
def test_owner_intersection_rejects_each_real_bound_before_bus_writes(tmp_path, constraint):
    from workshop.station.ros_trajectory import path_digest
    session, arm, _ = owner_at_actual_elbow(tmp_path,
        calibration_max=3040 if constraint == 'calibration' else 3055)
    if constraint == 'firmware': arm.bus.data['Max_Position_Limit']['elbow_flex'] = 3040
    if constraint == 'configured': arm.settings['limits']['elbow_flex']['max'] = 96.
    session.initialize()
    start = session._arm_radians(arm.bus.data['Present_Position'])
    end = dict(start, elbow_flex=1.685)
    points = [{'time': 0., 'positions': list(start.values())},
              {'time': 2., 'positions': list(end.values())}]
    digest = path_digest(prepare(start, points), held_gripper_raw=2000)
    with pytest.raises(ValueError, match='position limit exceeded: elbow_flex'):
        session.execute_trajectory('outside-'+constraint, ARM_JOINTS, points, expected_path_sha256=digest)
    assert not arm.bus.writes and not arm.bus.goal_history


def test_admitted_bound_conversion_stays_outside_owner_sample_and_tick_loops():
    import ast
    import inspect
    import textwrap
    from workshop.station.arm_session import ArmSession
    tree = ast.parse(textwrap.dedent(inspect.getsource(ArmSession.execute_trajectory)))
    conversions = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and
        isinstance(node.func, ast.Attribute) and node.func.attr == '_arm_radians' and
        node.args and isinstance(node.args[0], ast.Attribute) and
        node.args[0].attr in {'admitted_min', 'admitted_max'}]
    assert len(conversions) == 2
    for loop in ast.walk(tree):
        if isinstance(loop, (ast.For, ast.While, ast.ListComp, ast.DictComp)):
            assert not any(node is conversion for node in ast.walk(loop) for conversion in conversions)
