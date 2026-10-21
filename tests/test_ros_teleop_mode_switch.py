"""Real bridge methods with fake owner/leader state; no ROS or actuator I/O."""
import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import uuid

import pytest

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / 'ros2/src/so101_station_control/so101_station_control'
SOURCE = CONTROL / 'trajectory_server.py'
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


def bridge_class():
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'TrajectoryBridge')
    wanted = {'_active_goal_is', '_prepare_owner_state', '_teleop_start_error',
              '_set_teleop_enabled', '_set_teleop_absolute_mode', '_clear_teleop', '_teleop_tick'}
    body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in wanted]
    for node in body:
        node.decorator_list = []
    helpers = [node for node in tree.body if isinstance(node, ast.FunctionDef) and
               node.name in {'_arm_bounds_observation', '_teleop_leader_readiness'}]
    leader = ast.parse((CONTROL / 'leader_reader.py').read_text())
    helpers += [node for node in leader.body if isinstance(node, ast.FunctionDef) and node.name == 'matched_pose']
    namespace = dict(math=math, time=time, uuid=uuid, TELEOP_JOINTS=JOINTS, JOINTS=JOINTS)
    unit = ast.Module(body=helpers + [ast.ClassDef(name='Bridge', bases=[], keywords=[],
                      body=body, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(unit), str(SOURCE), 'exec'), namespace)
    return namespace['Bridge']


def state(torque=1, **changes):
    stamp = time.monotonic()
    value = dict(monotonic=stamp, torque_monotonic=stamp, observe_only=False,
        arm_id='fake', calibration_sha256='a' * 64, owner_source_sha256={'owner': 'b' * 64},
        raw=dict.fromkeys(JOINTS, 2000), torque=dict.fromkeys(JOINTS, torque),
        pose=dict.fromkeys(JOINTS, 25.), arm_bounds=dict(schema='arm-bounds-observation-v1',
            classification='normal', position_monotonic=stamp,
            admitted_min=dict.fromkeys(JOINTS, 1000), admitted_max=dict.fromkeys(JOINTS, 3000),
            firmware_min=dict.fromkeys(JOINTS, 500), firmware_max=dict.fromkeys(JOINTS, 3500)))
    value.update(changes)
    return value


@pytest.fixture
def bridge():
    node = bridge_class()()
    node._process = SimpleNamespace(poll=lambda: None)
    node._owner_started = time.monotonic() - 1
    node._home_completed_owner_started = node._owner_started
    node._expected_arm_id, node._expected_calibration_sha256 = 'fake', 'a' * 64
    node._owner_source_expected = {'owner': 'b' * 64}
    node._closed = node._external_state_only = node._owner_faulted = False
    node._reconnect_observe = node._stop_latched = node._cancel_before_start = False
    node._active_goal = None
    node._active_lock = threading.Lock()
    node._teleop_latest_lock = threading.Lock()
    node._teleop_condition = threading.Condition()
    node._teleop_acknowledgements = {}
    node._teleop_state = 'disabled'
    node._teleop_mode = 'relative'
    node._teleop_stop_reason = None
    node._teleop_step_done = threading.Event()
    node._teleop_step_done.set()
    node._teleop_latest = dict(stamp_s=time.monotonic(), sequence=1, pose=dict.fromkeys(JOINTS, 5.))
    node.get_parameter = lambda key: SimpleNamespace(value={'state_freshness_s': .25,
        'teleop_pose_stale_s': .15, 'teleop_owner_timeout_s': .1}[key])
    node._publish_teleop_status = lambda *args, **kwargs: None
    node._read_state = state
    node.commands = []
    node._send = node.commands.append
    def ack(key):
        if key == 'teleop_stop':
            return {'ok': True}
        # Newest leader reference is deliberately different from the pre-send
        # packet: the owner ACK baseline and this reference must win.
        node._teleop_latest = dict(stamp_s=time.monotonic(), sequence=2, pose=dict.fromkeys(JOINTS, 10.))
        return {'ok': True, 'state': state(pose=dict.fromkeys(JOINTS, 30.))}
    node._wait_teleop_ack = ack
    return node


def enable(node):
    return node._set_teleop_enabled(SimpleNamespace(data=True), SimpleNamespace(success=False, message=''))


def test_first_owner_session_uses_fresh_current_pose_without_home_completion(bridge):
    bridge._home_completed_owner_started = None
    result = enable(bridge)
    assert result.success
    assert bridge.commands[0]['command'] == 'teleop_start'
    assert bridge._home_completed_owner_started is None


def test_new_owner_generation_can_teleop_but_never_inherits_home_proof(bridge):
    bridge._owner_started += .2
    previous = bridge._home_completed_owner_started
    assert enable(bridge).success
    assert bridge._home_completed_owner_started == previous != bridge._owner_started


@pytest.mark.parametrize('torque', [0, 1])
def test_mode_switch_uses_ack_pose_and_post_ack_leader_origin_without_jump(bridge, torque):
    bridge._read_state = lambda: state(torque=torque)  # away from any saved home
    assert enable(bridge).success
    assert bridge._teleop_origin == dict.fromkeys(JOINTS, 10.)
    assert bridge._teleop_follower_start == dict.fromkeys(JOINTS, 30.)
    # Same measured leader pose at its next sequence yields exactly the
    # acknowledged current follower pose, rather than the pre-start snapshot.
    bridge._teleop_latest = dict(stamp_s=time.monotonic(), sequence=3, pose=dict.fromkeys(JOINTS, 10.))
    bridge._teleop_tick()
    assert bridge.commands[-1]['command'] == 'teleop_step'
    assert bridge.commands[-1]['pose'] == dict.fromkeys(JOINTS, 30.)


@pytest.mark.parametrize('fault', ['source', 'calibration', 'stale', 'mixed_torque', 'bounds', 'raw', 'pose'])
def test_mode_switch_preserves_current_pose_admission_guards(bridge, fault):
    current = state()
    if fault == 'source': current['owner_source_sha256'] = {'owner': 'c' * 64}
    if fault == 'calibration': current['calibration_sha256'] = 'c' * 64
    if fault == 'stale': current['torque_monotonic'] -= 1
    if fault == 'mixed_torque': current['torque']['gripper'] = 0
    if fault == 'bounds': current['arm_bounds']['classification'] = 'bounded_reentry_required'
    if fault == 'raw': current['raw']['elbow_flex'] = 3050
    if fault == 'pose': current['pose']['wrist_roll'] = float('nan')
    bridge._read_state = lambda: current
    assert not enable(bridge).success
    assert not bridge.commands


@pytest.mark.parametrize('fault', ['source', 'old_ack', 'cohort_changed', 'stale_leader'])
def test_owner_start_ack_must_remain_fresh_and_in_same_cohort(bridge, fault):
    def ack(key):
        if key == 'teleop_stop': return {'ok': True}
        current = state()
        if fault == 'source': current['owner_source_sha256'] = {'owner': 'c' * 64}
        if fault == 'old_ack': current['monotonic'] -= 1
        if fault == 'cohort_changed':
            bridge._owner_source_expected = current['owner_source_sha256'] = {'owner': 'c' * 64}
        if fault == 'stale_leader': bridge._teleop_latest['stamp_s'] -= 1
        return {'ok': True, 'state': current}
    bridge._wait_teleop_ack = ack
    assert not enable(bridge).success
    assert [command['command'] for command in bridge.commands] == ['teleop_start', 'teleop_stop']
    assert bridge._teleop_state == 'disabled'


def test_status_exposes_readiness_using_cached_state_without_io():
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'TrajectoryBridge')
    status = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '_status')
    strings = {node.value for node in ast.walk(status) if isinstance(node, ast.Constant)}
    assert {'teleop_start_ready', 'teleop_start_reason'} <= strings
    calls = {node.func.attr for node in ast.walk(status) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert '_teleop_start_error' in calls and '_read_state' not in calls
