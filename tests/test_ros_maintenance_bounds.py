"""Offline maintenance admission regressions; no ROS, services, or actuator access."""
import ast
import copy
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import uuid

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_control/so101_station_control/trajectory_server.py'
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


def bridge_class():
    tree = ast.parse(SOURCE.read_text())
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TrajectoryBridge')
    names = {'_active_goal_is', '_operator_maintenance', '_prepare_owner_state',
             '_assert_owner_reservation', '_expire_operator_modes'}
    methods = [n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names]
    for method in methods:
        method.decorator_list = []
    helper = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_arm_bounds_observation')
    tree = ast.Module(body=[helper, ast.ClassDef(name='Bridge', bases=[], keywords=[], body=methods,
                                                decorator_list=[])], type_ignores=[])
    namespace = dict(math=math, time=time, json=json, uuid=uuid, TELEOP_JOINTS=JOINTS)
    exec(compile(ast.fix_missing_locations(tree), str(SOURCE), 'exec'), namespace)
    return namespace['Bridge']


@pytest.fixture
def owner():
    node = bridge_class()()
    node._closed = node._external_state_only = node._reconnect_observe = False
    node._owner_faulted = node._stop_latched = node._cancel_before_start = False
    node._process = SimpleNamespace(poll=lambda: None)
    node._owner_started = time.monotonic() - .01
    node._expected_arm_id = 'fake-arm'
    node._expected_calibration_sha256 = 'a' * 64
    node._owner_source_expected = {'arm_session': 'b' * 64}
    node._active_lock = threading.Lock()
    node._state_cache_lock = threading.Lock()
    node._active_goal = node._prepare_reservation = None
    node._teleop_state = 'disabled'
    node._operator_external_job = node._operator_maintenance_token = None
    node._operator_maintenance_deadline = None
    node.get_parameter = lambda name: SimpleNamespace(value=.1)
    stamp = time.monotonic()
    state = dict(monotonic=stamp, torque_monotonic=stamp, observe_only=False,
        arm_id=node._expected_arm_id, calibration_sha256=node._expected_calibration_sha256,
        owner_source_sha256=copy.deepcopy(node._owner_source_expected), native_damping_restore_pending=False,
        raw=dict.fromkeys(JOINTS, 2000), torque=dict.fromkeys(JOINTS, 0))
    state['arm_bounds'] = dict(schema='arm-bounds-observation-v1', classification='normal',
        position_monotonic=stamp, admitted_min=dict.fromkeys(JOINTS, 100),
        admitted_max=dict.fromkeys(JOINTS, 3052), firmware_min=dict.fromkeys(JOINTS, 0),
        firmware_max=dict.fromkeys(JOINTS, 3055))
    node._read_state = lambda: copy.deepcopy(state)
    node._cached_owner_state = state
    node._cached_owner_generation = node._owner_started
    # Any arm command, including torque enable or position write, fails the test.
    node._send = lambda *args: pytest.fail('maintenance must never send arm commands')
    restarts = []
    node._operator_restart_begin = lambda token, unit: restarts.append((token, unit)) or 'job'
    return node, state, restarts


def maintenance(node, *, enable=True, token='', operation=''):
    request = SimpleNamespace(enable=enable, token=token, operation=operation, unit='so101-camera.service')
    return node._operator_maintenance(request, SimpleNamespace(success=False, message='', token=''))


@pytest.mark.parametrize('classification,raw', [('normal', 2000),
    ('bounded_reentry_required', 3054), ('unrecoverable', 3066)])
def test_all_off_maintenance_reserves_restarts_and_releases_outside_arm_bounds(owner, classification, raw):
    node, state, restarts = owner
    state['raw']['elbow_flex'] = raw
    state['arm_bounds']['classification'] = classification
    result = maintenance(node)
    assert result.success, result.message
    token = result.token
    assert token and node._active_goal == 'operator_maintenance'
    result = maintenance(node, operation='restart', token=token)
    assert result.success, result.message
    assert restarts == [(token, 'so101-camera.service')]
    result = maintenance(node, enable=False, token=token)
    assert result.success, result.message
    assert node._active_goal is None and node._operator_maintenance_token is None
    assert node._operator_maintenance_deadline is None
    assert set(state['torque'].values()) == {0}


def test_ordinary_prepare_still_rejects_unrecoverable_and_maintenance_flag_cannot_admit_torque_on(owner):
    node, state, _ = owner
    state['raw']['elbow_flex'] = 3066
    state['arm_bounds']['classification'] = 'unrecoverable'
    with pytest.raises(RuntimeError, match='bounds'):
        node._prepare_owner_state(state, node._process, node._owner_started, torque=0)
    state['torque'] = dict.fromkeys(JOINTS, 1)
    with pytest.raises(RuntimeError, match='bounds'):
        node._prepare_owner_state(state, node._process, node._owner_started,
                                  torque=1, nonactuating_maintenance=True)


@pytest.mark.parametrize('failure', ['unknown', 'missing_bounds', 'malformed_bounds', 'bounds_stamp',
    'stale_position', 'stale_torque', 'future', 'nan', 'arm_id', 'calibration', 'source',
    'raw_missing', 'raw_extra', 'raw_bool', 'raw_range', 'torque_missing', 'torque_on',
    'observe', 'generation', 'dead', 'busy', 'teleop', 'stop', 'fault', 'cancel', 'closed'])
def test_invalid_or_unsafe_maintenance_admission_still_rejects(owner, failure):
    node, state, restarts = owner
    state['arm_bounds']['classification'] = 'unrecoverable'
    if failure == 'unknown': state['arm_bounds']['classification'] = 'unknown'
    if failure == 'missing_bounds': state.pop('arm_bounds')
    if failure == 'malformed_bounds': state['arm_bounds']['firmware_max']['elbow_flex'] = 10
    if failure == 'bounds_stamp': state['arm_bounds']['position_monotonic'] -= 1
    if failure == 'stale_position': state['monotonic'] -= 1
    if failure == 'stale_torque': state['torque_monotonic'] -= 1
    if failure == 'future': state['torque_monotonic'] += 1
    if failure == 'nan': state['torque_monotonic'] = math.nan
    if failure == 'arm_id': state['arm_id'] = 'other'
    if failure == 'calibration': state['calibration_sha256'] = 'other'
    if failure == 'source': state['owner_source_sha256'] = {}
    if failure == 'raw_missing': state['raw'].pop('gripper')
    if failure == 'raw_extra': state['raw']['extra'] = 2000
    if failure == 'raw_bool': state['raw']['gripper'] = True
    if failure == 'raw_range': state['raw']['gripper'] = 4096
    if failure == 'torque_missing': state['torque'].pop('gripper')
    if failure == 'torque_on': state['torque']['gripper'] = 1
    if failure == 'observe': state['observe_only'] = True
    if failure == 'generation': node._owner_started = time.monotonic() + 1
    if failure == 'dead': node._process.poll = lambda: 1
    if failure == 'busy': node._active_goal = object()
    if failure == 'teleop': node._teleop_state = 'active'
    if failure == 'stop': node._stop_latched = True
    if failure == 'fault': node._owner_faulted = True
    if failure == 'cancel': node._cancel_before_start = True
    if failure == 'closed': node._closed = True
    result = maintenance(node)
    assert not result.success, failure
    assert node._operator_maintenance_token is None
    assert not restarts


def test_restart_and_release_revalidate_fresh_torque_off_and_keep_reservation_on_failure(owner):
    node, state, restarts = owner
    state['arm_bounds']['classification'] = 'unrecoverable'
    result = maintenance(node)
    assert result.success, result.message
    token = result.token
    state['torque']['elbow_flex'] = 1
    assert not maintenance(node, operation='restart', token=token).success
    assert not maintenance(node, enable=False, token=token).success
    assert node._active_goal == 'operator_maintenance'
    assert node._operator_maintenance_token == token
    assert not restarts


def test_unresolved_job_or_wrong_token_cannot_release_and_expiry_cleanup_is_preserved(owner):
    node, state, _ = owner
    state['arm_bounds']['classification'] = 'unrecoverable'
    token = maintenance(node).token
    assert not maintenance(node, enable=False, token='wrong').success
    node._operator_external_job = {'job_id': 'pending'}
    assert not maintenance(node, enable=False, token=token).success
    node._operator_external_job = None
    node._operator_maintenance_deadline = time.monotonic() - 1
    node._expire_operator_modes()
    assert node._active_goal is None
    assert node._operator_maintenance_token is None
    assert node._operator_maintenance_deadline is None


def test_relaxed_bounds_option_is_scoped_to_nonactuating_maintenance():
    tree = ast.parse(SOURCE.read_text())
    callers = [method.name for method in ast.walk(tree) if isinstance(method, ast.FunctionDef)
        for call in ast.walk(method) if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute) and call.func.attr == '_prepare_owner_state'
        and any(keyword.arg == 'nonactuating_maintenance' for keyword in call.keywords)]
    assert callers == ['_operator_maintenance'] * 3


@pytest.mark.parametrize('failure', ['stale', 'source', 'stop', 'fault', 'cancel', 'replacement'])
def test_restart_revalidates_owner_after_reservation(owner, failure):
    node, state, restarts = owner
    state['arm_bounds']['classification'] = 'unrecoverable'
    result = maintenance(node)
    assert result.success, result.message
    token = result.token
    if failure == 'stale': state['torque_monotonic'] -= 1
    if failure == 'source': state['owner_source_sha256'] = {}
    if failure == 'stop': node._stop_latched = True
    if failure == 'fault': node._owner_faulted = True
    if failure == 'cancel': node._cancel_before_start = True
    if failure == 'replacement': node._owner_started = time.monotonic() + 1
    assert not maintenance(node, operation='restart', token=token).success
    assert node._active_goal == 'operator_maintenance'
    assert node._operator_maintenance_token == token
    assert not restarts


def test_unresolved_restart_expiry_retains_stop_and_fault_barrier(owner):
    node, state, _ = owner
    state['arm_bounds']['classification'] = 'unrecoverable'
    token = maintenance(node).token
    node._operator_external_job = {'job_id': 'pending'}
    node._operator_maintenance_deadline = time.monotonic() - 1
    faults = []
    node._mark_owner_fault = faults.append
    node._expire_operator_modes()
    assert node._active_goal == 'operator_maintenance_unknown'
    assert node._stop_latched
    assert node._operator_maintenance_token == token
    assert faults == ['external_restart_lease_expired']
