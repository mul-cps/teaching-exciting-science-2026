"""Source-bound offline bridge lifecycle proof, without ROS or generated interfaces."""
import ast
import copy
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from test_ros_maintenance_bounds import owner, JOINTS, SOURCE, maintenance


def lifecycle_class(base):
    tree = ast.parse(SOURCE.read_text())
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TrajectoryBridge')
    names = {'_reconnect', '_reset_owner', '_send', '_status', '_cache_owner_state'}
    methods = [n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names]
    helpers = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in
               {'_observe_owner_error', '_fault_observer_error', '_arm_bounds_observation'}]
    module = ast.Module(body=helpers + [ast.ClassDef(name='Lifecycle', bases=[ast.Name(id='Base', ctx=ast.Load())],
        keywords=[], body=methods, decorator_list=[])], type_ignores=[])
    namespace = dict(Base=base, math=math, time=time, json=json, queue=queue, subprocess=subprocess,
                     TELEOP_JOINTS=JOINTS, _teleop_leader_readiness=lambda *args: (False, 'offline', None))
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), 'exec'), namespace)
    return namespace['Lifecycle']


@pytest.fixture
def lifecycle(owner):
    node, state, _ = owner
    node.__class__ = lifecycle_class(type(node))
    node._stop_lock = threading.Lock()
    node._stop_condition = threading.Condition()
    node._write_lock = threading.Lock()
    node._control_callbacks = 0
    node._owner_readers = []
    node._response_queue = queue.Queue()
    node._home_completed_owner_started = node._owner_started
    node._recovery_depth_cache = node._recovery_camera_identity = None
    node._recovery_cache_error = None
    node._cached_owner_reason = None
    node._teleop_latest = None
    node._teleop_latest_lock = threading.Lock()
    node._prepare_phase = None
    node._operator_restart_result = None
    node._operator_restart_units = set()
    node._writer_faulted = False
    state['owner_session_id'] = 'arm-session-' + '1' * 32
    state['fault_observation'] = dict(schema=1, owner_session_id=state['owner_session_id'],
        command='home', error='deadline', error_type='TimeoutError',
        entered_monotonic=node._owner_started + .001)
    state['pose'] = dict.fromkeys(JOINTS, 0.)
    process = SimpleNamespace(alive=True, commands=[], wait_timeouts=[])
    process.poll = lambda: None if process.alive else 0
    def wait(**kwargs):
        process.wait_timeouts.append(kwargs['timeout'])
        if process.alive: raise subprocess.TimeoutExpired('owner', kwargs['timeout'])
        return 0
    process.wait = wait
    node._process = process
    def write(command, deadline):
        process.commands.append(command)
        if command['command'] == 'exit': process.alive = False
    node._write_owner_command = write
    node._send = type(node)._send.__get__(node)
    node._mark_owner_fault = lambda reason: setattr(node, '_owner_faulted', True)
    spawns = []
    def start(**kwargs):
        assert not process.alive
        spawns.append(kwargs)
        node._owner_started = time.monotonic()
        process.alive = True
        result = copy.deepcopy(state)
        result.update(observe_only=True, fault_observation=None)
        return result
    node._start_owner = start
    node._configure_recovery_depth_cache = lambda: None
    node._teleop_start_error = lambda value, *args: 'fault' if value.get('fault_observation') is not None else None
    stamp = time.monotonic()
    state.update(monotonic=stamp, torque_monotonic=stamp)
    state["arm_bounds"]["position_monotonic"] = stamp
    return node, state, process, spawns


def invoke(node, reset=False):
    return (node._reset_owner if reset else node._reconnect)(None, SimpleNamespace(success=False, message=''))


def test_explicit_fault_reconnect_exits_once_and_spawns_passive_generation(lifecycle):
    node, state, process, spawns = lifecycle
    result = invoke(node)
    assert result.success, result.message
    assert process.commands == [{'command': 'exit'}]
    assert spawns == [{'observe_only': True}]
    assert process.wait_timeouts == [8.]
    assert node._owner_faulted and node._reconnect_observe
    assert node._home_completed_owner_started is None


@pytest.mark.parametrize('reset', [False, True])
@pytest.mark.parametrize('fault', ['source', 'identity', 'calibration', 'stale', 'torque_stale',
    'on', 'incomplete', 'fault_identity', 'old_fault', 'future_fault', 'busy', 'callbacks', 'teleop'])
def test_reconnect_and_reset_reject_before_exit_without_complete_fresh_idle_alloff_proof(lifecycle, reset, fault):
    node, state, process, spawns = lifecycle
    if fault == 'source': state['owner_source_sha256'] = {'arm_session': 'c'*64}
    if fault == 'identity': state['arm_id'] = 'other'
    if fault == 'calibration': state['calibration_sha256'] = 'c'*64
    if fault == 'stale': state['monotonic'] -= 1
    if fault == 'torque_stale': state['torque_monotonic'] -= 1
    if fault == 'on': state['torque']['elbow_flex'] = 1
    if fault == 'incomplete': state['raw'].pop('gripper')
    if fault == 'fault_identity': state['fault_observation']['owner_session_id'] = 'other'
    if fault == 'old_fault': state['fault_observation']['entered_monotonic'] = node._owner_started - .01
    if fault == 'future_fault': state['fault_observation']['entered_monotonic'] = time.monotonic() + 1
    if fault == 'busy': node._active_goal = 'home'
    if fault == 'callbacks': node._control_callbacks = 1
    if fault == 'teleop': node._teleop_state = 'active'
    assert not invoke(node, reset).success
    assert process.alive and not process.commands and not spawns


def test_healthy_alive_owner_requires_separate_explicit_reset_endpoint(lifecycle):
    node, state, process, spawns = lifecycle
    state['fault_observation'] = None
    generation = node._owner_started
    assert not invoke(node).success
    assert not process.commands and not spawns and not node._owner_faulted
    assert node._home_completed_owner_started == generation
    assert invoke(node, reset=True).success
    assert process.commands == [{'command': 'exit'}]
    assert spawns == [{'observe_only': True}]
    assert node._home_completed_owner_started is None and node._owner_faulted


def test_live_exit_timeout_never_kills_or_spawns_replacement(lifecycle):
    node, state, process, spawns = lifecycle
    node._write_owner_command = lambda command, deadline: process.commands.append(command)
    assert not invoke(node).success
    assert process.alive and process.commands == [{'command': 'exit'}] and not spawns
    assert node._owner_faulted


def test_status_and_admission_retain_owner_fault_even_if_legacy_bit_cleared(lifecycle):
    node, state, process, _ = lifecycle
    node._cache_owner_state(state)
    node._owner_faulted = False
    node._home_completed_owner_started = node._owner_started
    result = node._status(None, SimpleNamespace(success=False, message=''))
    payload = json.loads(result.message)
    assert payload['connected'] and payload['fresh'] and payload['owner_faulted']
    assert payload['fault_observation'] == state['fault_observation']
    assert payload['recovery_action'] == 'explicit_reconnect_then_stop'
    assert not payload['home_completed_this_session'] and not payload['teleop_start_ready']
    with pytest.raises(RuntimeError, match='fresh matching'):
        node._prepare_owner_state(state, process, node._owner_started, torque=0)
    with pytest.raises(RuntimeError, match='explicit reconnect'):
        node._send({'command': 'home'})
    assert not process.commands


def test_station_reset_reservation_has_its_own_purpose_and_120_second_lease(owner):
    node, state, _ = owner
    before = time.monotonic()
    result = maintenance(node, operation='station_reset')
    assert result.success, result.message
    assert node._operator_maintenance_purpose == 'station_reset'
    assert before + 120 <= node._operator_maintenance_deadline <= time.monotonic() + 120
    assert maintenance(node, enable=False, token=result.token).success
    assert node._operator_maintenance_purpose is None
    normal = maintenance(node)
    assert normal.success
    assert node._operator_maintenance_purpose == 'maintenance'
    assert 59 < node._operator_maintenance_deadline - time.monotonic() <= 60


def test_station_reset_expiry_preserves_failed_acceptance_barrier(owner):
    node, state, _ = owner
    assert maintenance(node, operation='station_reset').success
    node._operator_maintenance_deadline = time.monotonic() - 1
    reasons = []
    node._mark_owner_fault = lambda reason: reasons.append(reason) or setattr(node, '_owner_faulted', True)
    node._expire_operator_modes()
    assert node._active_goal == 'operator_maintenance_unknown'
    assert node._stop_latched and node._owner_faulted
    assert reasons == ['station_reset_acceptance_expired']
    assert node._operator_maintenance_token


def test_normal_cached_sample_has_no_fault_metadata(lifecycle):
    node, state, _, _ = lifecycle
    state.pop('fault_observation')
    node._cache_owner_state(state)
    assert node._cached_owner_state['fault_observation'] is None
    assert not node._owner_faulted
