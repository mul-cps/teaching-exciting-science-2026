"""Native follower recovery/status boundary; fake owner only, domain75."""
import ast
import inspect
import json
import math
import subprocess
import sys
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import textwrap

import pytest
pytest.importorskip('rclpy')
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from test_ros_stop_service import bridge, snapshot, stop
from so101_station_control.trajectory_server import TrajectoryBridge
from tools.ros_station_action import parse_args


@pytest.fixture
def ready(bridge):
    node, process = bridge
    node._owner_started = time.monotonic() - .01
    node._owner_source_expected = {'arm': 'a'*64, 'arm_session': 'b'*64,
        'recovery_runtime': 'c'*64, 'recovery_types': 'd'*64}
    node._read_state = lambda **kwargs: state(node)
    node._cache_owner_state(state(node))
    yield node, process


def state(node, **changes):
    result = snapshot()
    result.update(observe_only=False, torque_monotonic=result['monotonic'],
                  owner_source_sha256=dict(node._owner_source_expected))
    result.update(changes)
    return result


def status(node):
    response = node._status(Trigger.Request(), Trigger.Response())
    assert response.success
    return json.loads(response.message)


def install_recovery(node, process, transform=lambda value: value):
    original = process.write
    def write(line):
        command = json.loads(line)
        if command['command'] != 'recover_camera':
            return original(line)
        process.commands.append(command)
        reply = transform({'ok': True, 'state': state(node, event='camera_recovery_complete',
            torque=dict.fromkeys(snapshot()['torque'], 1))})
        node._response_queue.put(reply)
    process.write = write


def recover(node):
    return node._recover_camera(Trigger.Request(), Trigger.Response())


def test_recovery_forwards_only_existing_command_and_never_sets_home(ready):
    node, process = ready
    install_recovery(node, process)
    node._home_completed_owner_started = node._owner_started
    result = recover(node)
    assert result.success
    assert process.commands == [{'command': 'recover_camera'}]
    assert node._home_completed_owner_started is None
    assert not status(node)['home_completed_this_session']
    assert node._active_goal is None


@pytest.mark.parametrize('mode', ['busy', 'teleop', 'faulted', 'stopped', 'dead', 'readonly',
                                  'observe', 'reconnect', 'stale', 'old_generation', 'source',
                                  'identity', 'torque_stale', 'nan', 'closed'])
def test_recovery_rejects_before_any_owner_command(ready, mode):
    node, process = ready
    if mode == 'busy': node._active_goal = 'home'
    if mode == 'teleop': node._teleop_state = 'active'
    if mode == 'faulted': node._owner_faulted = True
    if mode == 'stopped': node._stop_latched = True
    if mode == 'dead': process.alive = False
    if mode == 'readonly': node._external_state_only = True
    if mode == 'reconnect': node._reconnect_observe = True
    if mode == 'closed': node._closed = True
    changes = {'observe': {'observe_only': True}, 'stale': {'monotonic': time.monotonic()-1},
        'old_generation': {'monotonic': node._owner_started-.001},
        'source': {'owner_source_sha256': {'arm': 'c'*64}}, 'identity': {'arm_id': 'other'},
        'torque_stale': {'torque_monotonic': time.monotonic()-1}, 'nan': {'monotonic': math.nan}}
    if mode in changes:
        node._read_state = lambda **kwargs: state(node, **changes[mode])
    assert not recover(node).success
    assert process.commands == []


@pytest.mark.parametrize('mode', ['event', 'identity', 'torque', 'stale', 'torque_stale',
                                  'future', 'cancel', 'stop', 'error', 'malformed', 'generation',
                                  'all_off', 'source', 'observe', 'truthy', 'missing_stamp', 'raw'])
def test_recovery_failure_never_invents_success_or_home_or_retry(ready, mode):
    node, process = ready
    def change(reply):
        if mode == 'event': reply['state']['event'] = 'home_complete'
        if mode == 'identity': reply['state']['arm_id'] = 'other'
        if mode == 'torque': reply['state']['torque']['gripper'] = 0
        if mode == 'all_off': reply['state']['torque'] = dict.fromkeys(snapshot()['torque'], 0)
        if mode == 'source': reply['state']['owner_source_sha256']['arm'] = 'e'*64
        if mode == 'observe': reply['state']['observe_only'] = True
        if mode == 'truthy': reply['ok'] = 1
        if mode == 'missing_stamp': reply['state'].pop('torque_monotonic')
        if mode == 'raw': reply['state']['raw']['gripper'] = 5000
        if mode == 'stale': reply['state']['monotonic'] -= 1
        if mode == 'torque_stale': reply['state']['torque_monotonic'] -= 1
        if mode == 'future': reply['state']['monotonic'] += 1
        if mode == 'cancel': reply['cancelled'] = True
        if mode == 'stop': reply['stopped'] = True
        if mode == 'error': reply.update(ok=False, error='unqualified scene provider')
        if mode == 'malformed': return None
        if mode == 'generation': node._owner_started += .001
        return reply
    install_recovery(node, process, change)
    node._home_completed_owner_started = node._owner_started
    result = recover(node)
    assert not result.success and node._owner_faulted
    assert node._home_completed_owner_started is None
    assert process.commands == [{'command': 'recover_camera'}]
    if mode == 'error': assert 'unqualified scene provider' in result.message


def test_stop_before_reserved_recovery_write_cannot_submit(ready, monkeypatch):
    node, process = ready
    entered, release = threading.Event(), threading.Event()
    original = node._send
    def send(command):
        entered.set()
        assert release.wait(1)
        original(command)
    monkeypatch.setattr(node, '_send', send)
    results = []
    worker = threading.Thread(target=lambda: results.append(recover(node)))
    worker.start()
    assert entered.wait(1)
    node._stop_latched = True
    release.set()
    worker.join(1)
    assert results and not results[0].success
    assert process.commands == []
    assert node._active_goal is None


def test_status_never_reads_files_hashes_or_sends_owner_commands(ready, monkeypatch):
    node, process = ready
    node._home_completed_owner_started = node._owner_started
    monkeypatch.setattr(node, '_read_state', lambda **kwargs: pytest.fail('status read filesystem'))
    monkeypatch.setattr(node, '_send', lambda *args: pytest.fail('status wrote owner'))
    monkeypatch.setattr(Path, 'read_text', lambda *args, **kwargs: pytest.fail('status filesystem'))
    data = status(node)
    assert data['schema'] == 1 and data['kind'] == 'follower_status'
    assert data['connected'] and data['fresh'] and data['observe_only'] is False
    assert data['home_completed_this_session']
    assert data['owner_source_sha256'] == node._owner_source_expected
    assert data['position_age_s'] >= 0 and data['torque_age_s'] >= 0
    assert process.commands == []


@pytest.mark.parametrize('mode', ['missing', 'disconnected', 'none_generation', 'old_state', 'source',
                                  'identity', 'nan', 'infinity', 'missing_torque', 'future'])
def test_status_missing_or_old_fields_are_conservative_valid_json(ready, mode):
    node, process = ready
    saved = state(node)
    if mode == 'missing': node._cached_owner_state = None
    if mode == 'disconnected': process.alive = False
    if mode == 'none_generation':
        node._owner_started = None
        node._home_completed_owner_started = None
    if mode == 'old_state': saved['monotonic'] = node._owner_started-.001
    if mode == 'source': saved['owner_source_sha256'] = {'arm': 'c'*64, 'arm_session': 'b'*64}
    if mode == 'identity': saved['arm_id'] = 'other'
    if mode == 'nan': saved['monotonic'] = math.nan
    if mode == 'infinity': saved['torque_monotonic'] = math.inf
    if mode == 'missing_torque': saved.pop('torque_monotonic')
    if mode == 'future': saved['monotonic'] += 1
    if mode not in {'missing', 'disconnected', 'none_generation'}:
        node._cache_owner_state(saved)
    data = status(node)
    assert not data['fresh'] and not data['home_completed_this_session']
    assert data['reasons']
    assert process.commands == []


def test_cache_is_pruned_owned_and_rejects_late_reader_generation(ready):
    node, process = ready
    saved = state(node)
    saved['unbounded_extra'] = list(range(1000))
    node._cache_owner_state(saved)
    saved['raw']['gripper'] = 9999
    assert status(node)['raw']['gripper'] != 9999
    assert 'unbounded_extra' not in node._cached_owner_state
    old_process, old_generation = process, node._owner_started
    node._process = SimpleNamespace(poll=lambda: None)
    node._owner_started += .001
    before = node._cached_owner_state
    node._cache_owner_state(state(node), generation=old_generation, process=old_process)
    assert node._cached_owner_state is before
    assert not status(node)['fresh']
    node._process = process


def test_status_reports_reconnect_stop_home_provenance(ready):
    node, process = ready
    node._owner_faulted = True
    node._reconnect_observe = True
    node._cache_owner_state(state(node, observe_only=True))
    data = status(node)
    assert data['connected'] and data['observe_only'] and data['owner_faulted']
    assert not data['home_completed_this_session']
    node._read_state = lambda **kwargs: state(node, observe_only=True)
    process.transform = lambda reply: dict(reply, state=state(node))
    assert stop(node).success
    node._read_state = lambda **kwargs: state(node)
    data = status(node)
    assert not data['observe_only'] and not data['owner_faulted'] and not data['stop_latched']
    assert not data['home_completed_this_session']
    # The existing home callback is the only producer of this provenance.
    original = process.write
    def write(line):
        command = json.loads(line)
        if command['command'] != 'home': return original(line)
        process.commands.append(command)
        node._response_queue.put({'ok': True, 'state': state(node, event='home_complete')})
    process.write = write
    assert node._home(Trigger.Request(), Trigger.Response()).success
    assert status(node)['home_completed_this_session']
    assert [command['command'] for command in process.commands] == ['stop', 'home']


def test_native_status_is_available_while_recovery_reserved(ready):
    node, process = ready
    node._active_goal = 'recover_camera'
    client_node = Node('follower_status_native_client')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        client = client_node.create_client(Trigger, '/follower/status')
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        until = time.monotonic()+2
        while not future.done() and time.monotonic() < until: time.sleep(.005)
        assert future.done() and future.result().success
        assert json.loads(future.result().message)['active_operation'] == 'recover_camera'
        assert process.commands == []
        assert node._recover_camera_service.callback_group is node._teleop_service_group
        assert node._status_service.callback_group is not node._teleop_service_group
    finally:
        executor.shutdown()
        thread.join(1)
        executor.remove_node(node)
        client_node.destroy_node()


def test_status_static_no_io_or_hashing():
    tree = ast.parse(textwrap.dedent(inspect.getsource(TrajectoryBridge._status)))
    calls = {item.func.attr for item in ast.walk(tree)
             if isinstance(item, ast.Call) and isinstance(item.func, ast.Attribute)}
    assert not calls.intersection({'_read_state', '_send', '_write_owner_command', 'read_text',
                                   'read_bytes', 'sha256', 'open', 'stat'})


def test_quick_recovery_and_status_accept_no_motion_arguments():
    assert parse_args(['arm-recover']).command == 'arm-recover'
    assert parse_args(['arm-status']).command == 'arm-status'
    with pytest.raises(SystemExit): parse_args(['arm-recover', '--target', 'home'])


@pytest.mark.parametrize('provider_mode', ['missing', 'unqualified'])
def test_real_owner_recovery_rejection_propagates_without_goal_writes(ready, tmp_path, provider_mode):
    from test_arm_session import FakeArm, target
    from workshop.station.arm_session import ArmSession, CameraRecoveryPolicy
    from test_arm_camera_recovery import Scene
    node, process = ready
    arm = FakeArm(tmp_path, initial=target(elbow_flex=2993))
    arm.settings['limits']['elbow_flex']['max'] = (2975.1-2027.5)*360/4095
    def acquire(raw, stamp):
        scene = Scene(raw, stamp)
        scene.registration_qualified = False
        return scene
    policy = CameraRecoveryPolicy(model_sha256='a'*64, calibration_sha256='b'*64,
        camera_serial='qualified-test-camera', arm_calibration_sha256=arm.settings['calibration']['sha256'])
    session = ArmSession(arm, tmp_path/'owner-state.json', tmp_path/'owner-logs',
        camera_scene_provider=acquire if provider_mode == 'unqualified' else None,
        camera_recovery_policy=policy if provider_mode == 'unqualified' else None)
    try:
        observed = session.initialize()
        node._expected_arm_id = observed['arm_id']
        node._expected_calibration_sha256 = observed['calibration_sha256']
        node._owner_source_expected = observed['owner_source_sha256']
        node._read_state = lambda **kwargs: session.publish_state(event='sample')
        before = len(arm.bus.goal_history)
        def send(command):
            process.commands.append(command)
            try:
                reply = session.handle(command)
            except (RuntimeError, ValueError) as error:
                reply = {'ok': False, 'error': str(error)}
            node._response_queue.put(reply)
        node._send = send
        result = recover(node)
        assert not result.success
        assert process.commands == [{'command': 'recover_camera'}]
        assert len(arm.bus.goal_history) == before == 0
        assert not status(node)['home_completed_this_session']
        assert 'provider' in result.message or 'qualified' in result.message
    finally:
        session.shutdown()
        session.close_log()


def test_old_still_fresh_file_cannot_supply_new_owner_status(ready):
    node, process = ready
    old = state(node, monotonic=node._owner_started-.001)
    node._state_file.write_text(json.dumps(old))
    # Exercise the production file reader, not the fixture's synthetic provider.
    TrajectoryBridge._read_state(node)
    data = status(node)
    assert not data['fresh'] and data['raw'] is None
    assert 'position_sample_precedes_owner_or_invalid' in data['reasons']
    assert process.commands == []


def test_quick_status_and_recovery_native_routes(ready):
    node, process = ready
    install_recovery(node, process)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    root = Path(__file__).resolve().parents[1]
    try:
        for action in ('arm-status', 'arm-recover'):
            result = subprocess.run([sys.executable, str(root/'tools/ros_station_action.py'), action],
                                    cwd=root, capture_output=True, text=True, timeout=12)
            assert result.returncode == 0, result.stderr + result.stdout
            payload = json.loads(result.stdout)
            assert payload['success']
            if action == 'arm-status':
                assert payload['status']['schema'] == 1
                assert payload['status']['kind'] == 'follower_status'
                assert process.commands == []
        assert process.commands == [{'command': 'recover_camera'}]
    finally:
        executor.shutdown()
        thread.join(1)
        executor.remove_node(node)


def test_failed_telemetry_read_invalidates_cached_freshness(ready):
    node, process = ready
    assert status(node)['fresh']
    assert TrajectoryBridge._read_state(node) is None
    data = status(node)
    assert not data['fresh'] and data['raw'] is None
    assert 'state_read_failed' in data['reasons']
    assert process.commands == []


def test_cache_and_control_hot_paths_static_guard():
    tree = ast.parse(textwrap.dedent(inspect.getsource(TrajectoryBridge._cache_owner_state)))
    calls = {item.func.attr for item in ast.walk(tree)
             if isinstance(item, ast.Call) and isinstance(item.func, ast.Attribute)}
    assert not calls.intersection({'_send', 'read_text', 'read_bytes', 'sha256', 'open', 'stat'})
    control = inspect.getsource(TrajectoryBridge._teleop_tick)
    assert '_cache_owner_state' not in control
    reader = inspect.getsource(TrajectoryBridge._read_responses)
    assert 'response.get("command") != "teleop_step"' in reader


def test_bridge_owner_source_map_uses_shared_four_module_contract():
    from so101_station_control.trajectory_server import _load_owner_source_hashes
    from workshop.station.recovery_types import OWNER_SOURCE_MODULES, owner_source_sha256
    root = Path(__file__).resolve().parents[1]
    expected = owner_source_sha256(root/'workshop/station')
    assert set(expected) == set(OWNER_SOURCE_MODULES)
    assert set(expected) == {'arm', 'arm_session', 'recovery_runtime', 'recovery_types'}
    assert _load_owner_source_hashes(root) == expected


@pytest.mark.parametrize('module', ['arm', 'arm_session', 'recovery_runtime', 'recovery_types'])
def test_status_and_recovery_reject_each_source_pin_mismatch(ready, module):
    node, process = ready
    observed = state(node)
    observed['owner_source_sha256'][module] = 'e'*64
    node._cache_owner_state(observed)
    data = status(node)
    assert not data['fresh'] and data['raw'] is None
    assert 'owner_source_hashes_unconfirmed' in data['reasons']
    node._read_state = lambda **kwargs: observed
    assert not recover(node).success
    assert process.commands == []
