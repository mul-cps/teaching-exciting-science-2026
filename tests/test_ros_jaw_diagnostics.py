"""Native zero-argument diagnostic service, fake owner only (domain 75)."""
import copy
import json
import time
import pytest
pytest.importorskip('rclpy')
from std_srvs.srv import Trigger
from test_ros_follower_recovery_status import ready, state
from test_ros_stop_service import bridge


def invoke(node):
    return node._gripper_diagnostics(Trigger.Request(), Trigger.Response())


def install(node, process, transform=lambda value: value):
    def write(line):
        command = json.loads(line)
        process.commands.append(command)
        assert command == {'command': 'jaw_diagnostics'}
        began = time.monotonic()
        before = state(node)
        names = ('CW_Dead_Zone', 'CCW_Dead_Zone', 'P_Coefficient', 'D_Coefficient',
                 'I_Coefficient', 'Minimum_Startup_Force', 'Max_Torque_Limit',
                 'Torque_Limit', 'Present_Load', 'Present_Current', 'Status', 'Moving')
        metadata = ((26, 1), (27, 1), (21, 1), (22, 1), (23, 1), (24, 2),
                    (16, 2), (48, 2), (60, 2), (69, 2), (65, 1), (66, 1))
        rows = [dict(register=name, motor_id=6, value=0, units='raw register value',
                     address=address, width_bytes=width, started_monotonic=began,
                     ended_monotonic=began) for name, (address, width) in zip(names, metadata)]
        result = dict(schema='jaw-register-diagnostics-v1', diagnostic_only=True,
            calibration_authority=False, motion_authority=False, available=True,
            arm_id=node._expected_arm_id, calibration_sha256=node._expected_calibration_sha256,
            owner_source_sha256=dict(node._owner_source_expected), motor_id=6,
            before=before, after=copy.deepcopy(before), reads=[], registers=rows,
            started_monotonic=began, ended_monotonic=time.monotonic())
        node._response_queue.put(transform({'ok': True, 'diagnostic': result}))
    process.write = write


def test_native_service_fixed_command_and_same_owner_generation(ready):
    node, process = ready
    install(node, process)
    reply = invoke(node)
    assert reply.success, reply.message
    payload = json.loads(reply.message)
    assert payload['owner_started_monotonic'] == node._owner_started
    assert payload['diagnostic_only'] and not payload['motion_authority']
    assert process.commands == [{'command': 'jaw_diagnostics'}]
    assert node._active_goal is None
    assert node._gripper_diagnostics_service.callback_group is not node._stop_group


@pytest.mark.parametrize('case', ['stale', 'identity', 'source', 'torque', 'busy', 'teleop', 'stop'])
def test_rejects_before_delivery(ready, case):
    node, process = ready
    value = state(node)
    if case == 'stale': value['monotonic'] -= 2
    if case == 'identity': value['arm_id'] = 'other'
    if case == 'source': value['owner_source_sha256'] = {}
    if case == 'torque': value['torque']['gripper'] = 1
    if case == 'busy': node._active_goal = 'trajectory'
    if case == 'teleop': node._teleop_state = 'active'
    if case == 'stop': node._stop_latched = True
    node._read_state = lambda **kwargs: value
    response = invoke(node)
    assert not response.success
    assert not process.commands


@pytest.mark.parametrize('case', ['drift', 'torque', 'stale', 'identity', 'source', 'generation', 'replaced', 'stop', 'read_error', 'bad_reply', 'inventory', 'moving', 'clock', 'extra_nan', 'extra_object', 'address', 'width', 'negative_unsigned', 'oversized_unsigned', 'oversized_load', 'negative_load', 'width_bool', 'address_float'])
def test_unavailable_or_changed_cohort_never_dispatches_cleanup(ready, case):
    node, process = ready
    def transform(reply):
        result = reply['diagnostic']
        if case == 'drift': result['after']['raw']['shoulder_pan'] += 1
        if case == 'torque': result['after']['torque']['gripper'] = 1
        if case == 'stale': result['after']['monotonic'] -= 2
        if case == 'identity': result['arm_id'] = 'other'
        if case == 'source': result['owner_source_sha256'] = {}
        if case == 'generation': node._owner_started += .001
        if case == 'replaced': node._process = None
        if case == 'stop': node._stop_latched = True
        if case == 'width_bool': result['registers'][0]['width_bytes'] = True
        if case == 'address_float': result['registers'][0]['address'] = 26.0
        if case == 'address': result['registers'][0]['address'] = 25
        if case == 'width': result['registers'][0]['width_bytes'] = 2
        if case == 'negative_unsigned': result['registers'][0]['value'] = -1
        if case == 'oversized_unsigned': result['registers'][0]['value'] = 256
        if case == 'oversized_load': result['registers'][8]['value'] = 1024
        if case == 'negative_load': result['registers'][8]['value'] = -1024
        if case == 'extra_nan': result['extra'] = float('nan')
        if case == 'extra_object': result['extra'] = object()
        if case == 'inventory': result['registers'][0]['motor_id'] = 1
        if case == 'moving': result['registers'][-1]['value'] = 1
        if case == 'clock': result['registers'][0]['ended_monotonic'] = float('nan')
        if case == 'read_error': reply['ok'] = result['available'] = False
        if case == 'bad_reply': return []
        return reply
    install(node, process, transform)
    response = invoke(node)
    assert not response.success
    assert process.commands == [{'command': 'jaw_diagnostics'}]
    assert node._active_goal is None


def test_actual_stopped_owner_bridge_is_read_only(ready, tmp_path):
    from test_jaw_diagnostics import passive
    node, process = ready
    session, arm, clock, calls = passive(tmp_path)
    session.handle({'command': 'stop'})
    arm.bus.writes.clear(); calls.clear()
    node._expected_arm_id = arm.settings['id']
    node._expected_calibration_sha256 = session.calibration_digest
    node._owner_source_expected = dict(session._owner_source_sha256)
    def fresh(**kwargs):
        clock.value = time.monotonic()
        return session.read_state()
    node._read_state = fresh
    def write(line):
        command = json.loads(line)
        process.commands.append(command)
        clock.value = time.monotonic()
        node._response_queue.put(session.handle(command))
    process.write = write
    response = invoke(node)
    assert response.success, response.message
    result = json.loads(response.message)
    assert result['registers'][7]['value'] == 500
    assert session._shutdown_complete
    assert not arm.bus.writes and not arm.bus.goal_history


def test_quick_alias_calls_native_zeroarg_service(ready):
    import subprocess
    import sys
    import threading
    from pathlib import Path
    from rclpy.executors import MultiThreadedExecutor
    node, process = ready
    install(node, process)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    root = Path(__file__).resolve().parents[1]
    try:
        response = subprocess.run([sys.executable, str(root/'tools/ros_station_action.py'),
            'gripper-diagnostics'], cwd=root, capture_output=True, text=True, timeout=12)
        assert response.returncode == 0, response.stderr + response.stdout
        result = json.loads(response.stdout)
        assert result['success']
        assert json.loads(result['message'])['diagnostic_only']
        assert process.commands == [{'command': 'jaw_diagnostics'}]
    finally:
        executor.shutdown()
        thread.join(1)
        executor.remove_node(node)


def test_explicit_stop_interrupts_reserved_read_and_preserves_barrier(ready):
    import threading
    from test_ros_stop_service import stop
    node, process = ready
    delivered = threading.Event()
    original = process.write
    def write(line):
        command = json.loads(line)
        if command['command'] == 'jaw_diagnostics':
            process.commands.append(command)
            delivered.set()
        else:
            original(line)
    process.write = write
    replies = []
    worker = threading.Thread(target=lambda: replies.append(invoke(node)))
    worker.start()
    assert delivered.wait(1.)
    result = stop(node)
    worker.join(1.)
    assert not worker.is_alive()
    assert result.success, result.message
    assert replies and not replies[0].success
    assert [row['command'] for row in process.commands] == ['jaw_diagnostics', 'stop']
    assert node._active_goal is None and not node._stop_latched


def test_read_timeout_poisoned_admission_has_no_invented_cleanup(ready):
    node, process = ready
    process.write = lambda line: process.commands.append(json.loads(line))
    began = time.monotonic()
    response = invoke(node)
    assert not response.success and time.monotonic()-began < 1.5
    assert node._owner_faulted
    assert process.commands == [{'command': 'jaw_diagnostics'}]


@pytest.mark.parametrize('load', [-1023, -1, 1023])
def test_signed_driver_load_boundaries_are_raw_observations(ready, load):
    node, process = ready
    def transform(reply):
        reply['diagnostic']['registers'][8]['value'] = load
        return reply
    install(node, process, transform)
    response = invoke(node)
    assert response.success, response.message
    assert json.loads(response.message)['registers'][8]['value'] == load
