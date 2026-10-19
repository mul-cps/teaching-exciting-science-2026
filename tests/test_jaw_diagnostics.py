"""Fixed passive jaw inventory; fake transport only, never hardware."""
import pytest
from test_arm_session import make_session
from workshop.station.arm_session import ArmSessionCancelled, JOINTS
from tools.ros_station_action import parse_args

REGISTERS = ('CW_Dead_Zone', 'CCW_Dead_Zone', 'P_Coefficient', 'D_Coefficient',
             'I_Coefficient', 'Minimum_Startup_Force', 'Max_Torque_Limit',
             'Torque_Limit', 'Present_Load', 'Present_Current', 'Status', 'Moving')


def passive(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    calls = []
    read = arm.bus.read
    sync = arm.bus.sync_read
    for register in REGISTERS:
        arm.bus.data.setdefault(register, dict.fromkeys(JOINTS, 0))
    arm.bus.data['Torque_Limit']['gripper'] = 500
    def single(register, joint, **kwargs):
        calls.append((register, joint, kwargs))
        return read(register, joint, **kwargs)
    def multiple(register, **kwargs):
        calls.append((register, tuple(JOINTS), kwargs))
        return sync(register, **kwargs)
    arm.bus.read, arm.bus.sync_read = single, multiple
    return session, arm, clock, calls


def test_fixed_inventory_raw_clocks_alloff_and_zero_writes(tmp_path):
    session, arm, clock, calls = passive(tmp_path)
    reply = session.handle({'command': 'jaw_diagnostics'})
    assert reply['ok'] and reply['diagnostic']['available']
    result = reply['diagnostic']
    assert result['diagnostic_only'] is True
    assert result['calibration_authority'] is result['motion_authority'] is False
    assert [row['register'] for row in result['registers']] == list(REGISTERS)
    assert result['registers'][7]['value'] == 500
    assert all(row['motor_id'] == 6 and row['units'] == 'raw register value' for row in result['registers'])
    assert all(row['started_monotonic'] <= row['ended_monotonic'] for row in result['reads'])
    assert result['before']['raw'] == result['after']['raw']
    assert result['before']['torque'] == result['after']['torque'] == dict.fromkeys(JOINTS, 0)
    assert [entry[0] for entry in calls] == ['Present_Position', 'Torque_Enable', *REGISTERS, 'Present_Position', 'Torque_Enable']
    assert all(entry[1] == 'gripper' for entry in calls[2:-2])
    assert all(entry[2] == {'normalize': False, 'num_retry': 0} for entry in calls)
    assert not arm.bus.writes and not arm.bus.goal_history and arm.connect_calls == 1


@pytest.mark.parametrize('extra', [{'joint': 'shoulder_pan'}, {'register': 'ID'}, {'motor_id': 1}])
def test_arbitrary_arguments_rejected_without_io(tmp_path, extra):
    session, arm, clock, calls = passive(tmp_path)
    with pytest.raises(ValueError): session.handle({'command': 'jaw_diagnostics', **extra})
    assert not calls and not arm.bus.writes


@pytest.mark.parametrize('case', ['torque', 'moving', 'drift', 'bad_raw', 'bad_register', 'read_error', 'slow', 'identity', 'source', 'calibration', 'teleop', 'uninitialized'])
def test_unavailable_snapshot_never_writes_or_claims_authority(tmp_path, case):
    session, arm, clock, calls = passive(tmp_path)
    if case == 'torque': arm.bus.data['Torque_Enable']['elbow_flex'] = 1
    if case == 'moving': arm.bus.data['Moving']['gripper'] = 1
    if case == 'bad_raw': arm.bus.data['Present_Position']['gripper'] = True
    if case == 'bad_register': arm.bus.data['CW_Dead_Zone']['gripper'] = True
    if case == 'identity': arm.bus.motors['gripper'].id = 5
    if case == 'source': session._owner_source_sha256 = {'wrong': 'hash'}
    if case == 'calibration':
        from pathlib import Path
        Path(arm.settings['calibration']['path']).write_text('{}')
    if case == 'teleop': session._teleop = object()
    if case == 'uninitialized': session._initialized = False
    read = arm.bus.read
    def altered(register, joint, **kwargs):
        if case == 'read_error': raise ConnectionError('diagnostic packet failure')
        if case == 'slow': clock.value += .2
        value = read(register, joint, **kwargs)
        if case == 'drift': arm.bus.data['Present_Position']['shoulder_pan'] += 1
        return value
    arm.bus.read = altered
    reply = session.handle({'command': 'jaw_diagnostics'})
    assert not reply['ok'] and not reply['diagnostic']['available']
    assert reply['diagnostic']['motion_authority'] is False
    assert not arm.bus.writes and not arm.bus.goal_history and arm.connect_calls == 1


def test_stop_polled_between_register_reads(tmp_path):
    session, arm, clock, calls = passive(tmp_path)
    def poll():
        if any(row[0] == 'CW_Dead_Zone' for row in calls):
            session._cancel('busy_stop')
        session._check_cancel()
    session._poll_busy_input = poll
    with pytest.raises(ArmSessionCancelled): session.handle({'command': 'jaw_diagnostics'})
    assert [row[0] for row in calls] == ['Present_Position', 'Torque_Enable', 'CW_Dead_Zone']
    assert not arm.bus.writes


def test_caller_has_zero_argument_alias():
    assert parse_args(['gripper-diagnostics']).command == 'gripper-diagnostics'
    with pytest.raises(SystemExit): parse_args(['gripper-diagnostics', 'ID'])


@pytest.mark.parametrize('case', ['register_nan', 'register_object', 'raw_nan', 'raw_object', 'torque_nan'])
def test_real_serve_unavailable_serializes_without_cleanup_or_owner_exit(tmp_path, case):
    import json
    import os
    session, arm, clock, calls = passive(tmp_path)
    if case.startswith('register'):
        arm.bus.data['CW_Dead_Zone']['gripper'] = float('nan') if case.endswith('nan') else object()
    else:
        register = 'Present_Position' if case.startswith('raw') else 'Torque_Enable'
        original_sync = arm.bus.sync_read
        def malformed_read(name, **kwargs):
            value = original_sync(name, **kwargs)
            if session._motion_busy and name == register:
                value['gripper'] = float('nan') if case.endswith('nan') else object()
            return value
        arm.bus.sync_read = malformed_read
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b'{"command":"jaw_diagnostics"}\n')
    # serve's broad exception handler can catch emitter failure, so use a worker
    # thread and leave its input open, observing the live owner after response.
    import threading
    done = threading.Event()
    replies = []
    def output(value):
        replies.append(json.loads(json.dumps(value, allow_nan=False)))
        done.set()
    worker = threading.Thread(target=lambda: session.serve(fd=read_fd, emit=output), daemon=True)
    worker.start()
    assert done.wait(1.)
    assert worker.is_alive()
    assert not replies[0]['ok'] and not replies[0]['diagnostic']['available']
    assert not arm.bus.writes and not arm.bus.goal_history
    # Explicit exit terminates the test owner after write-free evidence above.
    # Restore malformed data so the separately authorized cleanup can read it.
    for register in ('CW_Dead_Zone', 'Present_Position', 'Torque_Enable'):
        arm.bus.data[register]['gripper'] = 2000 if register == 'Present_Position' else 0
    os.write(write_fd, b'{"command":"exit"}\n')
    worker.join(1.)
    os.close(read_fd); os.close(write_fd)
    assert not worker.is_alive()


def test_verified_stopped_owner_diagnostics_preserve_shutdown_and_zero_writes(tmp_path):
    session, arm, clock, calls = passive(tmp_path)
    session.handle({'command': 'stop'})
    assert session._shutdown_complete
    arm.bus.writes.clear(); calls.clear()
    reply = session.handle({'command': 'jaw_diagnostics'})
    assert reply['ok'] and session._shutdown_complete
    assert not arm.bus.writes and not arm.bus.goal_history


def test_passive_body_outside_normal_bounds_remains_observation(tmp_path):
    session, arm, clock, calls = passive(tmp_path)
    arm.bus.data['Present_Position']['shoulder_pan'] = 2993
    reply = session.handle({'command': 'jaw_diagnostics'})
    assert reply['ok']
    assert reply['diagnostic']['after']['arm_bounds']['classification'] == 'unrecoverable'
    assert not reply['diagnostic']['motion_authority'] and not arm.bus.writes


def test_real_serve_stop_interrupts_inventory_with_original_correlation(tmp_path):
    import json
    import os
    session, arm, clock, calls = passive(tmp_path)
    read_fd, write_fd = os.pipe()
    request_id = 'a' * 32
    original = arm.bus.read
    def read(register, joint, **kwargs):
        value = original(register, joint, **kwargs)
        if register == 'CW_Dead_Zone':
            os.write(write_fd, (json.dumps({'command': 'stop', 'request_id': request_id})+'\n').encode())
        return value
    arm.bus.read = read
    results = []
    def emit(value):
        results.append(json.loads(json.dumps(value, allow_nan=False)))
        if value.get('stopped'):
            os.write(write_fd, b'{"command":"exit"}\n')
    os.write(write_fd, b'{"command":"jaw_diagnostics"}\n')
    try:
        assert session.serve(fd=read_fd, emit=emit) == 0
    finally:
        os.close(read_fd); os.close(write_fd)
    stopped = [row for row in results if row.get('stopped')]
    assert stopped[0]['request_id'] == request_id
    assert stopped[0]['reason'] == 'busy_stop'
    assert not any(row[0] == 'CCW_Dead_Zone' for row in calls)
    # These writes derive from the independently received stop, never diagnostic
    # error cleanup. No profile, goal, torque-enable or EEPROM write is admitted.
    assert arm.bus.writes and all(row[0] == 'Torque_Enable' and row[2] == 0 for row in arm.bus.writes)
    assert not arm.bus.goal_history


@pytest.mark.parametrize('failure', ['late_publication', 'source_replaced', 'identity_replaced', 'calibration_replaced'])
def test_late_or_replaced_cohort_is_unavailable_without_writes(tmp_path, monkeypatch, failure):
    session, arm, clock, calls = passive(tmp_path)
    if failure == 'late_publication':
        publish = session.publish_state
        def slow(**kwargs):
            result = publish(**kwargs)
            clock.value += .2
            return result
        monkeypatch.setattr(session, 'publish_state', slow)
    else:
        read = arm.bus.read
        def changed(register, joint, **kwargs):
            value = read(register, joint, **kwargs)
            if failure == 'identity_replaced': arm.settings['id'] = 'different'
            if failure == 'source_replaced': session._owner_source_sha256 = {'changed': 'hash'}
            if failure == 'calibration_replaced':
                from pathlib import Path
                Path(arm.settings['calibration']['path']).write_text('{}')
            return value
        arm.bus.read = changed
    reply = session.handle({'command': 'jaw_diagnostics'})
    assert not reply['ok'] and not reply['diagnostic']['available']
    assert not arm.bus.writes and not arm.bus.goal_history


def test_diagnostic_static_zero_write_single_read_guard():
    """Keep this explicit-request path passive when future code changes."""
    import ast
    import inspect
    import textwrap
    from workshop.station.arm_session import ArmSession, JAW_DIAGNOSTIC_REGISTERS
    tree = ast.parse(textwrap.dedent(inspect.getsource(ArmSession.jaw_diagnostics)))
    forbidden = {'write', 'sync_write', 'connect', 'disconnect', 'reconnect', 'shutdown',
                 '_write_each', '_ensure_profile', '_component_shutdown', 'cancel_hold',
                 '_hold_measured_pose', 'move_plan', 'move_gripper'}
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    assert not [node for node in calls if isinstance(node.func, ast.Attribute) and
                node.func.attr in forbidden]
    reads = [node for node in calls if isinstance(node.func, ast.Attribute) and
             node.func.attr in {'read', 'sync_read'}]
    assert len(reads) == 2
    for node in reads:
        assert {kw.arg: ast.literal_eval(kw.value) for kw in node.keywords} == {
            'normalize': False, 'num_retry': 0}
    assert [name for name, _, _ in JAW_DIAGNOSTIC_REGISTERS] == list(REGISTERS)
    fixed_joint_calls = [node for node in calls if isinstance(node.func, ast.Name) and
        node.func.id == 'acquire' and any(kw.arg == 'joint' for kw in node.keywords)]
    assert len(fixed_joint_calls) == 1
    assert ast.literal_eval(next(kw.value for kw in fixed_joint_calls[0].keywords if kw.arg == 'joint')) == 'gripper'


def test_periodic_publish_and_ticks_do_not_hash_source_or_read_calibration():
    import ast
    import inspect
    import textwrap
    from pathlib import Path
    from workshop.station.arm_session import ArmSession
    owner = ast.parse(textwrap.dedent(inspect.getsource(ArmSession.publish_state)))
    bridge = ast.parse((Path(__file__).resolve().parents[1]/
        'ros2/src/so101_station_control/so101_station_control/trajectory_server.py').read_text())
    periodic = [node for node in ast.walk(bridge) if isinstance(node, ast.FunctionDef) and
        node.name in {'_publish_joint_state', '_publish_teleop_monitor', '_teleop_tick'}]
    assert len(periodic) == 3
    forbidden = {'owner_source_sha256', '_calibration_guard', 'read_bytes', 'read_text', 'sha256'}
    for tree in [owner, *periodic]:
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, 'id', None)
                assert name not in forbidden
