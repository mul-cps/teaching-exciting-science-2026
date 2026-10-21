"""ROS-only lifecycle recovery via an in-memory owner; no hardware."""
import io
import time
import pytest
pytest.importorskip('rclpy')
from test_ros_stop_service import bridge, snapshot, stop
from std_srvs.srv import Trigger
from so101_station_control.trajectory_server import _observe_owner_error


def observed(node, torque=0):
    state = snapshot()
    state.update(observe_only=True, torque_monotonic=state['monotonic'], owner_source_sha256={'arm':'a'*64,'arm_session':'b'*64})
    state['torque']['gripper'] = torque
    return state


def install(node, process, torque=0):
    process.alive = False
    process.wait = lambda **kw: 1
    calls = []
    def start(**kwargs):
        calls.append(kwargs)
        process.alive = True
        node._owner_started = time.monotonic()
        node._owner_source_expected = {'arm':'a'*64,'arm_session':'b'*64}
        return observed(node, torque)
    node._start_owner = start
    node._read_state = lambda **kw: observed(node, torque)
    return calls


@pytest.mark.parametrize('torque', [0, 1])
def test_reconnect_preserves_fault_until_explicit_stop_keeps_torque_on_owner(bridge, torque):
    node, process = bridge
    calls = install(node, process, torque)
    node._owner_faulted = True
    node._stop_latched = True
    node._home_completed_owner_started = 123
    response = node._reconnect(Trigger.Request(), Trigger.Response())
    assert response.success is (torque==0)
    assert process.alive and calls == [{'observe_only':True}]
    assert node._owner_faulted and node._stop_latched and node._home_completed_owner_started is None
    assert not process.commands
    assert stop(node).success
    assert not node._owner_faulted and not node._stop_latched
    assert [c['command'] for c in process.commands] == ['stop']


@pytest.mark.parametrize('case', ['alive', 'active', 'callback', 'readonly', 'closed', 'locked'])
def test_reconnect_never_replaces_live_or_busy_owner(bridge, case):
    node, process = bridge
    calls = install(node, process)
    if case == 'alive': process.alive=True
    if case == 'active': node._active_goal='home'
    if case == 'callback': node._control_callbacks=1
    if case == 'readonly': node._external_state_only=True
    if case == 'closed': node._closed=True
    if case == 'locked': node._stop_lock.acquire()
    try:
        assert not node._reconnect(Trigger.Request(), Trigger.Response()).success
        assert not calls and not process.commands
    finally:
        if case=='locked': node._stop_lock.release()


@pytest.mark.parametrize('case', ['identity', 'calibration', 'source', 'raw', 'torque', 'future', 'stale', 'torque_stale', 'mode'])
def test_passive_generation_requires_exact_fresh_provenance(bridge, case):
    node, process = bridge
    install(node, process)
    node._owner_started = time.monotonic()-.01
    state=observed(node)
    if case=='identity': state['arm_id']='other'
    if case=='calibration': state['calibration_sha256']='c'*64
    if case=='source': state['owner_source_sha256']['arm']='d'*64
    if case=='raw': state['raw'].pop('gripper')
    if case=='torque': state['torque']['gripper']=128
    if case=='future': state['monotonic']+=1
    if case=='stale': state['monotonic']-=1
    if case=='torque_stale': state['torque_monotonic']-=1
    if case=='mode': state['observe_only']=False
    node._owner_source_expected={'arm':'a'*64,'arm_session':'b'*64}
    assert _observe_owner_error(state,arm_id=node._expected_arm_id,calibration_sha256=node._expected_calibration_sha256,
        owner_started=node._owner_started,source_sha256=node._owner_source_expected,now=time.monotonic())
    node._reconnect_observe=True
    process.alive=True
    node._read_state=lambda **kw: state
    assert not stop(node).success
    assert not process.commands


def test_old_owner_reader_cannot_route_into_new_generation(bridge):
    node, process = bridge
    class Old:
        stdout=io.StringIO('{"ok":true}\n')
        stderr=io.StringIO('old error\n')
    node._read_responses(Old())
    node._read_stderr(Old())
    assert node._response_queue.empty()


def test_ros_reconnect_then_explicit_stop_roundtrip(bridge):
    import threading
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    node, process = bridge
    calls = install(node, process, torque=1)
    node._owner_faulted=True
    client_node=Node('passive_reconnect_test_client')
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.add_node(client_node)
    thread=threading.Thread(target=executor.spin)
    thread.start()
    try:
        def invoke(name):
            client=client_node.create_client(Trigger,name)
            assert client.wait_for_service(timeout_sec=2)
            future=client.call_async(Trigger.Request())
            until=time.monotonic()+2
            while not future.done() and time.monotonic()<until: time.sleep(.005)
            assert future.done()
            return future.result()
        assert node._reconnect_service.callback_group is node._stop_group
        assert not invoke('/follower/reconnect').success
        assert process.alive and node._owner_faulted and not process.commands
        assert invoke('/follower/stop').success
        assert not node._owner_faulted
        assert calls == [{'observe_only': True}]
    finally:
        executor.shutdown()
        thread.join(1)
        executor.remove_node(node)
        client_node.destroy_node()


def test_observe_startup_failure_keeps_new_passive_owner_for_stop(bridge):
    node,process=bridge
    install(node,process)
    original=node._start_owner
    def start(**kwargs):
        original(**kwargs)
        raise RuntimeError('state wait timed out')
    node._start_owner=start
    assert not node._reconnect(Trigger.Request(),Trigger.Response()).success
    assert process.alive and node._owner_faulted
    assert stop(node).success


def test_unquiesced_old_reader_prohibits_new_owner(bridge):
    node,process=bridge
    calls=install(node,process)
    class Reader:
        def join(self,timeout): pass
        def is_alive(self): return True
    node._owner_readers=[Reader()]
    assert not node._reconnect(Trigger.Request(),Trigger.Response()).success
    assert not calls and not process.commands


@pytest.mark.parametrize('phase', ['before_spawn', 'during_startup'])
def test_destroy_serializes_reconnect_and_leaves_no_live_generation(bridge, monkeypatch, phase):
    import threading
    from rclpy.node import Node
    node,process=bridge
    calls=install(node,process)
    entered,release=threading.Event(),threading.Event()
    if phase=='before_spawn':
        class Reader:
            def join(self,timeout): entered.set(); assert release.wait(2)
            def is_alive(self): return False
        node._owner_readers=[Reader()]
    else:
        original=node._start_owner
        def start(**kwargs):
            result=original(**kwargs)
            entered.set()
            assert release.wait(2)
            return result
        node._start_owner=start
    original_write=process.write
    def write(line):
        import json
        payload=json.loads(line)
        if payload['command']=='exit':
            process.commands.append(payload)
            process.alive=False
        else: original_write(line)
    process.write=write
    response=Trigger.Response()
    reconnect=threading.Thread(target=node._reconnect,args=(Trigger.Request(),response))
    reconnect.start()
    assert entered.wait(1)
    with monkeypatch.context() as patch:
        patch.setattr(Node,'destroy_node',lambda self: True)
        destroy=threading.Thread(target=node.destroy_node)
        destroy.start()
        until=time.monotonic()+1
        while not node._closed and time.monotonic()<until: time.sleep(.001)
        assert node._closed
        release.set()
        reconnect.join(2)
        destroy.join(2)
        assert not reconnect.is_alive() and not destroy.is_alive()
    assert not response.success and not process.alive
    if phase=='before_spawn': assert not calls and not process.commands
    else: assert process.commands == [{'command':'exit'}]


def fault_observed(node):
    state = observed(node)
    state.update(observe_only=False, write_scope='fault_observation', owner_session_id='arm-session-' + '1' * 32)
    state['fault_observation'] = dict(schema=1, owner_session_id=state['owner_session_id'],
        entered_monotonic=node._owner_started + .001, command='home',
        error='leg deadline', error_type='TimeoutError')
    return state


def install_fault_observer(node, process):
    calls = install(node, process)
    node._owner_started = time.monotonic() - .02
    node._owner_source_expected = {'arm': 'a' * 64, 'arm_session': 'b' * 64}
    process.alive = True
    node._read_state = lambda **kw: fault_observed(node)
    original_write = process.write
    def write(line):
        import json
        command = json.loads(line)
        if command['command'] == 'exit':
            process.commands.append(command)
            process.alive = False
        else:
            original_write(line)
    process.write = write
    def wait(**kwargs):
        assert not process.alive
        return 0
    process.wait = wait
    return calls


def test_explicit_reconnect_exits_only_verified_live_fault_observer_then_passively_replaces(bridge):
    node, process = bridge
    calls = install_fault_observer(node, process)
    node._home_completed_owner_started = node._owner_started
    response = node._reconnect(Trigger.Request(), Trigger.Response())
    assert response.success, response.message
    assert process.commands == [{'command': 'exit'}]
    assert calls == [{'observe_only': True}]
    assert node._owner_faulted and node._reconnect_observe
    assert node._home_completed_owner_started is None


@pytest.mark.parametrize('case', ['normal', 'identity', 'calibration', 'source', 'stale', 'torque_stale',
    'torque_on', 'missing_joint', 'fault_session', 'old_fault', 'future_fault', 'active', 'callback'])
def test_explicit_live_reconnect_requires_complete_current_fault_observation(bridge, case):
    node, process = bridge
    calls = install_fault_observer(node, process)
    state = fault_observed(node)
    if case == 'normal': state['fault_observation'] = None
    if case == 'identity': state['arm_id'] = 'other'
    if case == 'calibration': state['calibration_sha256'] = 'c' * 64
    if case == 'source': state['owner_source_sha256'] = {'arm': 'z' * 64}
    if case == 'stale': state['monotonic'] -= 1
    if case == 'torque_stale': state['torque_monotonic'] -= 1
    if case == 'torque_on': state['torque']['elbow_flex'] = 1
    if case == 'missing_joint': state['raw'].pop('gripper')
    if case == 'fault_session': state['fault_observation']['owner_session_id'] = 'other'
    if case == 'old_fault': state['fault_observation']['entered_monotonic'] = node._owner_started - .01
    if case == 'future_fault': state['fault_observation']['entered_monotonic'] = time.monotonic() + 1
    if case == 'active': node._active_goal = 'home'
    if case == 'callback': node._control_callbacks = 1
    node._read_state = lambda **kw: state
    home_proof = node._home_completed_owner_started
    assert not node._reconnect(Trigger.Request(), Trigger.Response()).success
    assert process.alive and not process.commands and not calls
    if case == 'normal':
        assert not node._owner_faulted
        assert node._home_completed_owner_started == home_proof


def test_verified_stop_cannot_clear_owner_fault_observation_or_home_proof(bridge):
    node, process = bridge
    install_fault_observer(node, process)
    node._owner_faulted = True
    node._home_completed_owner_started = node._owner_started
    def transform(reply):
        state = fault_observed(node)
        state['event'] = 'torque_off'
        reply['state'] = state
        return reply
    process.transform = transform
    result = stop(node)
    assert result.success, result.message
    assert node._owner_faulted
    assert node._home_completed_owner_started is None


def test_explicit_reset_owner_reinitializes_healthy_idle_alloff_without_weakening_reconnect(bridge):
    node, process = bridge
    calls = install_fault_observer(node, process)
    def healthy(**kwargs):
        value = fault_observed(node)
        value['fault_observation'] = None
        value['write_scope'] = 'normal'
        return value
    node._read_state = healthy
    node._home_completed_owner_started = node._owner_started
    assert node._reset_owner_service.callback_group is node._stop_group
    assert not node._reconnect(Trigger.Request(), Trigger.Response()).success
    assert not process.commands and not calls
    response = node._reset_owner(Trigger.Request(), Trigger.Response())
    assert response.success, response.message
    assert process.commands == [{'command': 'exit'}]
    assert calls == [{'observe_only': True}]
    assert node._home_completed_owner_started is None and node._owner_faulted


@pytest.mark.parametrize('failure', ['torque_on', 'stale', 'source', 'busy', 'teleop'])
def test_explicit_reset_owner_requires_fresh_idle_alloff_proof(bridge, failure):
    node, process = bridge
    calls = install_fault_observer(node, process)
    state = fault_observed(node)
    state['fault_observation'] = None
    if failure == 'torque_on': state['torque']['gripper'] = 1
    if failure == 'stale': state['monotonic'] -= 1
    if failure == 'source': state['owner_source_sha256'] = {}
    if failure == 'busy': node._active_goal = 'home'
    if failure == 'teleop': node._teleop_state = 'active'
    node._read_state = lambda **kwargs: state
    assert not node._reset_owner(Trigger.Request(), Trigger.Response()).success
    assert not calls and not process.commands
