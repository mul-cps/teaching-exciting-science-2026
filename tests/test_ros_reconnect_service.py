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
