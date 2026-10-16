"""Native ROS bridge tests with an in-memory owner; no actuator access."""
import json
import os
from pathlib import Path
import sys
import threading
import time

import pytest
rclpy = pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_control'))
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from so101_station_control.trajectory_server import TrajectoryBridge, TELEOP_JOINTS


@pytest.fixture
def bridge(monkeypatch, tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '75'
    rclpy.init()
    def start(node):
        node._expected_arm_id = 'fake-arm'
        node._expected_calibration_sha256 = 'a' * 64
    monkeypatch.setattr(TrajectoryBridge, '_start_owner', start)
    root = Path(__file__).resolve().parents[1]
    node = TrajectoryBridge(parameter_overrides=[
        Parameter('station_root', value=str(root)),
        Parameter('state_file', value=str(tmp_path / 'state.json')),
        Parameter('stop_timeout_s', value=.35)])
    class Process:
        alive = True
        transform = staticmethod(lambda response: response)
        commands = []
        def poll(self):
            return None if self.alive else 1
        def write(self, line):
            command = json.loads(line)
            self.commands.append(command)
            if command['command'] == 'stop':
                response = {'ok': True, 'stopped': True, 'request_id': command['request_id'],
                            'state': snapshot()}
                transformed = self.transform(response)
                if transformed:
                    node._route_stop_response(transformed)
        def flush(self):
            pass
    process = Process()
    process.stdin = process
    node._process = process
    real_write_stop = node._write_owner_command
    def fake_write_stop(command, deadline):
        if process.stdin is process:
            process.stdin.write(json.dumps(command) + "\n")
        else:
            real_write_stop(command, deadline)
    monkeypatch.setattr(node, '_write_owner_command', fake_write_stop)
    node._read_state = lambda **kw: snapshot()
    yield node, process
    node._process = None
    node.destroy_node()
    rclpy.shutdown()


def snapshot():
    return dict(monotonic=time.monotonic(), arm_id='fake-arm',
                calibration_sha256='a' * 64, event='sample',
                raw=dict.fromkeys(TELEOP_JOINTS, 2000),
                pose=dict.fromkeys(TELEOP_JOINTS, 0.),
                torque=dict.fromkeys(TELEOP_JOINTS, 0))


def stop(node):
    return node._stop(Trigger.Request(), Trigger.Response())


def test_stop_correlated_fresh_and_independent(bridge):
    node, process = bridge
    assert node._stop_service.callback_group is node._stop_group
    assert node._stop_group is not node._teleop_service_group
    assert node._stop_group is not node._action_group
    assert stop(node).success
    assert not node._owner_faulted and not node._stop_latched
    assert process.commands[0]['command'] == 'stop'
    assert len(process.commands[0]['request_id']) == 32


@pytest.mark.parametrize('failure', ['missing', 'wrong_id', 'stale', 'future', 'identity',
                                    'calibration', 'raw', 'torque_missing', 'torque_on', 'dead'])
def test_stop_never_acknowledges_unverified_state(bridge, failure):
    node, process = bridge
    def transform(response):
        state = response['state']
        if failure == 'missing': return None
        if failure == 'wrong_id': response['request_id'] = '0' * 32
        if failure == 'stale': state['monotonic'] -= 10
        if failure == 'future': state['monotonic'] += 10
        if failure == 'identity': state['arm_id'] = 'other'
        if failure == 'calibration': state['calibration_sha256'] = 'b' * 64
        if failure == 'raw': state['raw'].pop('gripper')
        if failure == 'torque_missing': state['torque'].pop('gripper')
        if failure == 'torque_on': state['torque']['gripper'] = 1
        if failure == 'dead': process.alive = False
        return response
    process.transform = transform
    assert not stop(node).success
    assert node._stop_latched and node._owner_faulted
    with pytest.raises(RuntimeError, match='barrier|not running'):
        node._send({'command': 'home'})
    # Retrying stop is allowed; no old ack may satisfy the next request.
    process.alive = True
    process.transform = lambda response: response
    assert stop(node).success


@pytest.mark.parametrize('mode', ['readonly', 'unavailable'])
def test_stop_requires_owner(bridge, mode):
    node, process = bridge
    if mode == 'readonly': node._external_state_only = True
    else: node._process = None
    assert not stop(node).success
    assert process.commands == []


def test_parallel_home_reserved_before_send_cannot_rearm_after_stop(bridge, monkeypatch):
    node, process = bridge
    entered, release = threading.Event(), threading.Event()
    original = node._send
    def delayed_send(command):
        if command['command'] == 'home':
            entered.set()
            assert release.wait(1)
        return original(command)
    monkeypatch.setattr(node, '_send', delayed_send)
    home_response = Trigger.Response()
    worker = threading.Thread(target=node._home, args=(Trigger.Request(), home_response))
    worker.start()
    assert entered.wait(1)
    def resume():
        while not node._stop_latched:
            time.sleep(.001)
        release.set()
    releaser = threading.Thread(target=resume)
    releaser.start()
    started = time.monotonic()
    assert stop(node).success
    assert time.monotonic() - started < .35
    worker.join(1)
    releaser.join(1)
    assert not worker.is_alive() and not home_response.success
    assert [c['command'] for c in process.commands] == ['stop']
    assert node._active_goal is None


def test_pending_trajectory_cannot_be_released_by_stop(bridge):
    node, process = bridge
    node._active_goal = 'pending'
    assert not stop(node).success
    class Goal:
        aborted = False
        def abort(self): self.aborted = True
    goal = Goal()
    result = node._execute(goal)
    assert goal.aborted
    assert 'torque disabled' in result.error_string
    assert node._active_goal is None
    assert stop(node).success


def test_ros_stop_service_completes_while_home_callback_is_waiting(bridge, monkeypatch):
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    node, process = bridge
    entered = threading.Event()
    original_write = process.write
    def write(line):
        if json.loads(line)['command'] == 'home':
            process.commands.append(json.loads(line))
            entered.set()  # no home completion: the stop ack must unblock it
        else:
            original_write(line)
    process.write = write
    client_node = Node('synthetic_stop_service_client')
    home = client_node.create_client(Trigger, '/follower/home')
    stop_client = client_node.create_client(Trigger, '/follower/stop')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert home.wait_for_service(timeout_sec=2)
        assert stop_client.wait_for_service(timeout_sec=2)
        home_future = home.call_async(Trigger.Request())
        assert entered.wait(2)
        stop_future = stop_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2
        while not stop_future.done() and time.monotonic() < deadline:
            time.sleep(.005)
        assert stop_future.done() and stop_future.result().success
        assert home_future.done() and not home_future.result().success
        assert [c['command'] for c in process.commands] == ['home', 'stop']
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(2)
        client_node.destroy_node()


def test_external_stop_cannot_report_trajectory_success(bridge, monkeypatch):
    from types import SimpleNamespace
    node, process = bridge
    monkeypatch.setattr(node, '_collision_check_path', lambda *args: {'effective_path_sha256': 'c' * 64})
    artifacts = []
    monkeypatch.setattr(node, '_write_artifact', lambda artifact: artifacts.append(dict(artifact)))
    monkeypatch.setattr(node, '_feedback', lambda *args: True)
    original_write = process.write
    def write(line):
        if json.loads(line)['command'] == 'execute_trajectory':
            node._route_stop_response({'ok': True, 'stopped': True, 'final_state': snapshot()})
        else:
            original_write(line)
    process.write = write
    point = SimpleNamespace(time_from_start=SimpleNamespace(sec=1, nanosec=0),
                            positions=[0.] * 5, velocities=[], accelerations=[])
    class Goal:
        request = SimpleNamespace(trajectory=SimpleNamespace(
            joint_names=list(TELEOP_JOINTS[:5]), points=[point, point]))
        is_cancel_requested = False
        outcome = None
        def abort(self): self.outcome = 'aborted'
        def canceled(self): self.outcome = 'cancelled'
        def succeed(self): self.outcome = 'succeeded'
    goal = Goal()
    result = node._execute(goal)
    assert goal.outcome == 'aborted'
    assert result.error_code != 0
    assert 'torque-off confirmation' in result.error_string
    assert artifacts[-1]['status'] == 'stopped_torque_off'


def test_busy_writer_has_bounded_stop_timeout_and_keeps_barrier(bridge):
    node, process = bridge
    node._write_lock.acquire()
    try:
        started = time.monotonic()
        assert not stop(node).success
        assert time.monotonic() - started < .6
        assert node._stop_latched and node._owner_faulted
        assert not process.commands
    finally:
        node._write_lock.release()
    assert stop(node).success


def test_stop_pipe_backpressure_obeys_deadline(bridge):
    node, process = bridge
    read_fd, write_fd = os.pipe()
    original = process.stdin
    try:
        os.set_blocking(write_fd, False)
        while True:
            try:
                os.write(write_fd, b'x' * 4096)
            except BlockingIOError:
                break
        process.stdin = os.fdopen(write_fd, 'w')
        started = time.monotonic()
        assert not stop(node).success
        assert time.monotonic() - started < .6
        assert node._stop_latched
    finally:
        process.stdin.close()
        process.stdin = original
        os.close(read_fd)


def test_stop_broken_reader_fails_closed(bridge):
    node, process = bridge
    read_fd, write_fd = os.pipe()
    os.close(read_fd)
    stream = os.fdopen(write_fd, 'w')
    original = process.stdin
    process.stdin = stream
    try:
        started = time.monotonic()
        assert not stop(node).success
        assert time.monotonic() - started < .35
        assert node._stop_latched
    finally:
        stream.close()
        process.stdin = original


def test_stop_rejects_previous_write_failure(bridge, monkeypatch):
    node, process = bridge
    read_fd, write_fd = os.pipe()
    original = process.stdin
    stream = os.fdopen(write_fd, 'w')
    process.stdin = stream
    monkeypatch.setattr(os, 'write', lambda descriptor, data: len(data) - 1)
    try:
        with pytest.raises(RuntimeError, match='partial'):
            node._send({'command': 'home'})
        assert node._writer_faulted
        response = stop(node)
        assert not response.success
        assert 'previous owner write failed' in response.message
        assert node._stop_latched
    finally:
        stream.close()
        process.stdin = original
        os.close(read_fd)


def test_stop_missing_fd_and_partial_write_fail_closed(bridge, monkeypatch):
    node, process = bridge
    class MissingFD:
        pass
    original = process.stdin
    process.stdin = MissingFD()
    assert not stop(node).success
    read_fd, write_fd = os.pipe()
    stream = os.fdopen(write_fd, 'w')
    process.stdin = stream
    monkeypatch.setattr(os, 'write', lambda descriptor, data: len(data) - 1)
    try:
        response = stop(node)
        assert not response.success and 'partial' in response.message
        assert node._writer_faulted and node._stop_latched
    finally:
        stream.close()
        process.stdin = original
        os.close(read_fd)


def test_normal_large_command_delivery_uses_bounded_atomic_chunks(bridge):
    node, process = bridge
    read_fd, write_fd = os.pipe()
    original = process.stdin
    stream = os.fdopen(write_fd, 'w')
    process.stdin = stream
    received = []
    def read():
        with os.fdopen(read_fd, 'rb') as reader:
            received.append(json.loads(reader.readline()))
    reader = threading.Thread(target=read)
    reader.start()
    command = {'command': 'execute_trajectory', 'points': [{'positions': [0.] * 5}] * 1000}
    try:
        node._send(command)
        reader.join(1)
        assert not reader.is_alive() and received == [command]
        assert not node._writer_faulted
        assert os.get_blocking(write_fd)  # original file status restored
    finally:
        stream.close()
        process.stdin = original
        reader.join(1)


def test_saturated_normal_write_releases_writer_for_parallel_stop(bridge):
    node, process = bridge
    read_fd, write_fd = os.pipe()
    stream = os.fdopen(write_fd, 'w')
    original = process.stdin
    os.set_blocking(write_fd, False)
    filled = 0
    while True:
        try:
            filled += os.write(write_fd, b'x' * 4096)
        except BlockingIOError:
            break
    os.set_blocking(write_fd, True)
    process.stdin = stream
    errors = []
    done = threading.Event()
    def normal_write():
        try:
            node._send({'command': 'execute_trajectory', 'points': [0.] * 10000})
        except TimeoutError as error:
            errors.append(error)
        finally:
            done.set()
    worker = threading.Thread(target=normal_write)
    worker.start()
    deadline = time.monotonic() + 1
    while not node._write_lock.locked() and time.monotonic() < deadline:
        time.sleep(.001)
    assert node._write_lock.locked()
    def drain_and_ack():
        assert done.wait(1)
        with os.fdopen(read_fd, 'rb') as reader:
            assert reader.read(filled) == b'x' * filled
            command = json.loads(reader.readline())
            node._route_stop_response({'ok': True, 'stopped': True,
                'request_id': command['request_id'], 'state': snapshot()})
    reader = threading.Thread(target=drain_and_ack)
    reader.start()
    try:
        started = time.monotonic()
        assert stop(node).success
        assert time.monotonic() - started < .35
        worker.join(1)
        reader.join(1)
        assert errors and not worker.is_alive() and not reader.is_alive()
        assert not node._writer_faulted
    finally:
        stream.close()
        process.stdin = original


def test_incomplete_large_line_prohibits_future_writes(bridge):
    node, process = bridge
    read_fd, write_fd = os.pipe()
    stream = os.fdopen(write_fd, 'w')
    original = process.stdin
    process.stdin = stream
    try:
        with pytest.raises(TimeoutError):
            node._send({'command': 'execute_trajectory', 'points': [0.] * 100000})
        assert node._writer_faulted
        response = stop(node)
        assert not response.success and 'stream integrity' in response.message
    finally:
        stream.close()
        process.stdin = original
        os.close(read_fd)


def test_normal_write_broken_reader_and_missing_fd_never_use_textio(bridge):
    node, process = bridge
    class NoFD:
        def write(self, value): raise AssertionError('blocking TextIO write forbidden')
        def flush(self): raise AssertionError('blocking TextIO flush forbidden')
    original = process.stdin
    process.stdin = NoFD()
    with pytest.raises(RuntimeError, match='file descriptor'):
        node._send({'command': 'home'})
    read_fd, write_fd = os.pipe()
    os.close(read_fd)
    stream = os.fdopen(write_fd, 'w')
    process.stdin = stream
    try:
        started = time.monotonic()
        with pytest.raises(BrokenPipeError):
            node._send({'command': 'home'})
        assert time.monotonic() - started < .2
    finally:
        stream.close()
        process.stdin = original
