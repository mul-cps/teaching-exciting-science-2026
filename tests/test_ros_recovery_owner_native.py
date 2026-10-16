"""ROS -> pipe -> production owner/factory/proof, synthetic bus and depth only."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from test_ros_stop_service import TELEOP_JOINTS, TrajectoryBridge
from so101_station_control.trajectory_server import _load_owner_source_hashes


def test_native_camera_recovery_uses_actual_factory_and_proof(tmp_path, monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID') == '75'
    child_python = os.environ.get('SO101_RECOVERY_TEST_PYTHON')
    if not child_python or not Path(child_python).is_file():
        pytest.skip('set SO101_RECOVERY_TEST_PYTHON to Python with recovery geometry dependencies')
    root = Path(__file__).resolve().parents[1]
    folder = tmp_path/'fake-owner'
    folder.mkdir()
    state_file = folder/'state.json'
    processes = []
    def start(node, **kwargs):
        assert not kwargs  # the synthetic owner starts normally, never through reconnect
        node._owner_started = time.monotonic()
        node._owner_source_expected = _load_owner_source_hashes(root)
        env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(root), str(root/'tests'))))
        process = subprocess.Popen([child_python, str(root/'tests/fake_recovery_owner.py'),
            '--fixture-root', str(folder), '--state-file', str(state_file),
            '--logdir', str(folder/'logs'), '--lock', str(folder/'owner.lock')],
            cwd=root, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1)
        processes.append(process)
        node._process = process
        node._owner_readers = [threading.Thread(target=callback, args=(process,), daemon=True)
                              for callback in (node._read_responses, node._read_stderr)]
        for reader in node._owner_readers:
            reader.start()
        until = time.monotonic()+10
        while time.monotonic() < until:
            if process.poll() is not None:
                raise RuntimeError('synthetic actual owner exited during initialization')
            state = node._read_state()
            if state:
                node._expected_arm_id = state['arm_id']
                node._expected_calibration_sha256 = state['calibration_sha256']
                # The helper enters normal serve only after actual scene preparation.
                if state.get('event') == 'sample' and time.monotonic()-state['monotonic'] < .05:
                    return
            time.sleep(.005)
        raise TimeoutError('actual factory/prepared scene did not become ready')
    monkeypatch.setattr(TrajectoryBridge, '_start_owner', start)
    rclpy.init()
    node = client_node = executor = thread = None
    commands = []
    try:
        node = TrajectoryBridge(parameter_overrides=[Parameter('station_root', value=str(root)),
            Parameter('state_file', value=str(state_file))])
        original = node._send
        def send(command):
            commands.append(dict(command))
            return original(command)
        node._send = send
        client_node = Node('actual_recovery_factory_client')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.add_node(client_node)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        def invoke(name, timeout=2):
            client = client_node.create_client(Trigger, name)
            assert client.wait_for_service(timeout_sec=2)
            future = client.call_async(Trigger.Request())
            until = time.monotonic()+timeout
            while not future.done() and time.monotonic() < until:
                time.sleep(.005)
            assert future.done(), name
            return future.result()
        before = json.loads(invoke('/follower/status').message)
        assert before['connected'] and before['fresh'] and not before['home_completed_this_session']
        began = time.monotonic()
        result = invoke('/follower/recover_camera', timeout=10)
        elapsed = time.monotonic()-began
        assert result.success, result.message
        after = json.loads(invoke('/follower/status').message)
        assert after['connected'] and after['fresh'] and not after['home_completed_this_session']
        assert after['raw']['elbow_flex'] == 2973
        assert all(value == 1 for value in after['torque'].values())
        assert after['owner_source_sha256'] == _load_owner_source_hashes(root)
        assert commands == [{'command': 'recover_camera'}]
        operation_commands = list(commands)
        assert node._home_completed_owner_started is None
        assert elapsed <= 8.0
    finally:
        if executor: executor.shutdown()
        if thread: thread.join(2)
        if node: node.destroy_node()
        if client_node: client_node.destroy_node()
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        rclpy.shutdown()
    audit = json.loads((folder/'fake-owner-write-audit.json').read_text())
    goals = audit['goal_history']
    assert goals and goals[0]['elbow_flex'] == 2993 and goals[-1]['elbow_flex'] == 2973
    for previous, current in zip(goals, goals[1:]):
        assert 0 <= previous['elbow_flex']-current['elbow_flex'] <= 2
    assert all(goal[joint] == 2000 for goal in goals for joint in TELEOP_JOINTS if joint != 'elbow_flex')
    assert audit['eeprom_writes'] == []
    assert all(value == 0 for value in audit['final_torque'].values())
    evidence = os.environ.get('SO101_RECOVERY_TEST_EVIDENCE')
    if evidence:
        Path(evidence).write_text(json.dumps(dict(schema=1, hardware_access=False,
            physical_qualification=False, before=before, after=after, elapsed_s=elapsed,
            owner_commands=operation_commands, cleanup_commands=commands[len(operation_commands):],
            actual_bus=audit), indent=2)+'\n')
