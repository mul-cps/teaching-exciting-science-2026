"""Typed gripper calibration service with a fake resident owner, never hardware."""
import json
import threading
import time

import pytest
pytest.importorskip('rclpy')
from test_ros_stop_service import bridge, snapshot, stop
from so101_station_workflow.srv import CommissionGripperStep
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def command(delta=8):
    return CommissionGripperStep.Request(delta_counts=delta)


def install_completion(node, process, transform=lambda r: r):
    original = process.write
    def write(line):
        payload = json.loads(line)
        if payload['command'] != 'commission_gripper_step':
            return original(line)
        process.commands.append(payload)
        state = snapshot()
        state['event'] = 'commission_gripper_complete'
        state['torque_monotonic'] = state['monotonic']
        result = transform({'ok': True, 'state': state})
        node._response_queue.put(result)
    process.write = write


def test_component_typed_ros_service_roundtrip(bridge):
    node, process = bridge
    install_completion(node, process)
    client_node = Node('fake_component_client')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        client = client_node.create_client(CommissionGripperStep, '/follower/commission_gripper_step')
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(command())
        until = time.monotonic() + 2
        while not future.done() and time.monotonic() < until: time.sleep(.005)
        assert future.done() and future.result().success
        assert process.commands == [{'command': 'commission_gripper_step', 'delta_counts': 8}]
        assert node._active_goal is None
        assert json.loads(future.result().message)['state'] == 'commission_gripper_complete'
    finally:
        executor.shutdown()
        thread.join(1)
        executor.remove_node(node)
        client_node.destroy_node()


@pytest.mark.parametrize('mode', ['busy', 'faulted', 'stale', 'readonly', 'stopped', 'delta'])
def test_component_rejects_before_owner_write(bridge, mode):
    node, process = bridge
    if mode == 'busy': node._active_goal = 'trajectory'
    if mode == 'faulted': node._owner_faulted = True
    if mode == 'stale': node._read_state = lambda: None
    if mode == 'readonly': node._external_state_only = True
    if mode == 'stopped': node._stop_latched = True
    result = node._commission_gripper_step(command(17 if mode == 'delta' else 8), CommissionGripperStep.Response())
    assert not result.success
    assert not process.commands


@pytest.mark.parametrize('mode', ['wrong_identity', 'wrong_event', 'torque_on', 'stopped', 'error', 'stale', 'torque_stale', 'delayed', 'future', 'torque_future', 'malformed'])
def test_component_bad_completion_faults_and_never_retries(bridge, mode):
    node, process = bridge
    def change(result):
        if mode == 'wrong_identity': result['state']['arm_id'] = 'other'
        if mode == 'wrong_event': result['state']['event'] = 'home_complete'
        if mode == 'torque_on': result['state']['torque']['wrist_roll'] = 1
        if mode == 'stopped': result.update(stopped=True, cancelled=True)
        if mode == 'error': result.update(ok=False, error='owner failed')
        if mode == 'stale': result['state']['monotonic'] -= 10
        if mode == 'torque_stale': result['state']['torque_monotonic'] -= 10
        if mode == 'delayed': time.sleep(.6)
        if mode == 'future': result['state']['monotonic'] += 1
        if mode == 'torque_future': result['state']['torque_monotonic'] += 1
        if mode == 'malformed': return None
        return result
    install_completion(node, process, change)
    result = node._commission_gripper_step(command(), CommissionGripperStep.Response())
    assert not result.success and node._owner_faulted
    assert [c['command'] for c in process.commands] == ['commission_gripper_step', 'stop']


def test_component_reserved_before_stop_never_submits_after_stop(bridge, monkeypatch):
    node, process = bridge
    entered, release = threading.Event(), threading.Event()
    original = node._send
    def send(command):
        if command['command'] == 'commission_gripper_step':
            entered.set()
            assert release.wait(1)
        return original(command)
    monkeypatch.setattr(node, '_send', send)
    response = CommissionGripperStep.Response()
    worker = threading.Thread(target=node._commission_gripper_step, args=(command(), response))
    worker.start()
    assert entered.wait(1)
    def resume():
        while not node._stop_latched: time.sleep(.001)
        release.set()
    releaser = threading.Thread(target=resume)
    releaser.start()
    assert stop(node).success
    worker.join(1)
    releaser.join(1)
    assert not worker.is_alive() and not response.success
    assert all(c['command'] == 'stop' for c in process.commands)
