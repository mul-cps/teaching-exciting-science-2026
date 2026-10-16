"""Synthetic ROS/EV3 API vertical; never opens a physical device."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from std_srvs.srv import Trigger
interfaces = pytest.importorskip('so101_station_workflow.srv')
from so101_station_workflow.srv import ConfirmReference, EV3Command, RunCycles

ROOT = Path(__file__).resolve().parents[1]


def test_resident_one_connection_reference_status_fault_and_cancel(tmp_path):
    if os.environ.get('ROS_DOMAIN_ID') != '75':
        pytest.skip('synthetic test requires explicitly isolated ROS_DOMAIN_ID=75')
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen(3)
    listener.settimeout(.2)
    port = listener.getsockname()[1]
    halt = threading.Event()
    connections, operations = [], []
    refs = {'sorter': None, 'feeder': None}
    def serve():
        while not halt.is_set():
            try:
                conn, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            connections.append(conn)
            with conn, conn.makefile('rwb') as stream:
                stream.write(b'{"protocol":"ev3-live-api","version":1,"ready":true,"session":"fake-session"}\n')
                stream.flush()
                for raw in stream:
                    req = json.loads(raw)
                    op = req['operation']
                    operations.append(op)
                    status = {'session': 'fake-session', 'busy': False, 'references': refs.copy()}
                    if op == 'status':
                        result = status
                    elif op == 'stop':
                        result = {'stopped': True, 'status': status}
                    elif op == 'confirm_reference':
                        refs[req['args']['name']] = req['args']['reference']
                        result = {'confirmed': True}
                    else:
                        result = {'alive': True}
                    stream.write((json.dumps({'id': req['id'], 'ok': True, 'result': result})+'\n').encode())
                    stream.flush()
    server = threading.Thread(target=serve, daemon=True)
    server.start()
    env = dict(os.environ, SO101_STATION_ROOT=str(ROOT))
    # Simulated perception/arm adapter preserves the real cycle core and ROS
    # EV3 proxy, while no hardware executable can be reached by this test.
    fake_adapter = """
from tools import ros_sort_station as sorting
import time
class FakeDevices:
    def __init__(self, settings, client, output, **kwargs):
        self.ev3 = client
    def require_ready(self, deadline, cancel):
        return self.ev3.call('status')
    def wait_clear(self, deadline, cancel):
        if cancel.wait(.08):
            raise InterruptedError('synthetic cycle cancelled')
        return True
    def dispense(self, deadline, cancel):
        return self.ev3.call('eject', {'cycles': 1})
    def detect(self, deadline, cancel):
        return {'track_id': 'red-0001', 'color': 'red'}
    def pick_place(self, target, destination, deadline, cancel):
        return dict(state='pick_place_complete', track_id=target['track_id'],
                    pickup_verified=True, lift_verified=True, release_verified=True)
    def stop(self):
        return self.ev3.call('stop')
sorting.ROSSortingDevices = FakeDevices
from tools.ros_station_api import main
main()
"""
    config_path = tmp_path/'sorting.json'
    command = [sys.executable, '-c', fake_adapter,
               '--ros-args', '-p', f'ev3_port:={port}', '-p', f'evidence_dir:={tmp_path}',
               '-p', f'sorting_config:={config_path}']
    process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('synthetic_station_api_client')
    stop_service = node.create_service(Trigger, '/follower/stop',
        lambda req, res: _success(res, 'all six torque bits zero'))
    def call(type_, name, request=None):
        client = node.create_client(type_, '/station/'+name)
        assert client.wait_for_service(timeout_sec=8), process.poll()
        future = client.call_async(request or type_.Request())
        until = time.monotonic()+5
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        assert future.done(), name
        return future.result()
    try:
        assert call(Trigger, 'ev3/connect').success
        assert call(Trigger, 'ev3/connect').success
        assert len(connections) == 1
        assert not call(ConfirmReference, 'ev3/confirm_reference',
                        ConfirmReference.Request(name='feeder', reference='eject', physically_verified=False)).success
        assert 'confirm_reference' not in operations
        assert call(ConfirmReference, 'ev3/confirm_reference',
                    ConfirmReference.Request(name='feeder', reference='eject', physically_verified=True)).success
        assert refs['feeder'] == 'eject'
        assert not call(EV3Command, 'ev3/command',
                        EV3Command.Request(operation='eject', arguments_json='{}')).success
        # Missing trusted configuration blocks cycles before any dispensing.
        assert not call(RunCycles, 'run_cycles', RunCycles.Request(cycles=1)).success
        assert 'eject' not in operations
        from tools.ros_plan_current import ARM_JOINTS
        (tmp_path/'station.json').write_text('{}')
        (tmp_path/'execution.json').write_text('{}')
        config_path.write_text(json.dumps(dict(schema='so101-ros-sorting-v1', enabled=True,
            commissioned=True, automatic_start=True, pickup_bounds_m=[0, 1, 0, 1, 0, 1],
            frame_id='base_link', destinations={'red': [0, 0, .2, 0, 0, 0, 1]},
            clear_joint_pose_rad={name: 0 for name in ARM_JOINTS}, station_config='station.json',
            grasp_execution_config='execution.json')))
        assert call(RunCycles, 'run_cycles', RunCycles.Request(cycles=2)).success
        until = time.monotonic()+5
        while time.monotonic() < until:
            state = json.loads(call(Trigger, 'status').message)['workflow']
            if state['state'] == 'idle':
                break
            time.sleep(.05)
        assert state['result'] is not None, state
        assert state['result']['completed'] == 2
        assert operations.count('eject') == 2
        assert call(RunCycles, 'run_cycles', RunCycles.Request(cycles=100)).success
        assert call(Trigger, 'cancel').success
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.02)
        state = json.loads(call(Trigger, 'status').message)
        assert state['stop']['ev3_stopped']
        assert state['stop']['arm_disarmed']
        assert not state['stop']['pending']
        until = time.monotonic()+5
        while time.monotonic() < until:
            state = json.loads(call(Trigger, 'status').message)['workflow']
            if state['state'] == 'fault':
                break
            time.sleep(.05)
        assert state['state'] == 'fault'
        assert state['result'] is None
        assert operations.count('eject') < 102
        assert not call(RunCycles, 'run_cycles', RunCycles.Request(cycles=1)).success
        assert len(connections) == 1
    finally:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        node.destroy_node()
        rclpy.shutdown()
        halt.set()
        listener.close()
        server.join(1)
    assert b'Traceback' not in output, output.decode()


def _success(response, message):
    response.success, response.message = True, message
    return response
