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
from so101_station_workflow.srv import ConfirmReference, EV3Command, RunCycles, SorterPosition

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
    axis = {'angle': 0., 'job': None, 'last_job': None, 'hold': False, 'power': 'awake'}
    axis_arguments = []
    arm_stop_seen = threading.Event()
    arm_stop_times, ev3_stop_response_times, preparations = [], [], []
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
                    if axis['job'] and not axis['hold'] and time.monotonic() >= axis['job']['complete_at']:
                        axis['angle'] = axis['job']['target']
                        axis['last_job'] = dict(axis['job'], state='complete')
                        axis['job'] = None
                    sampled = int(time.monotonic()*1000)
                    status = dict(session='fake-session', connected=True, busy=axis['job'] is not None,
                        references=refs.copy(), power_mode=axis['power'], last_error=None, sampled_ms=sampled,
                        angles={'sorter': axis['angle'], 'feeder': 0.}, job=axis['job'], last_job=axis['last_job'],
                        motor_control={name: {'angle_sample_ms': sampled, 'angle_read_ms': 1,
                            'speed_sample': {'speed_dps': 0., 'sampled_ms': sampled, 'duration_ms': 1}}
                            for name in ('sorter', 'feeder')})
                    if op in ('status', 'critical_status'):
                        result = status
                    elif op == 'stop':
                        if axis.get('wait_for_arm_stop'):
                            assert arm_stop_seen.wait(.6), 'arm stop queued behind blocking EV3 RPC'
                            ev3_stop_response_times.append(time.monotonic())
                        if axis['job']:
                            axis['last_job'] = dict(axis['job'], state='aborted')
                            axis['job'] = None
                        result = {'stopped': True, 'status': status}
                    elif op == 'wake':
                        axis['power'] = 'awake'
                        result = {'power_mode': 'awake'}
                    elif op == 'motor_target':
                        axis_arguments.append(req['args'])
                        axis['job'] = dict(id=req['id'], operation=op, motor='sorter', target=req['args']['target'],
                                           state='running', complete_at=time.monotonic()+.15)
                        result = {'accepted': True, 'job': axis['job']}
                    elif op == 'confirm_reference':
                        refs[req['args']['name']] = req['args']['reference']
                        result = {'confirmed': True}
                    else:
                        result = {'alive': True}
                    stream.write((json.dumps({'id': req['id'], 'ok': True, 'result': result})+'\n').encode())
                    stream.flush()
    server = threading.Thread(target=serve, daemon=True)
    server.start()
    env = dict(os.environ, SO101_STATION_ROOT=str(ROOT), XDG_STATE_HOME=str(tmp_path/'state'))
    # Simulated perception/arm adapter preserves the real cycle core and ROS
    # EV3 proxy, while no hardware executable can be reached by this test.
    fake_adapter = """
from tools import ros_sort_station as sorting
from pathlib import Path
import time
class FakeDevices:
    def __init__(self, settings, client, output, **kwargs):
        self.ev3 = client
        self.output = Path(output)
        self.block = settings.get('synthetic_block', False)
        self.readiness_block = settings.get('synthetic_readiness_block', False)
    def require_ready(self, deadline, cancel):
        if self.readiness_block:
            (self.output/'readiness-entered').write_text('ready')
            if cancel.wait(1.2):
                raise InterruptedError('readiness cancelled')
        return self._require_native_ready(deadline, cancel)
    def _require_native_ready(self, deadline, cancel):
        from workshop.station.ev3_readiness import require_fresh_stationary
        return require_fresh_stationary(self.ev3, session=self.ev3.session, deadline=deadline,
            cancel=cancel, spin=lambda deadline, cancel: time.sleep(.01))
    def wait_clear(self, deadline, cancel):
        (self.output/'entered-wait-clear').write_text('ready')
        if cancel.wait(3. if self.block else .08):
            # Keep this job alive after braking to exercise stale queued RPCs.
            time.sleep(.8)
            try:
                self.ev3.call('wake')
            except InterruptedError:
                (self.output/'proxy-cancelled').write_text('blocked before RPC')
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
    def stopped(request, response):
        arm_stop_times.append(time.monotonic())
        arm_stop_seen.set()
        return _success(response, 'all six torque bits zero')
    stop_service = node.create_service(Trigger, '/follower/stop',
        stopped)
    def prepared(request, response):
        preparations.append(time.monotonic())
        return _success(response, json.dumps(dict(schema=1,kind='arm_prepare',state='ready',
            owner_started_monotonic=time.monotonic()-1,home_completed_this_session=True)))
    prepare_service = node.create_service(Trigger, '/follower/prepare', prepared)
    captures = []
    def captured(request, response):
        captures.append(time.monotonic())
        return _success(response, json.dumps(dict(schema='resident-home-capture-v1', captured=True,
            qualified=False, command_id='a'*32)))
    capture_service = node.create_service(Trigger, '/follower/capture_home', captured)
    def call(type_, name, request=None):
        client = node.create_client(type_, '/station/'+name)
        if not client.wait_for_service(timeout_sec=8):
            if process.poll() is not None:
                raise AssertionError(process.communicate(timeout=2)[0].decode())
            raise AssertionError('station API service unavailable')
        future = client.call_async(request or type_.Request())
        until = time.monotonic()+5
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        assert future.done(), name
        return future.result()
    try:
        admission = call(Trigger, 'arm/prepare')
        assert admission.success
        until=time.monotonic()+3
        while time.monotonic()<until:
            standalone=json.loads(call(Trigger,'status').message)['workflow']
            if standalone['state']=='idle':break
        assert standalone['result']['state']=='ready'
        assert len(preparations)==1 and connections==[] and operations==[]
        assert call(Trigger, 'arm/capture_home').success
        until=time.monotonic()+3
        while time.monotonic()<until:
            captured_job=json.loads(call(Trigger,'status').message)['workflow']
            if captured_job['state']=='idle':break
        assert captured_job['result']['capture_home']['captured'] is True
        assert len(captures)==1 and connections==[] and operations==[]
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
        assert call(ConfirmReference, 'ev3/confirm_reference',
                    ConfirmReference.Request(name='sorter', reference='home', physically_verified=True)).success
        configured = json.loads(config_path.read_text())
        configured['automatic_start'] = False
        config_path.write_text(json.dumps(configured))
        before_auto_rejection = (list(operations),len(preparations))
        assert not call(RunCycles,'run_cycles',RunCycles.Request(cycles=1)).success
        assert (operations,len(preparations)) == before_auto_rejection
        configured['automatic_start'] = True
        configured['synthetic_readiness_block'] = True
        config_path.write_text(json.dumps(configured))
        readiness = call(Trigger, 'check_ready')
        assert readiness.success
        readonly_job = json.loads(readiness.message)['job_id']
        until = time.monotonic()+3
        while not (tmp_path/'readiness-entered').exists() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        assert (tmp_path/'readiness-entered').exists()
        before_readiness = {op: operations.count(op) for op in ('eject', 'home_touch', 'wake', 'power_save', 'motor_target')}
        for operation in before_readiness:
            rejected = call(EV3Command, 'ev3/command', EV3Command.Request(operation=operation,
                arguments_json='{}', job_id=readonly_job))
            assert not rejected.success and 'private single-use' in rejected.message
        assert all(operations.count(op) == count for op, count in before_readiness.items())
        until = time.monotonic()+3
        while time.monotonic() < until:
            state = json.loads(call(Trigger, 'status').message)['workflow']
            if state['state'] == 'idle':
                break
            time.sleep(.02)
        assert state['result'] == {'ready': True}
        configured['synthetic_readiness_block'] = False
        config_path.write_text(json.dumps(configured))
        axis['power'] = 'power_save'
        wakes_before_run = operations.count('wake')
        assert call(RunCycles, 'run_cycles', RunCycles.Request(cycles=2)).success
        until = time.monotonic()+5
        while time.monotonic() < until:
            state = json.loads(call(Trigger, 'status').message)['workflow']
            if state['state'] == 'idle':
                break
            time.sleep(.05)
        assert state['result'] is not None, state
        assert state['result']['completed'] == 2
        assert len(preparations)==2  # once in the same sorting job, not per cycle
        assert axis['power'] == 'awake'
        assert operations.count('wake') == wakes_before_run+1
        assert operations.index('wake') < operations.index('eject')
        assert operations.count('eject') == 2
        configured = json.loads(config_path.read_text())
        configured['synthetic_block'] = True
        config_path.write_text(json.dumps(configured))
        admission = call(RunCycles, 'run_cycles', RunCycles.Request(cycles=100))
        assert admission.success
        cancelled_job_id = json.loads(admission.message)['job_id']
        until = time.monotonic()+3
        while not (tmp_path/cancelled_job_id/'entered-wait-clear').exists() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        assert (tmp_path/cancelled_job_id/'entered-wait-clear').exists()
        arm_stop_seen.clear()
        axis['wait_for_arm_stop']=True
        before_injection = operations.count('eject')
        rejected = call(EV3Command, 'ev3/command', EV3Command.Request(operation='eject',
            arguments_json='{"cycles":100}', job_id=cancelled_job_id))
        assert not rejected.success and 'private single-use' in rejected.message
        assert operations.count('eject') == before_injection
        assert call(Trigger, 'cancel').success
        mutations_after_stop = {op: operations.count(op) for op in ('eject', 'home_touch', 'wake', 'power_save', 'motor_target')}
        for operation in mutations_after_stop:
            rejected = call(EV3Command, 'ev3/command', EV3Command.Request(operation=operation,
                arguments_json='{}', job_id=cancelled_job_id))
            assert not rejected.success, operation
            assert 'private single-use' in rejected.message, (operation, rejected.message)
        assert all(operations.count(op) == count for op, count in mutations_after_stop.items())
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.02)
        state = json.loads(call(Trigger, 'status').message)
        assert arm_stop_times[-1] <= ev3_stop_response_times[-1]
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
        assert (tmp_path/cancelled_job_id/'proxy-cancelled').read_text() == 'blocked before RPC'
        assert all(operations.count(op) == count for op, count in mutations_after_stop.items())
        assert not call(RunCycles, 'run_cycles', RunCycles.Request(cycles=1)).success
        assert len(connections) == 1
        def terminal(expected):
            until = time.monotonic()+4
            while time.monotonic() < until:
                state = json.loads(call(Trigger, 'status').message)['workflow']
                if state['state'] == expected:
                    return state
                time.sleep(.02)
            raise AssertionError(state)
        assert call(Trigger, 'reset_fault').success
        # No home reference: accepted bounded job fails before motor dispatch.
        refs['sorter'] = None
        assert call(SorterPosition, 'ev3/sorter_position', SorterPosition.Request(target_deg=20.)).success
        assert 'home reference' in terminal('fault')['fault']
        assert not axis_arguments
        failures = [json.loads(line) for line in (tmp_path/'station-events.jsonl').read_text().splitlines()
                    if json.loads(line).get('event') == 'sorter_position_failed']
        assert failures
        assert failures[-1]['native_observation']['session'] == 'fake-session'
        assert failures[-1]['native_observation']['references']['sorter'] is None
        assert failures[-1]['native_observation']['busy'] is False
        assert call(Trigger, 'reset_fault').success
        assert call(ConfirmReference, 'ev3/confirm_reference',
                    ConfirmReference.Request(name='sorter', reference='home', physically_verified=True)).success
        before_boundary = json.loads(call(Trigger, 'status').message)['workflow']
        wake_count = operations.count('wake')
        for boundary in (0., 550., 551.):
            rejected = call(SorterPosition, 'ev3/sorter_position', SorterPosition.Request(target_deg=boundary))
            assert not rejected.success
            after_boundary = json.loads(call(Trigger, 'status').message)['workflow']
            assert after_boundary['job_id'] == before_boundary['job_id']
            assert after_boundary['state'] == before_boundary['state']
        assert operations.count('wake') == wake_count
        assert not axis_arguments
        assert call(SorterPosition, 'ev3/sorter_position', SorterPosition.Request(target_deg=549.)).success
        assert 'six-second' in terminal('fault')['fault']
        assert not axis_arguments
        assert call(Trigger, 'reset_fault').success
        assert call(SorterPosition, 'ev3/sorter_position', SorterPosition.Request(target_deg=20.)).success
        result = terminal('idle')['result']
        assert result['measured_deg'] == 20.
        assert axis_arguments == [dict(name='sorter', target=20., speed=50., acceleration=400., actuation=30.)]
        axis['hold'] = True
        assert call(SorterPosition, 'ev3/sorter_position', SorterPosition.Request(target_deg=40.)).success
        until = time.monotonic()+3
        while not axis['job'] and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        assert axis['job'] is not None
        assert call(Trigger, 'cancel').success
        assert terminal('fault')['result'] is None
        assert axis['job'] is None
        assert len(axis_arguments) == 2
        assert operations.count('eject') == 2
        assert refs == {'sorter': 'home', 'feeder': 'eject'}
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
