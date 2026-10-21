"""Resident EV3 owner/panel contract using loopback fake motors, domain 75 only."""
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
from std_msgs.msg import String
from std_srvs.srv import Trigger
pytest.importorskip('so101_station_workflow.srv')
from so101_station_workflow.srv import ConfirmReference
ROOT = Path(__file__).resolve().parents[1]


def test_explicit_connection_endpoints_eject_and_terminal_status(tmp_path):
    if os.environ.get('ROS_DOMAIN_ID') != '75':
        pytest.skip('requires isolated domain 75')
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0)); listener.listen(1); listener.settimeout(.1)
    halt = threading.Event()
    calls = []
    angle = [0.]
    refs = {'sorter':'home','feeder':None}
    job = [None]
    last = [None]
    def serve():
        while not halt.is_set():
            try: connection, _ = listener.accept(); break
            except socket.timeout: continue
        else: return
        with connection, connection.makefile('rwb') as stream:
            stream.write(b'{"protocol":"ev3-live-api","version":1,"ready":true,"session":"panel-fake"}\n'); stream.flush()
            for raw in stream:
                request = json.loads(raw); op = request['operation']; calls.append(op)
                sampled = int(time.monotonic()*1000)
                if job[0] and time.monotonic() >= job[0]['until']:
                    angle[0] = job[0].get('target', angle[0]); last[0] = dict(job[0], state='complete'); job[0] = None
                status = dict(session='panel-fake', connected=True, busy=job[0] is not None,
                    references=refs.copy(), power_mode='awake', last_error=None,
                    angles={'sorter':angle[0],'feeder':0.}, sampled_ms=sampled, job=job[0], last_job=last[0],
                    touch=False, color='RED', motor_control={name:dict(angle_sample_ms=sampled,angle_read_ms=1,
                        speed_sample=dict(speed_dps=0.,sampled_ms=sampled,duration_ms=1)) for name in ('sorter','feeder')})
                if op in ('status','critical_status'): result = status
                elif op in ('motor_target','eject'):
                    job[0] = dict(id=request['id'],operation=op,motor='sorter' if op=='motor_target' else 'feeder',
                                  state='running', until=time.monotonic()+.15)
                    if op == 'motor_target': job[0]['target'] = request['args']['target']
                    result = dict(accepted=True,job=job[0])
                elif op == 'confirm_reference':
                    refs[request['args']['name']] = request['args']['reference']
                    result = dict(confirmed=True)
                elif op == 'stop': result = dict(stopped=True)
                else: result = dict(alive=True)
                stream.write((json.dumps(dict(id=request['id'],ok=True,result=result))+'\n').encode()); stream.flush()
    server = threading.Thread(target=serve,daemon=True); server.start()
    process = subprocess.Popen([sys.executable,'-c','from tools.ros_station_api import main; main()',
        '--ros-args','-p',f'ev3_port:={listener.getsockname()[1]}','-p',f'evidence_dir:={tmp_path}',
        '-p','ev3_left_end_deg:=10.0','-p','ev3_right_end_deg:=530.0'], cwd=ROOT,
        env=dict(os.environ,XDG_STATE_HOME=str(tmp_path/'state')),stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    rclpy.init(); node = Node('ev3_panel_contract_test'); snapshots = []
    subscription = node.create_subscription(String,'/station/ev3/operator_status',
        lambda message:snapshots.append(json.loads(message.data)),1)
    service = node.create_service(Trigger,'/follower/stop',lambda request,response: success(response))
    def call(name):
        client = node.create_client(Trigger,'/station/'+name)
        assert client.wait_for_service(timeout_sec=8), process.poll()
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+5
        while not future.done() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.02)
        assert future.done(), name
        return future.result()
    def terminal(job_id):
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            current = json.loads(call('status').message)['workflow']
            if current['job_id']==job_id and current['state'] in ('idle','fault'): return current
            time.sleep(.02)
        raise AssertionError('missing terminal workflow acknowledgement')
    try:
        assert call('status').success
        assert calls == [], 'startup must not connect or actuate'
        assert call('ev3/connect').success
        assert not set(calls) & {'home_touch','motor_target','eject','confirm_reference'}
        reference_client = node.create_client(ConfirmReference,'/station/ev3/confirm_reference')
        assert reference_client.wait_for_service(timeout_sec=3)
        reference = reference_client.call_async(ConfirmReference.Request(name='feeder',reference='eject',physically_verified=True))
        deadline=time.monotonic()+3
        while not reference.done() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.02)
        assert reference.done() and reference.result().success
        endpoint_ids=[]
        for name in ('ev3/right_end','ev3/left_end','ev3/eject'):
            response = call(name); assert response.success, response.message
            done = terminal(json.loads(response.message)['job_id'])
            assert done['state']=='idle',done
            if name.endswith('_end'):
                assert done['result']['phase_count']==3
                endpoint_ids.append(json.loads(response.message)['job_id'])
                assert len({phase['native_job_id'] for phase in done['result']['phases']})==3
        assert angle[0]==10.
        assert len(set(endpoint_ids))==2
        assert calls.count('motor_target')==6 and calls.count('eject')==1
        assert calls.count('confirm_reference')==1
        assert snapshots and snapshots[-1]['session']=='panel-fake'
        assert len(json.dumps(snapshots[-1]))<8192
        assert call('stop').success
    finally:
        process.terminate()
        deadline=time.monotonic()+3
        while process.poll() is None and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.02)
        if process.poll() is None: process.kill()
        output=process.communicate(timeout=3)[0].decode()
        node.destroy_node(); rclpy.shutdown(); halt.set(); listener.close(); server.join(2)
        assert process.returncode==0,output


def success(response):
    response.success=True; response.message='fake arm stop acknowledged'; return response
