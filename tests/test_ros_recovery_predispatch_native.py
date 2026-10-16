"""Force a genuine ROS identity renewal inside passive owner admission."""
import json
import os
from pathlib import Path
import threading
import subprocess
import time

import pytest
pytest.importorskip('rclpy')
from rclpy.client import Client
from rclpy.node import Node
from so101_station_control.recovery_camera_identity import RecoveryCameraIdentity
from test_ros_recovery_producer_native import test_native_ros_depth_reaches_actual_station_owner as run_recovery_fixture


def test_actual_rpc_renewal_during_proof_waits_passively_then_completes(tmp_path,monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID')=='75'
    marker=tmp_path/'external-owner/fake-proof-phase.json'
    managers=[];armed=threading.Event();pending=threading.Event();confirmations=[]
    original_init=RecoveryCameraIdentity.__init__
    def initialize(self,*args,**kwargs):
        original_init(self,*args,**kwargs);managers.append(self)
    monkeypatch.setattr(RecoveryCameraIdentity,'__init__',initialize)
    original_tick=RecoveryCameraIdentity._tick
    def tick(self):
        result=original_tick(self)
        value=self._identity
        if armed.is_set() and value is not None:
            record=dict(value['record'])
            if not confirmations or confirmations[-1]!=record:confirmations.append(record)
        return result
    monkeypatch.setattr(RecoveryCameraIdentity,'_tick',tick)
    original_service=Node.create_service
    def service(self,service_type,name,callback,*args,**kwargs):
        if name=='/camera/device_info':
            original_callback=callback
            def callback(request,response):
                if armed.is_set() and not pending.is_set():
                    pending.set()
                    until=time.monotonic()+.200
                    while not marker.is_file() and time.monotonic()<until:time.sleep(.001)
                    assert marker.is_file(),'owner never began its first proof'
                return original_callback(request,response)
        return original_service(self,service_type,name,callback,*args,**kwargs)
    monkeypatch.setattr(Node,'create_service',service)
    original_popen=subprocess.Popen
    def popen(arguments,*args,**kwargs):
        if isinstance(arguments,(list,tuple)) and any(str(value).endswith('/tests/fake_recovery_owner.py') for value in arguments):
            arguments=[*arguments,'--predispatch-proof-barrier']
        return original_popen(arguments,*args,**kwargs)
    monkeypatch.setattr(subprocess,'Popen',popen)
    original_call=Client.call_async
    forced={}
    def call(self,request):
        if self.srv_name=='/follower/recover_camera':
            assert not armed.is_set(),'no recovery request retry permitted'
            assert len(managers)==1
            manager=managers[0]
            snapshot=manager.snapshot()
            assert snapshot.get('valid') is True
            armed.set()
            manager._last_request=float('-inf')  # schedule ONE genuine normal timer query
            assert pending.wait(.200),'normal identity timer never issued renewal'
            actual=manager._pending
            assert actual is not None
            while time.monotonic()<actual['started']+.035:time.sleep(.001)
            forced.update(request_started_monotonic=actual['started'],
                original_identity=snapshot,parent_scheduled_delay_s=.035,
                command_sent_monotonic=time.monotonic())
        return original_call(self,request)
    monkeypatch.setattr(Client,'call_async',call)
    run_recovery_fixture(tmp_path,monkeypatch,'predispatch_renewal_required')
    audit=json.loads((tmp_path/'external-owner/fake-owner-write-audit.json').read_text())
    first_goal=audit['goal_write_observations'][0]
    renewal=next(value for value in confirmations if value['request_started_monotonic']==forced['request_started_monotonic'])
    evidence=os.environ.get('SO101_PRODUCER_TEST_EVIDENCE')
    if evidence:
        path=Path(evidence)
        path.with_name(path.stem+'-predispatch-coordination.json').write_text(json.dumps(dict(
            hardware_access=False,forced=forced,actual_renewal=renewal,
            passive_wait=audit.get('predispatch_observations'),proofs=audit['proof_observations'][:3],
            first_goal=first_goal),indent=2)+'\n')
    assert any(proof['check_started_monotonic']<=renewal['confirmed_monotonic']<=proof['check_finished_monotonic']
        and proof['check_finished_monotonic']<first_goal['started_monotonic'] for proof in audit['proof_observations'])
    waits=audit['predispatch_observations'];assert waits
    entered=next(item for item in waits if item['state']=='entered')
    exited=next(item for item in waits if item['state']=='exited')
    assert entered['goal_writes']==exited['goal_writes']==0
    assert entered['bus_write_count']==exited['bus_write_count']
    assert not any(entered['torque'].values()) and not any(exited['torque'].values())
    assert entered['raw']==exited['raw'] and exited['raw']['elbow_flex']==2993
    proof=next(item for item in reversed(audit['proof_observations'])
        if item['check_finished_monotonic']<=first_goal['started_monotonic'] and 'error' not in item)
    floor=max(exited['context_entered_monotonic'],renewal['confirmed_monotonic'])
    assert min(proof['scene_stamp_monotonic'],proof['original_declared_depth_stamp_monotonic'])>floor
    assert proof['camera_identity_confirmed_monotonic']>=renewal['confirmed_monotonic']
    assert first_goal['started_monotonic']<exited['context_deadline_monotonic']
    evidence=os.environ.get('SO101_PRODUCER_TEST_EVIDENCE')
    if evidence:
        path=Path(evidence)
        path.with_name(path.stem+'-predispatch-coordination.json').write_text(json.dumps(dict(
            hardware_access=False,forced=forced,actual_renewal=renewal,passive_wait=waits,
            first_goal=first_goal,final_raw=audit['final_raw']),indent=2)+'\n')
