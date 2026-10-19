"""Native ROS service boundary with in-memory owner, no hardware."""
import copy
import hashlib
import json
from pathlib import Path
import time
import pytest
pytest.importorskip('rclpy')
from std_srvs.srv import Trigger
from test_ros_follower_recovery_status import ready,state
from test_ros_stop_service import bridge


def install(node,process,transform=lambda v:v):
    def sample():
        value=state(node,owner_session_id='session-1',failed_endpoint_hold=None,
            component_restore_pending=False,write_scope='normal',profile='unmodified',camera_recovery_phase=None,
            arm_bounds={'schema':'arm-bounds-observation-v1','classification':'normal',
                'position_monotonic':time.monotonic(),'admitted_min':dict.fromkeys(node._saved_home_pose,1000),
                'admitted_max':dict.fromkeys(node._saved_home_pose,3000),'firmware_min':dict.fromkeys(node._saved_home_pose,1000),
                'firmware_max':dict.fromkeys(node._saved_home_pose,3000)})
        value['arm_bounds']['position_monotonic']=value['monotonic']
        return value
    from so101_station_control.trajectory_server import TELEOP_JOINTS
    node._saved_home_pose=dict.fromkeys(TELEOP_JOINTS,0.);node._saved_home_tolerance=3.
    node._read_state=lambda **kwargs:sample()
    config=json.loads(Path(node._prepare_config_path).read_text());path=str(Path(config['arm']['home_file']).resolve())
    def write(line):
        command=json.loads(line);process.commands.append(command)
        now=time.monotonic()
        observation=dict(raw=command['expected_start_raw'],torque=dict.fromkeys(command['expected_start_raw'],0),
            monotonic=now,torque_monotonic=now,position_started_monotonic=now,torque_started_monotonic=now)
        value=dict(schema='resident-home-capture-v1',captured=True,qualified=False,calibration_authority=False,
            motion_authority=False,command_id=command['command_id'],owner_session_id=command['owner_session_id'],
            arm_id=node._expected_arm_id,calibration_sha256=node._expected_calibration_sha256,
            owner_source_sha256=dict(node._owner_source_expected),path=path,sha256='d'*64,
            previous_sha256='e'*64,previous_artifact=path+'.backup-fixture',acquisition=observation)
        node._response_queue.put(transform(dict(ok=True,command='capture_home',command_id=command['command_id'],capture=value)))
    process.write=write
    return sample


def call(node):return node._capture_home(Trigger.Request(),Trigger.Response())


def test_native_capture_reserves_exact_existing_owner_command_no_cache_promotion(ready):
    node,p=ready;install(node,p);old=copy.deepcopy(node._saved_home_pose)
    node._home_completed_owner_started=node._owner_started
    response=call(node);assert response.success,response.message
    assert len(p.commands)==1 and set(p.commands[0])=={'command','command_id','expected_start_raw','owner_session_id'}
    assert p.commands[0]['command']=='capture_home'
    assert node._saved_home_pose==old and node._home_completed_owner_started is None
    assert node._active_goal is None and not node._owner_faulted


@pytest.mark.parametrize('fault',['busy','faulted','hold','torque','stale','source','component','recovery'])
def test_capture_rejected_before_owner_dispatch(ready,fault):
    node,p=ready;sample=install(node,p)
    if fault=='busy':node._active_goal='trajectory'
    elif fault=='faulted':node._owner_faulted=True
    else:
        value=sample()
        if fault=='hold':value['failed_endpoint_hold']={'held':True}
        if fault=='torque':value['torque']['elbow_flex']=1
        if fault=='stale':value['torque_monotonic']-=1
        if fault=='source':value['owner_source_sha256']={}
        if fault=='component':value['component_restore_pending']=True
        if fault=='recovery':value['camera_recovery_phase']='stationary_recapture'
        node._read_state=lambda **kwargs:value
    assert not call(node).success and not p.commands


@pytest.mark.parametrize('fault',['command_id','session','late','raw','path','source','unknown'])
def test_untrusted_or_late_reply_latches_without_implicit_motor_stop(ready,fault):
    node,p=ready
    def change(value):
        cap=value['capture']
        if fault=='command_id':value['command_id']='0'*32
        if fault=='session':cap['owner_session_id']='other'
        if fault=='late':cap['acquisition']['torque_monotonic']-=1
        if fault=='raw':cap['acquisition']['raw']=dict(cap['acquisition']['raw'],elbow_flex=0)
        if fault=='path':cap['path']='/tmp/arbitrary'
        if fault=='source':cap['owner_source_sha256']={}
        if fault=='unknown':raise TimeoutError('unknown capture result')
        return value
    install(node,p,change)
    assert not call(node).success and node._owner_faulted
    assert len(p.commands)==1 and p.commands[0]['command']=='capture_home'
    assert node._active_goal is None


def test_capture_service_registered_and_quick_action_present(ready):
    from tools.ros_station_action import parse_args
    node,p=ready
    assert node._capture_home_service.srv_name=='/follower/capture_home'
    assert node._capture_home_service.callback_group is node._teleop_service_group
    assert parse_args(['arm-capture-home']).command=='arm-capture-home'


def test_explicit_stop_interrupts_capture_and_keeps_correlated_alloff_barrier(ready):
    import threading
    from test_ros_stop_service import stop,snapshot
    node,p=ready;sample=install(node,p);entered=threading.Event()
    original=p.write
    def write(line):
        command=json.loads(line)
        if command['command']=='capture_home':p.commands.append(command);entered.set()
        else:original_stop(line)
    def original_stop(line):
        command=json.loads(line);p.commands.append(command)
        if command['command']=='stop':
            node._route_stop_response(dict(ok=True,stopped=True,request_id=command['request_id'],state=snapshot()))
    p.write=write
    result=Trigger.Response();worker=threading.Thread(target=node._capture_home,args=(Trigger.Request(),result))
    worker.start();assert entered.wait(1.)
    stopped=stop(node);worker.join(1.)
    assert stopped.success and not worker.is_alive() and not result.success
    assert [c['command'] for c in p.commands]==['capture_home','stop']
    assert node._active_goal is None and not node._owner_faulted and not node._stop_latched


def test_real_dds_capture_request_uses_resident_owner_only(ready):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    import uuid
    node,p=ready;install(node,p)
    caller=Node('capture_caller_'+uuid.uuid4().hex);executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node);executor.add_node(caller)
    try:
        client=caller.create_client(Trigger,'/follower/capture_home')
        assert client.wait_for_service(timeout_sec=1.)
        future=client.call_async(Trigger.Request())
        until=time.monotonic()+2.
        while not future.done() and time.monotonic()<until:executor.spin_once(timeout_sec=.01)
        assert future.done() and future.result().success
        assert len(p.commands)==1 and p.commands[0]['command']=='capture_home'
    finally:
        executor.shutdown(timeout_sec=1.);executor.remove_node(node);executor.remove_node(caller);caller.destroy_node()


def test_acquisition_clock_order_is_not_renewed_or_repaired(ready):
    node,p=ready
    def change(value):
        value['capture']['acquisition']['position_started_monotonic']+=.001
        return value
    install(node,p,change)
    assert not call(node).success and node._owner_faulted
    assert len(p.commands)==1


def test_station_capture_forwarding_is_typed_and_no_sdk(tmp_path):
    import ast
    from tools.ros_station_action import parse_args
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'tools/ros_station_api.py').read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='arm_capture_home')
    source=ast.unparse(fn)
    assert '/follower/capture_home' in source and "worker_task('arm_capture_home'" in source
    assert not any(token in source for token in ('SOFollower','LeRobotArm','connect(',"'stop'",'.bus'))
    assert parse_args(['arm-capture-home']).command=='arm-capture-home'


def test_absolute_response_deadline_rechecked_after_fresh_reply_access(ready,monkeypatch):
    import types
    import so101_station_control.trajectory_server as module
    node,p=ready;install(node,p)
    clock=types.SimpleNamespace(offset=0.)
    actual_time=module.time
    class TimeProxy:
        def monotonic(self):return actual_time.monotonic()+clock.offset
        def __getattr__(self,key):return getattr(actual_time,key)
    monkeypatch.setattr(module,'time',TimeProxy())
    reserved=node._reserved_owner_reply
    def slow_access(*args,**kwargs):
        reply=reserved(*args,**kwargs);clock.offset=2.1
        now=module.time.monotonic()
        for key in ('monotonic','torque_monotonic','position_started_monotonic','torque_started_monotonic'):
            reply['capture']['acquisition'][key]=now
        return reply
    monkeypatch.setattr(node,'_reserved_owner_reply',slow_access)
    response=call(node)
    assert not response.success and node._owner_faulted
    assert len(p.commands)==1 and p.commands[0]['command']=='capture_home'


def test_original_acquisition_freshness_rechecked_after_json_materialization(ready,monkeypatch):
    import types
    import so101_station_control.trajectory_server as module
    node,p=ready;install(node,p)
    offset=types.SimpleNamespace(value=0.)
    actual_time,actual_json=module.time,module.json
    class TimeProxy:
        def monotonic(self):return actual_time.monotonic()+offset.value
        def __getattr__(self,key):return getattr(actual_time,key)
    class JSONProxy:
        def dumps(self,value,**kwargs):
            text=actual_json.dumps(value,**kwargs)
            if isinstance(value,dict) and value.get('captured') is True:offset.value=.15
            return text
        def __getattr__(self,key):return getattr(actual_json,key)
    monkeypatch.setattr(module,'time',TimeProxy());monkeypatch.setattr(module,'json',JSONProxy())
    response=call(node)
    assert not response.success and node._owner_faulted
    assert json.loads(response.message)['result_unknown'] is True
    assert len(p.commands)==1 and p.commands[0]['command']=='capture_home'
