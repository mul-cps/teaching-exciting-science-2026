"""One follower preparation reservation; in-memory owner, never hardware."""
import json
import copy
import threading
import time
import os
from pathlib import Path

import pytest
pytest.importorskip('rclpy')
from std_srvs.srv import Trigger
from test_ros_stop_service import bridge, snapshot, stop
from so101_station_control.trajectory_server import TELEOP_JOINTS


@pytest.fixture
def prepared_owner(bridge, tmp_path, monkeypatch):
    node, process = bridge
    # These are explicit fake acquisitions, not a continuous hardware reader.
    # Host scheduling must not age them before the intended injected boundary.
    # Replace ONLY this production module binding; Condition/DDS clocks remain
    # real and deliberate freshness faults advance this independent test clock.
    import so101_station_control.trajectory_server as server
    class MeasurementClock:
        value = time.monotonic()
        def monotonic(self): return self.value
        def advance(self, seconds): self.value += seconds
        def __getattr__(self, name): return getattr(time, name)
    clock = MeasurementClock()
    monkeypatch.setattr(server, 'time', clock)
    node._prepare_clock = clock
    node._owner_started = clock.monotonic()-.01
    node._owner_source_expected = {'arm': '1'*64, 'arm_session': '2'*64,
        'recovery_runtime': '3'*64, 'recovery_types': '4'*64,
        'recovery_runtime_config': '5'*64, 'camera_recovery': '6'*64}
    config = tmp_path/'station.json'
    config.write_text(json.dumps(dict(enabled=True, commissioned=True, automatic_start=True,
        arm=dict(id=node._expected_arm_id, calibration=dict(sha256=node._expected_calibration_sha256)))))
    node._prepare_config_path = config
    node._gripper_jaw = object()
    node._saved_home_pose = dict.fromkeys(TELEOP_JOINTS, 0.)
    node._saved_home_tolerance = 1.
    current = {}
    def sample(classification='normal', event='sample', torque=0):
        value = snapshot()
        value['monotonic'] = clock.monotonic()
        value.update(observe_only=False, event=event, torque_monotonic=value['monotonic'],
            owner_source_sha256=dict(node._owner_source_expected),
            torque=dict.fromkeys(TELEOP_JOINTS, torque))
        value['arm_bounds'] = dict(schema='arm-bounds-observation-v1',
            classification=classification, position_monotonic=value['monotonic'],
            admitted_min=dict.fromkeys(TELEOP_JOINTS, 100),
            admitted_max=dict.fromkeys(TELEOP_JOINTS, 3000),
            firmware_min=dict.fromkeys(TELEOP_JOINTS, 0),
            firmware_max=dict.fromkeys(TELEOP_JOINTS, 4095))
        return value
    current.update(sample())
    node._read_state = lambda **kwargs: copy.deepcopy(current)
    node._cache_owner_state(current)
    node._prepare_test_state=lambda:copy.deepcopy(current)
    node._prepare_test_commands=[]
    original = process.write
    def write(line):
        command = json.loads(line)
        node._prepare_test_commands.append(dict(command=command,monotonic=clock.monotonic(),
            phase=node._prepare_phase,position_monotonic=current.get('monotonic'),
            torque_monotonic=current.get('torque_monotonic')))
        if command['command'] == 'stop':
            process.commands.append(command)
            current.clear();current.update(sample(event='stopped'))
            node._route_stop_response(dict(ok=True, stopped=True,
                request_id=command['request_id'], state=copy.deepcopy(current)))
            return
        process.commands.append(command)
        assert node._active_goal in {'prepare', 'home', 'recover_camera'}
        event = {'home': 'home_complete', 'recover_camera': 'camera_recovery_complete'}[command['command']]
        current.clear();current.update(sample(event=event, torque=1))
        node._response_queue.put(dict(ok=True, state=copy.deepcopy(current)))
    process.write = write
    yield node, process, current, sample


def prepare(node):
    before=time.monotonic()
    original=node._prepare_test_state()
    response=node._prepare(Trigger.Request(), Trigger.Response())
    diagnostic=os.environ.get('SO101_PREPARE_TEST_DIAGNOSTICS')
    if not response.success and diagnostic:
        value=dict(test=os.environ.get('PYTEST_CURRENT_TEST'),before=before,after=time.monotonic(),
            original=original,after_state=node._prepare_test_state(),response_message=response.message,
            state_freshness_s=node.get_parameter('state_freshness_s').value,
            phase_after=node._prepare_phase,owner_started=node._owner_started,
            commands=node._prepare_test_commands)
        with Path(diagnostic).open('a') as stream:
            stream.write(json.dumps(value,default=str)+'\n')
    return response


def test_normal_pose_homes_once_and_ready_noop_does_not_send(prepared_owner):
    node, process, current, sample = prepared_owner
    result = prepare(node)
    assert result.success, result.message
    observation=json.loads(result.message)['observation']
    assert observation=={key:current[key] for key in ('arm_id','calibration_sha256',
        'owner_source_sha256','raw','torque','monotonic','torque_monotonic','arm_bounds')}
    assert process.commands == [{'command': 'home'}]
    assert node._home_completed_owner_started == node._owner_started
    assert node._active_goal is None
    result=prepare(node)
    assert result.success,result.message
    assert process.commands == [{'command': 'home'}]


def test_bounded_recovery_then_home_under_one_reservation(prepared_owner):
    node, process, current, sample = prepared_owner
    current.clear();current.update(sample('bounded_reentry_required'))
    node._recovery_cache_config = object()
    seen = []
    original = process.write
    def write(line):
        seen.append(node._active_goal)
        if json.loads(line)['command'] == 'home':
            assert node._home_completed_owner_started is None
            assert not node._home(Trigger.Request(), Trigger.Response()).success
            assert not node._recover_camera(Trigger.Request(), Trigger.Response()).success
        return original(line)
    process.write = write
    result = prepare(node)
    assert result.success, result.message
    assert process.commands == [{'command': 'recover_camera'}, {'command': 'home'}]
    assert seen == ['prepare', 'prepare']


@pytest.mark.parametrize('failure', ['unknown', 'unrecoverable', 'missing_bounds', 'stale',
    'source', 'observe', 'fault', 'busy', 'teleop', 'commission', 'stop'])
def test_pre_delivery_rejects_without_stop_or_owner_command(prepared_owner, failure):
    node, process, current, sample = prepared_owner
    if failure in {'unknown', 'unrecoverable'}:
        current['arm_bounds']['classification'] = failure
    if failure == 'missing_bounds':current.pop('arm_bounds')
    if failure == 'stale':current['monotonic'] -= 1
    if failure == 'source':current['owner_source_sha256'] = {}
    if failure == 'observe':current['observe_only'] = True
    if failure == 'fault':node._owner_faulted = True
    if failure == 'busy':node._active_goal = 'trajectory'
    if failure == 'teleop':node._teleop_state = 'active'
    if failure == 'commission':node._prepare_config_path.write_text('{}')
    if failure == 'stop':node._stop_latched = True
    result = prepare(node)
    assert not result.success
    assert process.commands == []


def test_failed_delivered_recovery_stops_and_never_homes(prepared_owner):
    node, process, current, sample = prepared_owner
    current.clear();current.update(sample('bounded_reentry_required'))
    node._recovery_cache_config = object()
    original = process.write
    def write(line):
        if json.loads(line)['command'] == 'recover_camera':
            process.commands.append({'command': 'recover_camera'})
            node._response_queue.put({'ok': False, 'error': 'scene unqualified'})
        else:original(line)
    process.write = write
    result = prepare(node)
    assert not result.success
    assert [c['command'] for c in process.commands] == ['recover_camera', 'stop']
    assert node._stop_latched and node._owner_faulted
    assert node._home_completed_owner_started is None
    assert node._active_goal is None
    assert stop(node).success  # only public quiescent stop clears the barrier


def test_public_stop_waits_for_prepare_and_keeps_its_correlation(prepared_owner):
    node, process, current, sample = prepared_owner
    entered = threading.Event()
    original = process.write
    def write(line):
        if json.loads(line)['command'] == 'home':
            process.commands.append({'command': 'home'})
            entered.set()
        else:original(line)
    process.write = write
    response = Trigger.Response()
    thread = threading.Thread(target=node._prepare,args=(Trigger.Request(),response))
    thread.start();assert entered.wait(1)
    stopped = stop(node)
    thread.join(1)
    assert not thread.is_alive() and not response.success
    assert stopped.success, stopped.message
    assert [c['command'] for c in process.commands] == ['home', 'stop']
    assert not node._stop_latched and node._active_goal is None


@pytest.mark.parametrize('stage',['recovery_reply','before_home','home_reply','before_ready'])
@pytest.mark.parametrize('fault',['stop','generation','source','identity','calibration','raw','clock'])
def test_each_phase_boundary_fails_closed_without_next_motion(prepared_owner,stage,fault):
    node,process,current,sample=prepared_owner
    current.clear();current.update(sample('bounded_reentry_required'))
    node._recovery_cache_config=object()
    hit=[]
    def inject(state):
        hit.append(True)
        if fault=='stop':node._stop_latched=True
        if fault=='generation':node._owner_started+=.001
        if fault=='source':state['owner_source_sha256']['arm']='b'*64
        if fault=='identity':state['arm_id']='other'
        if fault=='calibration':state['calibration_sha256']='b'*64
        if fault=='raw':state['raw']['gripper']+=1
        if fault=='clock':state['arm_bounds']['position_monotonic']-=.001
    write=process.write
    if stage.endswith('_reply'):
        command='recover_camera' if stage=='recovery_reply' else 'home'
        def intercept(line):
            write(line)
            if json.loads(line)['command']==command:
                response=node._response_queue.get_nowait()
                inject(response['state'])
                node._response_queue.put(response)
        process.write=intercept
    else:
        read=node._read_state
        def intercept(**kwargs):
            state=read(**kwargs)
            expected='camera_recovery_complete' if stage=='before_home' else 'home_complete'
            phase='recover_camera' if stage=='before_home' else 'readiness'
            if not hit and state['event']==expected and node._prepare_phase==phase:
                inject(state)
            return state
        node._read_state=intercept
    result=prepare(node)
    assert hit and not result.success,result.message
    commands=[c['command'] for c in process.commands]
    assert commands==(['recover_camera','stop'] if stage in {'recovery_reply','before_home'} else
                      ['recover_camera','home','stop'])
    assert node._home_completed_owner_started is None
    assert node._active_goal is None and node._stop_latched


@pytest.mark.parametrize('fault',['timestamp','bool_timestamp','unknown','missing_joint','unordered','noninteger'])
def test_malformed_owner_bounds_never_dispatches(prepared_owner,fault):
    node,process,current,sample=prepared_owner
    value=current['arm_bounds']
    if fault=='timestamp':value['position_monotonic']+=.01
    if fault=='bool_timestamp':value['position_monotonic']=True
    if fault=='unknown':value['classification']='unknown'
    if fault=='missing_joint':value['admitted_min'].pop('gripper')
    if fault=='unordered':value['admitted_max']['gripper']=1
    if fault=='noninteger':value['firmware_min']['gripper']=False
    assert not prepare(node).success
    assert process.commands==[]


def test_ready_noop_still_rejects_stale_source_and_missing_commission(prepared_owner):
    node,process,current,sample=prepared_owner
    result=prepare(node)
    assert result.success,result.message
    process.commands.clear()
    current['owner_source_sha256']={}
    assert not prepare(node).success and process.commands==[]
    current.clear();current.update(sample())
    node._prepare_config_path.write_text('{}')
    assert not prepare(node).success and process.commands==[]


def test_pending_public_stop_without_new_correlation_cannot_borrow_old_ack(prepared_owner):
    node,process,current,sample=prepared_owner
    entered=threading.Event()
    original=process.write
    def write(line):
        if json.loads(line)['command']=='home':
            process.commands.append({'command':'home'});entered.set()
        else:original(line)
    process.write=write
    prepared=Trigger.Response()
    worker=threading.Thread(target=node._prepare,args=(Trigger.Request(),prepared))
    worker.start();assert entered.wait(1)
    node._write_lock.acquire()
    node._stop_ack=dict(ok=True,stopped=True,request_id='old',state=sample())
    stopped=Trigger.Response()
    stopper=threading.Thread(target=node._stop,args=(Trigger.Request(),stopped))
    stopper.start()
    until=time.monotonic()+.1
    while not node._stop_latched and time.monotonic()<until:time.sleep(.001)
    assert node._stop_latched
    time.sleep(.025)
    assert worker.is_alive() and stopper.is_alive() and node._stop_pending is None
    node._write_lock.release()
    worker.join(1);stopper.join(1)
    assert not worker.is_alive() and not stopper.is_alive()
    assert stopped.success and not prepared.success
    assert [c['command'] for c in process.commands]==['home','stop']
    assert process.commands[-1]['request_id']!='old'
    assert node._control_callbacks==0 and not node._stop_latched


@pytest.mark.parametrize('case',['stale_torque','future_position','nan_position','source','observe','bounds','torque_off'])
def test_public_sorting_readiness_revalidates_actual_owner_observation(prepared_owner,case):
    node,process,current,sample=prepared_owner
    result=prepare(node)
    assert result.success,result.message
    assert node._sorting_ready(Trigger.Request(),Trigger.Response()).success
    if case=='stale_torque':current['torque_monotonic']=node._prepare_clock.monotonic()-1
    if case=='future_position':current['monotonic']=node._prepare_clock.monotonic()+1
    if case=='nan_position':current['monotonic']=float('nan')
    if case=='source':current['owner_source_sha256']={}
    if case=='observe':current['observe_only']=True
    if case=='bounds':current['arm_bounds']['classification']='bounded_reentry_required'
    if case=='torque_off':current['torque'][TELEOP_JOINTS[0]]=0
    assert not node._sorting_ready(Trigger.Request(),Trigger.Response()).success
    assert process.commands==[{'command':'home'}]


@pytest.mark.parametrize('flag,value,accepted',[('automatic_start',False,True),
    ('automatic_start',None,False),('automatic_start',1,False),
    ('enabled',False,False),('commissioned',False,False)])
def test_explicit_prepare_preserves_physical_admission_without_automation_release(prepared_owner,flag,value,accepted):
    node,process,current,sample=prepared_owner
    config=json.loads(node._prepare_config_path.read_text())
    if value is None:config.pop(flag)
    else:config[flag]=value
    node._prepare_config_path.write_text(json.dumps(config))
    result=prepare(node)
    assert result.success is accepted,result.message
    assert process.commands==([{'command':'home'}] if accepted else [])


def test_private_bounds_cache_helper_has_no_io_hash_or_owner_calls():
    import ast
    import inspect
    import textwrap
    from so101_station_control.trajectory_server import _arm_bounds_observation
    for helper in (_arm_bounds_observation,):
        tree=ast.parse(textwrap.dedent(inspect.getsource(helper)))
        calls={item.func.attr if isinstance(item.func,ast.Attribute) else item.func.id
            for item in ast.walk(tree) if isinstance(item,ast.Call)
            and isinstance(item.func,(ast.Attribute,ast.Name))}
        assert not calls.intersection({'_send','read_text','read_bytes','sha256','open','stat',
            '_read_state','_prepare_configuration','_load_owner_source_hashes','Popen'})


def test_direct_motion_and_reconnect_cannot_enter_prepare_reservation(prepared_owner):
    from std_srvs.srv import SetBool
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rclpy.action import GoalResponse
    from so101_station_control.trajectory_server import ARM_JOINTS
    node,process,current,sample=prepared_owner
    original=process.write
    def write(line):
        if json.loads(line)['command']=='home':
            assert not node._set_teleop_enabled(SetBool.Request(data=True),SetBool.Response()).success
            assert not node._reconnect(Trigger.Request(),Trigger.Response()).success
            assert not node._sorting_ready(Trigger.Request(),Trigger.Response()).success
            goal=FollowJointTrajectory.Goal()
            goal.trajectory.joint_names=list(ARM_JOINTS)
            goal.trajectory.points=[JointTrajectoryPoint(),JointTrajectoryPoint()]
            assert node._goal_request(goal)==GoalResponse.REJECT
            assert node._active_goal=='prepare'
        original(line)
    process.write=write
    result=prepare(node)
    assert result.success,result.message
    assert process.commands==[{'command':'home'}]


@pytest.mark.parametrize('config',[[],None,{'arm':[]},{'arm':None},
    {'arm':{'calibration':[]}},{'arm':{'calibration':None}}])
def test_malformed_configuration_returns_structured_pre_delivery_rejection(prepared_owner,config):
    node,process,current,sample=prepared_owner
    node._prepare_config_path.write_text(json.dumps(config))
    result=prepare(node)
    assert not result.success
    value=json.loads(result.message)
    assert value['state']=='rejected' and value['command_delivery']=='not_sent'
    assert value['phases']==[] and value['stop_verified'] is None
    assert process.commands==[]


@pytest.mark.parametrize('classification',[[],{},True,None])
def test_malformed_classification_is_unknown_on_cache_and_admission(prepared_owner,classification):
    from so101_station_control.trajectory_server import _arm_bounds_observation
    node,process,current,sample=prepared_owner
    current['arm_bounds']['classification']=classification
    assert _arm_bounds_observation(current) is None
    node._cache_owner_state(current)
    assert node._cached_owner_state['arm_bounds'] is None
    result=prepare(node)
    assert not result.success and process.commands==[]
    assert json.loads(result.message)['command_delivery']=='not_sent'



def test_host_scheduling_pause_does_not_expire_fake_admission(prepared_owner):
    node,process,current,sample=prepared_owner
    original=current['monotonic']
    time.sleep(.150)  # wall scheduling changes; the fake sensor has not ticked
    result=prepare(node)
    assert result.success,result.message
    assert current['monotonic']==original
    assert process.commands==[{'command':'home'}]


def test_explicit_virtual_sensor_expiry_still_rejects_before_dispatch(prepared_owner):
    node,process,current,sample=prepared_owner
    original=copy.deepcopy(current)
    node._prepare_clock.advance(.101)
    result=prepare(node)
    assert not result.success and 'fresh position/torque' in result.message
    assert process.commands==[] and current==original


def test_explicit_virtual_postrecovery_expiry_prevents_home(prepared_owner):
    node,process,current,sample=prepared_owner
    current.clear();current.update(sample('bounded_reentry_required'))
    node._recovery_cache_config=object();write=process.write
    def expire(line):
        write(line)
        if json.loads(line)['command']=='recover_camera':node._prepare_clock.advance(.101)
    process.write=expire
    result=prepare(node)
    assert not result.success and json.loads(result.message)['phases']==[]
    assert [c['command'] for c in process.commands]==['recover_camera','stop']
