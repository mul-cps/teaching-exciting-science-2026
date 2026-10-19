"""Native service and real fake-bus owner share typed held-jaw terminal proofs."""
import copy,json,time
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from so101_station_workflow.srv import CommissionGripperStep
from test_ros_stop_service import bridge
from test_held_jaw import held


def setup(bridge,tmp_path,monkeypatch,*,blocked=False,collision=True,transform=lambda x:x):
    node,process=bridge
    s,b,c,cmd=held(tmp_path);c.value=time.monotonic();s.publish_state(event='sample')
    import so101_station_control.trajectory_server as module
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=c.monotonic))
    node.set_parameters([Parameter('calibration_endpoint_mode',value=True)])
    node._owner_started=c.value-.01;node._expected_arm_id=s.arm.settings['id']
    node._expected_calibration_sha256=s.arm.settings['calibration']['sha256']
    node._owner_source_expected=dict(s._owner_source_sha256);node._gripper_raw_limits=(1000,3000)
    node._read_state=s._latest_state;checks=[];stops=[]
    node._state_validity=SimpleNamespace(wait_for_service=lambda **kw:True)
    monkeypatch.setattr(node,'_get_validity',lambda p,j,d:checks.append((p,j)) or SimpleNamespace(valid=collision))
    monkeypatch.setattr(node,'_prepare_stop',lambda p,g:stops.append((p,g)) or True)
    b.blocked_gripper=blocked
    def write(line):
        value=json.loads(line);process.commands.append(value)
        node._response_queue.put(transform(s.handle(value)))
    process.write=write
    return node,process,s,b,c,checks,stops


def call(node):
    return node._commission_gripper_held_step(CommissionGripperStep.Request(delta_counts=8),CommissionGripperStep.Response())


@pytest.mark.parametrize('blocked',[False,True])
def test_service_accepts_actual_owner_terminal_without_stop(bridge,tmp_path,monkeypatch,blocked):
    node,p,s,b,c,checks,stops=setup(bridge,tmp_path,monkeypatch,blocked=blocked)
    response=call(node)
    assert response.success is (not blocked),response.message
    payload=json.loads(response.message);assert payload['qualified'] is False
    assert len(checks)==9 and len(p.commands)==1 and not stops
    assert set(b.data['Torque_Enable'].values())=={1}
    assert all(j=='gripper' for r,j,v in b.writes) and not b.goal_history
    assert node._owner_faulted is blocked
    assert node._active_goal is None
    if blocked:assert payload['result']['status']=='failed_held_jaw'


@pytest.mark.parametrize('fault',['mode','collision','mixed','alloff','bounds','session','stale','rawlimits','busy','teleop'])
def test_service_rejects_before_dispatch(bridge,tmp_path,monkeypatch,fault):
    node,p,s,b,c,checks,stops=setup(bridge,tmp_path,monkeypatch,collision=fault!='collision')
    if fault=='mode':node.set_parameters([Parameter('calibration_endpoint_mode',value=False)])
    if fault in {'mixed','alloff'}:
        b.data['Torque_Enable']['gripper']=0
        if fault=='alloff':b.data['Torque_Enable']=dict.fromkeys(b.data['Torque_Enable'],0)
        s.publish_state(event='sample')
    if fault=='bounds':
        bad=s._latest_state();bad['arm_bounds']['classification']='bounded_reentry_required';node._read_state=lambda:bad
    if fault=='session':
        bad=s._latest_state();bad['owner_session_id']=None;node._read_state=lambda:bad
    if fault=='stale':c.value+=.2
    if fault=='rawlimits':node._gripper_raw_limits=None
    if fault=='busy':node._active_goal='other'
    if fault=='teleop':node._teleop_state='active'
    response=call(node);assert not response.success
    assert not p.commands and not b.writes and not stops


@pytest.mark.parametrize('field',['source','identity','command','raw','goalclock','profile','schema','anchor','success','nan','profile_type','body_goals','target'])
def test_bad_actual_terminal_proof_demands_stop(bridge,tmp_path,monkeypatch,field):
    def corrupt(reply):
        x=copy.deepcopy(reply)
        if field=='source':x['state']['owner_source_sha256']={}
        if field=='identity':x['state']['arm_id']='other'
        if field=='command':x['state']['component_command']['command_id']='other'
        if field=='raw':x['state']['raw']['shoulder_pan']+=3
        if field=='goalclock':x['state']['component_command']['goal_monotonic']-=1
        if field=='profile':x['state']['component_command']['restored_jaw_ram']['Torque_Limit']+=1
        if field=='schema':x['hold']['schema']='failed-endpoint-hold-v1'
        if field=='anchor':x['hold']['body_anchor_raw']['shoulder_pan']+=1
        if field=='success':x['ok']=True
        if field=='nan':x['extra']=float('nan')
        if field=='profile_type':x['state']['component_command']['entry_jaw_ram']={'Acceleration':True,'Goal_Velocity':220,'Torque_Limit':250};x['state']['component_command']['restored_jaw_ram']=dict(x['state']['component_command']['entry_jaw_ram'])
        if field=='body_goals':x['state']['component_command']['preserved_body_goals']={}
        if field=='target':x['hold']['target_raw']['gripper']+=1
        return x
    node,p,s,b,c,checks,stops=setup(bridge,tmp_path,monkeypatch,blocked=True,transform=corrupt)
    response=call(node);assert not response.success and node._owner_faulted
    assert len(stops)==1 and len(p.commands)==1


def test_service_accepts_actual_post_install_cancel_proof(bridge,tmp_path,monkeypatch):
    node,p,s,b,c,checks,stops=setup(bridge,tmp_path,monkeypatch,blocked=True)
    record=s._record
    def terminal(value):
        record(value)
        if value.get('event')=='commission_gripper_held_failed':
            b.stalled_joints.add('shoulder_pan');b.data['Present_Position']['shoulder_pan']+=2
            c.sleep(.01)
            s._cancel_requested=True;s._cancel_reason='trajectory_cancel_hold'
    s._record=terminal
    response=call(node)
    assert response.success is False and node._owner_faulted,response.message
    assert json.loads(response.message)['result']['state']['event']=='read_state'
    assert not stops and set(b.data['Torque_Enable'].values())=={1}


def test_collision_deadline_cannot_be_refreshed_by_fresh_owner(bridge,tmp_path,monkeypatch):
    node,p,s,b,c,checks,stops=setup(bridge,tmp_path,monkeypatch)
    node._read_state=lambda:s.publish_state(event='sample')
    def delayed(positions,jaw,deadline):
        c.sleep(30.01)
        return SimpleNamespace(valid=True)
    monkeypatch.setattr(node,'_get_validity',delayed)
    response=call(node)
    assert response.success is False
    assert not p.commands and not b.writes


def test_collision_preflight_deadline_checked_again_immediately_before_send(bridge,tmp_path,monkeypatch):
    node,p,s,b,c,checks,stops=setup(bridge,tmp_path,monkeypatch)
    original=node._prepare_owner_state;calls=0
    def delayed(*args,**kwargs):
        nonlocal calls
        result=original(*args,**kwargs);calls+=1
        if calls==2:c.sleep(30.01)
        return result
    monkeypatch.setattr(node,'_prepare_owner_state',delayed)
    response=call(node)
    assert not response.success and not p.commands and not b.writes
