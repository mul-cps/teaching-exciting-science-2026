"""Native typed failed-hold bridge admission and abort; fake owner only."""
import copy
import json
import math
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from control_msgs.action import FollowJointTrajectory
from test_ros_follower_recovery_status import ready,state
from test_ros_stop_service import bridge


def sample(node,*,held=False):
    value=state(node,owner_session_id='arm-session-fixed-test-owner')
    value['arm_bounds']=dict(schema='arm-bounds-observation-v1',classification='normal',
        position_monotonic=value['monotonic'],admitted_min=dict.fromkeys(value['raw'],1100),
        admitted_max=dict.fromkeys(value['raw'],2900),firmware_min=dict.fromkeys(value['raw'],1100),
        firmware_max=dict.fromkeys(value['raw'],2900))
    if held:value.update(event='trajectory_failed_endpoint_held',torque=dict.fromkeys(value['raw'],1))
    return value


def install(node,process,monkeypatch,transform=lambda reply:reply,*,artifact=None):
    node._read_state=lambda **kwargs:sample(node)
    monkeypatch.setattr(node,'_collision_check_path',lambda *args:{'effective_path_sha256':'c'*64})
    monkeypatch.setattr(node,'_feedback',lambda *args:True)
    artifacts=[];stops=[]
    monkeypatch.setattr(node,'_write_artifact',artifact or (lambda item:artifacts.append(dict(item))))
    monkeypatch.setattr(node,'_prepare_stop',lambda p,g:stops.append((p,g)) or True)
    def write(line):
        command=json.loads(line);process.commands.append(command)
        value=sample(node,held=True)
        hold=dict(schema='failed-endpoint-hold-v1',goal_id=command['goal_id'],
            owner_session_id=value['owner_session_id'],held_raw=dict(value['raw']),
            target_raw=dict(value['raw'],elbow_flex=1974),
            goal_error_rad={name:(.04 if name=='elbow_flex' else 0.) for name in value['pose'] if name!='gripper'},
            qualified=False,target_reached=False,settled_samples=0,lifetime='while_healthy_until_explicit_stop_or_transport_loss',
            entry_position_monotonic=value['monotonic'],entry_torque_monotonic=value['torque_monotonic'])
        value['failed_endpoint_hold']=copy.deepcopy(hold)
        reply=dict(ok=False,status='failed_endpoint_held',qualified=False,target_reached=False,
            goal_id=command['goal_id'],hold=hold,state=value)
        node._response_queue.put(transform(reply))
    process.write=write
    return artifacts,stops


class Goal:
    point=SimpleNamespace(time_from_start=SimpleNamespace(sec=1,nanosec=0),
        positions=[0.]*5,velocities=[],accelerations=[])
    request=SimpleNamespace(trajectory=SimpleNamespace(joint_names=
        ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll'],points=[point,point]))
    is_cancel_requested=False
    outcome=None
    def abort(self):self.outcome='aborted'
    def succeed(self):self.outcome='succeeded'
    def canceled(self):self.outcome='cancelled'


@pytest.mark.parametrize('cancel',[False,True])
def test_failed_hold_aborts_even_with_simultaneous_cancel_and_latches(ready,monkeypatch,cancel):
    node,process=ready;artifacts,stops=install(node,process,monkeypatch)
    goal=Goal();goal.is_cancel_requested=cancel
    result=node._execute(goal)
    assert result.error_code==FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
    assert result.error_string.startswith('failed_endpoint_held:')
    assert goal.outcome=='aborted' and node._owner_faulted
    assert node._home_completed_owner_started is None
    assert artifacts[-1]['target_reached'] is False and artifacts[-1]['qualified'] is False
    assert not stops and len(process.commands)==1


@pytest.mark.parametrize('failure',['goal','session','source','calibration','identity','raw','torque',
    'clock','generation','bounds','error','success','qualified','lifetime','status','nan'])
def test_malformed_or_stale_hold_requires_verified_stop(ready,monkeypatch,failure):
    node,process=ready
    def transform(reply):
        if failure=='goal':reply['goal_id']='other'
        if failure=='session':reply['state']['owner_session_id']='other'
        if failure=='source':reply['state']['owner_source_sha256']={}
        if failure=='calibration':reply['state']['calibration_sha256']='f'*64
        if failure=='identity':reply['state']['arm_id']='other'
        if failure=='raw':reply['hold']['held_raw']['gripper']+=5
        if failure=='torque':reply['state']['torque']['gripper']=0
        if failure=='clock':reply['state']['monotonic']-=1
        if failure=='generation':node._owner_started+=1
        if failure=='bounds':reply['state']['arm_bounds']['classification']='unknown'
        if failure=='error':reply['hold']['goal_error_rad']['elbow_flex']=.2
        if failure=='success':reply['ok']=True
        if failure=='qualified':reply['qualified']=True
        if failure=='lifetime':reply['hold']['lifetime']='forever'
        if failure=='status':reply['status']='succeeded'
        if failure=='nan':reply['extra']=float('nan')
        return reply
    artifacts,stops=install(node,process,monkeypatch,transform)
    goal=Goal();result=node._execute(goal)
    assert goal.outcome=='aborted' and result.error_code==-5
    assert node._owner_faulted and len(stops)==1
    assert not any(item.get('status')=='succeeded' for item in artifacts)


def test_explicit_stop_has_priority_over_failed_hold(ready,monkeypatch):
    node,process=ready
    def transform(reply):node._stop_latched=True;return reply
    artifacts,stops=install(node,process,monkeypatch,transform)
    goal=Goal();result=node._execute(goal)
    assert goal.outcome=='aborted' and 'owner stop interrupted' in result.error_string
    assert not stops and artifacts[-1]['status']=='stop_requested'


def test_last_in_tolerance_read_does_not_turn_unsettled_timeout_into_stop(ready,monkeypatch):
    node,process=ready
    def transform(reply):
        reply['hold']['goal_error_rad']['elbow_flex']=.03
        reply['hold']['settled_samples']=2
        reply['state']['failed_endpoint_hold']=copy.deepcopy(reply['hold'])
        return reply
    artifacts,stops=install(node,process,monkeypatch,transform)
    goal=Goal();result=node._execute(goal)
    assert result.error_code==-5 and result.error_string.startswith('failed_endpoint_held:')
    assert not stops and node._owner_faulted


def test_actual_owner_refreshed_cancel_proof_keeps_entry_anchor_and_aborts(ready,tmp_path,monkeypatch):
    import time as real_time
    import so101_station_control.trajectory_server as module
    from test_endpoint_failure_hold import plateau
    node,process=ready;session,arm,clock,owner_command=plateau(tmp_path)
    clock.value=real_time.monotonic()
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=clock.monotonic))
    node._owner_started=clock.value-.01
    node._expected_arm_id=arm.settings['id']
    node._expected_calibration_sha256=session.calibration_digest
    node._owner_source_expected=dict(session._owner_source_sha256)
    node._read_state=lambda **kwargs:session._latest_state()
    monkeypatch.setattr(node,'_collision_check_path',lambda *args:{'effective_path_sha256':owner_command['path_sha256']})
    monkeypatch.setattr(node,'_feedback',lambda *args:True)
    monkeypatch.setattr(node,'_write_artifact',lambda value:None)
    stops=[];proofs=[]
    monkeypatch.setattr(node,'_prepare_stop',lambda *args:stops.append(args))
    def write(line):
        request=json.loads(line);process.commands.append(request)
        first=session.handle(dict(owner_command,goal_id=request['goal_id']))
        entry=copy.deepcopy(first['hold'])
        arm.bus.stalled_joints.add('shoulder_pan');arm.bus.data['Present_Position']['shoulder_pan']+=4
        clock.sleep(.05)
        fresh=session.handle({'command':'cancel_hold'})
        assert fresh['hold']==entry and fresh['state']['raw']['shoulder_pan']!=entry['held_raw']['shoulder_pan']
        assert fresh['state']['monotonic']>entry['entry_position_monotonic']
        proofs.append(fresh);node._response_queue.put(fresh)
    process.write=write
    goal=Goal();goal.is_cancel_requested=True;result=node._execute(goal)
    assert proofs and result.error_code==-5 and result.error_string.startswith('failed_endpoint_held:')
    assert goal.outcome=='aborted' and node._owner_faulted and not stops
    assert set(arm.bus.data['Torque_Enable'].values())=={1}
