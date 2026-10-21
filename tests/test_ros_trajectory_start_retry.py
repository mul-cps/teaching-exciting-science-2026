"""Fake ROS owner: receipt-gated full collision revalidation, without hardware."""
import copy
import time
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from test_ros_stop_service import bridge
from so101_station_control.trajectory_server import TELEOP_JOINTS


@pytest.fixture
def retry(bridge, monkeypatch):
    node, process = bridge
    node._owner_started = time.monotonic() - 1.
    node._owner_source_expected = {'arm_session': 'a'*64}
    goal = SimpleNamespace(is_cancel_requested=False)
    node._active_goal = goal
    checked = dict.fromkeys(TELEOP_JOINTS, 2000)
    actual = dict(checked, wrist_flex=2001)
    command = dict(command='execute_trajectory', goal_id='goal', request_id='b'*32,
                   checked_start_raw=checked, path_sha256='c'*64, points=[])
    stamp = time.monotonic() - .001
    dispatched = stamp - .001
    session = 'arm-session-'+'d'*32
    proof = dict(schema='trajectory-start-rejection-v1', no_actuation=True,
                 request_id=command['request_id'], goal_id='goal', owner_session_id=session,
                 arm_id=node._expected_arm_id, calibration_sha256=node._expected_calibration_sha256,
                 owner_source_sha256=dict(node._owner_source_expected),
                 checked_path_sha256='c'*64, actual_path_sha256='e'*64,
                 checked_start_raw=checked, actual_start_raw=actual, monotonic=stamp)
    reply = dict(ok=False, command='execute_trajectory', error_type='ArmPlanPreflightError',
                 error='measured start or effective spline differs from collision-checked path',
                 trajectory_start_rejection=proof)
    def state():
        now=time.monotonic()
        return dict(monotonic=now, torque_monotonic=now, raw=dict(actual),
                    pose=dict.fromkeys(TELEOP_JOINTS,0.), torque=dict.fromkeys(TELEOP_JOINTS,1),
                    arm_id=node._expected_arm_id, calibration_sha256=node._expected_calibration_sha256,
                    owner_source_sha256=dict(node._owner_source_expected), owner_session_id=session,
                    observe_only=False, fault_observation=None,
                    arm_bounds=dict(schema='arm-bounds-observation-v1', classification='normal',
                                    position_monotonic=now, admitted_min=dict.fromkeys(TELEOP_JOINTS,1000),
                                    admitted_max=dict.fromkeys(TELEOP_JOINTS,3000),
                                    firmware_min=dict.fromkeys(TELEOP_JOINTS,1000),
                                    firmware_max=dict.fromkeys(TELEOP_JOINTS,3000)))
    monkeypatch.setattr(node,'_read_state',lambda **kwargs: state())
    calls=[]
    def collision(trajectory,points,goal_id,**kwargs):
        calls.append(kwargs)
        return dict(command_points=[dict(time=0.,positions=[0.]*5),dict(time=1.,positions=[.001]*5)],
                    start_state=dict(raw=kwargs['state']['raw']), effective_path_sha256='f'*64)
    monkeypatch.setattr(node,'_collision_check_path',collision)
    monkeypatch.setattr(node,'_write_artifact',lambda artifact: None)
    sent=[]
    monkeypatch.setattr(node,'_send',sent.append)
    arguments=dict(goal_handle=goal,trajectory=SimpleNamespace(points=[]),
                   artifact=dict(controller_action_goal_uuid=[1]*16),process=process,
                   generation=node._owner_started,owner_session_id=session,
                   dispatched_at=dispatched,deadline=time.monotonic()+1.)
    return node,reply,command,arguments,calls,sent


def test_jitter_receipt_rechecks_from_advancing_measured_cohort(retry):
    node,reply,command,args,calls,sent=retry
    artifact,new_command,feedback,sent_at=node._retry_trajectory_start(reply,command,**args)
    assert len(calls)==len(sent)==1
    assert calls[0]['deadline']==args['deadline']
    assert calls[0]['state']['monotonic']>reply['trajectory_start_rejection']['monotonic']
    assert new_command['request_id'] != command['request_id']
    assert new_command['path_sha256']=='f'*64 and new_command['path_sha256']!=command['path_sha256']
    assert new_command['checked_start_raw']==reply['trajectory_start_rejection']['actual_start_raw']
    assert len(artifact['preflight_rejections'])==1 and len(feedback.points)==2
    assert node._active_goal is args['goal_handle']


@pytest.mark.parametrize('failure', ['missing','no_actuation','request','goal','session','source','calibration',
    'path','checked','unchanged_raw','large_jitter','actual_hash','future','before_dispatch','generic','unknown_ack'])
def test_unproven_or_nonjitter_rejection_never_retries(retry,failure):
    node,reply,command,args,calls,sent=retry
    proof=reply['trajectory_start_rejection']
    if failure=='missing': reply.pop('trajectory_start_rejection')
    if failure=='no_actuation': proof['no_actuation']=False
    if failure=='request': proof['request_id']='0'*32
    if failure=='goal': proof['goal_id']='old'
    if failure=='session': proof['owner_session_id']='old'
    if failure=='source': proof['owner_source_sha256']={}
    if failure=='calibration': proof['calibration_sha256']='0'*64
    if failure=='path': proof['checked_path_sha256']='0'*64
    if failure=='checked': proof['checked_start_raw']=dict(proof['checked_start_raw'],wrist_roll=1999)
    if failure=='unchanged_raw': proof['actual_start_raw']=dict(proof['checked_start_raw'])
    if failure=='large_jitter': proof['actual_start_raw']['wrist_flex']+=2
    if failure=='actual_hash': proof['actual_path_sha256']=command['path_sha256']
    if failure=='future': proof['monotonic']=time.monotonic()+1
    if failure=='before_dispatch': proof['monotonic']=args['dispatched_at']-1
    if failure=='generic': reply['error']='other preflight failure'
    if failure=='unknown_ack': reply=dict(ok=False,error_type='TimeoutError')
    assert node._retry_trajectory_start(reply,command,**args) is None
    assert not calls and not sent


@pytest.mark.parametrize('failure',['stop','cancel','owner','generation','deadline','native_fault','nonadvancing'])
def test_changed_lifecycle_or_missing_new_cohort_never_retries(retry,monkeypatch,failure):
    node,reply,command,args,calls,sent=retry
    if failure=='stop': node._stop_latched=True
    if failure=='cancel': args['goal_handle'].is_cancel_requested=True
    if failure=='owner': node._process=SimpleNamespace(poll=lambda: None)
    if failure=='generation': node._owner_started+=.1
    if failure=='deadline': args['deadline']=time.monotonic()-1
    if failure in {'native_fault','nonadvancing'}:
        old=node._read_state
        def state(**kwargs):
            result=old()
            if failure=='native_fault': result['fault_observation']={}
            else: result['monotonic']=reply['trajectory_start_rejection']['monotonic']
            return result
        monkeypatch.setattr(node,'_read_state',state)
    with pytest.raises((RuntimeError,TimeoutError)):
        node._retry_trajectory_start(reply,command,**args)
    assert not calls and not sent


def test_stop_during_collision_check_prevents_retry_dispatch(retry,monkeypatch):
    node,reply,command,args,calls,sent=retry
    collision=node._collision_check_path
    def stop(*a,**kw):
        result=collision(*a,**kw)
        node._stop_latched=True
        return result
    monkeypatch.setattr(node,'_collision_check_path',stop)
    with pytest.raises(RuntimeError): node._retry_trajectory_start(reply,command,**args)
    assert len(calls)==1 and not sent


@pytest.mark.parametrize('terminal,expected_sends', [('success',3),('exhausted',3),('unknown',1)])
def test_action_retries_at_most_twice_and_keeps_one_action(retry,monkeypatch,terminal,expected_sends):
    node,reply,command,args,calls,sent=retry
    names=list(TELEOP_JOINTS[:5])
    points=[SimpleNamespace(time_from_start=SimpleNamespace(sec=t,nanosec=0),
                            positions=[.001*t]*5,velocities=[],accelerations=[]) for t in (0,1)]
    trajectory=SimpleNamespace(joint_names=names,points=points)
    outcome=[]
    goal=SimpleNamespace(request=SimpleNamespace(trajectory=trajectory),
        goal_id=SimpleNamespace(uuid=[1]*16),is_cancel_requested=False,
        succeed=lambda: outcome.append('success'),abort=lambda: outcome.append('aborted'),
        canceled=lambda: outcome.append('cancelled'))
    raw=dict(command['checked_start_raw'])
    previous_read=node._read_state
    def state(**kwargs):
        value=previous_read()
        value['raw']=dict(raw)
        return value
    monkeypatch.setattr(node,'_read_state',state)
    def collision(_trajectory,_points,goal_id,**kwargs):
        frozen=kwargs.get('state') or state()
        calls.append(kwargs)
        return dict(command_points=[dict(time=0.,positions=[0.]*5),dict(time=1.,positions=[.001]*5)],
                    start_state=dict(raw=dict(frozen['raw'])),effective_path_sha256=str(len(calls)%10)*64,
                    retiming_factor=1.,ros_motion_profile='default')
    monkeypatch.setattr(node,'_collision_check_path',collision)
    monkeypatch.setattr(node,'_feedback',lambda *a: True)
    def send(payload):
        sent.append(payload)
        if terminal=='unknown':
            node._response_queue.put(dict(ok=False,error_type='TimeoutError',error='unknown acknowledgement'))
        elif terminal=='success' and len(sent)==3:
            node._response_queue.put(dict(ok=True))
        else:
            raw['wrist_flex']+=1
            proof=copy.deepcopy(reply['trajectory_start_rejection'])
            proof.update(request_id=payload['request_id'],goal_id=payload['goal_id'],
                         checked_start_raw=dict(payload['checked_start_raw']),actual_start_raw=dict(raw),
                         checked_path_sha256=payload['path_sha256'],actual_path_sha256='e'*64,
                         monotonic=time.monotonic())
            node._response_queue.put(dict(reply,trajectory_start_rejection=proof))
    monkeypatch.setattr(node,'_send',send)
    node._execute(goal)
    assert len(sent)==expected_sends
    assert len(calls)==expected_sends
    assert len({item['goal_id'] for item in sent})==1
    assert len({item['request_id'] for item in sent})==len(sent)
    assert outcome==(['success'] if terminal=='success' else ['aborted'])


def test_exact_two_count_jitter_is_admitted(retry):
    node,reply,command,args,calls,sent=retry
    reply['trajectory_start_rejection']['actual_start_raw']['wrist_flex']+=1
    assert node._retry_trajectory_start(reply,command,**args) is not None
    assert len(calls)==len(sent)==1


@pytest.mark.parametrize('failure',['stop','deadline'])
def test_stop_or_deadline_during_artifact_write_prevents_dispatch(retry,monkeypatch,failure):
    node,reply,command,args,calls,sent=retry
    def artifact_io(_artifact):
        if failure=='stop': node._stop_latched=True
        else:
            # The same absolute deadline must still apply after durable I/O.
            time.sleep(max(0.,args['deadline']-time.monotonic())+.001)
    if failure=='deadline': args['deadline']=time.monotonic()+.03
    monkeypatch.setattr(node,'_write_artifact',artifact_io)
    with pytest.raises(RuntimeError): node._retry_trajectory_start(reply,command,**args)
    assert len(calls)==1 and not sent


@pytest.mark.parametrize('fresh_delta,accepted', [(2,True),(3,False)])
def test_advancing_cohort_must_still_be_small_encoder_jitter(retry,monkeypatch,fresh_delta,accepted):
    node,reply,command,args,calls,sent=retry
    previous=node._read_state
    def state(**kwargs):
        value=previous()
        value['raw']['wrist_flex']=command['checked_start_raw']['wrist_flex']+fresh_delta
        return value
    monkeypatch.setattr(node,'_read_state',state)
    if accepted:
        assert node._retry_trajectory_start(reply,command,**args) is not None
    else:
        with pytest.raises(RuntimeError,match='encoder jitter'):
            node._retry_trajectory_start(reply,command,**args)
    assert len(calls)==len(sent)==int(accepted)


@pytest.mark.parametrize('phase',['collision','artifact'])
def test_larger_drift_during_retry_work_never_dispatches(retry,monkeypatch,phase):
    node,reply,command,args,calls,sent=retry
    changed=False
    previous=node._read_state
    def state(**kwargs):
        value=previous()
        if changed: value['raw']['wrist_flex']=command['checked_start_raw']['wrist_flex']+3
        return value
    monkeypatch.setattr(node,'_read_state',state)
    if phase=='collision':
        collision=node._collision_check_path
        def work(*a,**kw):
            nonlocal changed
            result=collision(*a,**kw)
            changed=True
            return result
        monkeypatch.setattr(node,'_collision_check_path',work)
    else:
        def work(_artifact):
            nonlocal changed
            changed=True
        monkeypatch.setattr(node,'_write_artifact',work)
    with pytest.raises(RuntimeError,match='encoder jitter'):
        node._retry_trajectory_start(reply,command,**args)
    assert len(calls)==1 and not sent
