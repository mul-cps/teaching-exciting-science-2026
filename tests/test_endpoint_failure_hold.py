"""Stable normal endpoint failure holds measured state without claiming arrival."""
import math
import pytest
from test_ros_elbow_bounds import owner_at_actual_elbow
from workshop.station.ros_trajectory import (ARM_JOINTS, ROS_TRAJECTORY_PERIOD_S,
                                             prepare_trajectory, path_digest)


def plateau(tmp_path):
    session,arm,clock=owner_at_actual_elbow(tmp_path)
    arm.bus.data['Present_Position']['elbow_flex']=2540
    arm.bus.data['Goal_Position']['elbow_flex']=2540
    session.initialize()
    arm.bus.stalled_joints.add('elbow_flex')
    def physical_plateau():
        current=arm.bus.data['Present_Position']['elbow_flex']
        goal=arm.bus.data['Goal_Position']['elbow_flex']
        arm.bus.data['Present_Position']['elbow_flex']=min(current,max(2517,goal+23))
    arm.bus.on_present_read=physical_plateau
    start=session._arm_radians(arm.bus.data['Present_Position'])
    end=dict(start,elbow_flex=(2493.889298102783-1951.5)*2*math.pi/4095)
    points=[{'time':0.,'positions':list(start.values())},
            {'time':8.8,'positions':list(end.values())}]
    prepared=prepare_trajectory(ARM_JOINTS,points,start_radians=start,
        max_velocity=.125,max_acceleration=.2,max_jerk=2.)
    command=dict(command='execute_trajectory',goal_id='hardware-plateau',
        joint_names=list(ARM_JOINTS),points=points,
        ros_motion_profile=session._ros_motion_profile.name,
        path_sha256=path_digest(prepared,held_gripper_raw=2000))
    return session,arm,clock,command


def test_actual_23_count_plateau_is_failed_held_not_success_or_disarm(tmp_path):
    session,arm,clock,command=plateau(tmp_path)
    result=session.handle(command)
    assert result['ok'] is False
    assert result['status']=='failed_endpoint_held'
    assert result['target_reached'] is False and result['qualified'] is False
    assert result['goal_id']==command['goal_id']
    assert result['state']['raw']['elbow_flex']==2517
    assert result['hold']['held_raw']==result['state']['raw']
    assert result['hold']['owner_session_id']==result['state']['owner_session_id']
    assert arm.bus.data['Goal_Position']==result['state']['raw']
    assert set(arm.bus.data['Torque_Enable'].values())=={1}
    assert session.saved_ram is not None


@pytest.mark.parametrize('name',['move_plan','move_gripper','commission_gripper_step',
    'recover_camera','home','execute_trajectory','teleop_start','teleop_step','teleop_stop'])
def test_owner_local_fault_gate_requires_explicit_stop(tmp_path,name):
    from workshop.station.arm_session import ArmPlanPreflightError
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    anchor=dict(session._failed_endpoint_hold['held_raw']);goals=len(arm.bus.goal_history)
    with pytest.raises(ArmPlanPreflightError,match='verified all-off STOP'):
        session.handle({'command':name})
    assert len(arm.bus.goal_history)==goals and session._failed_endpoint_hold['held_raw']==anchor
    session.handle({'command':'stop'})
    assert session._failed_endpoint_hold is None and session.saved_ram is None
    assert set(arm.bus.data['Torque_Enable'].values())=={0}


def test_cancel_cannot_rebase_failed_anchor(tmp_path):
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    anchor=dict(session._failed_endpoint_hold['held_raw']);goals=len(arm.bus.goal_history)
    arm.bus.stalled_joints.add('shoulder_pan')
    arm.bus.data['Present_Position']['shoulder_pan']+=4
    reply=session.handle({'command':'cancel_hold'})
    assert reply['ok'] is False and reply['status']=='failed_endpoint_held'
    assert session._failed_endpoint_hold['held_raw']==anchor
    assert len(arm.bus.goal_history)==goals
    arm.bus.data['Present_Position']['shoulder_pan']+=1
    with pytest.raises(RuntimeError,match='held axis drift'):
        session.handle({'command':'cancel_hold'})


def serve_held(session,arm,clock,*,fault=None):
    import os,select,json
    read_fd,write_fd=os.pipe();tick=0
    def wait(readers,writers,errors,timeout):
        nonlocal tick
        tick+=1;clock.sleep(.11 if fault=='loop' and tick==3 else timeout)
        if tick==3:
            if fault=='drift':
                arm.bus.stalled_joints.add('shoulder_pan')
                arm.bus.data['Present_Position']['shoulder_pan']+=5
            if fault=='bounds':
                arm.bus.stalled_joints.add('shoulder_pan')
                arm.bus.data['Present_Position']['shoulder_pan']=session.admitted_max['shoulder_pan']+1
            if fault=='torque':arm.bus.data['Torque_Enable']['gripper']=0
            if fault=='read':arm.bus.sync_read_failures['Present_Position']=10
            if fault=='slow_read':arm.bus.present_read_delays.append(.11)
        if tick==12:
            assert set(arm.bus.data['Torque_Enable'].values())=={1}
            os.write(write_fd,b'{"command":"stop"}\n')
        if tick==13:os.write(write_fd,b'{"command":"exit"}\n')
        return select.select(readers,writers,errors,0)
    try:return session.serve(fd=read_fd,emit=lambda value:None,wait=wait)
    finally:os.close(read_fd);os.close(write_fd)


def test_healthy_failed_hold_outlives_ordinary_idle_timeout(tmp_path):
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    session.idle_timeout=.1;last_command=session.last_command
    assert serve_held(session,arm,clock)==0
    assert clock.value-last_command>.5
    assert session._failed_endpoint_hold is None
    assert set(arm.bus.data['Torque_Enable'].values())=={0}


@pytest.mark.parametrize('fault',['drift','bounds','torque','read','slow_read','loop'])
def test_held_health_loss_disarms_and_restores(tmp_path,fault):
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    with pytest.raises(Exception):serve_held(session,arm,clock,fault=fault)
    assert set(arm.bus.data['Torque_Enable'].values())=={0}
    assert session.saved_ram is None


def test_eof_disarms_failed_hold(tmp_path):
    import os
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    read_fd,write_fd=os.pipe();os.close(write_fd)
    try:assert session.serve(fd=read_fd,emit=lambda value:None)==0
    finally:os.close(read_fd)
    assert set(arm.bus.data['Torque_Enable'].values())=={0}
    assert session.saved_ram is None


def test_typed_failure_serve_avoids_generic_shutdown_and_queued_motion(tmp_path):
    import os,json
    session,arm,clock,command=plateau(tmp_path);read_fd,write_fd=os.pipe()
    os.write(write_fd,(json.dumps(command)+'\n').encode());replies=[]
    def emit(value):
        json.dumps(value,allow_nan=False);replies.append(value)
        if value.get('status')=='failed_endpoint_held':
            assert set(arm.bus.data['Torque_Enable'].values())=={1}
            os.write(write_fd,b'{"command":"home"}\n{"command":"stop"}\n')
        elif value.get('failed_hold_cleared'):os.write(write_fd,b'{"command":"exit"}\n')
    try:assert session.serve(fd=read_fd,emit=emit)==0
    finally:os.close(read_fd);os.close(write_fd)
    assert replies[0]['status']=='failed_endpoint_held'
    assert any(row.get('error_type')=='ArmPlanPreflightError' for row in replies)
    assert set(arm.bus.data['Torque_Enable'].values())=={0}


def test_reply_transport_loss_disarms(tmp_path):
    import os,json
    session,arm,clock,command=plateau(tmp_path);read_fd,write_fd=os.pipe()
    os.write(write_fd,(json.dumps(command)+'\n').encode())
    def broken(value):raise BrokenPipeError('bridge reader gone')
    try:
        with pytest.raises(BrokenPipeError):session.serve(fd=read_fd,emit=broken)
    finally:os.close(read_fd);os.close(write_fd)
    assert set(arm.bus.data['Torque_Enable'].values())=={0}
    assert session.saved_ram is None


def test_rejected_queue_flood_cannot_refresh_health_clock(tmp_path):
    import os,json
    session,arm,clock,command=plateau(tmp_path);read_fd,write_fd=os.pipe()
    os.write(write_fd,(json.dumps(command)+'\n').encode())
    fault_replies=[];rejections=[];held_goals=[];write_open=True
    def emit(value):
        nonlocal write_open
        if value.get('status')=='failed_endpoint_held':
            held_goals.append(len(arm.bus.goal_history))
            os.write(write_fd,b'{"command":"home"}\n'*40)
        elif value.get('error_type')=='ArmPlanPreflightError':
            rejections.append(value);clock.sleep(.04)
        elif value.get('error_type')=='RuntimeError':
            fault_replies.append(value)
            # Verified cleanup now leaves an observable resident fault. EOF
            # ends this fixture after checking that queued motion never ran.
            os.close(write_fd);write_open=False
    try:
        assert session.serve(fd=read_fd,emit=emit)==0
    finally:
        os.close(read_fd)
        if write_open:os.close(write_fd)
    assert len(fault_replies)==1
    assert fault_replies[0]['error']=='failed endpoint hold buffered command health deadline expired'
    assert rejections and len(rejections)<40
    assert held_goals==[len(arm.bus.goal_history)]
    assert session._fault_observation is not None
    assert session._fault_observation['error']==fault_replies[0]['error']
    assert set(arm.bus.data['Torque_Enable'].values())=={0}
    assert session.saved_ram is None


def test_two_in_tolerance_final_samples_still_failed_held(tmp_path):
    session,arm,clock,command=plateau(tmp_path)
    original=arm.bus.on_present_read
    late_sample_time=command['points'][-1]['time']+3.-1.5*ROS_TRAJECTORY_PERIOD_S
    def late_convergence():
        original()
        # The 50 Hz endpoint loop ends after its unchanged three-second
        # settling budget. Enter tolerance for only its final two reads.
        if clock.value>=late_sample_time:arm.bus.data['Present_Position']['elbow_flex']=2515
    arm.bus.on_present_read=late_convergence
    reply=session.handle(command)
    assert reply['ok'] is False and reply['status']=='failed_endpoint_held'
    assert reply['hold']['settled_samples']==2
    assert reply['hold']['goal_error_rad']['elbow_flex']<math.radians(2)
    assert set(arm.bus.data['Torque_Enable'].values())=={1}


@pytest.mark.parametrize('fault',['read','slow','source','bounds','torque','stop','readback'])
def test_failure_hold_admission_fault_never_leaves_torque_enabled(tmp_path,monkeypatch,fault):
    from workshop.station import arm_session as module
    session,arm,clock,command=plateau(tmp_path);original=session.cancel_hold;counts={}
    def intercept(*,failure_context=None):
        counts['before']=len(arm.bus.goal_history)
        if fault=='read':arm.bus.sync_read_failures['Present_Position']=10
        if fault=='slow':arm.bus.present_read_delays.append(.11)
        if fault=='source':monkeypatch.setattr(module,'owner_source_sha256',lambda path:{})
        if fault=='bounds':
            arm.bus.stalled_joints.add('shoulder_pan')
            arm.bus.data['Present_Position']['shoulder_pan']=session.admitted_max['shoulder_pan']+1
        if fault=='torque':arm.bus.data['Torque_Enable']['gripper']=0
        if fault=='stop':session._cancel_requested=True;session._cancel_reason='busy_stop'
        if fault=='readback':arm.bus.clamp_goal=True
        try:return original(failure_context=failure_context)
        finally:counts['after']=len(arm.bus.goal_history)
    monkeypatch.setattr(session,'cancel_hold',intercept)
    with pytest.raises(Exception):session.handle(command)
    assert set(arm.bus.data['Torque_Enable'].values())=={0}
    assert session.saved_ram is None
    if fault!='readback':assert counts['after']==counts['before']
    else:assert counts['after']==counts['before']+1


def test_unstable_endpoint_keeps_original_shutdown(tmp_path):
    session,arm,clock,command=plateau(tmp_path);original=arm.bus.on_present_read;flip=False
    arm.bus.stalled_joints.add('shoulder_pan')
    def unstable():
        nonlocal flip
        original()
        if clock.value>=8.8:
            flip=not flip;arm.bus.data['Present_Position']['shoulder_pan']=2000+(3 if flip else -3)
    arm.bus.on_present_read=unstable
    with pytest.raises(TimeoutError,match='final goal tolerance'):session.handle(command)
    assert session._failed_endpoint_hold is None and set(arm.bus.data['Torque_Enable'].values())=={0}


def test_hold_health_checks_add_no_periodic_bus_reads(tmp_path):
    import os
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    before=dict(arm.bus.sync_read_calls);read_fd,write_fd=os.pipe();os.close(write_fd)
    counts={};original=session.shutdown
    def shutdown(*,reason='stop'):
        counts.update(arm.bus.sync_read_calls)
        return original(reason=reason)
    session.shutdown=shutdown
    try:assert session.serve(fd=read_fd,emit=lambda value:None)==0
    finally:os.close(read_fd)
    delta={reg:counts.get(reg,0)-before.get(reg,0) for reg in counts}
    assert {key:value for key,value in delta.items() if value}=={'Present_Position':1,'Torque_Enable':1}


def test_failed_hold_tick_guard_forbids_io_hashing_or_motion_calls():
    import ast,inspect,textwrap
    from workshop.station.arm_session import ArmSession
    for method,allowed in [(ArmSession._check_failed_hold_snapshot,{'self.clock','RuntimeError','self._guard_failed_hold_observation','self._failed_endpoint_hold.get','any','abs'}),
                           (ArmSession._guard_failed_hold_observation,{'self.clock','any','type','math.isfinite','isinstance','set','RuntimeError','torque.values','self._guard_supervised_actual'})]:
        tree=ast.parse(textwrap.dedent(inspect.getsource(method)))
        calls={ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node,ast.Call)}
        assert calls<=allowed


def test_failed_stop_drops_buffered_motion_tail_even_nonpersistent(tmp_path):
    import os,json,select
    session,arm,clock,command=plateau(tmp_path);session.handle(command)
    assert not session.persistent_idle
    read_fd,write_fd=os.pipe();replies=[];stage=0
    os.write(write_fd,b'{"command":"stop"}\n{"command":"home"}\n')
    def wait(readers,writers,errors,timeout):
        nonlocal stage
        if stage:
            os.write(write_fd,b'{"command":"exit"}\n')
        stage+=1
        return select.select(readers,writers,errors,0)
    goals=len(arm.bus.goal_history)
    try:assert session.serve(fd=read_fd,emit=replies.append,wait=wait)==0
    finally:os.close(read_fd);os.close(write_fd)
    assert replies[0]['failed_hold_cleared'] is True
    assert not any(row.get('state',{}).get('event')=='home_complete' for row in replies)
    assert len(arm.bus.goal_history)==goals


def test_cancel_after_install_preserves_failure_context_and_enabled_hold(tmp_path,monkeypatch):
    session,arm,clock,command=plateau(tmp_path);record=session._record;injected=False
    def cancel_after_record(value):
        nonlocal injected
        record(value)
        if value.get('event')=='trajectory_failed_endpoint_held' and 'feedback_samples' in value and not injected:
            injected=True
            session._cancel_requested=True;session._cancel_reason='trajectory_cancel_hold'
    monkeypatch.setattr(session,'_record',cancel_after_record)
    reply=session.handle(command)
    assert injected and reply['ok'] is False and reply['status']=='failed_endpoint_held'
    assert reply['state']['event']=='read_state'
    assert set(arm.bus.data['Torque_Enable'].values())=={1}
