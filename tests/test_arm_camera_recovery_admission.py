"""A rejected recovery must not energize hardware via a purported hold."""
import pytest
from test_arm_camera_recovery import recovery_session
from workshop.station.arm_session import ArmPlanPreflightError,ArmSessionCancelled


def energizing_bus(arm):
    original=arm.bus.sync_write
    def write(register,values,**kwargs):
        original(register,values,**kwargs)
        if register=='Goal_Position':arm.bus.data['Torque_Enable'].update(dict.fromkeys(values,1))
    arm.bus.sync_write=write


@pytest.mark.parametrize('fault',['changed_raw','profile_write','cancel_hold','disarm_failure'])
def test_pre_dispatch_failures_disarm_without_goal_or_accidental_enable(tmp_path,fault):
    def provider(scene,index,session,arm,clock):
        if index==1 and fault in {'changed_raw','disarm_failure'}:
            arm.bus.data['Present_Position']['wrist_roll']+=1
        return scene
    session,arm,_,_=recovery_session(tmp_path,provider=provider)
    energizing_bus(arm)
    original=arm.bus.write
    fired=False
    def write(reg,joint,value,**kwargs):
        nonlocal fired
        original(reg,joint,value,**kwargs)
        if fault in {'profile_write','cancel_hold'} and reg=='Acceleration' and value==8 and not fired:
            fired=True
            if fault=='profile_write':raise ConnectionError('partial profile write')
            session._cancel('trajectory_cancel_hold')
        if fault=='disarm_failure' and reg=='Torque_Enable' and value==0:
            arm.bus.data[reg][joint]=1
            raise ConnectionError('cannot disarm')
    arm.bus.write=write
    expected=RuntimeError if fault=='disarm_failure' else ArmSessionCancelled if fault=='cancel_hold' else ArmPlanPreflightError
    with pytest.raises(expected):session.handle({'command':'recover_camera'})
    assert arm.bus.goal_history==[]
    if fault!='disarm_failure':
        assert not any(arm.bus.data['Torque_Enable'].values())
        assert session.saved_ram is None
        assert arm.bus.data['Acceleration']==dict.fromkeys(arm.bus.motors,2)
    session.close_log()


def test_first_goal_write_uncertain_receipt_is_post_dispatch_not_preflight(tmp_path):
    session,arm,_,_=recovery_session(tmp_path)
    energizing_bus(arm)
    original=arm.bus.sync_write
    fired=False
    def write(reg,values,**kwargs):
        nonlocal fired
        original(reg,values,**kwargs)
        if reg=='Goal_Position' and not fired:
            fired=True
            raise ConnectionError('goal applied but reply lost')
    arm.bus.sync_write=write
    result=session.handle({'command':'recover_camera'})
    assert result['ok'] is False and result['recovery']['status']=='held'
    assert len(arm.bus.goal_history)==2
    assert all(value==1 for value in arm.bus.data['Torque_Enable'].values())
    session.shutdown();session.close_log()


def test_preflight_abort_does_not_reuse_earlier_shutdown_ack(tmp_path):
    def provider(scene,*_):scene.clearance=None;return scene
    session,arm,_,_=recovery_session(tmp_path,provider=provider)
    session.shutdown()
    arm.bus.writes.clear()
    # A previous stop is not a fresh readback for this new explicit operation.
    arm.bus.data['Torque_Enable']['wrist_roll']=1
    with pytest.raises(ArmPlanPreflightError):session.handle({'command':'recover_camera'})
    assert arm.bus.goal_history==[]
    assert not any(arm.bus.data['Torque_Enable'].values())
    assert any(reg=='Torque_Enable' and value==0 for reg,_,value in arm.bus.writes)
    session.close_log()


@pytest.mark.parametrize('fault',['slow_read_state','changed_raw','cancel'])
def test_first_goal_final_input_pump_cannot_invalidate_proof(tmp_path,fault,monkeypatch):
    session,arm,clock,_=recovery_session(tmp_path)
    energizing_bus(arm)
    original_enable=session._enable_current_goal
    original_poll=session._poll_busy_input
    in_enable=False
    fired=False
    def enable(*args,**kwargs):
        nonlocal in_enable
        in_enable=True
        return original_enable(*args,**kwargs)
    def poll():
        nonlocal fired
        original_poll()
        if in_enable and not fired:
            fired=True
            session.read_state()  # the real queued-status I/O path
            if fault=='slow_read_state':clock.value+=.251
            if fault=='changed_raw':arm.bus.data['Present_Position']['wrist_roll']+=1
            if fault=='cancel':session._cancel('trajectory_cancel_hold')
    monkeypatch.setattr(session,'_enable_current_goal',enable)
    monkeypatch.setattr(session,'_poll_busy_input',poll)
    with pytest.raises(ArmSessionCancelled if fault=='cancel' else ArmPlanPreflightError):
        session.handle({'command':'recover_camera'})
    assert fired and arm.bus.goal_history==[]
    assert not any(arm.bus.data['Torque_Enable'].values())
    assert session.saved_ram is None
    session.close_log()
