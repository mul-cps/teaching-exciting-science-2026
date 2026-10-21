"""Owner-session home provenance and measured-current teleop admission."""
from pathlib import Path

import pytest

from test_arm_session import make_session, target
from workshop.station.arm_session import JOINTS


@pytest.mark.parametrize('torque',[0,1])
def test_completed_home_allows_normal_current_pose_and_installs_exact_hold(tmp_path,torque):
    session,arm,_=make_session(tmp_path)
    session.initialize()
    assert session._home_completed_this_session is False
    session.home()
    assert session._home_completed_this_session is True
    session.move_plan([{'label':'valid-later-move','target_raw':target(shoulder_pan=2100,wrist_flex=2120)}])
    current=dict(arm.bus.data['Present_Position'])
    assert current!=session.home_raw
    arm.bus.data['Torque_Enable']=dict.fromkeys(JOINTS,torque)
    arm.bus.goal_history.clear()
    state=session.handle({'command':'teleop_start','session_id':'current-baseline'})['state']
    assert arm.bus.goal_history==[current]
    assert state['raw']==current
    assert session._teleop['start']==session._teleop['previous']==current
    assert arm.bus.data['Torque_Enable']==dict.fromkeys(JOINTS,1)
    session.shutdown(reason='test_cleanup');session.close_log()


def test_saved_home_file_and_current_home_pose_do_not_prove_session_completion(tmp_path):
    session,arm,_=make_session(tmp_path)
    session.initialize()
    assert arm.bus.data['Present_Position']==session.home_raw
    before=len(arm.bus.writes)
    with pytest.raises(ValueError,match='completed measured home in this owner session'):
        session.handle({'command':'teleop_start','session_id':'not-homed'})
    assert len(arm.bus.writes)==before
    session.close_log()


def test_home_provenance_does_not_survive_new_owner_or_failed_home(tmp_path,monkeypatch):
    session,_,_=make_session(tmp_path)
    session.initialize();session.home()
    assert session._home_completed_this_session is True
    session.shutdown(reason='restart');session.close_log()
    new,arm,_=make_session(tmp_path)
    new.initialize()
    assert new._home_completed_this_session is False
    def fail(*args,**kwargs): raise RuntimeError('home leg failed')
    monkeypatch.setattr(new,'_run_leg',fail)
    with pytest.raises(RuntimeError,match='home leg failed'):new.home()
    assert new._home_completed_this_session is False
    new.close_log()


@pytest.mark.parametrize('fault',['mixed_torque','out_of_bounds','calibration','source','stale'])
def test_current_baseline_guards_refuse_before_any_goal_or_profile_write(tmp_path,monkeypatch,fault):
    session,arm,clock=make_session(tmp_path)
    session.initialize();session.home()
    if fault=='mixed_torque':arm.bus.data['Torque_Enable']['wrist_roll']=0
    if fault=='out_of_bounds':
        arm.bus.data['Present_Position']['wrist_flex']=session.admitted_max['wrist_flex']+1
        arm.bus.data['Goal_Position']=dict(arm.bus.data['Present_Position'])
    if fault=='calibration':Path(arm.settings['calibration']['path']).write_text('{}')
    if fault=='source':session._owner_source_sha256={}
    if fault=='stale':
        read=session._read
        def delayed(register):
            result=read(register)
            if register=='Torque_Enable':clock.sleep(.101)
            return result
        monkeypatch.setattr(session,'_read',delayed)
    before=len(arm.bus.writes)
    with pytest.raises((ValueError,RuntimeError)):
        session.handle({'command':'teleop_start','session_id':'unsafe-baseline'})
    assert len(arm.bus.writes)==before
    assert session._teleop is None
    session.close_log()


def test_home_failure_revokes_prior_completion_and_invalid_receipt_cannot_grant_it(tmp_path,monkeypatch):
    session,arm,_=make_session(tmp_path)
    session.initialize();session.home()
    assert session._home_completed_this_session is True
    publish=session.publish_state
    def invalid_receipt(**kwargs):
        state=publish(**kwargs)
        if kwargs.get('event')=='home_complete':
            state['raw']=dict(state['raw'],wrist_roll=session.admitted_max['wrist_roll']+1)
        return state
    monkeypatch.setattr(session,'publish_state',invalid_receipt)
    with pytest.raises(ValueError,match='configured limits'):session.home()
    assert session._home_completed_this_session is False
    session.close_log()


def test_teleop_reacquires_current_hold_after_profile_readback(tmp_path,monkeypatch):
    session,arm,_=make_session(tmp_path)
    session.initialize();session.home()
    profile=session._ensure_profile
    def drift_during_profile(**kwargs):
        profile(**kwargs)
        current=dict(arm.bus.data['Present_Position'],wrist_roll=2020)
        arm.bus.data['Present_Position']=current
        arm.bus.data['Goal_Position']=dict(current)
    monkeypatch.setattr(session,'_ensure_profile',drift_during_profile)
    arm.bus.goal_history.clear()
    state=session.handle({'command':'teleop_start','session_id':'profile-reanchor'})['state']
    assert state['raw']['wrist_roll']==2020
    assert arm.bus.goal_history==[state['raw']]
    assert session._teleop['start']==state['raw']
    session.shutdown(reason='test_cleanup');session.close_log()
