"""Owner-authoritative bounds observations; no additional device reads."""
import ast
import inspect
import textwrap

import pytest

from test_arm_camera_recovery import recovery_session
from workshop.station.arm_session import ArmSession


@pytest.mark.parametrize('elbow,gripper,expected',[
    (2975,2000,'normal'),(2993,2000,'bounded_reentry_required'),
    (3005,2000,'bounded_reentry_required'),(3006,2000,'unrecoverable'),
    (3056,2000,'unrecoverable'),(2975,999,'unrecoverable')])
def test_bounds_classification_exact_cached_intersection_and_sample(tmp_path,elbow,gripper,expected):
    session,arm,clock,_=recovery_session(tmp_path)
    raw=dict(arm.bus.data['Present_Position'],elbow_flex=elbow,gripper=gripper)
    before=len(arm.bus.writes)
    session._read=lambda *_:pytest.fail('classification telemetry added a device read')
    state=session.publish_state(raw=raw,torque=dict.fromkeys(raw,0),
        position_monotonic=.123,position_utc=clock.utc(),
        torque_monotonic=.124,torque_utc=clock.utc())
    bounds=state['arm_bounds']
    assert bounds['schema']=='arm-bounds-observation-v1'
    assert bounds['classification']==expected
    assert bounds['position_monotonic']==state['monotonic']==.123
    assert bounds['admitted_min']==session.admitted_min
    assert bounds['admitted_max']==session.admitted_max
    assert bounds['firmware_min']==session.firmware_min
    assert bounds['firmware_max']==session.firmware_max
    assert len(arm.bus.writes)==before
    session.close_log()


def test_missing_bounds_are_unknown_never_normal(tmp_path):
    session,arm,_,_=recovery_session(tmp_path)
    session._arm_bounds_metadata=None
    assert session._arm_bounds_observation(dict(arm.bus.data['Present_Position']),.1)['classification']=='unknown'
    session.close_log()


def test_bounds_observation_static_guard_has_no_io_or_calibration_conversion():
    tree=ast.parse(textwrap.dedent(inspect.getsource(ArmSession._arm_bounds_observation)))
    forbidden={'open','read','_read','write','sync_read','sync_write','read_bytes','read_text','stat','owner_source_sha256','_calibration_guard','plan_bound_reentry'}
    calls={node.func.attr if isinstance(node.func,ast.Attribute) else node.func.id
        for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,(ast.Attribute,ast.Name))}
    assert not calls&forbidden
    assert not any(isinstance(node,ast.BinOp) and isinstance(node.op,(ast.Div,ast.Mult)) for node in ast.walk(tree))


@pytest.mark.parametrize('stamp',[True,False,float('nan'),float('inf'),-.001,None])
def test_invalid_acquisition_clock_is_unknown(tmp_path,stamp):
    session,arm,_,_=recovery_session(tmp_path)
    assert session._arm_bounds_observation(dict(arm.bus.data['Present_Position']),stamp)['classification']=='unknown'
    session.close_log()


@pytest.mark.parametrize('fault',['extra','missing','bool','not_dict'])
def test_unknown_raw_shape_cannot_classify_normal(tmp_path,fault):
    session,arm,_,_=recovery_session(tmp_path)
    raw=dict(arm.bus.data['Present_Position'])
    if fault=='extra':raw['other']=1
    if fault=='missing':raw.pop('gripper')
    if fault=='bool':raw['gripper']=True
    if fault=='not_dict':raw=list(raw)
    assert session._arm_bounds_observation(raw,.1)['classification']=='unknown'
    session.close_log()


def test_published_bounds_cannot_mutate_cached_metadata(tmp_path):
    session,arm,_,_=recovery_session(tmp_path)
    first=session._arm_bounds_observation(dict(arm.bus.data['Present_Position']),.1)
    for key in ('admitted_min','admitted_max','firmware_min','firmware_max'):
        with pytest.raises(TypeError):first[key]['gripper']=-1
    second=session._arm_bounds_observation(dict(arm.bus.data['Present_Position']),.2)
    assert first['admitted_min'] is second['admitted_min']
    assert second['admitted_min']==session.admitted_min
    session.close_log()


@pytest.mark.parametrize('overrun',[30,31,32])
def test_owner_classification_matches_actual_planner_inward_distance_boundary(tmp_path,overrun):
    from test_arm_camera_recovery import Scene
    from workshop.station.camera_recovery import plan_bound_reentry
    from workshop.station.recovery_types import CAMERA_RECOVERY_MAX_DISPLACEMENT_COUNTS,CAMERA_RECOVERY_INWARD_COUNTS
    session,arm,_,_=recovery_session(tmp_path)
    raw=dict(arm.bus.data['Present_Position'],elbow_flex=session.admitted_max['elbow_flex']+overrun)
    state=session._arm_bounds_observation(raw,10.)
    bounds={j:(session.admitted_min[j],session.admitted_max[j]) for j in raw}
    firmware={j:(session.firmware_min[j],session.firmware_max[j]) for j in raw}
    scene=Scene(raw,10.)
    arguments=dict(scene=scene,now=10.1,expected_model_sha256=scene.model_sha256,
        expected_calibration_sha256=scene.calibration_sha256,expected_camera_serial=scene.camera_serial)
    if overrun+CAMERA_RECOVERY_INWARD_COUNTS<=CAMERA_RECOVERY_MAX_DISPLACEMENT_COUNTS:
        result=plan_bound_reentry(raw,bounds,firmware,**arguments)
        assert result.target_raw['elbow_flex']==session.admitted_max['elbow_flex']-2
        assert state['classification']=='bounded_reentry_required'
    else:
        with pytest.raises(ValueError,match='envelope'):plan_bound_reentry(raw,bounds,firmware,**arguments)
        assert state['classification']=='unrecoverable'
    session.close_log()


def test_empty_intersection_never_publishes_ready_bounds(tmp_path):
    from test_arm_session import make_session
    session,arm,_=make_session(tmp_path)
    arm.bus.data['Min_Position_Limit']['gripper']=3001
    arm.bus.data['Max_Position_Limit']['gripper']=3055
    with pytest.raises(ValueError,match='empty'):session.initialize()
    assert session._arm_bounds_metadata is None
    assert session._arm_bounds_observation(dict(arm.bus.data['Present_Position']),.1)['classification']=='unknown'
    assert arm.bus.goal_history==[]
    session.close_log()


def test_source_scope_cache_still_pins_current_owner_and_shared_policy(tmp_path,monkeypatch):
    from pathlib import Path
    from workshop.station.recovery_types import OWNER_SOURCE_MODULES,owner_source_sha256
    session,arm,_,_=recovery_session(tmp_path)
    expected=owner_source_sha256(Path('workshop/station'))
    assert len(OWNER_SOURCE_MODULES)==7
    assert session._owner_source_sha256==expected
    monkeypatch.setattr(Path,'read_bytes',lambda *_:pytest.fail('tick source read'))
    monkeypatch.setattr('workshop.station.arm_session.owner_source_sha256',lambda *_:pytest.fail('tick source hash'))
    assert session._arm_bounds_observation(dict(arm.bus.data['Present_Position']),.1)['classification']=='bounded_reentry_required'
    assert session._owner_source_sha256==expected
    session.close_log()
