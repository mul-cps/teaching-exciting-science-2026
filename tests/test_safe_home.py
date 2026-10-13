import json
import numpy as np
import pytest
from workshop.choreography import JOINTS, smooth_state
from workshop.safe_home import make_home_move, load_home, home_at

START=dict(zip(JOINTS,(64,-67,76,62,-69,58)))
BOUNDS={k:(-180,180) for k in JOINTS};BOUNDS['gripper']=(0,100)


def test_home_move_has_bounded_smooth_ramps_and_never_overshoots():
    target={k:v+3 for k,v in START.items()}
    move=make_home_move(START,target,BOUNDS,125)
    assert home_at(move,0)[0]==START
    assert home_at(move,move['duration'])[0]==pytest.approx(target)
    for t in np.linspace(0,move['duration'],2001):
        p,v,a,j=smooth_state(START,target,dict.fromkeys(JOINTS,0),dict.fromkeys(JOINTS,0),move['duration'],t/move['duration'])
        for k in JOINTS:
            assert START[k]-1e-6<=p[k]<=target[k]+1e-6
            assert abs(v[k])<=move['speed_limits'][k]
            assert abs(a[k])<=move['acceleration_limits'][k]
            assert abs(j[k])<=move['jerk_limits'][k]
    for field in (1,2,3):
        assert smooth_state(START,target,dict.fromkeys(JOINTS,0),dict.fromkeys(JOINTS,0),move['duration'],1)[field]==pytest.approx(dict.fromkeys(JOINTS,0),abs=1e-7)


def test_home_move_refuses_large_returns_and_calibration_bound_violations():
    with pytest.raises(ValueError,match='return envelope'):
        make_home_move(START,{**START,'shoulder_pan':START['shoulder_pan']+80},BOUNDS,125)
    with pytest.raises(ValueError,match='calibrated'):
        make_home_move(START,{**START,'gripper':101},BOUNDS,125)


def test_home_file_rejects_wrong_arm_calibration_units_and_nonfinite_positions(tmp_path):
    path=tmp_path/'home.json'
    data=dict(schema=1,kind='measured-home',arm_id='rig',calibration_sha256='abc',units='body degrees; gripper percent',position=dict(START))
    path.write_text(json.dumps(data))
    assert load_home(path,'rig','abc')==START
    with pytest.raises(ValueError,match='calibration'):
        load_home(path,'rig','bad')
    with pytest.raises(ValueError,match='arm'):
        load_home(path,'other','abc')
    data['position']['shoulder_pan']=float('nan');path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='finite'):
        load_home(path,'rig','abc')


def test_restart_must_follow_home_before_saved_choreography():
    from workshop.safe_home import restart_at
    from workshop.rhythm import make_rhythm_plan
    from workshop.choreography import plan_at
    current={**START,'shoulder_pan':START['shoulder_pan']+3}
    move=make_home_move(current,START,BOUNDS,125)
    plan=make_rhythm_plan(START,125)
    assert restart_at(move,plan,0)[0]==current
    assert restart_at(move,plan,move['duration']-.001)[1]=='safe home'
    assert restart_at(move,plan,move['duration'])[0]==START
    assert restart_at(move,plan,move['duration']+1)==plan_at(plan,1)


def test_saved_replay_cannot_start_from_a_different_home():
    from workshop.safe_home import validate_home_start
    validate_home_start(START,START)
    with pytest.raises(ValueError,match='saved home'):
        validate_home_start({**START,'elbow_flex':START['elbow_flex']+2},START)
