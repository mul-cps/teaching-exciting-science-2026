import math
import pytest
from workshop.choreography import JOINTS
from workshop.teleop_replay import prepare_samples, validate_samples, reverse_return

def rows():
    return [dict(command_t=t,goal=dict.fromkeys(JOINTS,x)) for t,x in [(0.002,0),(0.010,0),(0.018,.1),(0.026,.2)]]

def test_original_command_timing_and_targets_are_preserved():
    result=prepare_samples(rows())
    assert result[2]['t']==pytest.approx(.016)
    assert result[2]['goal']==rows()[2]['goal']

def test_return_uses_reverse_path_and_omits_initial_idle():
    result=reverse_return(prepare_samples(rows()))
    assert result[0]['goal']['shoulder_pan']==.2
    assert result[-1]['goal']['shoulder_pan']==0
    assert result[-1]['t']==pytest.approx(.016)

def test_validation_rejects_time_nan_bounds_and_pose_jump():
    samples=prepare_samples(rows()); bounds={k:(-1,1) for k in JOINTS}; rates=dict.fromkeys(JOINTS,100)
    validate_samples(samples,bounds,rates)
    bad=[dict(s,goal=dict(s['goal'])) for s in samples];bad[1]['goal']['wrist_roll']=math.nan
    with pytest.raises(ValueError):validate_samples(bad,bounds,rates)
    bad=[dict(s) for s in samples];bad[1]['t']=0
    with pytest.raises(ValueError):validate_samples(bad,bounds,rates)
    bad=[dict(s,goal=dict(s['goal'])) for s in samples];bad[1]['goal']['elbow_flex']=2
    with pytest.raises(ValueError):validate_samples(bad,bounds,rates)
    with pytest.raises(ValueError):validate_samples(samples,bounds,dict.fromkeys(JOINTS,.1))

def test_recorded_command_latency_jitter_uses_original_tick_step_bound():
    samples=[{'t':0,'goal':dict.fromkeys(JOINTS,0)}, {'t':.0062,'goal':dict.fromkeys(JOINTS,.48)}]
    validate_samples(samples,dict.fromkeys(JOINTS,(-1,1)),dict.fromkeys(JOINTS,60),nominal_dt=.008)
