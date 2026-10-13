import json
import numpy as np
import pytest
from workshop.choreography import JOINTS, plan_at, profile_at, validate_replay
from workshop.rhythm import make_rhythm_plan

START = dict(zip(JOINTS, (64, -95, 64, 96, -67, 3)))


def test_rhythm_has_smooth_start_stop_and_continuous_interior_acceleration():
    plan = make_rhythm_plan(START, 125)
    assert plan['duration'] < 30
    for t in (0, plan['duration']):
        state = profile_at(plan, t)
        assert state['position'] == pytest.approx(START, abs=1e-7)
        for field in ('velocity', 'acceleration', 'jerk'):
            assert state[field] == pytest.approx(dict.fromkeys(JOINTS, 0), abs=1e-6)
    active_acceleration = []
    for t in np.linspace(0, plan['duration'], 2001):
        state = profile_at(plan, t)
        active_acceleration.append(abs(state['acceleration']['shoulder_pan']))
        for field, limit in [('velocity', 'speed_limits'), ('acceleration', 'acceleration_limits'), ('jerk', 'jerk_limits')]:
            assert all(abs(state[field][k]) <= plan[limit][k] + 1e-6 for k in JOINTS)
        assert abs(state['position']['shoulder_pan'] - START['shoulder_pan']) <= 20.00001
    assert max(active_acceleration) > 10
    assert plan_at(plan, plan['duration'] + 1)[0] == pytest.approx(START)


def test_rhythm_serialization_replays_identically_and_rejects_modified_parameters():
    plan = make_rhythm_plan(START, 125)
    plan.update(arm_id='rig', calibration_sha256='abc')
    saved = json.loads(json.dumps(plan))
    validate_replay(saved, START, 'rig', 'abc')
    for t in np.linspace(0, plan['duration'], 100):
        assert plan_at(saved, t) == plan_at(plan, t)
    saved['harmonics']['shoulder_pan'][0]['amplitude'] = 100
    with pytest.raises(ValueError, match='workspace'):
        validate_replay(saved, START, 'rig', 'abc')


def test_rhythm_rejects_calibrated_bounds_before_actuation():
    from workshop.choreography import validate_calibrated_bounds
    plan = make_rhythm_plan(START, 125)
    bounds = {k: (-180, 180) for k in JOINTS}
    bounds['gripper'] = (0, 100)
    validate_calibrated_bounds(plan, bounds)
    bounds['shoulder_pan'] = (START['shoulder_pan']-10, START['shoulder_pan']+10)
    with pytest.raises(ValueError, match='calibrated'):
        validate_calibrated_bounds(plan, bounds)
