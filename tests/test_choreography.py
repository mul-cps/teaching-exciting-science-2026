import pytest

from workshop.choreography import (build_dance, interpolate, motion_duration,
                                  make_plan, plan_at, validate_replay, smooth_state)


def test_spline_stops_smoothly_at_both_ends():
    a, b = {"wrist_roll": -20.0}, {"wrist_roll": 30.0}
    assert interpolate(a, b, 0) == a
    assert interpolate(a, b, 1) == b
    h = 1e-4
    # Endpoint displacement shrinks cubically: no velocity or acceleration jump.
    assert abs(interpolate(a, b, h)["wrist_roll"] - a["wrist_roll"]) < 1e-8
    assert abs(interpolate(a, b, 1 - h)["wrist_roll"] - b["wrist_roll"]) < 1e-8
    values = [interpolate(a, b, i / 100)["wrist_roll"] for i in range(101)]
    assert values == sorted(values)


def test_duration_enforces_speed_and_acceleration():
    # Quintic blend peaks at 1.875 / T velocity and (10/sqrt(3)) / T² acceleration.
    a, b = {"shoulder_pan": 0}, {"shoulder_pan": 40}
    duration = motion_duration(a, b)
    assert 1.875 * 40 / duration <= 10 + 1e-9
    assert (10 / 3**0.5) * 40 / duration**2 <= 20 + 1e-9


def test_dance_has_holds_two_waves_and_returns_to_start():
    start = dict(shoulder_pan=2, shoulder_lift=-95, elbow_flex=69,
                 wrist_flex=98, wrist_roll=2, gripper=90)
    segments = build_dance(start)
    assert segments[-1].target == start
    assert sum(s.name.startswith("wave ") for s in segments) == 4
    assert any(s.name == "bow" for s in segments)
    assert any(s.hold >= 0.8 for s in segments)
    assert all(0 <= s.target["gripper"] <= 100 for s in segments)
    for s in segments:
        assert s.duration > 0
        assert abs(s.target["shoulder_pan"] - start["shoulder_pan"]) <= 20
        assert abs(s.target["shoulder_lift"] - start["shoulder_lift"]) <= 22


def test_rejects_incomplete_pose():
    with pytest.raises(ValueError):
        build_dance({"shoulder_pan": 0})


def test_saved_plan_replays_exactly_and_holds():
    import json
    start = dict(shoulder_pan=2, shoulder_lift=-95, elbow_flex=69,
                 wrist_flex=98, wrist_roll=2, gripper=1)
    plan = make_plan(start, 100)
    saved = json.loads(json.dumps(plan))
    assert plan_at(saved, 0)[0] == start
    for stage in plan["stages"]:
        t = stage["begin"] + stage["move"] + stage["hold"] / 2
        assert plan_at(saved, t)[0] == pytest.approx(stage["target"])
    assert plan_at(saved, plan["duration"] + 1)[0] == start
    for i in range(int(plan["duration"] * 100)):
        assert plan_at(plan, i / 100) == plan_at(saved, i / 100)


def test_replay_refuses_a_different_start_or_calibration():
    start = dict(shoulder_pan=2, shoulder_lift=-95, elbow_flex=69,
                 wrist_flex=98, wrist_roll=2, gripper=1)
    plan = make_plan(start, 100)
    plan.update(arm_id="rig", calibration_sha256="abc")
    validate_replay(plan, start, "rig", "abc")
    with pytest.raises(ValueError, match="starting pose"):
        validate_replay(plan, dict(start, shoulder_pan=20), "rig", "abc")
    with pytest.raises(ValueError, match="calibration"):
        validate_replay(plan, start, "rig", "def")


@pytest.mark.parametrize("body_wave", [False, True])
def test_flow_has_continuous_velocity_acceleration_and_jerk(body_wave):
    start = dict(shoulder_pan=2, shoulder_lift=-95, elbow_flex=69,
                 wrist_flex=98, wrist_roll=2, gripper=1)
    plan = make_plan(start, 100, body_wave)
    assert plan["interpolation"] == "septic-flow"
    previous = start
    moving_boundary = False
    last_end = None
    for stage in plan["stages"]:
        initial = smooth_state(previous, stage["target"], stage["v0"], stage["v1"], stage["move"], 0)
        final = smooth_state(previous, stage["target"], stage["v0"], stage["v1"], stage["move"], 1)
        if last_end:
            for derivative in (1, 2, 3):
                assert initial[derivative] == pytest.approx(last_end[derivative], abs=1e-7)
        assert final[0] == pytest.approx(stage["target"])
        for derivative in (2, 3):
            assert final[derivative] == pytest.approx({k: 0 for k in start}, abs=1e-7)
        moving_boundary |= any(abs(v) > 0.1 for v in final[1].values())
        for i in range(201):
            p, v, a, j = smooth_state(previous, stage["target"], stage["v0"], stage["v1"], stage["move"], i / 200)
            for k in start:
                assert min(previous[k], stage["target"][k]) - 1e-7 <= p[k] <= max(previous[k], stage["target"][k]) + 1e-7
                assert abs(v[k]) <= plan["speed_limits"][k]
                assert abs(a[k]) <= plan["acceleration_limits"][k]
                assert abs(j[k]) <= plan["jerk_limits"][k]
        previous, last_end = stage["target"], final
    assert moving_boundary
    assert plan["duration"] < 60


def test_replay_rejects_nonfinite_timing_and_outside_workspace():
    start = dict(shoulder_pan=2, shoulder_lift=-95, elbow_flex=69,
                 wrist_flex=98, wrist_roll=2, gripper=1)
    plan = make_plan(start, 125)
    plan.update(arm_id="rig", calibration_sha256="abc")
    plan["stages"][0]["hold"] = float("nan")
    with pytest.raises(ValueError, match="timing"):
        validate_replay(plan, start, "rig", "abc")
    plan = make_plan(start, 125)
    plan.update(arm_id="rig", calibration_sha256="abc")
    plan["stages"][0]["target"]["shoulder_pan"] += 100
    with pytest.raises(ValueError, match="displacement"):
        validate_replay(plan, start, "rig", "abc")


def test_retained_hardware_trajectory_remains_replayable():
    import json
    from pathlib import Path
    plan = json.loads((Path(__file__).parents[1] / "trajectories/rig01-flow-2026-10-12.json").read_text())
    validate_replay(plan, plan["start"], plan["arm_id"], plan["calibration_sha256"])
