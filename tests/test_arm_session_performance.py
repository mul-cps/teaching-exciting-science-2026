"""Prevent setup/EEPROM work from entering the resident motion feedback loop."""
import ast
import inspect
import textwrap
import pytest

from workshop.station.arm_session import ArmSession, JOINTS


def test_motion_feedback_has_only_position_goal_and_torque_reads():
    tree = ast.parse(textwrap.dedent(inspect.getsource(ArmSession._run_leg)))
    registers = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        assert node.func.attr not in {"_ensure_profile", "_calibration_guard", "ping"}
        if node.func.attr == "_read":
            assert node.args and isinstance(node.args[0], ast.Constant)
            registers.add(node.args[0].value)
        if node.func.attr == "sync_write":
            assert isinstance(node.args[0], ast.Constant)
            assert node.args[0].value == "Goal_Position"
        assert node.func.attr != "write", "individual ACK writes belong outside the feedback loop"
    assert registers == {"Present_Position", "Goal_Position", "Torque_Enable"}


def test_feedback_accepts_only_a_bounded_cumulative_miss_budget_and_resynchronizes():
    from workshop.station import arm_session
    assert arm_session.MAX_FEEDBACK_GAP_S == pytest.approx(0.25)
    assert arm_session.MAX_MISSED_FEEDBACK_TICKS_PER_LEG == 4
    source = textwrap.dedent(inspect.getsource(ArmSession._run_leg))
    assert source.count("_check_feedback_freshness(") == 2
    assert source.count("position_monotonic + PERIOD_S") == 2
    assert "now - scheduled > PERIOD_S" not in source


def test_live_state_snapshot_uses_atomic_replace_without_per_sample_fsync():
    from workshop.station import arm_session
    source = textwrap.dedent(inspect.getsource(arm_session._atomic_snapshot_json))
    assert "temporary.replace(path)" in source
    assert "os.fsync(" not in source
    publish = textwrap.dedent(inspect.getsource(ArmSession.publish_state))
    assert "self._publish_built_state(state)" in publish
    sink = textwrap.dedent(inspect.getsource(ArmSession._publish_built_state))
    assert "_atomic_snapshot_json(self.state_file, state)" in sink
    assert "os.fsync(" not in sink


def test_fake_benchmark_preserves_warm_profile_and_uses_bounded_idle_reads():
    from tools.benchmark_arm_session import benchmark
    result = benchmark(samples=25, plans=4)
    assert result["hardware_access"] is False
    assert result["idle_read_registers"] == ["Present_Position", "Torque_Enable"]
    assert result["idle_reads_per_sample"] == 2
    assert result["warm_profile_writes"] == []
    assert result["warm_goal_broadcasts"] == 4
    assert result["warm_read_registers"] == ["Acceleration", "Goal_Position", "Goal_Velocity",
                                             "Present_Position", "Torque_Enable", "Torque_Limit"]
    assert result["publish_p95_ms"] < result["target_period_ms"]


def test_recorder_pairing_reads_latest_only_and_never_scans_history():
    from tools.record_rgbd_experiment import (arm_transition, pair_arm_state,
                                             read_arm_state, snapshot_metadata)
    for function in (read_arm_state, pair_arm_state, arm_transition, snapshot_metadata):
        tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
        for node in ast.walk(tree):
            if isinstance(node, ast.Attribute):
                assert node.attr not in {"glob", "rglob", "iterdir", "scandir", "readlines", "splitlines"}
    from tools.benchmark_arm_session import benchmark_recorder_pairing
    result = benchmark_recorder_pairing(samples=100)
    assert result["hardware_access"] is False
    assert result["latest_json_bytes"] < 4096
    assert result["json_reads_per_sample"] == 1
    assert result["repeated_sample_keyframe_edges"] == 1
    assert result["pairing_p95_ms"] < 50


def test_unlabeled_samples_preserve_last_meaningful_label_without_keyframe_flood():
    from tools.record_rgbd_experiment import arm_transition
    label = event = None
    label_edges = []
    for current_label in ["reach", None, "reach"] * 20:
        token, label, event = arm_transition(
            {"raw": {}, "label": current_label, "event": "sample"},
            previous_label=label, previous_event=event)
        if token is not None:
            label_edges.append(token)
    assert label_edges == [("label", "reach")]


@pytest.mark.parametrize("raw", [{}, {"gripper": 1909},
                                    dict.fromkeys(JOINTS, True)])
def test_malformed_resident_raw_cannot_qualify_camera_pairing(raw):
    from tools.record_rgbd_experiment import pair_arm_state
    result = pair_arm_state({"monotonic": 10., "raw": raw}, 10.01)
    assert result is None or result["qualified_sync"] is False
