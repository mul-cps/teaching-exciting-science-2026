import json
from datetime import datetime, timezone

from tools.record_rgbd_experiment import (MAX_ARM_PAIRING_DELTA_S,
                                         arm_transition, atomic_json,
                                         manifest_pose_raw, pair_arm_state,
                                         read_arm_state,
                                         snapshot_metadata)


def resident_state(monotonic=10.0, *, event="sample", label="pick-cube"):
    return {
        "schema": 1,
        "event": event,
        "timestamp_utc": datetime(2026, 10, 14, tzinfo=timezone.utc).isoformat(),
        "monotonic": monotonic,
        "raw": {joint: 2000 for joint in (
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
            "wrist_roll", "gripper")},
        "pose": {"shoulder_pan": 0.0},
        "label": label,
    }


def test_pairs_camera_frame_with_recent_actual_resident_state():
    state = resident_state()
    paired = pair_arm_state(state, 10.12)
    assert paired["monotonic"] == 10.0
    assert paired["timestamp_utc"] == state["timestamp_utc"]
    assert paired["raw"] == state["raw"]
    assert paired["pose"] == state["pose"]
    assert round(paired["delta_s"], 2) == 0.12
    assert paired["qualified_sync"] is True


def test_stale_sample_is_attached_but_not_marked_synchronized():
    paired = pair_arm_state(resident_state(9.8), 10.0)
    assert paired["raw"] is not None
    assert paired["qualified_sync"] is False
    assert paired["delta_s"] > MAX_ARM_PAIRING_DELTA_S


def test_unlabeled_telemetry_sample_does_not_create_repeated_keyframes():
    state = resident_state(label=None)
    token, label, event = arm_transition(state)
    assert token is None
    assert label is None
    assert event is None
    token, label, event = arm_transition(state, previous_label=label,
                                         previous_event=event)
    assert token is None
    assert label is None
    assert event is None


def test_label_and_lifecycle_edges_create_one_keyframe_each():
    first = resident_state()
    first["phase"] = "move_plan"
    token, label, event = arm_transition(first,
                                         previous_label=None,
                                         previous_event=None)
    assert token == ("label", "pick-cube")
    token, label, event = arm_transition(resident_state(10.05),
                                         previous_label=label,
                                         previous_event=event)
    assert token is None
    token, label, event = arm_transition(resident_state(
        10.1, event="plan_complete"), previous_label=label,
        previous_event=event)
    assert token == ("event", "plan_complete")
    token, label, event = arm_transition(resident_state(10.15),
                                         previous_label=label,
                                         previous_event=event)
    assert token is None


def test_legacy_phase_file_transition_and_atomic_latest_json(tmp_path):
    state_path = tmp_path / "state.json"
    atomic_json(state_path, {"phase": "reach", "event_id": 7,
                             "monotonic": 3.0, "pose_raw": {"joint": 1}})
    state = read_arm_state(state_path)
    assert arm_transition(state)[0] == ("legacy", "reach", 7)
    paired = pair_arm_state(state, 3.02)
    assert paired["raw"] == {"joint": 1}
    assert paired["qualified_sync"] is True
    assert not state_path.with_name("state.json.tmp").exists()
    assert json.loads(state_path.read_text())["phase"] == "reach"


def test_manifest_pose_raw_exposes_only_qualified_resident_raw_counts():
    state = resident_state()
    fresh = pair_arm_state(state, 10.02)
    stale = pair_arm_state(state, 10.3)
    assert manifest_pose_raw(state, fresh) == state["raw"]
    assert manifest_pose_raw(state, stale) is None
    assert manifest_pose_raw({"pose_raw": {"joint": 1}}, None) == {"joint": 1}


def test_fake_camera_frame_metadata_keeps_acquisition_and_arm_provenance():
    timing = {"monotonic": 12.0, "utc": "2026-10-14T00:00:00+00:00",
              "device_ms": 500.0, "depth_device_ms": 499.0}
    arm = pair_arm_state(resident_state(11.96), timing["monotonic"])
    frame_metadata = snapshot_metadata(4, "frames/000004.png",
                                       "frames/000004.npy", timing, arm)
    assert frame_metadata["camera_acquisition"]["monotonic"] == 12.0
    assert frame_metadata["camera_acquisition"]["timestamp_utc"] == timing["utc"]
    assert frame_metadata["paths"]["snapshot_color"] == "frames/000004.png"
    assert frame_metadata["paths"]["snapshot_depth"] == "frames/000004.npy"
    assert frame_metadata["arm"]["raw"] == resident_state()["raw"]
    assert frame_metadata["arm"]["qualified_sync"] is True
