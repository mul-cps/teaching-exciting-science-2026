"""Offline qualification and independent pickup-evidence tests."""
import json

import numpy as np
import pytest

from tools.ros_plan_grasp import (load_execution_calibration, verify_camera_arm_calibration,
                                  verify_effective_model, verify_observed_lift,
                                  verify_observed_release, validate_model_links)


def _write(path, data):
    path.write_text(json.dumps(data))
    return str(path)


def test_execution_requires_qualified_si_jaw_and_tcp_identity(tmp_path):
    jaw = {"schema": "so101-jaw-gap-calibration-v1", "qualified": True,
           "closed_gap_m": 0.004, "open_gap_m": 0.075,
           "closed_percent": 0, "open_percent": 100}
    tcp = {"schema": "so101-grasp-tcp-calibration-v1", "qualified": True,
           "tcp_link": "gripper_frame_link",
           "units": {"translation": "m", "rotation": "quaternion_xyzw"},
           "tcp_from_object": np.eye(4).tolist()}
    jaw_path = _write(tmp_path / "jaw.json", jaw)
    tcp_path = _write(tmp_path / "tcp.json", tcp)
    loaded_jaw, loaded_tcp, matrix = load_execution_calibration(
        jaw_path, tcp_path, tip_link="gripper_frame_link")
    assert loaded_jaw["open_gap_m"] == pytest.approx(0.075)
    assert loaded_tcp["qualified"] is True
    np.testing.assert_allclose(matrix, np.eye(4))
    with pytest.raises(ValueError, match="match --tip-link"):
        load_execution_calibration(jaw_path, tcp_path, tip_link="other_link")


def test_pickup_evidence_requires_same_track_three_stable_elevated_samples():
    def sample(z):
        return {"objects": [{"track_id": 3, "position_world": [0.1, 0.2, z],
                              "geometric_only": True}]}

    assert verify_observed_lift([sample(.15), sample(.151), sample(.149)], 3, [.1, .2, .1])
    assert verify_observed_lift([sample(.15), sample(.151), sample(.149)], 3, [.1, .2, .1],
                                expected_position=[.1, .2, .15])
    assert not verify_observed_lift([sample(.15), sample(.151), sample(.149)], 3, [.1, .2, .1],
                                    expected_position=[.3, .2, .15])
    assert not verify_observed_lift([sample(.15), sample(.151)], 3, [.1, .2, .1])
    assert not verify_observed_lift([sample(.15), sample(.08), sample(.151)], 3, [.1, .2, .1])
    assert not verify_observed_lift([sample(.15), sample(.151), sample(.149)], 4, [.1, .2, .1])


def test_execution_requires_qualified_camera_registration_and_matching_model_digest(tmp_path):
    from hashlib import sha256

    camera = {"schema": "camera-arm-calibration-v1", "qualified": True,
              "transform_direction": "base_from_camera", "base_from_camera": np.eye(4).tolist()}
    path = _write(tmp_path / "camera.json", camera)
    assert verify_camera_arm_calibration(path)["qualified"]
    with pytest.raises(ValueError, match="qualified camera-arm"):
        verify_camera_arm_calibration(_write(tmp_path / "bad.json", {**camera, "qualified": False}))
    model = "<robot name='qualified'/>"
    digest = sha256(model.encode()).hexdigest()
    assert verify_effective_model(model, digest) == digest
    with pytest.raises(ValueError, match="does not match"):
        verify_effective_model(model, "0" * 64)


def test_release_requires_three_geometric_observations_at_calibrated_place():
    samples = [{"objects": [{"track_id": 3, "position_world": [0.3, 0.1, 0.2],
                             "geometric_only": True}]} for _ in range(3)]
    assert verify_observed_release(samples, 3, [.3, .1, .2])
    assert not verify_observed_release(samples[:2], 3, [.3, .1, .2])
    assert not verify_observed_release(samples, 4, [.3, .1, .2])
    assert not verify_observed_release(samples, 3, [.5, .1, .2])


def test_tool_and_touch_links_must_exist_in_effective_robot_description():
    model = '<robot name="test"><link name="tool"/><link name="finger"/></robot>'
    assert validate_model_links(model, ["tool", "finger"])
    with pytest.raises(ValueError, match="absent from effective robot model"):
        validate_model_links(model, ["missing"])
