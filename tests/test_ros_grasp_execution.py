"""Offline qualification and independent pickup-evidence tests."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from tools.ros_plan_grasp import (load_execution_calibration, verify_camera_arm_calibration,
                                  verify_effective_model, verify_observed_lift,
                                  verify_observed_release, validate_model_links,
                                  robot_description_from_parameter_response,
                                  ros_message_json, validate_clear_joint_pose,
                                  verify_live_registration, scene_ack_terminal_state)
from tools.ros_plan_grasp import (canonical_scene_track_id, scene_request_payload,
                                  require_live_registration)


def test_localization_expiring_during_server_discovery_prevents_goal_submission():
    from tools.ros_plan_grasp import submit_localized_goal
    import time
    sent = []
    client = SimpleNamespace(send_goal_async=lambda goal: sent.append(goal))
    stale = {"state": "tracking", "observation_valid": True, "stamp": 10.0}
    with pytest.raises(RuntimeError, match="immediately before arm motion"):
        submit_localized_goal(client, object(), registration=stale,
                              received_at=time.monotonic()-.6, now_ros=10.6,
                              deadline=time.monotonic()+1.)
    assert not sent


def test_expired_execution_deadline_prevents_a_freshly_localized_goal():
    from tools.ros_plan_grasp import submit_localized_goal
    import time
    sent = []
    client = SimpleNamespace(send_goal_async=lambda goal: sent.append(goal))
    fresh = {"state": "tracking", "observation_valid": True, "stamp": 10.0}
    with pytest.raises(TimeoutError):
        submit_localized_goal(client, object(), registration=fresh,
                              received_at=time.monotonic(), now_ros=10.0,
                              deadline=time.monotonic()-.01)
    assert not sent


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
           "grasp_frame_convention": "x_closing_z_approach",
           "tip_from_grasp": np.eye(4).tolist(),
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
    missing_grasp = {key: value for key, value in tcp.items() if key != "tip_from_grasp"}
    with pytest.raises(ValueError, match="grasp"):
        load_execution_calibration(jaw_path, _write(tmp_path / "missing-grasp.json", missing_grasp),
                                   tip_link="gripper_frame_link")
    with pytest.raises(ValueError, match="grasp"):
        load_execution_calibration(jaw_path, _write(tmp_path / "wrong-frame.json", {
            **tcp, "grasp_frame_convention": "unspecified"}), tip_link="gripper_frame_link")


def test_pickup_evidence_requires_same_track_three_stable_elevated_samples():
    base = __import__("time").monotonic()
    def sample(z, stamp):
        registration = {"state": "tracking", "observation_valid": True, "stamp": stamp}
        return (base, {"state": "tracking", "frame_id": "world", "stamp": stamp,
                       "registration": registration, "registration_received": base,
                       "objects": [{"track_id": 3, "position_world": [0.1, 0.2, z],
                                    "geometric_only": True}]})

    samples = [sample(.15, 10.01), sample(.151, 10.02), sample(.149, 10.03)]
    assert verify_observed_lift(samples, 3, [.1, .2, .1])
    assert verify_observed_lift(samples, 3, [.1, .2, .1], now_ros=10.04,
                                expected_frame_id="world")
    assert not verify_observed_lift(samples, 3, [.1, .2, .1], now_ros=11.0)
    assert not verify_observed_lift(samples, 3, [.1, .2, .1], expected_frame_id="camera")
    no_registration = [(received, {**status, "registration": {"state": "stale"}})
                      for received, status in samples]
    assert not verify_observed_lift(no_registration, 3, [.1, .2, .1])
    assert verify_observed_lift(samples, 3, [.1, .2, .1],
                                expected_position=[.1, .2, .15])
    assert not verify_observed_lift(samples, 3, [.1, .2, .1],
                                    expected_position=[.3, .2, .15])
    assert not verify_observed_lift(samples[:2], 3, [.1, .2, .1])
    assert not verify_observed_lift([sample(.15, 10.01), sample(.08, 10.02), sample(.151, 10.03)], 3, [.1, .2, .1])
    assert not verify_observed_lift(samples, 4, [.1, .2, .1])
    assert not verify_observed_lift([sample(.15, 10.01), sample(.15, 10.01), sample(.15, 10.01)], 3, [.1, .2, .1])


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
    base = __import__("time").monotonic()
    samples = [(base, {"state": "tracking", "frame_id": "world", "stamp": 10.01 + i * .01,
                       "registration": {"state": "tracking", "observation_valid": True,
                                        "stamp": 10.01 + i * .01},
                       "registration_received": base,
                       "objects": [{"track_id": 3, "position_world": [0.3, 0.1, 0.2],
                                    "geometric_only": True}]}) for i in range(3)]
    assert verify_observed_release(samples, 3, [.3, .1, .2])
    assert not verify_observed_release(samples[:2], 3, [.3, .1, .2])
    assert not verify_observed_release(samples, 4, [.3, .1, .2])
    assert not verify_observed_release(samples, 3, [.5, .1, .2])


def test_tool_and_touch_links_must_exist_in_effective_robot_description():
    model = '<robot name="test"><link name="tool"/><link name="finger"/></robot>'
    assert validate_model_links(model, ["tool", "finger"])
    with pytest.raises(ValueError, match="absent from effective robot model"):
        validate_model_links(model, ["missing"])


def test_robot_description_reads_native_get_parameters_response_values():
    try:
        from rcl_interfaces.msg import ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters
    except ImportError:
        native_response_shape = SimpleNamespace(values=[
            SimpleNamespace(string_value="<robot name='live'/>")])
    else:
        native_response_shape = GetParameters.Response()
        native_response_shape.values = [ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value="<robot name='live'/>")]
    native_response_shape_legacy = SimpleNamespace(values=[
        SimpleNamespace(string_value="<robot name='live'/>"),
    ])
    assert robot_description_from_parameter_response(native_response_shape) == "<robot name='live'/>"
    with pytest.raises(ValueError, match="unavailable"):
        robot_description_from_parameter_response(SimpleNamespace(values=[]))


def test_native_ros_trajectory_message_serializes_as_json():
    pytest.importorskip("moveit_msgs.msg")
    pytest.importorskip("trajectory_msgs.msg")
    from moveit_msgs.msg import RobotTrajectory
    message = RobotTrajectory()
    message.joint_trajectory.joint_names = ["shoulder_pan"]
    point = __import__("trajectory_msgs.msg", fromlist=["JointTrajectoryPoint"]).JointTrajectoryPoint()
    point.positions = [0.25]
    message.joint_trajectory.points = [point]
    encoded = ros_message_json(message)
    assert encoded["joint_trajectory"]["joint_names"] == ["shoulder_pan"]
    assert encoded["joint_trajectory"]["points"][0]["positions"] == [0.25]


def test_lift_release_require_live_registration_and_joint_clear_pose_is_exact():
    from tools.ros_plan_grasp import ARM_JOINTS
    base = __import__("time").monotonic()
    registration = {"state": "tracking", "observation_valid": True, "stamp": 10.0}
    assert verify_live_registration(registration, base, base + 0.1)
    assert not verify_live_registration(registration, base, base + 0.5)
    # A planning request can be admitted with fresh localization, then lose it
    # before the returned path is submitted to the arm action.
    with pytest.raises(RuntimeError, match="immediately before arm motion"):
        require_live_registration(registration, base, now=base + 0.6)
    assert validate_clear_joint_pose({name: 0.0 for name in ARM_JOINTS})
    with pytest.raises(ValueError, match="exactly the five"):
        validate_clear_joint_pose({"shoulder_pan": 0.0})


def test_scene_ack_waits_through_reserved_and_accepts_terminal_ack():
    assert scene_ack_terminal_state("attach", {"state": "reserved"}) is None
    assert scene_ack_terminal_state("attach", {"state": "applying"}) is None
    assert scene_ack_terminal_state("attach", {"state": "attached"}) == "attached"
    with pytest.raises(RuntimeError, match="scene attach failed"):
        scene_ack_terminal_state("attach", {"state": "failed", "reason": "apply failed"})


def test_scene_transport_uses_canonical_detection_id_for_every_ownership_action():
    candidate = {"candidate_id": "red-0003-g1", "track_id": 3}
    scene_id = canonical_scene_track_id(candidate)
    assert scene_id == "red-0003"
    requests = [scene_request_payload(action, scene_id, f"request-{i}")
                for i, action in enumerate(("lock", "allow_contact", "attach",
                                            "disallow_contact", "detach", "unlock"))]
    assert {request["track_id"] for request in requests} == {"red-0003"}
    with pytest.raises(ValueError, match="canonical track"):
        scene_request_payload("lock", 3, "numeric-is-not-scene-id")
