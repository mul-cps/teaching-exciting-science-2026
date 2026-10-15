import pytest

from tools.capture_ros_robot_registration import (
    GRIPPER_FRAME,
    closest_settled_joint_state,
    timeout_message,
    validate_frame_triplet,
    validate_tracking_status,
)
from pathlib import Path


def test_frame_triplet_rejects_stale_inputs_and_wrong_frames():
    stamps = {"color": 10.0, "depth": 10.01, "camera_info": 10.02}
    frames = {"color": "camera_color_optical_frame",
              "depth": "camera_color_optical_frame",
              "camera_info": "camera_color_optical_frame"}
    assert validate_frame_triplet(stamps, frames, now=10.2,
                                  expected_frame="camera_color_optical_frame") == 10.0

    with pytest.raises(ValueError, match="stale"):
        validate_frame_triplet(stamps, frames, now=10.6,
                               expected_frame="camera_color_optical_frame")
    frames["depth"] = "camera_depth_optical_frame"
    with pytest.raises(ValueError, match="frame"):
        validate_frame_triplet(stamps, frames, now=10.2,
                               expected_frame="camera_color_optical_frame")


def test_tracking_status_must_be_fresh_and_observationally_valid():
    status = {"state": "tracking", "observation_valid": True, "stamp": 10.0}
    assert validate_tracking_status(status, now=10.2) == 10.0
    with pytest.raises(ValueError, match="fresh"):
        validate_tracking_status(status, now=10.6)
    with pytest.raises(ValueError, match="tracking"):
        validate_tracking_status({**status, "state": "lost"}, now=10.2)


def test_joint_state_requires_nearby_sample_and_quiet_quarter_second_window():
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
             "wrist_roll", "gripper")
    settled = [(stamp, names, (0.1, -0.2, 0.3, 0.0, 0.4, 0.5))
               for stamp in (9.70, 9.80, 9.90, 9.98)]
    result = closest_settled_joint_state(settled, 10.0, names)
    assert result == dict(zip(names, settled[-1][2]))

    moving = settled[:-1] + [(9.98, names, (0.2, -0.2, 0.3, 0.0, 0.4, 0.5))]
    with pytest.raises(ValueError, match="settled"):
        closest_settled_joint_state(moving, 10.0, names)
    with pytest.raises(ValueError, match="nearby"):
        closest_settled_joint_state(settled, 10.2, names)


def test_default_gripper_frame_matches_station_urdf_convention():
    validator = Path("ros2/src/so101_station_scene/so101_station_scene/grasp_validator.py")
    assert GRIPPER_FRAME == "gripper_frame_link"
    assert '"ik_link": "gripper_frame_link"' in validator.read_text()


def test_timeout_message_reports_latest_rejected_input():
    message = timeout_message(
        "exact TF lookup failed",
        received_counts={"color": 4, "depth": 3},
        rejection_counts={"no matching CameraInfo": 1},
        parameter_state="response pending")
    assert "exact TF lookup failed" in message
    assert "color=4" in message and "depth=3" in message
    assert "no matching CameraInfo=1" in message
    assert "robot_description=response pending" in message


def test_capture_writes_complete_bundle_with_actual_description(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace
    import numpy as np
    import pytest
    rclpy = pytest.importorskip("rclpy")
    from sensor_msgs.msg import Image, CameraInfo
    import ament_index_python.packages
    from tools.capture_ros_robot_registration import RegistrationCaptureNode, JOINTS
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    (share / "config/model_offsets.yaml").write_text("gripper_mount_roll_deg: 90.0\n")
    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory", lambda _: str(share))
    output = tmp_path / "bundle"
    output.mkdir()
    rclpy.init(args=[])
    node = RegistrationCaptureNode(output, timeout_s=1)
    try:
        node.robot_description = '<robot name="captured"/>'
        node.tracking = {"state": "tracking", "observation_valid": True, "stamp": 1.0}
        rgb = np.zeros((3, 4, 3), dtype=np.uint8)
        depth = np.full((3, 4), 500, dtype=np.uint16)
        node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda m, desired_encoding: rgb if desired_encoding == "bgr8" else depth)
        color, depth_msg, info = Image(), Image(), CameraInfo()
        depth_msg.encoding = "16UC1"
        info.width, info.height = 4, 3
        transforms = dict.fromkeys(("world_from_camera", "world_from_base", "base_from_gripper_frame"), np.eye(4))
        node._write_capture(color, depth_msg, info, 1.0, dict.fromkeys(JOINTS, 0.), transforms, 1.01)
        assert node.failure is None
        assert (output / "robot.urdf").read_text() == node.robot_description
        assert np.array_equal(np.load(output / "depth.npy"), depth)
        metadata = json.loads((output / "capture.json").read_text())
        assert metadata["joint_positions"] == dict.fromkeys(JOINTS, 0.)
        assert metadata["frames"]["gripper"] == "gripper_frame_link"
    finally:
        node.destroy_node()
        rclpy.shutdown()
