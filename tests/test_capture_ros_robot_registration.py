import pytest

from tools.capture_ros_robot_registration import (
    GRIPPER_FRAME,
    JOINTS,
    closest_settled_joint_state,
    camera_identity_provenance,
    owner_state_provenance,
    timeout_message,
    _station_identity,
    validate_station_identity_snapshot,
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


def test_camera_identity_is_bound_to_device_info_parameter_and_expected_serial():
    info = {"serial_number": "243322071783", "firmware_version": "5.17.0.10"}
    bound = camera_identity_provenance(
        expected_serial="243322071783", device_info=info,
        parameter_serial="_243322071783", require_bound=True)
    assert bound["bound"] is True
    assert bound["serial_number"] == "243322071783"
    assert bound["firmware_version"] == "5.17.0.10"

    with pytest.raises(ValueError, match="serial"):
        camera_identity_provenance(
            expected_serial="243322071783", device_info=info,
            parameter_serial="_other", require_bound=True)

    diagnostic = camera_identity_provenance(
        expected_serial=None, device_info=None, parameter_serial=None,
        require_bound=False)
    assert diagnostic["bound"] is False
    assert diagnostic["reason"]


def test_owner_state_is_selected_by_acquisition_stamp_and_bound_to_arm_calibration():
    from datetime import datetime, timezone
    import math

    stamp = 1_797_000_000.0
    joints = {name: math.radians(index * 5.0)
              for index, name in enumerate(JOINTS[:-1])}
    owner = {
        "schema": 1, "event": "sample", "arm_id": "station-arm",
        "calibration_sha256": "a" * 64,
        "timestamp_utc": datetime.fromtimestamp(stamp + 0.01, timezone.utc).isoformat(),
        "monotonic": 123.0,
        "raw": dict.fromkeys(JOINTS, 2048),
        "pose": {**{name: math.degrees(value) for name, value in joints.items()},
                 "gripper": 50.0},
    }
    result = owner_state_provenance(
        [owner], image_stamp=stamp, joint_positions={**joints, "gripper": 0.0},
        expected_arm_id="station-arm", expected_calibration_sha256="a" * 64,
        now_utc=datetime.fromtimestamp(stamp + 0.02, timezone.utc), require_bound=True)
    assert result["bound"] is True
    assert result["pair_delta_s"] == pytest.approx(0.01)
    assert result["arm_joint_max_error_rad"] == pytest.approx(0.0)
    assert result["physical_gripper_mapping_qualified"] is False
    assert result["raw"] == dict.fromkeys(JOINTS, 2048)


def test_strict_owner_binding_rejects_stale_mismatched_or_inconsistent_state():
    from datetime import datetime, timezone

    owner = {
        "schema": 1, "event": "sample", "arm_id": "station-arm",
        "calibration_sha256": "a" * 64,
        "timestamp_utc": "2026-10-16T10:00:00+00:00", "monotonic": 123.0,
        "raw": dict.fromkeys(JOINTS, 2048),
        "pose": dict.fromkeys(JOINTS, 0.0),
    }
    stamp = datetime(2026, 10, 16, 10, 0, 0, tzinfo=timezone.utc).timestamp()
    with pytest.raises(ValueError, match="stale"):
        owner_state_provenance(
            [owner], image_stamp=stamp,
            joint_positions=dict.fromkeys(JOINTS, 0.0),
            expected_arm_id="station-arm", expected_calibration_sha256="a" * 64,
            now_utc=datetime.fromtimestamp(stamp + 1.0, timezone.utc), require_bound=True)
    with pytest.raises(ValueError, match="identity"):
        owner_state_provenance(
            [owner], image_stamp=stamp,
            joint_positions=dict.fromkeys(JOINTS, 0.0),
            expected_arm_id="different-arm", expected_calibration_sha256="a" * 64,
            now_utc=datetime.fromtimestamp(stamp, timezone.utc), require_bound=True)
    with pytest.raises(ValueError, match="identity"):
        owner_state_provenance(
            [owner], image_stamp=stamp,
            joint_positions=dict.fromkeys(JOINTS, 0.0),
            expected_arm_id="station-arm", expected_calibration_sha256="b" * 64,
            now_utc=datetime.fromtimestamp(stamp, timezone.utc), require_bound=True)
    with pytest.raises(ValueError, match="disagree"):
        owner_state_provenance(
            [owner], image_stamp=stamp,
            joint_positions={**dict.fromkeys(JOINTS, 0.0), "shoulder_pan": 0.2},
            expected_arm_id="station-arm", expected_calibration_sha256="a" * 64,
            now_utc=datetime.fromtimestamp(stamp, timezone.utc), require_bound=True)


def test_station_config_is_parsed_and_hashed_from_the_same_bytes(tmp_path, monkeypatch):
    import hashlib
    import json

    config_path = tmp_path / "station.json"
    original = json.dumps({"arm": {"id": "arm-a"}}).encode()
    replacement = json.dumps({"arm": {"id": "arm-b"}}).encode()
    config_path.write_bytes(original)
    read_bytes = Path.read_bytes

    def race_replacement(path):
        data = read_bytes(path)
        if path == config_path:
            path.write_bytes(replacement)
        return data

    monkeypatch.setattr(Path, "read_bytes", race_replacement)
    identity = _station_identity(config_path)
    assert identity["arm_id"] == "arm-a"
    assert identity["station_config_sha256"] == hashlib.sha256(original).hexdigest()
    assert config_path.read_bytes() == replacement


def test_station_identity_change_between_binding_and_write_is_rejected():
    initial = {"arm_id": "arm-a", "station_config_sha256": "a" * 64,
               "calibration_sha256": "b" * 64,
               "configured_calibration_sha256": "b" * 64}
    validate_station_identity_snapshot(initial, dict(initial))
    with pytest.raises(ValueError, match="changed"):
        validate_station_identity_snapshot(
            initial, {**initial, "calibration_sha256": "c" * 64})


def test_strict_capture_requires_explicit_station_owner_and_camera_identity(tmp_path):
    import subprocess
    import sys

    output = tmp_path / "should-not-be-created"
    result = subprocess.run(
        [sys.executable, "tools/capture_ros_robot_registration.py",
         "--output-directory", str(output), "--require-bound-provenance"],
        capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert "requires --station-config, --state-file, and --expected-camera-serial" in result.stderr
    assert not output.exists()


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
        assert metadata["schema"] == 2
        assert metadata["qualified"] is False
        assert metadata["provenance"]["qualification_inputs_bound"] is False
        assert metadata["provenance"]["physical_gripper_mapping_qualified"] is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_strict_capture_rejects_station_change_before_writing_bundle(tmp_path, monkeypatch):
    import hashlib
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
    monkeypatch.setattr(ament_index_python.packages, "get_package_share_directory",
                        lambda _: str(share))
    calibration = tmp_path / "calibration.json"
    calibration.write_text('{"calibration": true}\n')
    digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
    station = tmp_path / "station.json"
    station.write_text(json.dumps({"arm": {"id": "station-arm", "calibration": {
        "path": str(calibration), "sha256": digest}}}))
    output = tmp_path / "bundle"
    output.mkdir()

    rclpy.init(args=[])
    node = RegistrationCaptureNode(
        output, timeout_s=1, station_config=str(station),
        require_bound_provenance=True)
    try:
        node.robot_description = '<robot name="captured"/>'
        rgb = np.zeros((3, 4, 3), dtype=np.uint8)
        depth = np.full((3, 4), 500, dtype=np.uint16)
        node.bridge = SimpleNamespace(
            imgmsg_to_cv2=lambda m, desired_encoding: rgb if desired_encoding == "bgr8" else depth)
        color, depth_msg, info = Image(), Image(), CameraInfo()
        depth_msg.encoding = "16UC1"
        info.width, info.height = 4, 3
        transforms = dict.fromkeys(("world_from_camera", "world_from_base",
                                    "base_from_gripper_frame"), np.eye(4))
        station.write_text(json.dumps({"arm": {"id": "replaced-arm", "calibration": {
            "path": str(calibration), "sha256": digest}}}))
        node._write_capture(color, depth_msg, info, 1.0,
                            dict.fromkeys(JOINTS, 0.0), transforms, 1.01,
                            provenance={"qualification_inputs_bound": True})
        assert isinstance(node.failure, ValueError)
        assert "source changed" in str(node.failure)
        assert list(output.iterdir()) == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
