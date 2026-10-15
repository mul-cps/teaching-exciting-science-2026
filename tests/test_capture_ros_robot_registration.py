import pytest

from tools.capture_ros_robot_registration import (
    closest_settled_joint_state,
    validate_frame_triplet,
    validate_tracking_status,
)


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
               for stamp in (9.75, 9.82, 9.90, 9.98)]
    result = closest_settled_joint_state(settled, 10.0, names)
    assert result == dict(zip(names, settled[-1][2]))

    moving = settled[:-1] + [(9.98, names, (0.2, -0.2, 0.3, 0.0, 0.4, 0.5))]
    with pytest.raises(ValueError, match="settled"):
        closest_settled_joint_state(moving, 10.0, names)
    with pytest.raises(ValueError, match="nearby"):
        closest_settled_joint_state(settled, 10.2, names)
