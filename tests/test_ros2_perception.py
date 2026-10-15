import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="Optional ROS perception tests run in the RoboStack environment")


REPOSITORY = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = REPOSITORY / "ros2/src/so101_station_perception"
sys.path.insert(0, str(PACKAGE_SOURCE))

from so101_station_perception.calibration import (  # noqa: E402
    load_calibration_frames,
    matrix_quaternion,
    rigid_matrix,
)
from so101_station_perception.calibration_tf import (  # noqa: E402
    compose_base_from_camera_link,
)
from so101_station_perception.diagnostic_cloud import (  # noqa: E402
    MAX_POINTS,
    quaternion_matrix,
    transform_xyz_data,
)
from so101_station_perception.tag_observations import (  # noqa: E402
    depth_center_point,
    ensure_aruco_detector_api,
    select_region_detection,
    timestamps_paired,
)
from so101_station_perception.evidence_sink import (  # noqa: E402
    arm_pair_record,
    read_resident_state,
)


def _write(path, schema, direction, field, matrix, qualified=False):
    path.write_text(json.dumps({
        "schema": schema, "transform_direction": direction,
        field: np.asarray(matrix).tolist(), "qualified": qualified,
    }))


def test_unqualified_calibration_stays_in_diagnostic_subtree_and_keeps_directions(tmp_path):
    camera_from_target = np.eye(4)
    camera_from_target[:3, 3] = [0.1, 0.2, 0.5]
    base_from_camera = np.eye(4)
    base_from_camera[:3, 3] = [0.3, -0.4, 0.2]
    target = tmp_path / "target.json"
    arm = tmp_path / "arm.json"
    _write(target, "camera-target-pose-v1", "camera_from_target",
           "camera_from_target", camera_from_target)
    _write(arm, "camera-arm-calibration-v1", "base_from_camera",
           "base_from_camera", base_from_camera)

    transforms, driver_tf = load_calibration_frames(target, arm)

    assert driver_tf is True
    assert [(item["parent"], item["child"]) for item in transforms] == [
        ("base_link", "so101_diag_base"),
        ("so101_diag_base", "so101_diag_camera_optical"),
        ("so101_diag_camera_optical", "so101_diag_charuco_target"),
    ]
    np.testing.assert_allclose(transforms[0]["matrix"], np.eye(4))
    np.testing.assert_allclose(transforms[1]["matrix"], base_from_camera)
    np.testing.assert_allclose(transforms[2]["matrix"], camera_from_target)
    assert all(item["qualified"] is False for item in transforms)


def test_qualified_opt_in_requires_qualified_exact_direction_and_disables_camera_tf(tmp_path):
    matrix = np.eye(4)
    arm = tmp_path / "arm.json"
    _write(arm, "camera-arm-calibration-v1", "base_from_camera",
           "base_from_camera", matrix, qualified=True)
    transforms, driver_tf = load_calibration_frames(
        arm_path=arm, qualified_mode=True, base_frame="base_link",
        optical_frame="camera_color_optical_frame")
    assert driver_tf is True
    assert [(t["parent"], t["child"]) for t in transforms] == [
        ("base_link", "camera_color_optical_frame")]
    _write(arm, "camera-arm-calibration-v1", "camera_from_base",
           "base_from_camera", matrix, qualified=True)
    with pytest.raises(ValueError, match="base_from_camera"):
        load_calibration_frames(arm_path=arm, qualified_mode=True)


def test_qualified_camera_transform_composes_at_camera_link_without_reparenting_optical():
    base_from_link = np.eye(4)
    base_from_link[:3, 3] = [.4, -.2, .1]
    link_from_optical = np.eye(4)
    link_from_optical[:3, 3] = [.01, .02, .03]
    base_from_optical = base_from_link @ link_from_optical
    composed = compose_base_from_camera_link(base_from_optical, link_from_optical)
    np.testing.assert_allclose(composed, base_from_link, atol=1e-12)


@pytest.mark.parametrize("matrix", [
    np.diag([1.0, 1.0, 1.0, 2.0]),
    np.diag([-1.0, 1.0, 1.0, 1.0]),
    np.eye(4) + np.array([[0, .01, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]),
])
def test_nonrigid_or_reflected_calibration_is_rejected(matrix):
    with pytest.raises(ValueError, match=r"proper rigid SE\(3\)"):
        rigid_matrix(matrix)


def test_matrix_quaternion_round_trip_rotation():
    angle = math.pi / 2
    matrix = np.eye(4)
    matrix[:3, :3] = [[math.cos(angle), -math.sin(angle), 0],
                      [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
    x, y, z, w = matrix_quaternion(matrix)
    assert [x, y, z, w] == pytest.approx([0, 0, math.sqrt(.5), math.sqrt(.5)])


def test_diagnostic_cloud_transform_is_vectorized_preserves_other_fields_and_nan():
    from types import SimpleNamespace
    import struct

    fields = [SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
              for name, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))]
    data = bytearray(2 * 16)
    for i, point in enumerate(((1, 2, 3, 0x11223344), (math.nan, 5, 6, 0x55667788))):
        struct.pack_into("<fffI", data, i * 16, *point)
    output = transform_xyz_data(
        bytes(data), height=1, width=2, point_step=16, row_step=32,
        fields=fields, is_bigendian=False, rotation=np.eye(3), translation=[10, 20, 30])
    x, y, z, rgb = struct.unpack_from("<fffI", output, 0)
    assert (x, y, z, rgb) == pytest.approx((11, 22, 33, 0x11223344))
    x2, y2, z2, rgb2 = struct.unpack_from("<fffI", output, 16)
    assert math.isnan(x2) and (y2, z2, rgb2) == (5, 6, 0x55667788)
    assert len(output) == len(data) and MAX_POINTS == 640 * 480


def test_tf_quaternion_conversion_is_proper_rotation():
    rotation = quaternion_matrix(0, 0, math.sqrt(.5), math.sqrt(.5))
    np.testing.assert_allclose(rotation, [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)


def test_duplicate_tag_ids_are_separated_by_roi_and_temporal_identity():
    corners_a = [[10, 10], [20, 10], [20, 20], [10, 20]]
    corners_b = [[12, 11], [22, 11], [22, 21], [12, 21]]
    far = [[100, 100], [110, 100], [110, 110], [100, 110]]
    assert select_region_detection([{"code_id": 0, "corners_px": corners_a}]) is not None
    assert select_region_detection([
        {"code_id": 0, "corners_px": corners_a},
        {"code_id": 0, "corners_px": corners_b},
    ]) is None
    selected = select_region_detection([{"code_id": 0, "corners_px": corners_b}],
                                       previous_centers=[[15, 15]])
    assert selected["corners_px"] == corners_b
    assert select_region_detection([{"code_id": 0, "corners_px": far}],
                                   previous_centers=[[15, 15]]) is None


def test_marker_pairing_rejects_timestamp_skew_and_invalid_depth():
    assert timestamps_paired(10.0, 10.03)
    assert not timestamps_paired(10.0, 10.041)
    depth = np.zeros((3, 3), dtype=np.uint16)
    intrinsics = {"fx": 600, "fy": 600, "ppx": 1, "ppy": 1}
    assert depth_center_point([1, 1], depth, intrinsics, .001) is None
    depth[0:3, 0:3] = 500
    assert depth_center_point([1, 1], depth, intrinsics, .001) == pytest.approx((0, 0, .5))


def test_opencv_aruco_api_compatibility_shim_supports_legacy_detector():
    class LegacyAruco:
        @staticmethod
        def DetectorParameters_create():
            return "params"

        @staticmethod
        def detectMarkers(image, dictionary, parameters=None):
            return image, dictionary, parameters

    ensure_aruco_detector_api(LegacyAruco)
    detector = LegacyAruco.ArucoDetector("DICT", LegacyAruco.DetectorParameters())
    assert detector.detectMarkers("frame") == ("frame", "DICT", "params")


def test_resident_state_pairing_fails_closed_for_stale_or_unmatched_sample(tmp_path):
    state_path = tmp_path / "arm-resident-state.json"
    raw = dict.fromkeys(("shoulder_pan", "shoulder_lift", "elbow_flex",
                         "wrist_flex", "wrist_roll", "gripper"), 2000)
    sample_wall = 1000.0
    sample = {"schema": 1, "event": "sample", "timestamp_utc":
              datetime.fromtimestamp(sample_wall, timezone.utc).isoformat(),
              "monotonic": 100.0, "arm_id": "hs_rig01_follower", "raw": raw}
    state_path.write_text(json.dumps(sample))
    assert read_resident_state(state_path) == sample
    paired = arm_pair_record(sample, frame_wall=1000.1, frame_monotonic=100.1,
                             now_monotonic=100.12)
    assert paired["qualified_sync"] is True
    assert paired["delta_s"] == pytest.approx(.1)
    stale = arm_pair_record(sample, frame_wall=1001.0, frame_monotonic=101.0,
                            now_monotonic=101.0)
    assert stale["qualified_sync"] is False
    assert read_resident_state(state_path, max_bytes=16) is None


def test_evidence_sink_rate_is_hard_capped_at_five_hz():
    from so101_station_perception.evidence_sink import bounded_write_period

    assert bounded_write_period(5.0) == pytest.approx(.2)
    assert bounded_write_period(30.0) == pytest.approx(.2)
    with pytest.raises(ValueError, match="positive finite"):
        bounded_write_period(0.0)


def test_evidence_hot_path_has_fixed_read_and_write_bounds():
    import ast
    import inspect
    from so101_station_perception import evidence_sink

    source = inspect.getsource(evidence_sink.read_resident_state)
    tree = ast.parse(source)
    read_calls = [node for node in ast.walk(tree)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                  and node.func.attr == "read"]
    assert len(read_calls) == 1
    assert len(read_calls[0].args) == 1
    assert ast.unparse(read_calls[0].args[0]) == "max_bytes + 1"
    callback = inspect.getsource(evidence_sink.EvidenceSink._images)
    assert "self.min_period" in callback
    assert "atomic_npy" in callback
    assert "pyrealsense2" not in callback
    period = inspect.getsource(evidence_sink.bounded_write_period)
    assert "5.0" in period


def test_tag_observation_callback_does_not_open_device_or_write_history():
    import inspect
    from so101_station_perception.tag_observations import TagObservations

    callback = inspect.getsource(TagObservations._images)
    assert "pyrealsense2" not in callback
    assert "open(" not in callback
    assert "DELETEALL" in callback
    assert "StaticTransformBroadcaster" not in callback


def test_realsense_profile_is_low_bandwidth_aligned_and_keeps_native_tf():
    import yaml

    config = yaml.safe_load((REPOSITORY / "ros2/config/realsense.yaml").read_text())
    assert config["rgb_camera.color_profile"] == "640,480,15"
    assert config["depth_module.depth_profile"] == "640,480,15"
    assert config["align_depth.enable"] is True
    assert config["pointcloud.enable"] is True
    assert config["publish_tf"] is True


def test_station_rviz_config_contains_robot_camera_cloud_tags_and_diagnostic_frames():
    import yaml

    path = PACKAGE_SOURCE / "config/station_live.rviz"
    config = yaml.safe_load(path.read_text())
    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "base_link"
    displays = config["Visualization Manager"]["Displays"]
    text = str(displays)
    for value in ("RobotModel", "/follower/robot_description", "/camera/color/image_raw",
                  "/camera/depth/color/points", "/so101_perception/diagnostic/points",
                  "/so101_perception/tag_observations", "TF"):
        assert value in text


def test_diagnostic_cloud_hot_path_is_rate_limited_and_has_no_point_loop():
    import ast
    import inspect
    from so101_station_perception import diagnostic_cloud

    source = inspect.getsource(diagnostic_cloud.transform_xyz_data)
    tree = ast.parse(source)
    loops = [node for node in ast.walk(tree) if isinstance(node, (ast.For, ast.While))]
    assert not any(isinstance(node, ast.While) for node in loops)
    iterators = [ast.unparse(node.iter) for node in loops]
    assert "enumerate(coordinates)" in iterators
    assert "('x', 'y', 'z')" in iterators
    callback = inspect.getsource(diagnostic_cloud.DiagnosticCloud._cloud)
    assert "self.min_period" in callback
    assert "5.0" in inspect.getsource(diagnostic_cloud.DiagnosticCloud.__init__)
