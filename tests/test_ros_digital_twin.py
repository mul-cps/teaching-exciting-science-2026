from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from workshop.digital_twin.mujoco_ros import (
    apply_ros_state, compare_tool_frame, effective_urdf_to_mjcf_model,
    model_frame_pose, transform_between,
)
from workshop.digital_twin.ros_state import (
    RosTwinState, resolve_package_mesh_uris, transform_matrix,
)
from workshop.digital_twin.ros_viewer import _initial_camera_pose

sys.path.insert(0, str(Path("ros2/src/so101_station_control").resolve()))
from so101_station_control.model_offsets import apply_gripper_mount_roll


def _tf(parent, child, translation, rotation=(0, 0, 0, 1)):
    return {"parent": parent, "child": child,
            "matrix": transform_matrix(translation, rotation).tolist(), "static": False}


def test_ros_state_rejects_out_of_order_and_marks_stale_inputs():
    state = RosTwinState(max_age_s=.5)
    base = transform_matrix((.2, -.1, .4), (0, 0, 0, 1))
    assert state.update_transform("charuco_board", "base_link", 1_000_000_000,
                                  10., base)
    assert not state.update_transform("charuco_board", "base_link", 900_000_000,
                                      10.1, np.eye(4))
    assert state.update_joints(1_000_000_000, 10., ["shoulder_pan"], [.3])
    assert not state.update_joints(800_000_000, 10.1, ["shoulder_pan"], [.1])
    assert state.snapshot(10.1)["diagnostics"]["freshness"] == "fresh"
    stale = state.snapshot(11.)
    assert stale["diagnostics"]["held"]
    assert stale["transform_fresh"]["base_link"] is False


def test_twin_snapshot_stores_marker_and_grasp_objects_as_stamped_values():
    state = RosTwinState()
    marker = {"frame_id": "charuco_board", "stamp_ns": 1_000_000_000,
              "matrix": np.eye(4).tolist(), "source": "/so101/object_markers"}
    assert state.update_object("/so101/object_markers/colored_objects/0",
                               1_000_000_000, 10.0, marker)
    assert not state.update_object("/so101/object_markers/colored_objects/0",
                                   900_000_000, 10.1, {"source": "older"})
    snapshot = state.snapshot(10.1)
    assert snapshot["objects"]["/so101/object_markers/colored_objects/0"] == marker
    assert snapshot["object_fresh"]["/so101/object_markers/colored_objects/0"] is True
    assert snapshot["object_age_s"]["/so101/object_markers/colored_objects/0"] == pytest.approx(.1)
    stale = state.snapshot(10.6)
    assert stale["object_fresh"]["/so101/object_markers/colored_objects/0"] is False


def test_static_world_base_anchor_does_not_fail_joint_tf_synchrony():
    state = RosTwinState(max_age_s=.5, max_joint_tf_delta_s=.1)
    # A static registration transform is timeless, even if its message stamp
    # predates current arm state by an hour.
    assert state.update_transform("charuco_board", "base_link", 1_000_000_000,
                                  10., np.eye(4), static=True)
    assert state.update_transform("base_link", "shoulder_link", 3_600_000_000_000,
                                  20., np.eye(4))
    assert state.update_joints(3_600_000_000_000, 20., ["shoulder_pan"], [.3])
    snapshot = state.snapshot(20.05)
    assert snapshot["diagnostics"]["freshness"] == "fresh"
    assert snapshot["diagnostics"]["joint_tf_delta_s"] == pytest.approx(0.)
    assert snapshot["diagnostics"]["joint_tf_sync_basis"] == "dynamic_arm_tf_stamps_with_static_base"
    assert snapshot["transforms"]["base_link"]["stamp_ns"] == 1_000_000_000
    assert snapshot["transforms"]["shoulder_link"]["stamp_ns"] == 3_600_000_000_000


def test_dynamic_base_transform_must_remain_synchronized_with_joints():
    state = RosTwinState(max_age_s=.5, max_joint_tf_delta_s=.1)
    assert state.update_transform("charuco_board", "base_link", 1_000_000_000,
                                  10., np.eye(4))
    assert state.update_joints(1_300_000_000, 10., ["shoulder_pan"], [.3])
    snapshot = state.snapshot(10.1)
    assert snapshot["diagnostics"]["freshness"] == "stale_or_unsynchronized"
    assert snapshot["diagnostics"]["joint_tf_delta_s"] == pytest.approx(.3)


def test_fresh_but_old_arm_tf_stamp_is_not_joint_synchronized():
    state = RosTwinState(max_age_s=.5, max_joint_tf_delta_s=.1)
    assert state.update_transform("charuco_board", "base_link", 10_000_000_000,
                                  10., np.eye(4), static=True)
    assert state.update_transform("base_link", "shoulder_link", 9_000_000_000,
                                  10.05, np.eye(4))
    assert state.update_joints(10_000_000_000, 10.05, ["shoulder_pan"], [.3])
    snapshot = state.snapshot(10.1)
    assert snapshot["transform_fresh"]["shoulder_link"] is True
    assert snapshot["diagnostics"]["freshness"] == "stale_or_unsynchronized"
    assert snapshot["diagnostics"]["joint_tf_delta_s"] == pytest.approx(1.)


def test_package_mesh_resolution_preserves_separator_and_exact_asset_path():
    description = ('<robot><mesh filename="package://so101_description/'
                   'meshes/base.stl"/><mesh filename="package://other/meshes/tool.dae"/>'
                   '</robot>')
    resolved = resolve_package_mesh_uris(
        description, lambda package: f"/opt/ros/share/{package}")
    assert 'filename="/opt/ros/share/so101_description/meshes/base.stl"' in resolved
    assert 'filename="/opt/ros/share/other/meshes/tool.dae"' in resolved


@pytest.mark.parametrize("uri", [
    "package://so101_description/../outside.stl",
    "package://so101_description//outside.stl",
])
def test_package_mesh_resolution_rejects_paths_outside_package_share(uri):
    with pytest.raises(ValueError, match="escapes package share"):
        resolve_package_mesh_uris(f'<mesh filename="{uri}"/>',
                                  lambda package: "/opt/ros/share/" + package)


def test_initial_twin_camera_frames_effective_robot_and_ignores_scene_outliers():
    model = mujoco.MjModel.from_xml_string("""
        <mujoco model="initial_camera_bounds">
          <worldbody>
            <body pos="0 0 .2"><geom type="box" size=".1 .1 .1"/></body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    position, target, up = _initial_camera_pose(model, data)
    other_position, other_target, other_up = _initial_camera_pose(model, data)
    assert np.allclose(target, other_target)
    assert np.allclose(position, other_position)
    assert np.allclose(up, other_up)
    assert -1.0 < target[0] < 1.0
    assert np.allclose(up, [0., 0., 1.])
    assert .55 <= np.linalg.norm(position - target) <= .85


def test_tf_graph_composes_nonidentity_base_camera_tool_and_object():
    world_from_base = transform_matrix((.31, -.22, .13),
        Rotation.from_euler("z", 27, degrees=True).as_quat())
    base_from_camera = transform_matrix((.08, .02, .27),
        Rotation.from_euler("xyz", [90, 0, -5], degrees=True).as_quat())
    base_from_tool = transform_matrix((.03, .01, .18),
        Rotation.from_euler("y", 33, degrees=True).as_quat())
    world_from_object = transform_matrix((.6, .1, .02),
        Rotation.from_euler("z", -11, degrees=True).as_quat())
    graph = {
        "base_link": _tf("charuco_board", "base_link", world_from_base[:3, 3],
                          Rotation.from_matrix(world_from_base[:3, :3]).as_quat()),
        "camera_color_optical_frame": _tf("base_link", "camera_color_optical_frame",
                          base_from_camera[:3, 3],
                          Rotation.from_matrix(base_from_camera[:3, :3]).as_quat()),
        "gripper_frame_link": _tf("base_link", "gripper_frame_link", base_from_tool[:3, 3],
                          Rotation.from_matrix(base_from_tool[:3, :3]).as_quat()),
        "object": _tf("charuco_board", "object", world_from_object[:3, 3],
                          Rotation.from_matrix(world_from_object[:3, :3]).as_quat()),
    }
    assert np.allclose(transform_between(graph, "charuco_board", "base_link"), world_from_base)
    assert np.allclose(transform_between(graph, "charuco_board", "camera_color_optical_frame"),
                       world_from_base @ base_from_camera)
    assert np.allclose(transform_between(graph, "charuco_board", "gripper_frame_link"),
                       world_from_base @ base_from_tool)
    assert np.allclose(transform_between(graph, "charuco_board", "object"), world_from_object)


def test_effective_urdf_drives_measured_joint_radians_and_mount_offsets():
    urdf = Path("ros2/.cache/upstream/so101-ros-physical-ai/so101_description/urdf/so101_arm.urdf.xacro")
    if not urdf.is_file():
        pytest.skip("ROS SO-ARM101 description checkout unavailable")
    # The live bridge receives expanded URDF from RSP. Exercise MuJoCo's URDF
    # import with the checked-in canonical URDF after resolving its mesh URIs.
    source = Path("workshop/digital_twin/assets/so101_new_calib.urdf").resolve()
    offsets = yaml.safe_load(Path("ros2/src/so101_station_control/config/model_offsets.yaml").read_text())
    text = apply_gripper_mount_roll(source.read_text(), offsets["gripper_mount_roll_deg"])
    text = text.replace('filename="assets/', 'filename="' + str(source.parent / "cad") + "/")
    model = effective_urdf_to_mjcf_model(text)
    data = mujoco.MjData(model)
    joints = {name: value for name, value in zip(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"),
        (.2, -.4, .3, -.1, .5, .42))}
    for name, value in joints.items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)
    tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_frame_link")
    if tool_id < 0:
        tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_link")
    local_tool_position = data.xpos[tool_id].copy()
    local_tool_rotation = data.xmat[tool_id].reshape(3, 3).copy()
    root_geom_ids = np.flatnonzero(model.geom_bodyid == 0)
    local_root_geom_positions = data.geom_xpos[root_geom_ids].copy()
    local_root_geom_rotations = data.geom_xmat[root_geom_ids].reshape(-1, 3, 3).copy()
    local_link_poses = {
        int(body_id): (data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy())
        for body_id in range(1, model.nbody)
    }
    world_from_base = transform_matrix((.4, -.3, .1),
        Rotation.from_euler("xyz", [8, -13, 41], degrees=True).as_quat())
    snapshot = {"root_frame": "charuco_board", "transforms": {
        "base_link": _tf("charuco_board", "base_link", world_from_base[:3, 3],
                          Rotation.from_matrix(world_from_base[:3, :3]).as_quat())},
        "joint_state": {"fresh": True, "positions_rad": joints},
        "diagnostics": {"freshness": "fresh"}}
    result = apply_ros_state(model, data, snapshot)
    for name, value in joints.items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert data.qpos[model.jnt_qposadr[joint]] == pytest.approx(value)
    assert np.allclose(result["world_from_base"], world_from_base)
    assert result["gripper_physical_mapping_qualified"] is False
    assert "mapping unqualified" in result["gripper_source"]
    expected_tool_position = world_from_base[:3, :3] @ local_tool_position + world_from_base[:3, 3]
    expected_tool_rotation = world_from_base[:3, :3] @ local_tool_rotation
    assert data.xpos[tool_id] == pytest.approx(expected_tool_position)
    assert data.xmat[tool_id].reshape(3, 3) == pytest.approx(expected_tool_rotation)
    expected_root_geom_positions = (local_root_geom_positions @ world_from_base[:3, :3].T
                                    + world_from_base[:3, 3])
    assert data.geom_xpos[root_geom_ids] == pytest.approx(expected_root_geom_positions)
    expected_root_geom_rotations = np.einsum(
        "ij,njk->nik", world_from_base[:3, :3], local_root_geom_rotations)
    assert data.geom_xmat[root_geom_ids].reshape(-1, 3, 3) == pytest.approx(
        expected_root_geom_rotations)
    for body_id, (local_position, local_rotation) in local_link_poses.items():
        assert data.xpos[body_id] == pytest.approx(
            world_from_base[:3, :3] @ local_position + world_from_base[:3, 3])
        assert data.xmat[body_id].reshape(3, 3) == pytest.approx(
            world_from_base[:3, :3] @ local_rotation)
    tool_pose = model_frame_pose(model, data, "gripper_frame_link", text)
    tool_quat = Rotation.from_matrix(tool_pose[:3, :3]).as_quat()
    snapshot["transforms"]["gripper_frame_link"] = _tf(
        "charuco_board", "gripper_frame_link", tool_pose[:3, 3], tool_quat)
    parity = compare_tool_frame(model, data, snapshot, text)
    assert parity["translation_error_m"] == pytest.approx(0., abs=1e-9)
    assert parity["rotation_error_deg"] == pytest.approx(0., abs=1e-5)
    # A missing/doubled tool mount is visible as a 90 degree geodesic error.
    shifted = tool_pose[:3, :3] @ Rotation.from_euler("z", 90, degrees=True).as_matrix()
    snapshot["transforms"]["gripper_frame_link"] = _tf(
        "charuco_board", "gripper_frame_link", tool_pose[:3, 3],
        Rotation.from_matrix(shifted).as_quat())
    parity = compare_tool_frame(model, data, snapshot, text)
    assert parity["rotation_error_deg"] == pytest.approx(90., abs=1e-6)
    # ROS carries the physical jaw hinge angle in radians. It is never mapped
    # through the old offline viewer's percentage-only visual proxy.
    assert joints["gripper"] == pytest.approx(.42)


def test_effective_description_rejects_unresolved_mesh_uri():
    with pytest.raises(ValueError, match="unresolved package mesh URI"):
        effective_urdf_to_mjcf_model(
            '<robot name="x"><link name="base_link"><visual><geometry>'
            '<mesh filename="package://missing/mesh.stl"/>'
            '</geometry></visual></link></robot>')
