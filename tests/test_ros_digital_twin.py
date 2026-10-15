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
from workshop.digital_twin.ros_state import RosTwinState, transform_matrix

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
    expected_tool_position = world_from_base[:3, :3] @ local_tool_position + world_from_base[:3, 3]
    expected_tool_rotation = world_from_base[:3, :3] @ local_tool_rotation
    assert data.xpos[tool_id] == pytest.approx(expected_tool_position)
    assert data.xmat[tool_id].reshape(3, 3) == pytest.approx(expected_tool_rotation)
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
