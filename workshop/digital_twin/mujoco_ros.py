"""Synchronize MuJoCo directly from the common RSP inputs used by RViz."""
from __future__ import annotations

import tempfile
from pathlib import Path
from weakref import WeakKeyDictionary
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

_ROOT_BODY_BASELINES = WeakKeyDictionary()
_ROOT_GEOM_BASELINES = WeakKeyDictionary()


def effective_urdf_to_mjcf_model(robot_description: str):
    """Compile the exact effective URDF, resolving its RSP package mesh paths."""
    if not robot_description or "<robot" not in robot_description:
        raise ValueError("effective robot_description is missing or is not URDF")
    root = ET.fromstring(robot_description)
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://"):
            raise ValueError(f"unresolved package mesh URI in effective URDF: {filename}")
        if filename and not Path(filename).is_absolute():
            raise ValueError(f"effective URDF mesh path must be absolute: {filename}")
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", encoding="utf-8") as stream:
        stream.write(robot_description)
        stream.flush()
        model = mujoco.MjModel.from_xml_path(stream.name)
    return model


def transform_between(transforms: dict, target: str, source: str, freshness=None):
    """Resolve target_from_source through the received TF edge graph."""
    target, source = target.lstrip("/"), source.lstrip("/")
    if target == source:
        return np.eye(4)
    # Graph traversal stores source_from_current and composes parent_from_child.
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    for item in transforms.values():
        parent, child = item["parent"], item["child"]
        if freshness is not None and not freshness.get(child, False):
            continue
        matrix = np.asarray(item["matrix"], dtype=float).reshape(4, 4)
        graph.setdefault(child, []).append((parent, matrix))
        graph.setdefault(parent, []).append((child, np.linalg.inv(matrix)))
    pending = [(source, np.eye(4))]
    visited = {source}
    while pending:
        frame, frame_from_source = pending.pop(0)
        for neighbor, neighbor_from_frame in graph.get(frame, []):
            if neighbor in visited:
                continue
            neighbor_from_source = neighbor_from_frame @ frame_from_source
            if neighbor == target:
                return neighbor_from_source
            visited.add(neighbor)
            pending.append((neighbor, neighbor_from_source))
    raise KeyError(f"no TF chain from {source} to {target}")


def apply_ros_state(model, data, snapshot, *, root_frame=None, require_fresh=True):
    """Set MuJoCo joint qpos and base transform without simulation feedback."""
    diagnostics = snapshot.get("diagnostics", {})
    if require_fresh and diagnostics.get("freshness") != "fresh":
        raise ValueError("ROS joint/TF sample is stale or unsynchronized")
    joint_state = snapshot.get("joint_state")
    if not joint_state or not joint_state.get("fresh"):
        raise ValueError("fresh /follower/joint_states sample is required")
    missing = []
    for name, value in joint_state["positions_rad"].items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            continue
        address = int(model.jnt_qposadr[joint_id])
        data.qpos[address] = float(value)
    required = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                "wrist_roll", "gripper")
    for name in required:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
            missing.append(name)
        elif name not in joint_state["positions_rad"]:
            missing.append(name)
    if missing:
        raise ValueError("effective URDF/state lacks required joints: " + ", ".join(missing))
    base_from_root = transform_between(snapshot["transforms"],
                                       root_frame or snapshot.get("root_frame", "charuco_board"),
                                       "base_link", snapshot.get("transform_fresh"))
    from scipy.spatial.transform import Rotation
    world_from_base_quat = Rotation.from_matrix(base_from_root[:3, :3]).as_quat()[[3, 0, 1, 2]]
    # MuJoCo's URDF importer represents the URDF root link as world, folding
    # fixed root-link edges into the direct child body poses. Move every such
    # root body by world_from_base while preserving those folded local offsets.
    direct = np.flatnonzero(model.body_parentid == 0)
    direct = direct[direct != 0]
    if model not in _ROOT_BODY_BASELINES:
        _ROOT_BODY_BASELINES[model] = (direct.copy(), model.body_pos[direct].copy(),
                                       model.body_quat[direct].copy())
    direct, local_pos, local_quat = _ROOT_BODY_BASELINES[model]
    model.body_pos[direct] = local_pos @ base_from_root[:3, :3].T + base_from_root[:3, 3]
    model.body_quat[direct] = np.asarray([
        _quat_multiply(world_from_base_quat, quat) for quat in local_quat])
    root_geoms = np.flatnonzero(model.geom_bodyid == 0)
    if model not in _ROOT_GEOM_BASELINES:
        _ROOT_GEOM_BASELINES[model] = (root_geoms.copy(), model.geom_pos[root_geoms].copy(),
                                       model.geom_quat[root_geoms].copy())
    root_geoms, local_geom_pos, local_geom_quat = _ROOT_GEOM_BASELINES[model]
    model.geom_pos[root_geoms] = (local_geom_pos @ base_from_root[:3, :3].T
                                  + base_from_root[:3, 3])
    model.geom_quat[root_geoms] = np.asarray([
        _quat_multiply(world_from_base_quat, quat) for quat in local_geom_quat],
        dtype=float).reshape((-1, 4))
    mujoco.mj_forward(model, data)
    return {"world_from_base": base_from_root,
            "joint_positions_rad": dict(joint_state["positions_rad"]),
            "gripper_source": "reported ROS gripper joint radians; physical angle mapping unqualified",
            "gripper_physical_mapping_qualified": False,
            "actuation": "none; passive MuJoCo pose update"}


def _quat_multiply(wxyz_a, wxyz_b):
    aw, ax, ay, az = map(float, wxyz_a)
    bw, bx, by, bz = map(float, wxyz_b)
    return np.array([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw])


def ros_pose_to_viewer_xyz(position_m, frame_id, snapshot):
    """Express a ROS-frame metric point in the configured world frame."""
    root = snapshot.get("root_frame", "charuco_board")
    root_from_frame = transform_between(snapshot["transforms"], root, frame_id,
                                        snapshot.get("transform_fresh"))
    return (root_from_frame[:3, :3] @ np.asarray(position_m, dtype=float)
            + root_from_frame[:3, 3])


def model_frame_pose(model, data, frame_id, robot_description):
    """Return a fixed or movable URDF link pose even when MuJoCo merges a fixed link."""
    from scipy.spatial.transform import Rotation
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, frame_id)
    if body_id >= 0:
        matrix = np.eye(4)
        matrix[:3, :3] = data.xmat[body_id].reshape(3, 3)
        matrix[:3, 3] = data.xpos[body_id]
        return matrix
    root = ET.fromstring(robot_description)
    parents = {}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = np.zeros(3) if origin is None else np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.zeros(3) if origin is None else np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        local = np.eye(4)
        local[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        local[:3, 3] = xyz
        parents[child.get("link")] = (parent.get("link"), local)
    chain = []
    cursor = frame_id
    visited = set()
    while cursor not in visited:
        visited.add(cursor)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cursor)
        if body_id >= 0:
            result = np.eye(4)
            result[:3, :3] = data.xmat[body_id].reshape(3, 3)
            result[:3, 3] = data.xpos[body_id]
            for edge in reversed(chain):
                result = result @ edge
            return result
        if cursor not in parents:
            break
        parent, transform = parents[cursor]
        chain.append(transform)
        cursor = parent
    raise KeyError(f"MuJoCo model has no body or fixed-link ancestry for {frame_id}")


def compare_tool_frame(model, data, snapshot, robot_description,
                       frame_id="gripper_frame_link"):
    """Numerically compare passive MuJoCo FK with the received RSP TF pose."""
    root_frame = snapshot.get("root_frame", "charuco_board")
    model_pose = model_frame_pose(model, data, frame_id, robot_description)
    ros_pose = transform_between(snapshot["transforms"], root_frame, frame_id,
                                  snapshot.get("transform_fresh"))
    rotation_error = ros_pose[:3, :3].T @ model_pose[:3, :3]
    angle = np.arccos(np.clip((np.trace(rotation_error) - 1.) / 2., -1., 1.))
    delta = model_pose[:3, 3] - ros_pose[:3, 3]
    return {"frame": frame_id,
            "translation_error_m": float(np.linalg.norm(delta)),
            "rotation_error_matrix": rotation_error.tolist(),
            "rotation_error_deg": float(np.degrees(angle)),
            "ros_pose": ros_pose.tolist(), "mujoco_pose": model_pose.tolist()}
