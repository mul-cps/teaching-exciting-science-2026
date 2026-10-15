import json
import numpy as np
import pytest
import trimesh
from scipy.spatial.transform import Rotation

from tools.fit_ros_robot_joint_offsets import (
    fit_joint_offsets,
    forward_kinematics,
    validate_pose_diversity,
)
from tools.fit_ros_robot_registration import parse_urdf


def _tiny_chain():
    xml = """<robot name='tiny'>
      <link name='base'/><link name='shoulder'/><link name='elbow'/><link name='wrist'/>
      <joint name='shoulder_lift' type='revolute'><origin xyz='0 0 0.04'/>
        <parent link='base'/><child link='shoulder'/><axis xyz='0 1 0'/></joint>
      <joint name='elbow_flex' type='revolute'><origin xyz='0.12 0 0'/>
        <parent link='shoulder'/><child link='elbow'/><axis xyz='0 1 0'/></joint>
      <joint name='wrist_flex' type='revolute'><origin xyz='0.10 0 0'/>
        <parent link='elbow'/><child link='wrist'/><axis xyz='1 0 0'/></joint>
    </robot>"""
    return parse_urdf(xml)


def _frame(links, joints, points_by_link, normals_by_link, positions, offsets, base_delta):
    q = {name: value + offsets.get(name, 0.0) for name, value in positions.items()}
    fk = forward_kinematics(links, joints, q)
    rotation = Rotation.from_rotvec(base_delta[3:]).as_matrix()
    chunks = []
    for link, points in points_by_link.items():
        world = points @ fk[link][:3, :3].T + fk[link][:3, 3]
        chunks.append(world @ rotation.T + base_delta[:3])
    return {
        "links": links, "joints": joints, "joint_positions": positions,
        "points_by_link": points_by_link, "normals_by_link": normals_by_link,
        "observed_base": np.concatenate(chunks),
    }


def test_diversity_rejects_repeated_joint_pose_snapshots():
    frames = [{"joint_positions": {"shoulder_lift": 0.2, "elbow_flex": -0.3,
                                   "wrist_flex": 0.1}} for _ in range(4)]
    with pytest.raises(ValueError, match="distinct training poses"):
        validate_pose_diversity(frames, holdout_index=3)


def test_pan_zero_cannot_be_selected_because_it_shares_base_yaw_gauge():
    with pytest.raises(ValueError, match="gauge-equivalent"):
        fit_joint_offsets([], holdout_index=0, offset_names=("shoulder_pan",))


def test_multiview_fit_recovers_joint_zero_bias_and_shared_base_delta():
    links, joints = _tiny_chain()
    rng = np.random.default_rng(731)
    points_by_link, normals_by_link = {}, {}
    for index, link in enumerate(("base", "shoulder", "elbow", "wrist")):
        mesh = trimesh.creation.box(extents=(0.055 + index * .007, .035, .025))
        mesh.apply_translation((.018 * index, .012, .006 * index))
        points, face_ids = trimesh.sample.sample_surface(mesh, 450, seed=10 + index)
        points_by_link[link] = points
        normals_by_link[link] = mesh.face_normals[face_ids]
    positions = [
        {"shoulder_lift": -.42, "elbow_flex": .55, "wrist_flex": -.30},
        {"shoulder_lift": -.10, "elbow_flex": -.22, "wrist_flex": .36},
        {"shoulder_lift": .31, "elbow_flex": .17, "wrist_flex": -.51},
        {"shoulder_lift": .52, "elbow_flex": -.48, "wrist_flex": .12},
    ]
    expected_offsets = {"shoulder_lift": np.deg2rad(2.4),
                        "elbow_flex": np.deg2rad(-3.1),
                        "wrist_flex": np.deg2rad(1.8)}
    expected_base = np.r_[ [.006, -.004, .003],
                           Rotation.from_euler("xyz", [1.2, -1.0, 1.4], degrees=True).as_rotvec() ]
    frames = [_frame(links, joints, points_by_link, normals_by_link, q,
                     expected_offsets, expected_base) for q in positions]
    # Add realistic sub-millimetre depth noise to otherwise exact CAD samples.
    for frame in frames:
        frame["observed_base"] += rng.normal(0, .00025, frame["observed_base"].shape)

    result = fit_joint_offsets(frames, holdout_index=3, offset_names=tuple(expected_offsets),
                               samples_per_frame=900, max_nfev=45)

    assert result["status"] == "unqualified_multiview_candidate"
    for name, expected in expected_offsets.items():
        assert abs(result["joint_zero_offsets_rad"][name] - expected) < np.deg2rad(1.0)
    np.testing.assert_allclose(result["base_delta"], expected_base, atol=.0018)
    assert result["training_before"]["median_surface_residual_m"] > .002
    assert result["training_after"]["median_surface_residual_m"] < .0015
    assert result["holdout_after"]["median_surface_residual_m"] < .0015
    assert set(result["holdout_after"]["per_link"]) == {"base", "shoulder", "elbow", "wrist"}
    json.dumps(result)
