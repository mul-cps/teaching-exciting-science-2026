import numpy as np
import pytest

from tools.fit_ros_robot_registration import (forward_kinematics, parse_urdf,
                                               refine_point_to_plane,
                                               closest_surface_metrics,
                                               robust_rigid_fit)
import trimesh
from scipy.spatial.transform import Rotation


def test_robust_rigid_fit_recovers_bounded_transform_with_depth_outliers():
    rng = np.random.default_rng(81)
    source = rng.normal(size=(300, 3)) * [.12, .06, .04]
    angle = np.deg2rad(4.0)
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    expected = np.eye(4)
    expected[:3, :3] = rotation
    expected[:3, 3] = [.025, -.018, .012]
    target = source @ rotation.T + expected[:3, 3]
    target += rng.normal(scale=.0007, size=target.shape)
    target[:45] += rng.normal(scale=.04, size=(45, 3))

    fitted, inliers = robust_rigid_fit(source, target)

    assert inliers.sum() >= 240
    np.testing.assert_allclose(fitted, expected, atol=.0015)


def test_forward_kinematics_applies_urdf_origin_before_joint_motion():
    xml = """<robot name='tiny'>
      <link name='base'/><link name='arm'/>
      <joint name='pan' type='revolute'>
        <origin xyz='1 0 0' rpy='0 0 0'/><parent link='base'/><child link='arm'/>
        <axis xyz='0 0 1'/>
      </joint>
    </robot>"""
    links, joints = parse_urdf(xml)
    transforms = forward_kinematics(links, joints, {"pan": np.pi / 2})
    np.testing.assert_allclose(transforms["arm"][:3, 3], [1, 0, 0], atol=1e-10)
    np.testing.assert_allclose(transforms["arm"][:3, :3],
                               [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-10)


def test_bounded_point_to_plane_refinement_improves_fixed_observed_population():
    mesh = trimesh.creation.icosphere(subdivisions=3)
    mesh.vertices *= [.15, .07, .035]
    points, face_ids = trimesh.sample.sample_surface(mesh, 4000, seed=2)
    normals = mesh.face_normals[face_ids]
    initial = np.r_[ [.009, -.006, .004],
                     Rotation.from_euler("xyz", [2, -1.5, 2], degrees=True).as_rotvec() ]
    expected_rotation = Rotation.from_rotvec(initial[3:]).as_matrix()
    observed = points @ expected_rotation.T + initial[:3]
    observed += np.random.default_rng(3).normal(0, .0006, observed.shape)
    links = np.full(len(points), "ellipsoid")
    before, _, _ = closest_surface_metrics(points, normals, links, observed, np.zeros(6))

    fitted, history = refine_point_to_plane(points, normals, observed, initial,
                                             translation_limit_m=.03,
                                             rotation_limit_deg=6, iterations=8)
    after, _, _ = closest_surface_metrics(points, normals, links, observed, fitted)

    assert len(history) >= 2
    assert np.linalg.norm(fitted[:3]) <= .03
    assert np.rad2deg(np.linalg.norm(fitted[3:])) <= 6
    assert after["median_surface_residual_m"] < before["median_surface_residual_m"] * .3
    assert after["p95_surface_residual_m"] < before["p95_surface_residual_m"] * .3
