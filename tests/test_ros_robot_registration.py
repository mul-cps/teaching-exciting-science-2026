import numpy as np
import pytest

from tools.fit_ros_robot_registration import (forward_kinematics, parse_urdf,
                                               robust_rigid_fit)


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
