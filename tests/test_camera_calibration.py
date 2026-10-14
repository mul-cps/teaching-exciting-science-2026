import cv2
import numpy as np
import pytest
from workshop.digital_twin.camera_calibration import (validate_se3, eye_to_hand,
                                                       base_from_camera_from_board)


def pose(vector, translation):
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(np.asarray(vector, dtype=float))[0]
    result[:3, 3] = translation
    return result


def synthetic():
    rng = np.random.default_rng(42)
    extrinsic = pose([.3, -.5, .7], [.3, -.1, .5])
    mount = pose([.1, .2, -.3], [.02, .01, .05])
    base = [pose(rng.normal(0, .6, 3), rng.uniform(-.3, .3, 3)) for _ in range(12)]
    camera = [np.linalg.inv(extrinsic) @ b @ mount for b in base]
    return extrinsic, mount, base, camera


def test_synthetic_recovery_and_heldout():
    expected, mount, base, camera = synthetic()
    report = eye_to_hand(base[:9], camera[:9], heldout=(base[9:], camera[9:]))
    np.testing.assert_allclose(report['base_from_camera'], expected, atol=1e-6)
    for actual in report['gripper_from_target_estimates']:
        np.testing.assert_allclose(actual, mount, atol=1e-6)
    assert not report['qualified']
    assert max(r['translation_m'] for r in report['heldout_closure']) < 1e-6
    assert eye_to_hand(base[:9], camera[:9], heldout=(base[9:], camera[9:]),
                       measured_validation_tol=(1e-6, 1e-6))['qualified']
    assert not eye_to_hand(base, camera, measured_validation_tol=(1e-6, 1e-6))['qualified']


def test_wrong_inverse_cannot_recover():
    expected, _, base, camera = synthetic()
    with pytest.raises(ValueError, match='proper orthogonal'):
        eye_to_hand([np.linalg.inv(b) for b in base], camera)


def test_fixed_board_closure():
    extrinsic, _, _, _ = synthetic()
    target = pose([.2, .1, 0], [.1, .2, .3])
    observed = np.linalg.inv(extrinsic) @ target
    np.testing.assert_allclose(base_from_camera_from_board(target, observed), extrinsic, atol=1e-12)


@pytest.mark.parametrize('matrix', [np.zeros((3, 3)), np.full((4, 4), np.nan),
    np.diag([-1., 1, 1, 1]), np.diag([2., 1, 1, 1]), np.zeros((4, 4))])
def test_invalid_transform(matrix):
    with pytest.raises(ValueError):
        validate_se3(matrix)


def test_degenerate_and_mismatched():
    poses = [pose([0, 0, a], [a, 0, 0]) for a in (.1, .3, .5)]
    with pytest.raises(ValueError, match='degenerate'):
        eye_to_hand(poses, poses)
    with pytest.raises(ValueError):
        eye_to_hand(poses[:2], poses[:2])
    with pytest.raises(ValueError):
        eye_to_hand(poses, poses[:2])
