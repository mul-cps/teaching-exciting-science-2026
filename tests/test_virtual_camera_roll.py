"""In-place 180-degree optical roll, with unchanged eye/forward/intrinsics."""
import ast
from pathlib import Path

import numpy as np
import pytest

from workshop.digital_twin.virtual_camera import tool_from_virtual_optical, virtual_intrinsics

ROOT = Path(__file__).resolve().parents[1]


def previous_mount(axis, backoff):
    directions = {"+X": (1., 0., 0.), "-X": (-1., 0., 0.),
                  "+Y": (0., 1., 0.), "-Y": (0., -1., 0.),
                  "+Z": (0., 0., 1.), "-Z": (0., 0., -1.)}
    forward = np.asarray(directions[axis])
    up = np.array([0., 0., 1.]) if axis.endswith("Y") else np.array([0., -1., 0.])
    mount = np.eye(4)
    mount[:3, :3] = np.column_stack((np.cross(forward, up), -up, forward))
    mount[:3, 3] = -forward * backoff
    return mount


@pytest.mark.parametrize("axis", ["+X", "-X", "+Y", "-Y", "+Z", "-Z"])
@pytest.mark.parametrize("backoff", [0., .04, .15])
def test_camera_roll_is_exactly_180_in_place(axis, backoff):
    old = previous_mount(axis, backoff)
    new = tool_from_virtual_optical(axis, backoff)
    roll = np.diag([-1., -1., 1., 1.])
    assert new == pytest.approx(old @ roll)
    assert new[:3, 3] == pytest.approx(old[:3, 3])
    assert new[:3, 2] == pytest.approx(old[:3, 2])
    assert np.linalg.det(new[:3, :3]) == pytest.approx(1.)


def test_roll_reprojects_real_geometry_about_principal_point_without_changing_depth():
    old, new = previous_mount("+Z", .04), tool_from_virtual_optical()
    k = virtual_intrinsics()
    points = np.array([[.01, .02, .2, 1.], [-.005, .003, .005, 1.]])
    before, after = points @ np.linalg.inv(old).T, points @ np.linalg.inv(new).T
    a, b = before[:, :3] @ k.T, after[:, :3] @ k.T
    assert after[:, 2] == pytest.approx(before[:, 2])
    assert b[:, :2] / b[:, 2:] == pytest.approx(2 * k[:2, 2] - a[:, :2] / a[:, 2:])
    assert k == pytest.approx(np.array([[207.84609690826528, 0, 159.5],
                                       [0, 207.84609690826528, 119.5], [0, 0, 1]]))


def test_browser_and_ros_use_shared_mount_without_image_rotation():
    for filename in ("workshop/digital_twin/ros_viewer.py",
                     "ros2/src/so101_station_perception/so101_station_perception/virtual_gripper_camera.py"):
        source = (ROOT / filename).read_text()
        assert "tool_from_virtual_optical(" in source
        assert "np.rot90(" not in source and "cv2.rotate(" not in source
    tree = ast.parse((ROOT / "workshop/digital_twin/virtual_camera.py").read_text())
    mount = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "tool_from_virtual_optical")
    assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.GeneratorExp)) for n in ast.walk(mount))
