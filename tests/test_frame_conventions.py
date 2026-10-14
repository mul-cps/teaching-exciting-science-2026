"""Cross-engine regression, official URDF versus official native MuJoCo model."""
import numpy as np
import pytest

from workshop.digital_twin.frame_conventions import (
    NATIVE_SITE_TO_URDF_FRAME, NativeCADPinchProvider,
    convert_native_pinch_offset, frame_provenance,
)
from workshop.digital_twin.ik import ARM_JOINTS, SO101IK


def test_offset_rotation_and_axes_provenance():
    offset = convert_native_pinch_offset([.0059602, .0000590, .0163374])
    np.testing.assert_allclose(offset[:3, 3], [-.0163374, .0000590, .0059602])
    np.testing.assert_array_equal(offset[:3, :3], NATIVE_SITE_TO_URDF_FRAME[:3, :3])
    assert frame_provenance()["pinch_orientation_axes"] == "native-MuJoCo-gripper_frame_site"
    for bad in ([1, 2], [1, 2, np.nan]):
        with pytest.raises(ValueError):
            convert_native_pinch_offset(bad)


@pytest.mark.parametrize("values", [
    [0, 0, 0, 0, 0],
    [23, -35, 41, -21, 15],
    [-31, 22, -24, 30, -47],
])
def test_cross_engine_site_and_pinch(values):
    mujoco = pytest.importorskip("mujoco")
    if not SO101IK.available():
        pytest.skip("optional LeRobot/Placo backend unavailable")
    from workshop.digital_twin.scene import make_model
    from workshop.digital_twin.kinematics import set_pose, export_pinch_geometry
    cal = {name: {"range_min": 800, "range_max": 3300} for name in (*ARM_JOINTS, "gripper")}
    raw = {name: int(round(2050 + value * 4095 / 360)) for name, value in zip(ARM_JOINTS, values)}
    raw["gripper"] = 1300  # exactly 20% calibrated opening
    with SO101IK(cal, lambda percent: np.eye(4)) as solver:
        urdf_pose = solver.forward_raw(raw)
        degrees, opening = solver._raw_degrees(raw)
    model = mujoco.MjModel.from_xml_string(make_model([0, 0, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    set_pose(model, data, {**dict(zip(ARM_JOINTS, degrees)), "gripper": opening})
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper_frame_site")
    native_pose = np.eye(4)
    native_pose[:3, :3] = data.site_xmat[site].reshape(3, 3)
    native_pose[:3, 3] = data.site_xpos[site]
    converted = urdf_pose @ NATIVE_SITE_TO_URDF_FRAME
    assert np.linalg.norm(converted[:3, 3] - native_pose[:3, 3]) <= .00001
    angle = np.arccos(np.clip((np.trace(converted[:3, :3].T @ native_pose[:3, :3]) - 1) / 2, -1, 1))
    assert np.rad2deg(angle) <= .01
    geometry = export_pinch_geometry(model, data)
    offset = convert_native_pinch_offset(geometry["pinch_center_gripper_site_m"])
    predicted = urdf_pose @ offset
    assert np.linalg.norm(predicted[:3, 3] - geometry["pinch_center_world_m"]) <= .00001
    # Verify the provider, not just a manually converted result.
    provider = NativeCADPinchProvider()
    np.testing.assert_allclose(provider(opening), offset, atol=1e-7)


def test_provider_opening_range_and_qualification():
    pytest.importorskip("mujoco")
    provider = NativeCADPinchProvider()
    for opening in (0, 20, 30):
        assert provider(opening).shape == (4, 4)
    for opening in (-1, 31, float("nan")):
        with pytest.raises(ValueError):
            provider(opening)
    assert not provider.provenance()["grasp_eligible"]
    assert provider.provenance()["opening_percent_range"] == [0, 30]
