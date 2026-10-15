import math
import pytest
from tools.run_ros_calibration_sweep import JOINTS, offsets_to_reference, sweep_poses


def test_sweep_contains_independent_axes_holdout_and_reference_return():
    poses = sweep_poses(7)
    assert len(poses) == 7
    assert {next(iter(offsets)) for _, offsets in poses[:5]} == set(JOINTS)
    assert poses[-1] == ('return-reference', {})
    assert len(poses[-2][1]) == 3


def test_offsets_follow_measured_pose_and_reject_large_return():
    reference = dict.fromkeys(JOINTS, math.radians(20))
    measured = dict.fromkeys(JOINTS, 25.)
    offsets = offsets_to_reference(reference, measured, {'elbow_flex': math.radians(7)})
    assert offsets['elbow_flex'] == pytest.approx(math.radians(2))
    assert offsets['wrist_roll'] == pytest.approx(math.radians(-5))
    with pytest.raises(ValueError):
        offsets_to_reference(reference, dict.fromkeys(JOINTS, 80.), {})


@pytest.mark.parametrize('amplitude', [0, 3, 9, math.nan])
def test_sweep_rejects_outside_excitation_bounds(amplitude):
    with pytest.raises(ValueError):
        sweep_poses(amplitude)
