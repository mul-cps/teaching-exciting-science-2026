import math

import pytest

from workshop.station.ros_trajectory import (ARM_JOINTS, PreparedTrajectory, Sample,
                                             dense_path_samples, path_digest,
                                             prepare_trajectory)


def test_prepare_trajectory_reorders_joints_and_makes_smooth_samples():
    order = list(reversed(ARM_JOINTS))
    points = [
        {"time": 0.0, "positions": [0.0] * 5, "velocities": [0.0] * 5,
         "accelerations": [0.8] * 5},
        {"time": 1.0, "positions": [0.02] * 5, "velocities": [0.0] * 5,
         "accelerations": [-0.8] * 5},
    ]
    result = prepare_trajectory(order, points, start_radians={j: 0.0 for j in ARM_JOINTS},
                                max_velocity=1.0, max_acceleration=2.0,
                                period=0.05)
    assert result.input_joint_names == tuple(order)
    assert result.duration == 1.0
    assert len(result.samples) == 21
    assert result.samples[0].positions == (0.0,) * 5
    assert result.samples[-1].positions == pytest.approx((0.02,) * 5)
    assert result.ignored_derivatives is True
    assert all(math.isfinite(v) for sample in result.samples for v in sample.positions)


def test_prepare_trajectory_rejects_nonzero_start_mismatch():
    points = [{"time": 0.0, "positions": [0.1] * 5},
              {"time": 1.0, "positions": [0.2] * 5}]
    with pytest.raises(ValueError, match="start position"):
        prepare_trajectory(ARM_JOINTS, points, start_radians={j: 0.0 for j in ARM_JOINTS},
                           max_velocity=1.0, max_acceleration=2.0, period=0.05)


def test_prepare_trajectory_rejects_invalid_time_and_joint_names():
    points = [{"time": 0.0, "positions": [0.0] * 5},
              {"time": 0.0, "positions": [0.1] * 5}]
    with pytest.raises(ValueError, match="strictly increasing"):
        prepare_trajectory(ARM_JOINTS, points, start_radians={j: 0.0 for j in ARM_JOINTS},
                           max_velocity=1.0, max_acceleration=2.0, period=0.05)
    with pytest.raises(ValueError, match="exactly the five arm joints"):
        prepare_trajectory(ARM_JOINTS + ("gripper",), [], start_radians={},
                           max_velocity=1.0, max_acceleration=2.0, period=0.05)


def test_prepare_trajectory_rejects_velocity_limit_violation():
    points = [{"time": 0.0, "positions": [0.0] * 5},
              {"time": 0.1, "positions": [0.5] * 5}]
    with pytest.raises(ValueError, match="velocity limit"):
        prepare_trajectory(ARM_JOINTS, points, start_radians={j: 0.0 for j in ARM_JOINTS},
                           max_velocity=0.1, max_acceleration=10.0, period=0.05)


def test_final_command_stays_on_twenty_hz_grid_for_fractional_duration():
    points = [{"time": 0.0, "positions": [0.0] * 5},
              {"time": 0.98, "positions": [0.01] * 5}]
    result = prepare_trajectory(ARM_JOINTS, points,
                               start_radians={j: 0.0 for j in ARM_JOINTS},
                               max_velocity=0.125, max_acceleration=0.2,
                               period=0.05)
    times = [sample.time for sample in result.samples]
    assert times[-1] == pytest.approx(1.0)
    assert all(b - a >= 0.05 - 1e-9 for a, b in zip(times, times[1:]))
    assert result.samples[-1].positions == pytest.approx((0.01,) * 5)


def test_jerk_is_checked_analytically():
    points = [{"time": 0.0, "positions": [0.0] * 5},
              {"time": 1.0, "positions": [0.1] * 5}]
    with pytest.raises(ValueError, match="jerk limit"):
        prepare_trajectory(ARM_JOINTS, points,
                           start_radians={joint: 0.0 for joint in ARM_JOINTS},
                           max_velocity=1.0, max_acceleration=2.0, max_jerk=2.0)


def test_dense_collision_path_is_bounded_and_digest_is_stable():
    points = [{"time": 0.0, "positions": [0.0] * 5},
              {"time": 1.0, "positions": [0.03] * 5}]
    sparse = PreparedTrajectory(ARM_JOINTS, 1.0,
                                (Sample(0.0, (0.0,) * 5), Sample(1.0, (0.03,) * 5)),
                                tuple(), False)
    dense = dense_path_samples(sparse, max_joint_step_rad=0.01)
    assert len(dense) > len(sparse.samples)
    assert all(max(abs(b - a) for a, b in zip(left, right)) <= 0.010000001
               for left, right in zip(dense, dense[1:]))
    result = prepare_trajectory(ARM_JOINTS,
                                [{"time": 0.0, "positions": [0.0] * 5},
                                 {"time": 1.0, "positions": [0.01] * 5}],
                                start_radians={joint: 0.0 for joint in ARM_JOINTS},
                                max_velocity=0.125, max_acceleration=0.2, max_jerk=2.0)
    assert path_digest(result) == path_digest(result)
    with pytest.raises(ValueError, match="exceed"):
        dense_path_samples(sparse, max_joint_step_rad=1e-5, max_states=5)
