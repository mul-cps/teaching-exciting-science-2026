import math
import pytest
from workshop.station.sorter_position import position_arguments, stationary_snapshot, validate_target
from tools.ros_station_action import parse_args


def snapshot():
    return dict(session='s', connected=True, busy=False, power_mode='awake', last_error=None,
        references={'sorter': 'home'}, angles={'sorter': 0}, sampled_ms=1000,
        motor_control={'sorter': {'angle_sample_ms': 1000, 'angle_read_ms': 1,
        'speed_sample': {'speed_dps': 0, 'sampled_ms': 1001, 'duration_ms': 1}}})


def test_fixed_profile_and_conservative_distance_budget():
    assert position_arguments(snapshot(), session='s', target=200) == dict(
        name='sorter', target=200., speed=50., acceleration=400., actuation=30.)
    with pytest.raises(ValueError, match='six-second'):
        position_arguments(snapshot(), session='s', target=550)
    for invalid in (-1, 551, float('nan'), True):
        with pytest.raises(ValueError):
            validate_target(invalid)
    assert parse_args(['sorter-position', '200']).target_deg == 200


def test_reference_session_awake_idle_and_finite_telemetry_required():
    for changed in (dict(session='different'), dict(power_mode='power_save'), dict(busy=True),
                    dict(references={'sorter': None}), dict(connected=False), dict(angles={'sorter': 551})):
        with pytest.raises(ValueError):
            stationary_snapshot(dict(snapshot(), **changed), session='s')
    stale = snapshot()
    stale['motor_control']['sorter']['angle_sample_ms'] = 0
    with pytest.raises(ValueError):
        stationary_snapshot(stale, session='s')


def test_typed_pin_commands_allow_current_frame_or_explicit_generation():
    assert parse_args(['pin-anchor']).generation == ''
    assert parse_args(['pin-area', 'frozen-generation']).generation == 'frozen-generation'


def test_terminal_rejects_wrong_session_job_nonzero_speed_and_endpoint_error():
    from copy import deepcopy
    from workshop.station.sorter_position import terminal_position
    good = snapshot()
    good['last_job'] = dict(id=7, operation='motor_target', motor='sorter', state='complete')
    assert terminal_position(good, session='s', job_id=7, target=0., after_stamp=999) == 0.
    for changed in (dict(session='wrong'), dict(last_job=dict(good['last_job'], id=8)),
                    dict(last_job=dict(good['last_job'], cleanup_fault='brake failed')),
                    dict(angles={'sorter': 4.})):
        with pytest.raises(ValueError):
            terminal_position(dict(good, **changed), session='s', job_id=7, target=0., after_stamp=999)
    moving = deepcopy(good)
    moving['motor_control']['sorter']['speed_sample']['speed_dps'] = 4.
    with pytest.raises(ValueError):
        terminal_position(moving, session='s', job_id=7, target=0., after_stamp=999)


def test_frozen_encoder_fault_and_native_tolerance_diagnostic():
    from workshop.station.sorter_position import motion_progress, control_diagnostics
    status = snapshot()
    with pytest.raises(TimeoutError, match='stopped advancing'):
        motion_progress(status, session='s', previous_stamp=1000, progress_time=0., now=.751)
    with pytest.raises(ValueError, match='session'):
        motion_progress(status, session='wrong', previous_stamp=999, progress_time=0., now=.1)
    assert motion_progress(status, session='s', previous_stamp=999, progress_time=0., now=.1) == (1000,.1)
    status['motor_control']['sorter']['target_tolerances'] = [50, 5]
    assert control_diagnostics(status)['position_tolerance_exceeds_terminal_gate']
    status['motor_control']['sorter']['target_tolerances'] = [50, 3]
    assert not control_diagnostics(status)['position_tolerance_exceeds_terminal_gate']
