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
        position_arguments(snapshot(), session='s', target=549)
    for invalid in (-1, 0., 550., 551, float('nan'), True):
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


def test_observed_slow_terminal_read_is_discarded_then_fresh_zero_is_verified():
    import threading
    from workshop.station.sorter_position import critical_status_read, terminal_position
    clock, log, observed = [0.], [], []
    class Proxy:
        def __init__(self): self.durations = [.80,.10]; self.calls = []
        def call(self, operation):
            self.calls.append(operation)
            duration = self.durations.pop(0)
            clock[0] += duration
            backend = min(.39, duration)
            self.last_call_timing = dict(backend_call_s=backend, non_backend_s=duration-backend)
            result = snapshot()
            result['last_job'] = dict(id=1775, operation='motor_target', motor='sorter', state='complete')
            return result
    proxy = Proxy()
    result = critical_status_read(proxy, deadline=6., cancel=threading.Event(), stage='motion_or_terminal',
        slow_reads=[0], record=log.append, observe=lambda result,timing: observed.append(timing), clock=lambda: clock[0])
    assert terminal_position(result, session='s', job_id=1775, target=0., after_stamp=999) == 0.
    assert proxy.calls == ['critical_status', 'critical_status']
    assert len(log) == 1 and log[0]['timing']['measured_roundtrip_s'] == .8
    assert observed[-1]['measured_roundtrip_s'] == pytest.approx(.1)


def test_repeated_slow_terminal_samples_fail_and_cancel_prevents_retry():
    import threading
    from workshop.station.sorter_position import critical_status_read
    for cancel_after_first in (False, True):
        clock, cancel, calls = [0.], threading.Event(), []
        class Proxy:
            last_call_timing = None
            def call(self, operation):
                calls.append(operation)
                clock[0] += .8
                if cancel_after_first:
                    cancel.set()
                return snapshot()
        error = InterruptedError if cancel_after_first else TimeoutError
        with pytest.raises(error):
            critical_status_read(Proxy(), deadline=6., cancel=cancel, stage='motion_or_terminal',
                slow_reads=[0], record=lambda _: None, observe=lambda *_: None, clock=lambda: clock[0])
        assert len(calls) == (1 if cancel_after_first else 2)


def test_exact_boundary_positions_are_rejected_consistently_by_cli_and_arguments():
    for target, reason in ((0., 'ev3-home'), (550., 'not qualified')):
        with pytest.raises(ValueError, match=reason):
            position_arguments(snapshot(), session='s', target=target)
        with pytest.raises(SystemExit) as error:
            parse_args(['sorter-position', str(target)])
        assert error.value.code == 2
    # No unqualified broad inset or stopping-margin claim is introduced.
    validate_target(.001)
    validate_target(549.999)
