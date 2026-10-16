import threading
from copy import deepcopy
import pytest
from workshop.station.ev3_readiness import require_fresh_stationary, stationary_station_snapshot


def sample(stamp=1000):
    return dict(session='s', connected=True, busy=False, power_mode='awake', last_error=None,
        references={'sorter': 'home', 'feeder': 'eject'}, angles={'sorter': 0., 'feeder': 0.}, sampled_ms=stamp,
        motor_control={name: dict(angle_sample_ms=stamp, angle_read_ms=1,
            speed_sample=dict(speed_dps=0., sampled_ms=stamp, duration_ms=1)) for name in ('sorter','feeder')})


def test_both_axes_must_advance_and_remain_stopped():
    clock = [0.]
    frozen = sample()
    class Client:
        def call(self, operation):
            assert operation in ('status', 'critical_status')
            return frozen
    def spin(deadline, cancel):
        clock[0] += .1
    with pytest.raises(TimeoutError):
        require_fresh_stationary(Client(), session='s', deadline=3., cancel=threading.Event(), spin=spin, clock=lambda: clock[0])
    for axis in ('sorter','feeder'):
        moving = sample()
        moving['motor_control'][axis]['speed_sample']['speed_dps'] = 4.
        with pytest.raises(ValueError):
            stationary_station_snapshot(moving, session='s')
    faulty = sample()
    faulty['last_error'] = {}  # Even a malformed/non-null error cannot disappear as falsy.
    with pytest.raises(ValueError):
        stationary_station_snapshot(faulty, session='s')


def test_missing_fault_field_and_slow_status_cannot_pass():
    missing = sample()
    missing.pop('last_error')
    with pytest.raises(ValueError):
        stationary_station_snapshot(missing, session='s')
    clock = [0.]
    class Slow:
        def call(self, operation):
            clock[0] += .8
            return sample()
    with pytest.raises(TimeoutError, match='receipt too slow'):
        require_fresh_stationary(Slow(), session='s', deadline=3., cancel=threading.Event(),
            spin=lambda *_: None, clock=lambda: clock[0])


def test_one_advancing_axis_cannot_hide_the_other_frozen_axis():
    clock = [0.]
    frozen = sample()
    class OneAxis:
        def call(self, operation):
            result = deepcopy(frozen)
            result['motor_control']['sorter']['angle_sample_ms'] += int(clock[0]*1000)
            result['motor_control']['sorter']['speed_sample']['sampled_ms'] += int(clock[0]*1000)
            result['sampled_ms'] += int(clock[0]*1000)
            return result
    def spin(*_):
        clock[0] += .05
    with pytest.raises((TimeoutError, ValueError)):
        require_fresh_stationary(OneAxis(), session='s', deadline=3., cancel=threading.Event(),
            spin=spin, clock=lambda: clock[0])


def test_feeder_must_be_at_eject_endpoint_with_native_terminal_tolerance():
    for angle in (-3., 0., 3.):
        state = sample()
        state['angles']['feeder'] = angle
        stationary_station_snapshot(state, session='s')
    for angle in (-3.01, 3.01, 209.):
        state = sample()
        state['angles']['feeder'] = angle
        with pytest.raises(ValueError, match='eject endpoint'):
            stationary_station_snapshot(state, session='s')
