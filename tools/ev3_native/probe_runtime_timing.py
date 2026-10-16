#!/usr/bin/env pybricks-micropython
"""Timing-helper regression for the actual VM; imports no hardware drivers.

Run beside live_api.py with CPython or brickrun -r -- pybricks-micropython.
It uses fake clocks, axes, sensors, and peripheral call counters only. The persistent native API should be stopped
before brickrun takes its graphics console; no physical motors are constructed.
"""
import sys
import live_api


class Probe:
    def __init__(self):
        self.ms = 0
        self.records = []

    def clock(self):
        return self.ms

    def record_timing(self, job, name, elapsed):
        self.records.append((name, elapsed))

    def success(self, value):
        self.ms += 5
        return value

    def failure(self):
        self.ms += 7
        raise ValueError('expected probe failure')


class FakeControl:
    def __init__(self, axis):
        self.axis = axis
        self.settings = (800, 1600, 100)

    def limits(self, *args, **kwargs):
        if args:
            self.settings = tuple(args)
        if kwargs:
            self.settings = (kwargs['speed'], kwargs['acceleration'], kwargs['actuation'])
        return self.settings

    def target_tolerances(self):
        return (50, 5)

    def stall_tolerances(self):
        return (15, 200)

    def done(self):
        return self.axis.target is None or self.axis.value == self.axis.target

    def stalled(self):
        return False


class FakeAxis:
    def __init__(self):
        self.value = 0
        self.target = None
        self.angle_reads = 0
        self.speed_reads = 0
        self.dispatches = 0
        self.control = FakeControl(self)

    def angle(self):
        self.angle_reads += 1
        return self.value

    def speed(self):
        self.speed_reads += 1
        return 0

    def brake(self):
        self.target = None

    def run_target(self, speed, target, then, wait):
        assert then == 'BRAKE' and wait is False
        self.dispatches += 1
        self.target = target

    def advance(self):
        if self.target is not None and self.value != self.target:
            self.value += 1 if self.target > self.value else -1


class FakePeripherals:
    def __init__(self):
        self.calls = 0
        self.touch_reads = 0
        self.screen = self
        self.speaker = self

    def pressed(self):
        self.touch_reads += 1
        return False

    def color(self):
        self.calls += 1
        return 'fake'

    def clear(self):
        self.calls += 1

    def draw_text(self, *args):
        self.calls += 1

    def set_volume(self, *args):
        self.calls += 1

    def beep(self, **kwargs):
        self.calls += 1


class FakeStop:
    BRAKE = 'BRAKE'


class CriticalProbeAxis(FakeAxis):
    def __init__(self, clock):
        FakeAxis.__init__(self)
        self.sample_clock = clock

    def angle(self):
        self.sample_clock.ms += 2
        return FakeAxis.angle(self)

    def speed(self):
        self.sample_clock.ms += 3
        return FakeAxis.speed(self)


def critical_status_probe():
    clock = Probe()
    sorter, feeder = CriticalProbeAxis(clock), CriticalProbeAxis(clock)
    peripherals = FakePeripherals()
    api = live_api.LiveAPI({'sorter': sorter, 'feeder': feeder}, peripherals,
                          peripherals, peripherals, clock.clock, FakeStop,
                          'portable-fake-critical')
    api.connect()
    sorter.value = 13
    clock.ms += 400
    started = clock.ms
    reply = api.request({'id': 1, 'operation': 'critical_status', 'args': {}})
    assert reply['ok']
    status = reply['result']
    assert status['angles']['sorter'] == 13
    assert status['motor_control']['sorter']['angle_sample_ms'] == started + 2
    assert status['motor_control']['sorter']['speed_sample']['sampled_ms'] == started + 7
    assert status['motor_control']['feeder']['angle_sample_ms'] == started + 4
    assert status['motor_control']['feeder']['speed_sample']['sampled_ms'] == started + 10
    assert status['sampled_ms'] == started + 4
    assert status['critical_sample']['finished_ms'] == started + 10
    assert peripherals.calls == 0 and peripherals.touch_reads == 0
    assert sorter.target is None and feeder.target is None
    assert api.references == {'sorter': None, 'feeder': None}
    api.enter_power_save()
    reads = (sorter.angle_reads, feeder.angle_reads, sorter.speed_reads, feeder.speed_reads)
    sleeping = api.request({'id': 2, 'operation': 'critical_status', 'args': {}})
    assert sleeping['ok'] and sleeping['result']['power_mode'] == 'power_save'
    assert reads == (sorter.angle_reads, feeder.angle_reads, sorter.speed_reads, feeder.speed_reads)
    # Exercise guard_only on the actual VM too: fresh other-axis drift must
    # brake an existing target without issuing a second target or zeroing.
    api.wake()
    api.references['sorter'] = 'home'
    moving = api.request({'id': 3, 'operation': 'motor_target', 'args': {
        'name': 'sorter', 'target': 14, 'speed': 50, 'acceleration': 400, 'actuation': 30}})
    assert moving['ok']
    dispatches = sorter.dispatches
    feeder.value = 4
    guarded = api.request({'id': 4, 'operation': 'critical_status', 'args': {}})
    assert guarded['ok'] and not guarded['result']['busy']
    assert 'Other motor drift' in guarded['result']['last_error']['message']
    assert sorter.dispatches == dispatches and sorter.target is None
    # Normal duration expiry brakes and enters quiet settling, and repeated
    # critical reads must not turn that permitted settling into a fault.
    feeder.value = 0
    shaking = api.request({'id': 5, 'operation': 'shake', 'args': {
        'frequency_hz': 2, 'speed_dps': 50, 'acceleration': 1000, 'duration_ms': 100}})
    assert shaking['ok']
    clock.ms = api.job['_end'] - 1
    api.last_heartbeat = clock.ms
    dispatches = sorter.dispatches
    ended = api.request({'id': 6, 'operation': 'critical_status', 'args': {}})
    assert ended['ok'] and ended['result']['busy'] and ended['result']['last_error'] is None
    assert api.job['_stage'] == 'settling' and sorter.target is None
    assert sorter.dispatches == dispatches
    settling = api.request({'id': 7, 'operation': 'critical_status', 'args': {}})
    assert settling['ok'] and settling['result']['last_error'] is None
    assert api.job['_stage'] == 'settling' and sorter.dispatches == dispatches
    clock.ms += 220
    api.job_tick(clock.ms)
    assert api.job is None and api.last_job['state'] == 'complete'
    return {'paired_angle_speed_stamps': True, 'optional_peripheral_reads': 0,
            'read_duration_ms': 10, 'sleep_stayed_asleep': True,
            'fresh_other_axis_guard_no_dispatch': True,
            'active_shake_critical_settling_no_dispatch': True,
            'hardware_access': False}


def full_target_probe():
    clock = Probe()
    sorter, feeder = FakeAxis(), FakeAxis()
    peripherals = FakePeripherals()
    api = live_api.LiveAPI({'sorter': sorter, 'feeder': feeder}, peripherals,
                          peripherals, peripherals, clock.clock, FakeStop,
                          'portable-fake-target', diagnostic_home_quiet=True)
    api.connect()
    api.references['sorter'] = 'home'
    reply = api.request({'id': 1, 'operation': 'motor_target', 'args': {
        'name': 'sorter', 'target': 12, 'speed': 50, 'acceleration': 400, 'actuation': 30}})
    assert reply['ok']
    speed_reads = sorter.speed_reads + feeder.speed_reads
    stages = {}
    request_id = 1
    for index in range(40):
        stages[api.job['_stage']] = True
        clock.ms += 20
        sorter.advance()
        feeder.advance()
        api.tick()
        if api.job is None:
            break
        request_id += 1
        response = api.request({'id': request_id, 'operation': 'status', 'args': {}})
        assert response['ok'] and response['result']['busy']
        assert sorter.speed_reads + feeder.speed_reads == speed_reads
        assert peripherals.calls == 0
        reads = (sorter.angle_reads, feeder.angle_reads, peripherals.touch_reads)
        for burst in range(10):
            api.tick()
        assert reads == (sorter.angle_reads, feeder.angle_reads, peripherals.touch_reads)
    assert api.job is None
    assert api.last_job['state'] == 'complete'
    assert api.last_job['target'] == 12 and sorter.value == 12
    assert 'moving' in stages and 'settling' in stages
    assert sorter.angle_reads > 2 and feeder.angle_reads > 2
    assert peripherals.touch_reads > 0 and peripherals.calls > 0
    timing = api.last_job['timing_diagnostics']
    assert timing['touch_read_ms']['count'] > 0
    assert timing['job_tick_ms']['count'] > 0
    assert timing['status_speed_read_ms']['skipped'] > 0
    assert timing['captured_before_cleanup_ms'] is not None
    return {'target_deg': sorter.value, 'elapsed_ms': api.last_job['elapsed_ms'],
            'state': api.last_job['state'], 'timing_diagnostics': timing}


def run():
    probe = Probe()
    job = {'timing_diagnostics': True}
    result = live_api.LiveAPI.timed_call(probe, job, 'success', probe.success, False)
    assert result is False
    assert probe.records == [('success', 5)]
    try:
        live_api.LiveAPI.timed_call(probe, job, 'failure', probe.failure)
    except ValueError as error:
        assert str(error) == 'expected probe failure'
    else:
        raise AssertionError('timed call swallowed the callback failure')
    assert probe.records == [('success', 5), ('failure', 7)]
    result = live_api.LiveAPI.timed_call(probe, None, 'inactive', probe.success, 42)
    assert result == 42
    assert len(probe.records) == 2
    target = full_target_probe()
    critical = critical_status_probe()
    return {'timing_helper_passed': True, 'motion_target_passed': True,
            'critical_status_passed': True, 'fake_critical_status': critical,
            'fake_target': target, 'hardware_access': False,
            'runtime': str(sys.implementation)}


if __name__ == '__main__':
    print(live_api.json.dumps(run()))
