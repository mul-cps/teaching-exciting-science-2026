#!/usr/bin/env pybricks-micropython
"""Timing-helper regression for the actual VM; imports no hardware drivers.

Run beside live_api.py with CPython or brickrun -r -- pybricks-micropython.
It uses fake clocks/callables only. The persistent native API should be stopped
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
    return {'timing_helper_passed': True, 'hardware_access': False,
            'runtime': str(sys.implementation)}


if __name__ == '__main__':
    print(live_api.json.dumps(run()))
