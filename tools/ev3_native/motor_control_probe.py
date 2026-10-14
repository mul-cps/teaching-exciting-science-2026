#!/usr/bin/env pybricks-micropython
"""Motor-only, bounded diagnostic for the EV3 sorter motor.

Run only after the operator has independently verified EV3 identity and a clear
mechanical envelope. This program has no network, color, display, or sound I/O.
It does not reset the encoder or declare a home reference.
"""
import sys
try:
    import ujson as json
except ImportError:
    import json


SPEED_DPS = 50
NATIVE_LIMITS = (120, 400, 30)
TARGET_TRAVEL_DEG = 550
MAX_TRAVEL_DEG = 40
MAX_DURATION_MS = 2000
POLL_MS = 5
MAX_STEP_SPEED_DPS = 80
MAX_STEP_MIN_DEG = 20
MAX_STEP_ALLOWANCE_DEG = 6
MAX_RECORDS = 100
HOME_DIRECTION_SIGN = -1


class MotorControlProbe:
    def __init__(self, Motor, Port, Direction, Stop, TouchSensor, clock, wait,
                 emit=None):
        self.Motor = Motor
        self.Port = Port
        self.Direction = Direction
        self.Stop = Stop
        self.TouchSensor = TouchSensor
        self.clock = clock
        self.wait = wait
        self.emit = emit or print
        self.records = []
        self.dropped_records = 0
        self.motor = None
        self.touch = None
        self.saved_limits = None
        self.result = {'probe': 'ev3-motor-control-v1', 'outcome': 'fault',
                       'fault': None, 'records': self.records}

    def record(self, row):
        if len(self.records) == MAX_RECORDS:
            del self.records[0]
            self.dropped_records += 1
        self.records.append(row)

    def sample(self):
        started = self.clock()
        angle = self.motor.angle()
        touch = bool(self.touch.pressed())
        speed = self.motor.speed()
        done = bool(self.motor.control.done())
        stalled = bool(self.motor.control.stalled())
        sampled = self.clock()
        row = {'sample_ms': sampled, 'read_duration_ms': max(0, sampled - started),
               'angle_deg': angle, 'touch': touch, 'speed_dps': speed,
               'done': done, 'stalled': stalled}
        self.record(row)
        return row

    def cleanup(self):
        errors = []
        if self.motor is not None:
            try:
                self.motor.brake()
            except Exception as error:
                errors.append('brake: ' + str(error))
            if self.saved_limits is not None:
                try:
                    self.motor.control.limits(*self.saved_limits)
                except Exception as error:
                    errors.append('restore_limits: ' + str(error))
        self.result['cleanup_errors'] = errors
        if errors and self.result.get('fault') is None:
            self.result['fault'] = 'cleanup_failed: ' + '; '.join(errors)
            self.result['outcome'] = 'fault'

    def execute(self):
        self.motor = self.Motor(self.Port.D, self.Direction.CLOCKWISE)
        self.touch = self.TouchSensor(self.Port.S1)
        self.motor.brake()
        self.saved_limits = tuple(self.motor.control.limits())
        self.motor.control.limits(speed=NATIVE_LIMITS[0],
                                  acceleration=NATIVE_LIMITS[1],
                                  actuation=NATIVE_LIMITS[2])
        self.result.update(motor_port='D', touch_port='S1',
                           positive_direction='CLOCKWISE',
                           native_limits=list(NATIVE_LIMITS),
                           saved_limits=list(self.saved_limits),
                           requested_speed_dps=SPEED_DPS,
                           target_offset_deg=-TARGET_TRAVEL_DEG,
                           max_travel_deg=MAX_TRAVEL_DEG,
                           max_duration_ms=MAX_DURATION_MS,
                           poll_ms=POLL_MS)

        first = self.sample()
        start = first['angle_deg']
        started = first['sample_ms']
        self.result.update(start_ms=started, start_angle_deg=start,
                           initial_touch=first['touch'])
        if first['touch']:
            self.result['outcome'] = 'touch_pressed_at_start'
            self.result.update(end_ms=first['sample_ms'], end_angle_deg=start,
                               end_touch=first['touch'])
            return

        target = start - TARGET_TRAVEL_DEG
        command = {'issued_ms': self.clock(),
                                  'method': 'run_target', 'speed_dps': SPEED_DPS,
                                  'target_angle_deg': target,
                                  'then': 'BRAKE', 'wait': False}
        self.result['command'] = command
        self.motor.run_target(SPEED_DPS, target, then=self.Stop.BRAKE, wait=False)
        command['returned_ms'] = self.clock()
        previous_angle = first['angle_deg']
        previous_ms = first['sample_ms']

        terminal = first
        while True:
            row = self.sample()
            terminal = row
            now = row['sample_ms']
            angle = row['angle_deg']
            elapsed = now - started
            delta = angle - previous_angle
            dt = max(0, now - previous_ms)
            row['elapsed_ms'] = elapsed
            row['step_deg'] = delta
            row['step_interval_ms'] = dt
            row['step_bound_deg'] = max(MAX_STEP_MIN_DEG,
                                        MAX_STEP_SPEED_DPS * dt / 1000.0 +
                                        MAX_STEP_ALLOWANCE_DEG)

            if row['touch']:
                self.result['outcome'] = 'touch_pressed'
                break
            if delta * HOME_DIRECTION_SIGN < -3:
                self.result.update(outcome='fault', fault='wrong_direction')
                break
            if abs(delta) > row['step_bound_deg']:
                self.result.update(outcome='fault', fault='encoder_step_exceeds_bound')
                break
            if abs(angle - start) > MAX_TRAVEL_DEG:
                self.result.update(outcome='fault', fault='travel_limit')
                break
            if row['stalled']:
                self.result.update(outcome='fault', fault='native_stalled')
                break
            if row['done']:
                self.result['outcome'] = 'target_done'
                break
            if elapsed >= MAX_DURATION_MS:
                self.result.update(outcome='fault', fault='timeout')
                break

            previous_angle, previous_ms = angle, now
            self.wait(POLL_MS)

        self.result.update(end_ms=terminal['sample_ms'],
                           end_angle_deg=terminal['angle_deg'],
                           end_touch=terminal['touch'],
                           end_speed_dps=terminal['speed_dps'],
                           end_done=terminal['done'],
                           end_stalled=terminal['stalled'])

    def run(self):
        try:
            self.execute()
        except Exception as error:
            self.result.update(outcome='fault', fault='probe_error: ' + str(error))
        finally:
            self.cleanup()
            self.result['record_count'] = len(self.records)
            self.result['dropped_record_count'] = self.dropped_records
            self.emit(json.dumps(self.result))
        return self.result


def main():
    impl = sys.implementation
    if getattr(impl, 'name', None) != 'pybricks-micropython' or impl.version[0] != 2:
        raise RuntimeError('Requires pybricks-micropython major 2')
    from pybricks.ev3devices import Motor, TouchSensor
    from pybricks.parameters import Direction, Port, Stop
    from pybricks.tools import StopWatch, wait
    MotorControlProbe(Motor, Port, Direction, Stop, TouchSensor,
                      StopWatch().time, wait).run()


if __name__ == '__main__':
    main()
