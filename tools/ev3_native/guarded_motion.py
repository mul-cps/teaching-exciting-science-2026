#!/usr/bin/env pybricks-micropython
"""Bounded physical commissioning only; does not commission the station.

Modes: home, traverse, feed-home, eject [cycles=1..8].
Each command expires natively after 100 ms, refreshed every 60 ms.
"""
import sys
try:
    import ujson as json
except ImportError:
    import json


class CommissionFault(Exception):
    pass


def check_runtime():
    impl = sys.implementation
    if impl.name != 'pybricks-micropython' or impl.version[0] != 2:
        raise CommissionFault('Requires pybricks-micropython major 2')


def parse_args(argv):
    if len(argv) < 2 or argv[1] not in ('home', 'traverse', 'feed-home', 'eject'):
        raise CommissionFault('Usage: ev3-system-test.py home|traverse|feed-home|eject [1..8]')
    mode = argv[1]
    cycles = 1
    if len(argv) > 2:
        if mode != 'eject' or len(argv) != 3:
            raise CommissionFault('Cycle argument is only valid for eject')
        try:
            cycles = int(argv[2])
        except ValueError:
            raise CommissionFault('Cycles must be an integer 1..8')
        if cycles < 1 or cycles > 8:
            raise CommissionFault('Cycles must be 1..8')
    return mode, cycles


class Commissioning:
    def __init__(self, Motor, Port, Stop, TouchSensor, ColorSensor, StopWatch, wait):
        self.Motor = Motor
        self.Port = Port
        self.Stop = Stop
        self.wait = wait
        self.clock = StopWatch()
        self.touch = TouchSensor(Port.S1)
        self.color = ColorSensor(Port.S3)
        self.motors = []
        self.saved_limits = []
        self.d = None
        self.a = None

    def log(self, phase, **fields):
        fields['ms'] = self.clock.time()
        fields['phase'] = phase
        fields['scope'] = 'physicalcommissioning_only'
        print(json.dumps(fields))

    def safe_log(self, phase, **fields):
        # Cleanup and error reporting must never mask the original failure.
        try:
            self.log(phase, **fields)
        except Exception:
            pass

    def init_motor(self, port, label):
        motor = self.Motor(port)
        self.motors.append((label, motor))
        motor.brake()
        limits = motor.control.limits()
        self.saved_limits.append((label, motor, limits))
        motor.control.limits(speed=120, acceleration=400, actuation=30)
        self.log('motor_initialized', motor=label, angle=motor.angle(), saved_limits=limits)
        return motor

    def stopped(self, motor, phase):
        motor.brake()
        started = self.clock.time()
        before = motor.angle()
        minimum = before
        maximum = before
        samples = [(started, before)]
        self.log(phase + '_settling_start', angle=before, touch=self.touch.pressed(), total_drift=0)
        while True:
            now = self.clock.time()
            angle = motor.angle()
            minimum = min(minimum, angle)
            maximum = max(maximum, angle)
            total_drift = maximum - minimum
            if total_drift > 6:
                self.log(phase + '_settling_end', angle=angle, total_drift=total_drift,
                         elapsed_ms=now - started, outcome='drift_fault')
                raise CommissionFault(phase + ': settling drift exceeds 6 degrees')
            samples.append((now, angle))
            cutoff = now - 200
            while len(samples) > 1 and samples[1][0] <= cutoff:
                samples.pop(0)
            quiet_range = max(sample[1] for sample in samples) - min(sample[1] for sample in samples)
            if now - samples[0][0] >= 200 and quiet_range <= 1:
                self.log(phase + '_settling_end', angle=angle, touch=self.touch.pressed(),
                         elapsed_ms=now - started, total_drift=total_drift,
                         displacement=angle - before, quiet_range=quiet_range, outcome='stable')
                self.log(phase + '_stopped', angle=angle, touch=self.touch.pressed())
                return angle
            if now - started >= 1000:
                self.log(phase + '_settling_end', angle=angle, total_drift=total_drift,
                         elapsed_ms=now - started, outcome='unsettled_fault')
                raise CommissionFault(phase + ': motor did not settle within 1000 ms')
            self.wait(20)

    def motion(self, motor, phase, speed, travel, deadline, target=None,
               touch_goal=None, release_limit=None, stall_success=False,
               soft_limits=None, watch_d=False):
        """Finite slices, every-loop guards, with an independent native expiry."""
        start = motor.angle()
        last = start
        direction = 1 if speed > 0 else -1
        started = self.clock.time()
        progress_time = started
        progress_angle = start
        next_slice = started
        next_log = started
        d_start = self.d.angle() if watch_d else None
        released = not self.touch.pressed()
        self.log(phase + '_start', angle=start, speed=speed, budget=travel,
                 deadline_ms=deadline, target=target, touch=self.touch.pressed())
        try:
            while True:
                now = self.clock.time()
                angle = motor.angle()
                pressed = self.touch.pressed()
                displacement = direction * (angle - start)
                if abs(angle - last) > 15:
                    raise CommissionFault(phase + ': apparent encoder step exceeds 15 degrees')
                if displacement < -3 or direction * (angle - last) < -3:
                    raise CommissionFault(phase + ': wrong direction')
                if displacement > travel:
                    raise CommissionFault(phase + ': encoder travel budget exceeded')
                if watch_d and abs(self.d.angle() - d_start) > 3:
                    raise CommissionFault(phase + ': sorter D moved during feeder motion')
                if soft_limits is not None and not (soft_limits[0] <= angle <= soft_limits[1]):
                    raise CommissionFault(phase + ': sorter soft limit exceeded')
                if touch_goal is False:
                    if not pressed:
                        released = True
                    elif released:
                        raise CommissionFault(phase + ': unexpected touch press after release')
                    elif soft_limits is not None and direction > 0:
                        if target is not None and target > 20 and angle >= 20:
                            raise CommissionFault(phase + ': touch did not release by calibrated angle 20')
                    elif release_limit is not None and displacement >= release_limit:
                        raise CommissionFault(phase + ': touch did not release within bound')
                # A true homing switch wins before dispatching another motor slice.
                if touch_goal is True and pressed:
                    self.log(phase + '_touch', angle=angle)
                    return angle
                if target is not None:
                    remaining = direction * (target - angle)
                    if remaining < -5:
                        raise CommissionFault(phase + ': target overshoot exceeds 5 degrees')
                    if remaining <= 0:
                        near_home_positive = (soft_limits is not None and direction > 0
                                              and target <= 20)
                        if touch_goal is False and not released and not near_home_positive:
                            raise CommissionFault(phase + ': touch remained pressed at target')
                        self.log(phase + '_target', angle=angle, target=target)
                        return angle
                if displacement >= travel:
                    raise CommissionFault(phase + ': encoder travel budget exhausted')
                if now - started >= deadline:
                    raise CommissionFault(phase + ': deadline expired')
                if direction * (angle - progress_angle) >= 1:
                    progress_angle = angle
                    progress_time = now
                if now - progress_time >= 700:
                    if stall_success:
                        self.log(phase + '_stall', angle=angle)
                        return angle
                    raise CommissionFault(phase + ': less than 1 degree progress in 700 ms')
                if now >= next_log:
                    self.log(phase + '_sample', angle=angle, touch=pressed,
                             elapsed_ms=now - started, displacement=displacement)
                    next_log = now + 250
                if now >= next_slice:
                    motor.run_time(speed, 100, then=self.Stop.BRAKE, wait=False)
                    next_slice = now + 60
                last = angle
                self.wait(5)
        finally:
            try:
                motor.brake()
            except Exception as error:
                self.safe_log(phase + '_brake_error', error=str(error))

    def move(self, motor, phase, target, speed=70, watch_d=False, sorter=False):
        start = motor.angle()
        delta = target - start
        if abs(delta) <= 0:
            self.stopped(motor, phase)
            return
        signed_speed = abs(speed) if delta > 0 else -abs(speed)
        deadline = int(max(5000, min(40000, abs(delta) / 20.0 * 1000 + 3000)))
        result = self.motion(motor, phase, signed_speed, abs(delta) + 6, deadline,
                             target=target, touch_goal=False if sorter else None,
                             release_limit=20 if sorter else None,
                             soft_limits=(0, 550) if sorter else None, watch_d=watch_d)
        result = self.stopped(motor, phase)
        if abs(result - target) > 3:
            raise CommissionFault(phase + ': final target error exceeds 3 degrees')

    def home_d(self):
        self.log('home_D_start', touch=self.touch.pressed(), angle=self.d.angle())
        start = self.d.angle()
        if self.touch.pressed():
            self.motion(self.d, 'D_initial_release', 50, 20, 8000,
                        target=start + 19, touch_goal=False, release_limit=20)
            self.stopped(self.d, 'D_initial_release')
        else:
            self.motion(self.d, 'D_initial_away', 50, 16, 8000,
                        target=start + 10, touch_goal=False, release_limit=20)
            self.stopped(self.d, 'D_initial_away')
        self.motion(self.d, 'D_home_approach', -50, 720, 20000, touch_goal=True)
        self.stopped(self.d, 'D_home_approach')
        if not self.touch.pressed():
            raise CommissionFault('D home touch released after braking')
        self.d.reset_angle(0)
        self.log('D_zero_confirmed', angle=self.d.angle(), touch=True)
        self.motion(self.d, 'D_repeat_backoff', 50, 30, 8000,
                    target=25, touch_goal=False, release_limit=20)
        self.stopped(self.d, 'D_repeat_backoff')
        self.motion(self.d, 'D_repeat_approach', -25, 35, 8000, touch_goal=True)
        repeat = self.stopped(self.d, 'D_repeat_approach')
        if not self.touch.pressed():
            raise CommissionFault('D repeat touch released after braking')
        self.log('D_home_repeatability', prezero_difference=repeat, tolerance=4)
        if abs(repeat) > 4:
            raise CommissionFault('D home repeatability exceeds 4 degrees')
        self.d.reset_angle(0)
        self.log('home_D_complete', angle=self.d.angle(), touch=True)

    def home_a(self):
        self.a = self.init_motor(self.Port.A, 'A_feeder')
        self.motion(self.a, 'A_stall_home', 60, 360, 12000,
                    stall_success=True, watch_d=True)
        stop_angle = self.stopped(self.a, 'A_stall_home')
        self.move(self.a, 'A_home_backoff', stop_angle - 200, watch_d=True)
        self.a.reset_angle(0)
        self.log('home_A_complete', angle=self.a.angle())

    def execute(self, mode, cycles):
        self.log('commissioning_start', mode=mode, cycles=cycles)
        self.d = self.init_motor(self.Port.D, 'D_sorter')
        self.home_d()
        if mode == 'traverse':
            for target in (10, 132, 360, 530):
                self.move(self.d, 'D_traverse_' + str(target), target, sorter=True)
                self.dwell('D_traverse_dwell', 300)
            self.move(self.d, 'D_return_15', 15, sorter=True)
            self.home_d()
        elif mode in ('feed-home', 'eject'):
            self.home_a()
            if mode == 'eject':
                self.move(self.d, 'D_dispose_position', 10, sorter=True)
                for cycle in range(cycles):
                    start = self.a.angle()
                    self.log('eject_cycle_start', cycle=cycle + 1, color=str(self.color.color()))
                    self.move(self.a, 'A_eject_forward', start + 180, watch_d=True)
                    self.move(self.a, 'A_eject_return', start, watch_d=True)
                    if abs(self.a.angle() - start) > 3:
                        raise CommissionFault('A cycle return error exceeds 3 degrees')
                    self.log('eject_cycle_complete', cycle=cycle + 1, angle=self.a.angle())
        self.log('commissioning_complete', mode=mode, station_commissioned=False)

    def dwell(self, phase, duration):
        started = self.clock.time()
        next_log = started
        while self.clock.time() - started < duration:
            now = self.clock.time()
            if now >= next_log:
                self.log(phase, angle=self.d.angle(), touch=self.touch.pressed())
                next_log = now + 250
            self.wait(5)

    def cleanup(self):
        # Try every brake independently before restoring any RAM control setting.
        braked = []
        for label, motor in self.motors:
            try:
                motor.brake()
                braked.append(motor)
                self.safe_log('final_brake', motor=label)
            except Exception as error:
                self.safe_log('cleanup_brake_error', motor=label, error=str(error))
        try:
            self.wait(100)
        except Exception as error:
            self.safe_log('cleanup_wait_error', error=str(error))
        for label, motor, limits in self.saved_limits:
            try:
                if motor not in braked:
                    self.safe_log('limits_restore_skipped', motor=label, reason='brake_failed')
                    continue
                motor.control.limits(*limits)
                self.safe_log('limits_restored', motor=label)
            except Exception as error:
                self.safe_log('cleanup_restore_error', motor=label, error=str(error))
        angles = {}
        for label, motor in self.motors:
            try:
                angles[label] = motor.angle()
            except Exception as error:
                angles[label] = str(error)
        try:
            touch = self.touch.pressed()
        except Exception as error:
            touch = str(error)
        self.safe_log('final_state', angles=angles, touch=touch)


def main():
    check_runtime()
    mode, cycles = parse_args(sys.argv)
    from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
    from pybricks.parameters import Port, Stop
    from pybricks.tools import StopWatch, wait
    runner = Commissioning(Motor, Port, Stop, TouchSensor, ColorSensor, StopWatch, wait)
    try:
        runner.execute(mode, cycles)
    except BaseException as error:
        runner.safe_log('commissioning_error', error=str(error), error_type=type(error).__name__)
        raise
    finally:
        runner.cleanup()


if __name__ == '__main__':
    main()
