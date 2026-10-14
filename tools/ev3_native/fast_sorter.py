#!/usr/bin/env pybricks-micropython
"""One guarded fast sorter traverse; feeder remains braked throughout."""
import sys


def move_sorter(runner, target, a_reference):
    if not 0 <= target <= 550:
        raise ValueError('Sorter target must be within 0..550 degrees')
    motor = runner.d
    phase = 'D_fast_' + str(target)
    start = motor.angle()
    direction = 1 if target > start else -1
    distance = abs(target - start)
    started = runner.clock.time()
    last = start
    next_log = started
    runner.log(phase + '_start', angle=start, target=target, speed=600,
               acceleration=1200, actuation=50, deadline_ms=6000, budget=distance + 6)
    try:
        motor.run_target(600, target, then=runner.Stop.BRAKE, wait=False)
        while True:
            now = runner.clock.time()
            angle = motor.angle()
            displacement = direction * (angle - start)
            if not 0 <= angle <= 550:
                raise RuntimeError(phase + ': sorter soft limit exceeded')
            if abs(angle - last) > 20:
                raise RuntimeError(phase + ': encoder step exceeds 20 degrees')
            if direction * (angle - last) < -3 or displacement < -3:
                raise RuntimeError(phase + ': wrong direction')
            if displacement > distance + 6:
                raise RuntimeError(phase + ': travel budget exceeded')
            if abs(runner.a.angle() - a_reference) > 3:
                raise RuntimeError(phase + ': feeder A drift exceeds 3 degrees')
            if runner.touch.pressed() and angle > 20:
                raise RuntimeError(phase + ': unexpected touch above angle 20')
            if now - started >= 6000:
                raise RuntimeError(phase + ': 6000 ms deadline expired')
            if now >= next_log:
                runner.log(phase + '_sample', angle=angle, target=target,
                           elapsed_ms=now - started, touch=runner.touch.pressed())
                next_log = now + 250
            if motor.control.done():
                if abs(angle - target) > 3:
                    raise RuntimeError(phase + ': native target error exceeds 3 degrees')
                native_elapsed = now - started
                settled = runner.stopped(motor, phase)
                if not 0 <= settled <= 550:
                    raise RuntimeError(phase + ': settled sorter soft limit exceeded')
                if abs(settled - target) > 3:
                    raise RuntimeError(phase + ': settled target error exceeds 3 degrees')
                if abs(runner.a.angle() - a_reference) > 3:
                    raise RuntimeError(phase + ': feeder A drift exceeds 3 degrees')
                if runner.touch.pressed() and settled > 20:
                    raise RuntimeError(phase + ': unexpected touch above angle 20')
                runner.log(phase + '_complete', angle=settled, target=target,
                           native_elapsed_ms=native_elapsed,
                           elapsed_ms=runner.clock.time() - started, verified_complete=True)
                return
            if motor.control.stalled():
                raise RuntimeError(phase + ': native controller stalled')
            last = angle
            runner.wait(5)
    finally:
        try:
            motor.brake()
        except Exception as error:
            runner.safe_log(phase + '_brake_error', error=str(error))


def traverse(runner, speaker, wait):
    try:
        runner.d = runner.init_motor(runner.Port.D, 'D_sorter')
        runner.a = runner.init_motor(runner.Port.A, 'A_feeder')
        runner.a.brake()
        a_reference = runner.a.angle()
        speaker.beep(duration=100)
        runner.home_d()
        if abs(runner.a.angle() - a_reference) > 3:
            raise RuntimeError('Feeder A moved during sorter homing')
        runner.d.brake()
        runner.d.control.limits(speed=600, acceleration=1200, actuation=50)
        for target in (10, 132, 360, 530, 15):
            move_sorter(runner, target, a_reference)
            runner.wait(200)
            if abs(runner.a.angle() - a_reference) > 3:
                raise RuntimeError('Feeder A moved during camera hold')
        runner.d.brake()
        runner.d.control.limits(speed=120, acceleration=400, actuation=30)
        runner.home_d()
        if abs(runner.a.angle() - a_reference) > 3:
            raise RuntimeError('Feeder A moved during final sorter homing')
        runner.log('fast_sorter_complete', sorter_angle=runner.d.angle(),
                   feeder_angle=runner.a.angle(), station_commissioned=False)
    except BaseException as error:
        runner.safe_log('fast_sorter_error', error=str(error), error_type=type(error).__name__)
        raise
    finally:
        runner.cleanup()
    speaker.beep(duration=100)
    wait(150)
    speaker.beep(duration=100)


def main():
    if len(sys.argv) != 1:
        raise ValueError('Exactly one fixed traverse; no arguments')
    import guarded_motion as existing
    existing.check_runtime()
    from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
    from pybricks.hubs import EV3Brick
    from pybricks.parameters import Port, Stop
    from pybricks.tools import StopWatch, wait
    runner = existing.Commissioning(Motor, Port, Stop, TouchSensor, ColorSensor, StopWatch, wait)
    traverse(runner, EV3Brick().speaker, wait)


if __name__ == '__main__':
    main()
