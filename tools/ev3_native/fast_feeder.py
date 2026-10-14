#!/usr/bin/env pybricks-micropython
"""One feeder cycle using the accepted current relaxed-negative EJECT reference."""
import sys


def finite_target(runner, phase, target):
    motor = runner.a
    start = motor.angle()
    distance = abs(target - start)
    direction = 1 if target > start else -1
    d_start = runner.d.angle()
    started = runner.clock.time()
    last = start
    next_log = started
    runner.log(phase + '_start', angle=start, target=target, deadline_ms=6000,
               budget=distance + 6, speed=600, acceleration=2400, actuation=50)
    try:
        motor.run_target(600, target, then=runner.Stop.BRAKE, wait=False)
        while True:
            now = runner.clock.time()
            angle = motor.angle()
            displacement = direction * (angle - start)
            if abs(angle - last) > 15:
                raise RuntimeError(phase + ': encoder step exceeds 15 degrees')
            if direction * (angle - last) < -3 or displacement < -3:
                raise RuntimeError(phase + ': wrong direction')
            if displacement > distance + 6:
                raise RuntimeError(phase + ': travel budget exceeded')
            if abs(runner.d.angle() - d_start) > 3:
                raise RuntimeError(phase + ': sorter D drift exceeds 3 degrees')
            if now - started >= 6000:
                raise RuntimeError(phase + ': 6000 ms deadline expired')
            if now >= next_log:
                runner.log(phase + '_sample', angle=angle, target=target,
                           elapsed_ms=now - started, displacement=displacement,
                           sorter_angle=runner.d.angle())
                next_log = now + 250
            # A completed accurate native target wins over a residual stalled flag.
            if motor.control.done():
                if abs(angle - target) > 3:
                    raise RuntimeError(phase + ': native target error exceeds 3 degrees')
                settled = runner.stopped(motor, phase)
                if abs(settled - target) > 3:
                    raise RuntimeError(phase + ': settled target error exceeds 3 degrees')
                if abs(runner.d.angle() - d_start) > 3:
                    raise RuntimeError(phase + ': sorter D drift exceeds 3 degrees')
                runner.log(phase + '_complete', angle=settled, target=target,
                           verified_complete=True)
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


def cycle(runner, speaker, wait):
    try:
        runner.d = runner.init_motor(runner.Port.D, 'D_sorter')
        runner.a = runner.init_motor(runner.Port.A, 'A_feeder')
        runner.a.brake()
        runner.a.control.limits(speed=600, acceleration=2400, actuation=50)
        speaker.beep(duration=100)
        runner.log('one_eject_cycle_start', angle=runner.a.angle(),
                   reference='accepted_current_relaxed_negative_EJECT',
                   LOAD_READY=209, EJECT=0, encoder_reset=False)
        finite_target(runner, 'A_LOAD_READY', 209)
        finite_target(runner, 'A_EJECT', 0)
        runner.log('one_eject_cycle_complete', feeder_angle=runner.a.angle(),
                   sorter_angle=runner.d.angle(), stroke_verified=True,
                   cube_ejection_verified=False, station_commissioned=False)
    except BaseException as error:
        runner.safe_log('one_eject_cycle_error', error=str(error), error_type=type(error).__name__)
        raise
    finally:
        runner.cleanup()
    speaker.beep(duration=100)
    wait(150)
    speaker.beep(duration=100)


def main():
    if len(sys.argv) != 1:
        raise ValueError('Exactly one fixed feeder cycle; no arguments')
    import guarded_motion as existing
    existing.check_runtime()
    from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
    from pybricks.hubs import EV3Brick
    from pybricks.parameters import Port, Stop
    from pybricks.tools import StopWatch, wait
    runner = existing.Commissioning(Motor, Port, Stop, TouchSensor, ColorSensor, StopWatch, wait)
    cycle(runner, EV3Brick().speaker, wait)


if __name__ == '__main__':
    main()
