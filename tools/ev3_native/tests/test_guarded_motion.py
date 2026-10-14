from pathlib import Path
"""Host-only fake checks; never imports hardware modules or executes main()."""
import importlib.util
import unittest
import contextlib
import io
import ast
spec = importlib.util.spec_from_file_location('commission', str(Path(__file__).resolve().parents[1] / 'guarded_motion.py'))
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)

class Port:
    A = 'A'
    D = 'D'
    S1 = 'S1'
    S3 = 'S3'
class Stop:
    BRAKE = 'BRAKE'
class Control:
    def __init__(self):
        self.value = (999, 999, 99)
    def limits(self, *args, **kwargs):
        if args:
            self.value = args
        elif kwargs:
            self.value = (kwargs['speed'], kwargs['acceleration'], kwargs['actuation'])
        return self.value

class Env:
    def __init__(self, d_start=10, a_stall=50, frozen=False, wrong=False, jump=False):
        self.now = 0
        self.motors = {}
        self.d_start = d_start
        self.a_stall = a_stall
        self.frozen = frozen
        self.wrong = wrong
        self.jump = jump
        self.logs = []
    def time(self):
        return self.now
    def wait(self, ms):
        for motor in self.motors.values():
            active = max(0, min(ms, motor.expiry - self.now))
            delta = motor.speed * active / 1000.0
            if self.frozen:
                delta = 0
            if self.wrong:
                delta = -delta
            if self.jump and delta:
                delta = 16
            motor.raw += delta
            if motor.port == Port.D:
                motor.raw = max(0, motor.raw)
            elif self.a_stall is not None:
                motor.raw = min(self.a_stall, motor.raw)
        self.now += ms
    def Motor(self, port):
        env = self
        class Motor:
            def __init__(self):
                self.port = port
                self.raw = env.d_start if port == Port.D else 0
                self.zero = 0
                self.speed = 0
                self.expiry = 0
                self.control = Control()
                self.commands = []
                self.brakes = 0
            def angle(self):
                return int(round(self.raw - self.zero))
            def reset_angle(self, angle):
                self.zero = self.raw - angle
            def brake(self):
                self.speed = 0
                self.brakes += 1
            def run_time(self, speed, duration, then, wait):
                assert duration == 100 and then == Stop.BRAKE and wait is False
                self.speed = speed
                self.expiry = env.now + duration
                self.commands.append((env.now, speed))
        result = Motor()
        self.motors[port] = result
        return result
    def Touch(self, port):
        env = self
        class Touch:
            def pressed(self):
                return Port.D in env.motors and env.motors[Port.D].raw <= 0
        return Touch()
    def Color(self, port):
        class Color:
            def color(self):
                return 'BLUE'
        return Color()
    def runner(self):
        result = c.Commissioning(self.Motor, Port, Stop, self.Touch,
                                 self.Color, lambda: self, self.wait)
        result.log = lambda phase, **fields: self.logs.append((phase, fields))
        return result

class Tests(unittest.TestCase):
    def execute(self, env, mode, cycles=1):
        runner = env.runner()
        try:
            runner.execute(mode, cycles)
        finally:
            runner.cleanup()
        return runner
    def test_home_initially_released_no_feeder(self):
        env = Env()
        self.execute(env, 'home')
        self.assertEqual(set(env.motors), {'D'})
        self.assertEqual(env.motors['D'].angle(), 0)
        self.assertEqual(env.motors['D'].speed, 0)
        self.assertEqual(env.motors['D'].control.value, (999, 999, 99))
    def test_home_initially_pressed(self):
        env = Env(d_start=0)
        self.execute(env, 'home')
        self.assertIn('D_initial_release_start', [phase for phase, fields in env.logs])
    def test_traverse_and_rehome(self):
        env = Env()
        self.execute(env, 'traverse')
        self.assertEqual(env.motors['D'].angle(), 0)
        targets = [fields['target'] for phase, fields in env.logs if phase.endswith('_target')]
        for target in [10, 132, 360, 530, 15]:
            self.assertIn(target, targets)
    def test_feed_home(self):
        env = Env()
        self.execute(env, 'feed-home')
        self.assertEqual(env.motors['A'].angle(), 0)
        self.assertEqual(env.motors['D'].angle(), 0)
        self.assertEqual(env.motors['A'].speed, 0)
    def test_eject_cycles_and_return(self):
        env = Env(a_stall=300)
        runner = self.execute(env, 'eject', 2)
        self.assertEqual(env.motors['A'].angle(), 0)
        self.assertEqual(sum(phase == 'eject_cycle_complete' for phase, fields in env.logs), 2)
    def test_dead_motor_fault_and_brake(self):
        env = Env(frozen=True)
        with self.assertRaisesRegex(c.CommissionFault, 'progress'):
            self.execute(env, 'home')
        self.assertEqual(env.motors['D'].speed, 0)
        self.assertLessEqual(env.now, 1000)
    def test_wrong_direction(self):
        env = Env(wrong=True)
        with self.assertRaisesRegex(c.CommissionFault, 'wrong direction'):
            self.execute(env, 'home')
    def test_encoder_jump(self):
        env = Env(jump=True)
        with self.assertRaisesRegex(c.CommissionFault, 'encoder step'):
            self.execute(env, 'home')
    def test_feeder_without_stall_exhausts_bound(self):
        env = Env(a_stall=None)
        with self.assertRaisesRegex(c.CommissionFault, 'travel budget'):
            self.execute(env, 'feed-home')
        self.assertEqual(env.motors['A'].speed, 0)
    def test_pressed_switch_never_releases(self):
        env = Env(d_start=0)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.touch.pressed = lambda: True
        with self.assertRaisesRegex(c.CommissionFault, 'remained pressed'):
            runner.home_d()
        runner.cleanup()

    def test_unexpected_switch_after_release(self):
        env = Env(d_start=20)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.touch.pressed = lambda: env.now >= 100
        with self.assertRaisesRegex(c.CommissionFault, 'unexpected touch'):
            runner.move(runner.d, 'unexpected', 50, sorter=True)
        runner.cleanup()

    def test_feeder_checks_sorter_movement(self):
        env = Env()
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.a = runner.init_motor(Port.A, 'A')
        original_wait = runner.wait
        def disturbed_wait(ms):
            original_wait(ms)
            runner.d.raw += 1
        runner.wait = disturbed_wait
        with self.assertRaisesRegex(c.CommissionFault, 'sorter D moved'):
            runner.move(runner.a, 'disturbance', 100, watch_d=True)
        runner.cleanup()

    def test_soft_limit(self):
        env = Env(d_start=551)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        with self.assertRaisesRegex(c.CommissionFault, 'soft limit'):
            runner.move(runner.d, 'soft_limit', 530, sorter=True)
        runner.cleanup()

    def test_native_expiry_without_refresh(self):
        env = Env()
        motor = env.Motor(Port.D)
        motor.run_time(50, 100, then=Stop.BRAKE, wait=False)
        env.wait(1000)
        self.assertEqual(motor.angle(), 15)
        env.wait(1000)
        self.assertEqual(motor.angle(), 15)

    def test_cleanup_keeps_braking_if_logger_fails(self):
        env = Env()
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.a = runner.init_motor(Port.A, 'A')
        def failed_log(*args, **kwargs):
            raise IOError('logger failed')
        runner.log = failed_log
        runner.d.speed = 50
        runner.a.speed = 50
        runner.cleanup()
        self.assertEqual(runner.d.speed, 0)
        self.assertEqual(runner.a.speed, 0)
        self.assertEqual(runner.d.control.value, (999, 999, 99))
        self.assertEqual(runner.a.control.value, (999, 999, 99))

    def test_three_degree_spring_recoil_settles(self):
        env = Env(d_start=100)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        def recoil_wait(ms):
            env.wait(ms)
            if env.now <= 60:
                runner.d.raw += 1
        runner.wait = recoil_wait
        self.assertEqual(runner.stopped(runner.d, 'recoil'), 103)
        self.assertGreaterEqual(env.now, 240)
        self.assertLessEqual(env.now, 1000)
        end = [fields for phase, fields in env.logs if phase == 'recoil_settling_end'][0]
        self.assertEqual(end['total_drift'], 3)
        self.assertEqual(end['outcome'], 'stable')

    def test_continued_drift_fails(self):
        env = Env(d_start=100)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        def drift_wait(ms):
            env.wait(ms)
            runner.d.raw += 1
        runner.wait = drift_wait
        with self.assertRaisesRegex(c.CommissionFault, 'drift exceeds 6'):
            runner.stopped(runner.d, 'drift')
        self.assertLessEqual(env.now, 1000)

    def test_bounded_but_unsettled_motor_fails(self):
        env = Env(d_start=100)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        def oscillation_wait(ms):
            env.wait(ms)
            runner.d.raw = 100 if (env.now // 20) % 2 else 102
        runner.wait = oscillation_wait
        with self.assertRaisesRegex(c.CommissionFault, 'did not settle within 1000'):
            runner.stopped(runner.d, 'oscillation')
        self.assertEqual(env.now, 1000)

    def test_distance_deadline_is_finite_and_bounded(self):
        env = Env(d_start=530)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.move(runner.d, 'return515', 15, sorter=True)
        starts = [fields for phase, fields in env.logs if phase == 'return515_start']
        self.assertEqual(starts[0]['deadline_ms'], 28750)
        for target in (16, 550, 100000):
            captured = []
            def fake_motion(motor, phase, speed, travel, deadline, **kwargs):
                captured.append(deadline)
                motor.raw = kwargs['target']
                return motor.angle()
            runner.motion = fake_motion
            runner.move(runner.d, 'bounded', target)
            self.assertGreaterEqual(captured[0], 5000)
            self.assertLessEqual(captured[0], 40000)

    def test_slow_repeat_approach_bounded_parameters(self):
        env = Env()
        runner = env.runner()
        original_motion = runner.motion
        calls = {}
        def recorded_motion(motor, phase, speed, travel, deadline, **kwargs):
            calls[phase] = (speed, travel, deadline, kwargs)
            return original_motion(motor, phase, speed, travel, deadline, **kwargs)
        runner.motion = recorded_motion
        runner.execute('home', 1)
        runner.cleanup()
        self.assertEqual(calls['D_home_approach'][:3], (-50, 720, 20000))
        self.assertEqual(calls['D_repeat_backoff'][:3], (50, 30, 8000))
        self.assertEqual(calls['D_repeat_backoff'][3]['target'], 25)
        self.assertEqual(calls['D_repeat_backoff'][3]['release_limit'], 20)
        self.assertEqual(calls['D_repeat_approach'][:3], (-25, 35, 8000))
        self.assertTrue(calls['D_repeat_approach'][3]['touch_goal'])
        repeat = [fields for phase, fields in env.logs if phase == 'D_home_repeatability'][0]
        self.assertEqual(repeat['tolerance'], 4)
        self.assertEqual(set(env.motors), {'D'})

    def test_calibrated_blue_tolerates_initial_touch_hysteresis(self):
        env = Env()
        runner = env.runner()
        runner.execute('home', 1)
        runner.touch.pressed = lambda: runner.d.raw <= 12
        runner.move(runner.d, 'blue10', 10, sorter=True)
        self.assertEqual(runner.d.angle(), 10)
        self.assertTrue(runner.touch.pressed())
        runner.cleanup()

    def test_calibrated_beyond_blue_requires_release_by_absolute20(self):
        env = Env(d_start=10)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.touch.pressed = lambda: True
        with self.assertRaisesRegex(c.CommissionFault, 'calibrated angle 20'):
            runner.move(runner.d, 'beyondblue132', 132, sorter=True)
        self.assertEqual(runner.d.angle(), 20)
        self.assertEqual(runner.d.speed, 0)
        runner.cleanup()

    def test_calibrated_repress_after_release_even_near_home_faults(self):
        env = Env(d_start=0)
        runner = env.runner()
        runner.d = runner.init_motor(Port.D, 'D')
        runner.touch.pressed = lambda: runner.d.angle() <= 2 or runner.d.angle() >= 7
        with self.assertRaisesRegex(c.CommissionFault, 'unexpected touch'):
            runner.move(runner.d, 'blue_repress', 10, sorter=True)
        runner.cleanup()

    def test_arguments(self):
        for argv in [['p'], ['p', 'eject', '0'], ['p', 'eject', '9'],
                     ['p', 'home', '1'], ['p', 'eject', 'one']]:
            with self.assertRaises(c.CommissionFault):
                c.parse_args(argv)
        self.assertEqual(c.parse_args(['p', 'eject', '8']), ('eject', 8))
    def test_python34_syntax_and_no_unbounded_motor_api(self):
        with open(str(Path(__file__).resolve().parents[1] / 'guarded_motion.py')) as handle:
            source = handle.read()
        tree = ast.parse(source, feature_version=(3, 4))
        forbidden = {'run', 'run_target', 'run_angle', 'run_until_stalled'}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, forbidden)

if __name__ == '__main__':
    unittest.main()
