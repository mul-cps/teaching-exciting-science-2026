import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'motor_control_probe.py'
spec = importlib.util.spec_from_file_location('motor_control_probe', MODULE)
probe_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe_module)


class Clock:
    def __init__(self): self.ms = 0
    def time(self): return self.ms


class Port:
    D = 'D'
    S1 = 'S1'


class Direction:
    CLOCKWISE = 'CLOCKWISE'


class Stop:
    BRAKE = 'BRAKE'


class Control:
    def __init__(self): self.value = (999, 999, 99); self.is_done = False; self.is_stalled = False
    def limits(self, *args, **kwargs):
        if args: self.value = tuple(args)
        if kwargs: self.value = (kwargs['speed'], kwargs['acceleration'], kwargs['actuation'])
        return self.value
    def done(self): return self.is_done
    def stalled(self): return self.is_stalled


class Motor:
    def __init__(self, env):
        self.env = env; self.value = 0.0; self.current_speed = 0; self.target = None
        self.control = Control(); self.commands = []; self.brakes = 0
    def angle(self): return int(round(self.value))
    def speed(self): return self.current_speed
    def run_target(self, speed, target, then, wait):
        self.commands.append((self.env.clock.ms, speed, target, then, wait))
        self.target = target; self.current_speed = -50
    def brake(self): self.brakes += 1; self.target = None; self.current_speed = 0


class Touch:
    def __init__(self, env): self.env = env
    def pressed(self):
        return self.env.touch_pressed or (self.env.mode == 'normal' and self.env.motor.value <= -10)


class Env:
    def __init__(self, mode='normal', initially_pressed=False):
        self.clock = Clock(); self.mode = mode; self.touch_pressed = initially_pressed
        self.motor = Motor(self); self.waits = []
    def wait(self, ms):
        self.waits.append(ms)
        if self.motor.target is not None:
            if self.mode == 'normal': self.motor.value -= 2
            elif self.mode == 'reverse': self.motor.value += 4
            elif self.mode == 'overspeed': self.motor.value -= 25
            elif self.mode == 'timeout': pass
            self.motor.current_speed = {'normal': -50, 'reverse': 50,
                                        'overspeed': -500, 'timeout': 0}[self.mode]
        self.clock.ms += ms

    def run(self):
        probe = probe_module.MotorControlProbe(
            lambda port, direction: self.motor, Port, Direction, Stop,
            lambda port: Touch(self), self.clock.time, self.wait, emit=lambda value: None)
        return probe.run()


class Tests(unittest.TestCase):
    def test_result_persistence_writes_one_bounded_json_then_renames(self):
        writes = []
        renames = []
        class Output:
            def __init__(self, path): self.path, self.closed = path, False
            def __enter__(self): return self
            def __exit__(self, *args): self.closed = True
            def write(self, content): writes.append((self.path, content, self.closed))
        def rename(source, target):
            renames.append((source, target, len(writes), output.closed))
        output = None
        def open_file(path, mode):
            nonlocal output
            self.assertEqual(mode, 'w')
            output = Output(path)
            return output

        result = {'records': [{'sample_ms': n, 'angle_deg': -n} for n in range(100)],
                  'outcome': 'touch_pressed'}
        probe_module.persist_result(result, '/tmp/probe.json', open_file, rename)
        self.assertEqual(len(writes), 1)
        self.assertTrue(output.closed)
        self.assertEqual(writes[0][0], '/tmp/probe.json.tmp')
        self.assertFalse(writes[0][2])
        self.assertEqual(__import__('json').loads(writes[0][1]), result)
        self.assertLess(len(writes[0][1]), 20000)
        self.assertEqual(renames, [('/tmp/probe.json.tmp', '/tmp/probe.json', 1, True)])

    def test_native_runtime_guard_accepts_pybricks_tuple_and_named_forms(self):
        self.assertTrue(probe_module.native_runtime(
            ('pybricks-micropython', (2, 0, 0))))
        self.assertTrue(probe_module.native_runtime(SimpleNamespace(
            name='pybricks-micropython', version=(2, 0, 0))))
        self.assertFalse(probe_module.native_runtime(
            ('pybricks-micropython', (3, 0, 0))))
        self.assertFalse(probe_module.native_runtime(
            ('micropython', (2, 0, 0))))

    def test_touch_ends_bounded_probe_and_restores_limits(self):
        env = Env()
        result = env.run()
        self.assertEqual(result['outcome'], 'touch_pressed')
        self.assertEqual(env.motor.commands[0][1:], (50, -550, 'BRAKE', False))
        self.assertEqual(env.motor.control.value, (999, 999, 99))
        self.assertGreaterEqual(env.motor.brakes, 1)
        self.assertTrue(result['records'])
        self.assertTrue(all('touch' in row and 'speed_dps' in row and
                            'done' in row and 'stalled' in row for row in result['records']))

    def test_overspeed_jump_stops_before_travel_limit(self):
        env = Env('overspeed')
        result = env.run()
        self.assertEqual(result['outcome'], 'fault')
        self.assertEqual(result['fault'], 'encoder_step_exceeds_bound')
        self.assertLessEqual(abs(env.motor.value), 40)
        self.assertEqual(env.motor.control.value, (999, 999, 99))

    def test_wrong_direction_stops_immediately(self):
        env = Env('reverse')
        result = env.run()
        self.assertEqual(result['fault'], 'wrong_direction')
        self.assertLessEqual(env.clock.ms, 10)
        self.assertEqual(env.motor.control.value, (999, 999, 99))

    def test_timeout_brakes_restores_limits_and_caps_ring(self):
        env = Env('timeout')
        result = env.run()
        self.assertEqual(result['fault'], 'timeout')
        self.assertLessEqual(env.clock.ms, 2000)
        self.assertLessEqual(len(result['records']), 100)
        self.assertEqual(env.motor.control.value, (999, 999, 99))

    def test_initial_touch_does_not_dispatch_and_is_not_home_confirmation(self):
        env = Env(initially_pressed=True)
        result = env.run()
        self.assertEqual(result['outcome'], 'touch_pressed_at_start')
        self.assertFalse(env.motor.commands)
        self.assertNotIn('home_confirmed', result)


if __name__ == '__main__':
    unittest.main()
