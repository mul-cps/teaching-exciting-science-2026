from pathlib import Path
import ast
import importlib.util
import unittest
spec = importlib.util.spec_from_file_location('fast', str(Path(__file__).resolve().parents[1] / 'fast_sorter.py'))
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
fake_spec = importlib.util.spec_from_file_location('fakes', str(Path(__file__).resolve().parent / 'fakes.py'))
f = importlib.util.module_from_spec(fake_spec)
fake_spec.loader.exec_module(f)

class Touch:
    def __init__(self, runner):
        self.runner = runner
    def pressed(self):
        return self.runner.fault == 'touch' or self.runner.d.angle() <= 12
class Runner(f.Runner):
    def __init__(self, fault=None):
        super().__init__(fault)
        self.touch = Touch(self)
    def wait(self, ms):
        self.now += ms
        if hasattr(self, 'd') and self.d.target is not None:
            if self.fault == 'deadline':
                return
            delta = self.d.target - self.d.position
            step = min(2, abs(delta)) * (1 if delta > 0 else -1)
            if self.fault == 'jump':
                step = 21
            elif self.fault == 'wrong':
                step = -4
            elif self.fault == 'soft':
                self.d.position = 551
                return
            elif self.fault == 'budget':
                self.d.target = 30
                step = 2
            self.d.position += step
            if self.fault == 'A_drift':
                self.a.position += 4

class Tests(unittest.TestCase):
    def test_one_profile_with_exact_targets_then_slow_home(self):
        r = Runner()
        g.traverse(r, r.speaker, r.wait)
        targets = [call for call in r.calls if call[0] == 'run_target']
        self.assertEqual(targets, [('run_target', 600, target, 'BRAKE', False)
                                  for target in (10, 132, 360, 530, 15)])
        self.assertEqual(sum(call[0] == 'home_d' for call in r.calls), 2)
        self.assertEqual(r.d.angle(), 0)
        self.assertEqual(r.a.angle(), 0)
        self.assertFalse(any(call[0] in ('home_a', 'move', 'reset', 'run_angle') for call in r.calls))
        limits = [call for call in r.calls if call[0] == 'limits']
        self.assertEqual(limits, [('limits', {'speed': 600, 'acceleration': 1200, 'actuation': 50}),
                                  ('limits', {'speed': 120, 'acceleration': 400, 'actuation': 30})])
        self.assertTrue(r.cleaned)
        self.assertEqual(sum(call[0] == 'beep' for call in r.calls), 3)
        complete = [fields for phase, fields in r.logs if phase.startswith('D_fast_') and phase.endswith('_complete')]
        self.assertEqual([fields['target'] for fields in complete], [10, 132, 360, 530, 15])
        self.assertTrue(all('native_elapsed_ms' in fields for fields in complete))
    def test_faults_brake_both_no_retry_or_completion_beeps(self):
        for fault, message in [('deadline', 'deadline'), ('stall', 'stalled'),
                               ('A_drift', 'feeder A drift'), ('done_error', 'native target error'),
                               ('jump', 'encoder step'), ('wrong', 'soft limit'),
                               ('soft', 'soft limit'), ('budget', 'travel budget'),
                               ('touch', 'unexpected touch')]:
            r = Runner(fault)
            with self.assertRaisesRegex(RuntimeError, message):
                g.traverse(r, r.speaker, r.wait)
            self.assertTrue(r.cleaned, fault)
            self.assertIsNone(r.a.target, fault)
            self.assertIsNone(r.d.target, fault)
            self.assertEqual(sum(call[0] == 'beep' for call in r.calls), 1, fault)
            self.assertFalse(any(phase == 'fast_sorter_complete' for phase, fields in r.logs))
            if fault == 'deadline':
                self.assertEqual(r.now, 6000)
    def test_range_rejects_before_native_command(self):
        r = Runner()
        r.d = r.init_motor('D', 'D')
        r.a = r.init_motor('A', 'A')
        for target in (-1, 551):
            with self.assertRaises(ValueError):
                g.move_sorter(r, target, 0)
        self.assertFalse(any(call[0] == 'run_target' for call in r.calls))
    def test_done_accurate_target_before_residual_stalled_flag(self):
        r = Runner()
        r.d = r.init_motor('D', 'D')
        r.a = r.init_motor('A', 'A')
        r.d.control.done = lambda: True
        r.d.control.stalled = lambda: True
        g.move_sorter(r, 0, 0)
    def test_python34_no_timed_restart_or_feeder_motion(self):
        with open(str(Path(__file__).resolve().parents[1] / 'fast_sorter.py')) as handle:
            tree = ast.parse(handle.read(), feature_version=(3, 4))
        forbidden = {'run_time', 'run', 'run_until_stalled', 'home_a', 'reset_angle', 'run_angle', 'move'}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, forbidden)

if __name__ == '__main__':
    unittest.main()
