from pathlib import Path
import ast
import importlib.util
import unittest
spec = importlib.util.spec_from_file_location('cycle', str(Path(__file__).resolve().parents[1] / 'fast_feeder.py'))
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
fake_spec = importlib.util.spec_from_file_location('fakes', str(Path(__file__).resolve().parent / 'fakes.py'))
f = importlib.util.module_from_spec(fake_spec)
fake_spec.loader.exec_module(f)

class Tests(unittest.TestCase):
    def test_one_load_then_eject_without_D_motion_or_encoder_reset(self):
        r = f.Runner()
        c.cycle(r, r.speaker, r.wait)
        targets = [call for call in r.calls if call[0] == 'run_target']
        self.assertEqual(targets, [('run_target', 600, 209, 'BRAKE', False),
                                   ('run_target', 600, 0, 'BRAKE', False)])
        self.assertEqual(r.a.angle(), 0)
        self.assertEqual(r.d.angle(), 0)
        self.assertFalse(any(call[0] in ('home_d', 'home_a', 'move', 'reset', 'run_angle') for call in r.calls))
        self.assertIn(('limits', {'speed': 600, 'acceleration': 2400, 'actuation': 50}), r.calls)
        phases = [phase for phase, fields in r.logs if phase.endswith('_complete')]
        self.assertEqual(phases, ['A_LOAD_READY_complete', 'A_EJECT_complete', 'one_eject_cycle_complete'])
        self.assertTrue(r.cleaned)
        self.assertEqual(sum(call[0] == 'beep' for call in r.calls), 3)
    def test_first_leg_faults_brake_both_without_retry_or_success_beeps(self):
        for fault, message in [('deadline', 'deadline'), ('stall', 'stalled'),
                               ('drift', 'sorter D drift'), ('done_error', 'native target error'),
                               ('jump', 'encoder step'), ('wrong', 'wrong direction')]:
            r = f.Runner(fault)
            with self.assertRaisesRegex(RuntimeError, message):
                c.cycle(r, r.speaker, r.wait)
            self.assertTrue(r.cleaned)
            self.assertIsNone(r.a.target)
            self.assertIsNone(r.d.target)
            self.assertEqual(sum(call[0] == 'run_target' for call in r.calls), 1)
            self.assertEqual(sum(call[0] == 'beep' for call in r.calls), 1)
            if fault == 'deadline':
                self.assertEqual(r.now, 6000)
    def test_return_fault_stops_after_exactly_two_commands(self):
        r = f.Runner()
        original_init = r.init_motor
        def init_motor(port, label):
            motor = original_init(port, label)
            if port == 'A':
                original_target = motor.run_target
                def run_target(speed, target, then, wait):
                    original_target(speed, target, then, wait)
                    if target == 0:
                        r.fault = 'stall'
                motor.run_target = run_target
            return motor
        r.init_motor = init_motor
        with self.assertRaisesRegex(RuntimeError, 'stalled'):
            c.cycle(r, r.speaker, r.wait)
        self.assertTrue(r.cleaned)
        self.assertEqual(sum(call[0] == 'run_target' for call in r.calls), 2)
        self.assertEqual(sum(call[0] == 'beep' for call in r.calls), 1)
        self.assertFalse(any(phase == 'one_eject_cycle_complete' for phase, fields in r.logs))
    def test_done_accurate_target_wins_over_stall_flag(self):
        r = f.Runner()
        r.d = r.init_motor('D', 'D')
        r.a = r.init_motor('A', 'A')
        r.a.control.done = lambda: True
        r.a.control.stalled = lambda: True
        c.finite_target(r, 'same_target', 0)
        self.assertEqual(r.a.angle(), 0)
    def test_python34_no_homing_zeroing_or_timed_restart(self):
        with open(str(Path(__file__).resolve().parents[1] / 'fast_feeder.py')) as handle:
            tree = ast.parse(handle.read(), feature_version=(3, 4))
        forbidden = {'run_time', 'run', 'run_until_stalled', 'home_a', 'home_d', 'reset_angle', 'move', 'run_angle'}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, forbidden)

if __name__ == '__main__':
    unittest.main()
