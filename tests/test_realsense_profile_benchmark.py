"""Host-only fixtures: no SDK or device imports."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch
import itertools
import tempfile
import json

SPEC = importlib.util.spec_from_file_location('profile_benchmark', Path(__file__).parents[1] / 'tools/benchmark_realsense_profiles.py')
b = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(b)

class ProfileBenchmarkTests(unittest.TestCase):
    def test_exact_profile_selection_and_rejection(self):
        profiles = [p for pair in b.PROFILES.values() for p in pair]
        self.assertEqual(len(b.select_profiles(profiles)), 3)
        with self.assertRaises(ValueError):
            b.select_profiles([p for p in profiles if p['width'] != 1920])
        with self.assertRaises(ValueError):
            b.verify_profiles(b.PROFILES['maximum'], b.PROFILES['same_grid'])

    def test_order_is_fixed_complete_and_randomized(self):
        plan = b.make_plan()
        self.assertEqual(plan, b.make_plan())
        self.assertEqual(len(plan), 18)
        self.assertEqual(len({(x['profile'], x['mode'], x['repeat']) for x in plan}), 18)

    def test_quantile_and_median_of_trial_p95(self):
        self.assertEqual(b.quantile([0, 10], .95), 9.5)
        trials = [{'profile': 'baseline', 'mode': 'capture', 'status': 'ok', 'metrics': {'align_wall_ms': {'p95': n}}} for n in [1, 10, 100]]
        self.assertEqual(b.aggregate(trials)['baseline/capture']['median_of_trial_p95_align_wall_ms'], 10)

    def test_gaps_duplicates_resets_and_domains(self):
        values = [{'id': i, 'timestamp_ms': t, 'domain': d} for i,t,d in [(1,10,'hw'),(1,10,'hw'),(4,40,'hw'),(5,50,'sys'),(2,60,'sys')]]
        result = b.stream_summary(values)
        self.assertEqual(result['unique_ids'], 4)
        self.assertEqual(result['repeats'], 1)
        self.assertEqual(result['observed_counter_gaps'], 2)
        self.assertEqual(result['counter_resets'], 1)
        self.assertEqual(result['domain_changes'], 1)
        self.assertEqual(result['same_domain_positive_timestamp_delta_ms']['count'], 2)

    def test_cohort_identity_preserves_individual_ids(self):
        rows = [{'depth': {'id': d}, 'color': {'id': c}} for d,c in [(1,2),(1,3),(1,3),(2,3)]]
        self.assertEqual(b.cohort_summary(rows), {'unique_cohorts': 3, 'repeated_cohorts': 1})


    def test_preregistration_is_byte_immutable_across_dynamic_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = {key: {'fixture': key} for key in b.PROTOCOL_FIELDS}
            plan['ownership'] = {'opaque_process_count': 507}
            b.save_plan(root, plan)
            original = (root/'preregistered-plan.json').read_bytes()
            plan['ownership'] = {'opaque_process_count': 508}
            b.save_plan(root, plan, capture=True)
            self.assertEqual((root/'preregistered-plan.json').read_bytes(), original)
            self.assertEqual(json.loads((root/'capture-preflight.json').read_text())['ownership'], plan['ownership'])
            plan['steady_s'] = 9
            with self.assertRaises(ValueError):
                b.save_plan(root, plan, capture=True)
            self.assertEqual((root/'preregistered-plan.json').read_bytes(), original)

    def test_existing_capture_artifacts_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            b.refuse_existing_capture(root)
            with self.assertRaises(ValueError):
                b.refuse_existing_capture(root, internal_trial=True)
            (root/'results.json').write_text('{}')
            with self.assertRaises(ValueError):
                b.refuse_existing_capture(root)
            (root/'results.json').unlink()
            (root/'trial-00').mkdir()
            with self.assertRaises(ValueError):
                b.refuse_existing_capture(root)
            b.refuse_existing_capture(root/'trial-01', internal_trial=True)

    def exercise_trial(self, failure=None):
        rs, pipe, cfg, dev, queue, frames = [MagicMock() for _ in range(6)]
        rs.pipeline.return_value = pipe
        rs.frame_queue.return_value = queue
        active = pipe.start.return_value
        active.get_device.return_value = dev
        pair = b.PROFILES['baseline']
        cfg.resolve.return_value.get_streams.return_value = pair
        cfg.resolve.return_value.get_device.return_value.get_info.return_value = 'private'
        active.get_streams.return_value = pair
        depth, color = MagicMock(), MagicMock()
        frames.get_depth_frame.return_value = depth
        frames.get_color_frame.return_value = color
        queue.wait_for_frame.return_value.as_frameset.return_value = frames
        for frame, prof in [(depth,pair[1]), (color,pair[0])]:
            frame.get_profile.return_value = prof
            frame.get_units.return_value = .001
            frame.get_frame_number.side_effect = itertools.count(1)
            frame.get_timestamp.side_effect = itertools.count(10)
            frame.get_frame_timestamp_domain.return_value = 'hardware_clock'
        if failure == 'start':
            pipe.start.side_effect = RuntimeError('start failed')
        if failure == 'timeout':
            queue.wait_for_frame.side_effect = RuntimeError('timeout')
        if failure == 'profile':
            active.get_streams.return_value = pair[:1]
        if failure == 'align':
            rs.align.return_value.process.side_effect = RuntimeError('align failed')
        ticks = (i*.001 for i in itertools.count())
        with patch.object(b, 'refuse_busy'), patch.object(b, 'device', return_value=(dev,'private')), patch.object(b, 'configure', return_value=cfg), patch.object(b, 'sdk_profile', side_effect=lambda p:p), patch.object(b, 'options', return_value=[]), patch.object(b, 'provenance', return_value={}), patch.object(b, 'WARMUP_S', 0), patch.object(b, 'STEADY_S', .02), patch.object(b.time, 'monotonic', side_effect=lambda:next(ticks)):
            if failure:
                with self.assertRaises((RuntimeError, ValueError)):
                    b.trial_run(rs, dict(profile='baseline', mode='sdk_align' if failure=='align' else 'capture', repeat=0))
                self.assertEqual(pipe.stop.call_count, 0 if failure=='start' else 1)
            else:
                result = b.trial_run(rs, dict(profile='baseline', mode='capture', repeat=0))
                self.assertEqual(result['status'], 'ok')
                self.assertGreaterEqual(result['metrics']['cohorts'], 2)
                self.assertTrue(all('host_completed_s' in row for row in result['samples']))
                self.assertEqual(queue.poll_for_frame.call_count, 1)
                pipe.stop.assert_called_once()

    def test_fake_sdk_lifecycle_and_endwindow(self):
        self.exercise_trial()
        for failure in ('start', 'timeout', 'profile', 'align'):
            with self.subTest(failure=failure):
                self.exercise_trial(failure)

    def test_hot_path_has_no_pixel_conversion_or_option_writes(self):
        import ast
        tree = ast.parse(Path(b.__file__).read_text())
        forbidden = {'get_data', 'tobytes', 'asarray', 'set_option', 'resize'}
        calls = {node.func.attr for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        self.assertFalse(calls & forbidden)

if __name__ == '__main__':
    unittest.main()
