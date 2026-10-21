"""Bounded offline usefulness checks; no hardware fixtures or ROS services."""
import importlib.util
import math
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('servo_analysis', Path(__file__).with_name('analysis.py'))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
from workshop.station.dynamics_compensation import velocity_lag_candidate


class IdentificationChecks(unittest.TestCase):
    def test_irregular_samples_find_known_ripple_and_latency(self):
        rng = np.random.default_rng(21)
        command_t = np.arange(0., 5., .008)
        t = command_t + .001 + rng.uniform(-.0003, .0003, len(command_t))
        goal = 3*command_t
        actual = 3*(t-.12) + .32*np.sin(2*np.pi*8.7*t)
        metrics, _ = a.segment_metrics(t,actual,goal,command_t,cruise=(.25,4.75))
        self.assertAlmostEqual(metrics['spectrum']['candidate']['frequency_hz'],8.7,delta=.04)
        self.assertAlmostEqual(metrics['lag']['seconds'],.12,delta=.015)
        self.assertGreater(metrics['backward_intervals'], 0)

    def test_short_low_rate_window_withholds_frequency(self):
        t = np.arange(0.,.6,.05)
        metrics,_ = a.segment_metrics(t,np.sin(2*np.pi*8.7*t),t,t,cruise=(0.,.6))
        self.assertIsNone(metrics['spectrum']['candidate'])
        self.assertIn('withheld_reason',metrics['spectrum'])

    def test_20hz_search_has_alias_margin(self):
        t = np.arange(0.,5.,.05)
        result = a.spectrum(t,np.sin(2*np.pi*8.7*t),.01)
        self.assertLessEqual(result['search_high_hz'],8.0000001)
        self.assertIn('aliasing',result['interpretation'])

    def test_duplicate_and_nonfinite_times_rejected(self):
        for t in ([0,1,1,2],[0,1,math.nan,3]):
            with self.assertRaises(ValueError):
                a.segment_metrics(t,[0]*4,[0]*4,t)

    def test_goal_ambiguity_and_unmatched_condition_rejected(self):
        with self.assertRaises(ValueError):
            a.one_event([{'event':'x','goal_id':'g'}]*2,'x','g')
        run = dict(label='a',condition={},calibration_sha256='x',owner_source_sha256={},
                   profile_registers={},sample_period_s=.02,joints={})
        comparison = a.comparisons([run,{**run,'label':'b'}])[0]
        self.assertFalse(comparison['matched'])

    def test_wrong_owner_binding_fails_before_trace_analysis(self):
        binding = {'owner_trajectory_goal_id':'g','owner_session_id':'owner-a',
                   'calibration_sha256':'cal','effective_path_sha256':'path'}
        state = {'owner_session_id':'owner-b','calibration_sha256':'cal'}
        artifact = {'goal_id':'g','result':{'state':state},
                    'calibration_sha256':'cal','effective_path_sha256':'path'}
        report = {'expected_owner_session_id':'owner-a','controller_binding':binding}
        with tempfile.TemporaryDirectory() as directory:
            for name,value in [('controller-binding.json',binding),
                               ('controller-terminal-artifact.json',artifact),('report.json',report)]:
                (Path(directory)/name).write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'binding mismatch'):
                a.bound_run(directory,[],{},'cal',[],{})

    def test_disabled_and_bounded_offline_candidate(self):
        samples = [{'time':0.,'positions':{'j':0.},'velocities':{'j':0.}},
                   {'time':1.,'positions':{'j':1.},'velocities':{'j':2.}},
                   {'time':2.,'positions':{'j':2.},'velocities':{'j':0.}}]
        disabled = velocity_lag_candidate(samples)
        self.assertEqual(disabled['samples'][1]['positions']['j'],1.)
        candidate = velocity_lag_candidate(samples,enabled=True,lag_seconds={'j':.1},max_correction_rad={'j':.01})
        self.assertEqual(candidate['samples'][1]['positions']['j'],1.01)
        self.assertEqual(candidate['samples'][0]['positions']['j'],0.)
        self.assertEqual(candidate['samples'][-1]['positions']['j'],2.)
        self.assertEqual(samples[1]['positions']['j'],1.)
        self.assertEqual(candidate['validation_status'],'UNVALIDATED_OFFLINE_CANDIDATE')
        samples[0]['velocities']['j'] = 1.
        with self.assertRaises(ValueError):
            velocity_lag_candidate(samples,enabled=True)


if __name__ == '__main__':
    unittest.main()
