"""Static resource contract plus targeted negative controls; no ROS required."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('complete_resource_probe', ROOT / 'tools/performance/benchmark_complete_report_admission.py')
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def test_frozen_completion_resource_contract():
    assert probe.resource_guard(probe.SOURCE.read_text())


@pytest.mark.parametrize('old,new', [
    ('staged_tracker = copy.copy(self.tracker)', 'staged_tracker = copy.deepcopy(self.tracker)'),
    ('report = dict(report)', 'report = dict(report)\n                self.last_analysis_result_stamp = stamp'),
    ('report = dict(report)', 'report = dict(report)\n                frame = job["depth"]'),
    ('report = dict(report)', 'report = dict(report)\n                device.synchronize()'),
    ('report = dict(report)', 'report = dict(report)\n                device.readback()'),
    ('report = dict(report)', 'report = dict(report)\n                self._analyze_job(job)'),
    ('self.tracks = retained', 'self.tracks = retained\n        self.tracks[1]["stamp"] = stamp_s'),
    ('if commit is not None:\n                    commit()', 'if commit is not None:\n                    pass'),
])
def test_resource_guard_rejects_negative_mutation(old, new):
    source = probe.SOURCE.read_text()
    assert old in source
    with pytest.raises(AssertionError):
        probe.resource_guard(source.replace(old, new, 1))
