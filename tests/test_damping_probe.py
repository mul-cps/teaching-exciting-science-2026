import pytest
from workshop.system_probe import make_stages


def test_quick_probe_compares_same_path_at_three_speeds_and_finishes_centered():
    stages = make_stages(quick=True)
    sweeps = [s for s in stages if s['name'].startswith('speed-') and s['speed']]
    assert [s['speed'] for s in sweeps] == [1, 1, 3, 3, 8, 8]
    assert all(abs(s['to_angle'] - s['from_angle']) == 24 for s in sweeps)
    assert stages[-1]['to_angle'] == 0
    assert all(s['acceleration_register'] == 254 for s in stages)
    assert all(a['begin'] + a['duration'] == pytest.approx(b['begin'])
               for a, b in zip(stages, stages[1:]))


def test_focused_probe_keeps_both_directions_without_other_speed_trials():
    stages = make_stages(focus_speed=3)
    sweeps = [s for s in stages if s['name'].startswith('speed-') and s['speed']]
    assert [s['speed'] for s in sweeps] == [3, 3]
    assert sweeps[0]['from_angle'] == -12
    assert sweeps[1]['to_angle'] == -12
