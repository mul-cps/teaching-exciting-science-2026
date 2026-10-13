"""Reproduce steady-cruise and settled-hold metrics from a saved system probe."""
import argparse
import json
from pathlib import Path
import numpy as np
from workshop.motion_diagnostics import STEP, fit_ripple, reversal_metrics, spectrum_peak


def analyze_probe(directory):
    directory = Path(directory)
    rows = json.loads((directory / 'trace.json').read_text())
    plan = json.loads((directory / 'probe-plan.json').read_text())
    report = dict(result=json.loads((directory / 'result.json').read_text()),
                  settings={k: plan.get(k) for k in ('base_d_original', 'base_d_trial', 'base_extra_original', 'base_extra_trial', 'camera_fps')},
                  phases={}, holds={}, other_joint_motion={},
                  method='Exclude quintic ramps plus 0.2s; cubic-detrended 2-20Hz residual spectrum; 5-10Hz sine fit; settled holds exclude first 0.5s',
                  limitations=['Single pose and trial; no universal safe speed band.',
                               'Encoder residual does not distinguish mechanical from servo-loop oscillation.',
                               'Short cruise windows have limited frequency resolution.'])
    for stage in plan['stages']:
        phase = [r for r in rows if r['phase'] == stage['name']]
        if not phase:
            continue
        if not stage['speed']:
            settled = [r['actual']['shoulder_pan'] for r in phase if r['t'] >= stage['begin'] + .5]
            report['holds'][stage['name']] = dict(
                encoder_range_deg=float(np.ptp([r['actual']['shoulder_pan'] for r in phase])),
                settled_range_deg=float(np.ptp(settled)) if settled else None)
            continue
        if not stage['name'].startswith('speed-'):
            continue
        ramp = max(.8, stage['speed'] * .085)
        steady = [r for r in phase if stage['begin'] + ramp + .2 <= r['t'] <= stage['begin'] + stage['duration'] - ramp - .2]
        if len(steady) < 10:
            continue
        ct = np.array([r['command_t'] for r in steady])
        at = np.array([r['follower_t'] for r in steady])
        t = np.arange(max(ct[0], at[0]), min(ct[-1], at[-1]), .008)
        goal = np.interp(t, ct, [r['goal']['shoulder_pan'] for r in steady])
        actual = np.interp(t, at, [r['actual']['shoulder_pan'] for r in steady])
        residual = actual - goal
        report['phases'][stage['name']] = dict(
            speed_deg_s=stage['speed'], native_acceleration=stage['acceleration_register'],
            steady_seconds=float(t[-1] - t[0]), samples=len(t),
            tracking_mean_deg=float(np.mean(residual)),
            tracking_rms_deg=float(np.sqrt(np.mean(residual ** 2))),
            spectrum=spectrum_peak(t, residual), ripple_fit=fit_ripple(t, residual),
            backwards=reversal_metrics(t, goal, actual, STEP))
    for joint in plan['start']:
        if joint != 'shoulder_pan':
            report['other_joint_motion'][joint] = dict(
                max_deviation_from_start=max(abs(r['actual'][joint] - plan['start'][joint]) for r in rows),
                target_changed=any(abs(r['goal'][joint] - plan['start'][joint]) > 1e-9 for r in rows))
    gaps = np.diff([r['t'] for r in rows])
    report['timing'] = dict(samples=len(rows), mean_hz=float(1 / gaps.mean()),
                           max_gap_ms=float(gaps.max() * 1000), max_cycle_ms=max(r['cycle_ms'] for r in rows))
    (directory / 'probe-analysis.json').write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze_probe(args.directory), indent=2))


if __name__ == '__main__':
    main()
