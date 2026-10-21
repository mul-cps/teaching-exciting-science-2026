#!/usr/bin/env python3
"""Offline servo identification. Reads saved files only; never opens the servo bus."""
import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

SCHEMA = 'servo-identification-v1'
STEP_DEG = 360 / 4095  # Match station ArmController.pose, not nominal sensor spec.


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_json(path, provenance):
    path = Path(path)
    data = path.read_bytes()
    provenance.append({'path': str(path.resolve()), 'sha256': sha(data), 'bytes': len(data)})
    return json.loads(data)


def read_owner(path, provenance):
    """Freeze a complete-line prefix even while the root owner appends its log."""
    path = Path(path)
    with path.open('rb') as stream:
        size = path.stat().st_size
        data = stream.read(size)
    end = data.rfind(b'\n') + 1
    data = data[:end]
    provenance.append({'path': str(path.resolve()), 'sha256': sha(data),
                       'bytes': end, 'scope': 'complete-line prefix, not future appends'})
    wanted = {'connected', 'trajectory_preflight_passed', 'trajectory_feedback_complete',
              'trajectory_timing'}
    return [row for line in data.splitlines() if (row := json.loads(line)).get('event') in wanted]


def finite_array(values, label):
    result = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(result)):
        raise ValueError(f'{label}: nonfinite data')
    return result


def times(values, label):
    result = finite_array(values, label)
    if len(result) < 4 or np.any(np.diff(result) <= 0):
        raise ValueError(f'{label}: need at least four strictly increasing timestamps')
    return result


def summary(values):
    values = finite_array(values, 'summary')
    if not len(values):
        return None
    return {'count': len(values), 'min': float(np.min(values)), 'max': float(np.max(values)),
            'mean': float(np.mean(values)), 'rms': float(np.sqrt(np.mean(values ** 2))),
            'p99': float(np.percentile(values, 99))}


def spectrum(t, residual, quantum):
    """Fit sinusoids at real observation times; do not FFT interpolated encoders.

    Conservative search ceiling uses the largest gap, with 20% Nyquist margin.
    Every candidate includes a fresh cubic drift fit. Frequencies are descriptive
    candidates only: irregular sampling does not establish an anti-alias filter.
    """
    t = t - t[0]
    duration = float(t[-1])
    max_gap = float(np.max(np.diff(t)))
    high = min(20.0, 0.4 / max_gap)
    low = max(2.0, 3 / duration)
    result = {'method': 'least squares sinusoid + cubic drift at actual timestamps',
              'window_s': duration, 'rayleigh_resolution_hz': 1 / duration,
              'max_gap_nyquist_hz': 0.5 / max_gap, 'search_low_hz': low,
              'search_high_hz': high, 'candidate': None,
              'interpretation': 'descriptive; aliasing outside search band remains possible'}
    if len(t) < 30 or duration < 1.5 or high <= low:
        result['withheld_reason'] = 'short window, insufficient observations or conservative bandwidth'
        return result
    drift = np.polynomial.polynomial.polyvander(t / duration, 3)
    detrended = residual - drift @ np.linalg.lstsq(drift, residual, rcond=None)[0]
    rms = float(np.sqrt(np.mean(detrended ** 2)))
    result['cubic_detrended_rms_deg'] = rms
    if rms < quantum / math.sqrt(12):
        result['withheld_reason'] = 'detrended RMS below uniform quantization reference (not a noise estimate)'
        return result
    frequencies = np.linspace(low, high, min(2000, max(40, int((high-low)*duration*8))))
    best = None
    for f in frequencies:
        phase = 2 * np.pi * f * t
        design = np.column_stack((drift, np.sin(phase), np.cos(phase)))
        coefficients = np.linalg.lstsq(design, residual, rcond=None)[0]
        error = float(np.sum((residual - design @ coefficients) ** 2))
        if best is None or error < best[0]:
            best = (error, float(f), float(np.hypot(*coefficients[-2:])))
    result['candidate'] = {'frequency_hz': best[1], 'amplitude_peak_deg': best[2],
                           'fraction_detrended_variance_explained': float(1-best[0]/np.sum(detrended**2))}
    return result


def segment_metrics(t, actual, goal, command_t, *, quantum=STEP_DEG, cruise=None,
                    actual_writes=True):
    t = times(t, 'encoder timestamps')
    command_t = times(command_t, 'command timestamps')
    actual, goal = finite_array(actual, 'encoder'), finite_array(goal, 'command')
    if len(actual) != len(t) or len(goal) != len(command_t):
        raise ValueError('sample array lengths differ')
    if np.any(t < command_t[0]) or np.any(t > command_t[-1] + max(np.diff(t))):
        raise ValueError('encoder timestamps outside command coverage')
    # The bus holds the most recently written target, including quantization.
    held = goal[np.clip(np.searchsorted(command_t, t, side='right')-1, 0, len(goal)-1)]
    residual = actual-held
    delta = np.diff(actual)
    direction = int(np.sign(goal[-1]-goal[0]))
    active = np.abs(np.diff(held)) >= quantum * .5
    zero = np.abs(delta) < quantum * .5
    plateaus = []
    begin = None
    for i, stationary in enumerate(zero):
        if stationary and begin is None:
            begin = i
        if not stationary and begin is not None:
            plateaus.append({'begin_s': float(t[begin]-t[0]),
                             'duration_s': float(t[i]-t[begin]), 'jump_deg': float(delta[i]),
                             'command_change_deg': float(held[i]-held[begin])})
            begin = None
    if begin is not None:
        plateaus.append({'begin_s': float(t[begin]-t[0]),
                         'duration_s': float(t[-1]-t[begin]), 'jump_deg': None,
                         'command_change_deg': float(held[-1]-held[begin])})
    # Lag is a bounded kinematic fit, not a physical controller identification.
    lag = None
    if actual_writes and abs(goal[-1]-goal[0]) >= 4*quantum:
        max_lag = min(.5, (t[-1]-t[0])/4)
        mask = t >= command_t[0]+max_lag
        choices = []
        for delay in np.linspace(0, max_lag, 101):
            shifted = goal[np.clip(np.searchsorted(command_t, t[mask]-delay, side='right')-1, 0, len(goal)-1)]
            error = float(np.sqrt(np.mean((actual[mask]-shifted)**2)))
            choices.append((error, float(delay)))
        error, delay = min(choices)
        lag = {'seconds': delay, 'fit_rms_deg': error, 'search_max_s': max_lag,
               'at_search_boundary': delay in (0., max_lag),
               'method': 'nonnegative delay of held command; no offset compensation'}
    drift = np.polynomial.polynomial.polyvander((t-t[0])/(t[-1]-t[0]), 3)
    ripple = residual-drift@np.linalg.lstsq(drift, residual, rcond=None)[0]
    result = {'direction': direction, 'samples': len(t), 'duration_s': float(t[-1]-t[0]),
              'encoder_quantum_deg': quantum, 'tracking_mean_deg': float(np.mean(residual)),
              'tracking_rms_deg': float(np.sqrt(np.mean(residual**2))),
              'tracking_peak_abs_deg': float(np.max(np.abs(residual))),
              'cubic_detrended_ripple_rms_deg': float(np.sqrt(np.mean(ripple**2))),
              'cubic_detrended_ripple_peak_abs_deg': float(np.max(np.abs(ripple))),
              'cubic_detrended_ripple_peak_to_peak_deg': float(np.ptp(ripple)),
              'zero_encoder_interval_fraction': float(np.mean(zero)),
              'zero_while_command_changes_count': int(np.count_nonzero(zero & active)),
              'backward_intervals': int(np.count_nonzero(delta*direction < -quantum*.5)) if direction else None,
              'longest_zero_interval_s': max((p['duration_s'] for p in plateaus), default=0),
              'stick_then_jump': [p for p in plateaus if p['duration_s'] >= .04 and abs(p['command_change_deg']) >= quantum*.5],
              'encoder_interval_s': summary(np.diff(t)),
              'command_interval_s': summary(np.diff(command_t)) if actual_writes else None,
              'command_timestamp_kind': 'actual write' if actual_writes else 'feedback-associated goal; write timing unavailable',
              'lag': lag, 'spectrum': {'candidate': None, 'withheld_reason': 'no preregistered cruise window; full-path drift is not steady ripple'}}
    if cruise is not None:
        a, b = cruise
        mask = (t-t[0] >= a) & (t-t[0] <= b)
        if np.count_nonzero(mask) >= 4:
            result['spectrum'] = spectrum(t[mask], residual[mask], quantum)
        else:
            result['spectrum'] = {'candidate': None, 'withheld_reason': 'too few cruise samples'}
    records = [{'encoder_monotonic_s': float(a), 'command_held_deg': float(g),
                'encoder_deg': float(q), 'residual_deg': float(r), 'ripple_deg': float(d)}
               for a,g,q,r,d in zip(t,held,actual,residual,ripple)]
    return result, records


def one_event(rows, event, goal):
    matches = [r for r in rows if r.get('event') == event and r.get('goal_id') == goal]
    if len(matches) != 1:
        raise ValueError(f'{goal}: require exactly one {event}, got {len(matches)}')
    return matches[0]


def bound_run(directory, owner_rows, calibration, calibration_sha, provenance, conditions):
    directory = Path(directory)
    binding = read_json(directory/'controller-binding.json', provenance)
    artifact = read_json(directory/'controller-terminal-artifact.json', provenance)
    report = read_json(directory/'report.json', provenance)
    goal, owner = binding['owner_trajectory_goal_id'], binding['owner_session_id']
    state = artifact.get('result', {}).get('state')
    if state is None:  # Failed/aborted runs may have no terminal state; never invent it.
        state = read_json(directory/'pre-execute-state.json', provenance)
    if (artifact['goal_id'] != goal or state['owner_session_id'] != owner or
            report['expected_owner_session_id'] != owner or
            report['controller_binding'] != binding or
            artifact['calibration_sha256'] != calibration_sha or
            binding['calibration_sha256'] != calibration_sha or
            state['calibration_sha256'] != calibration_sha or
            artifact['effective_path_sha256'] != binding['effective_path_sha256']):
        raise ValueError(f'{directory}: owner/goal/path/calibration binding mismatch')
    # An owner-log event without session fields is admitted only inside a log
    # whose connected events all bind this exact owner/source/calibration.
    rows = None
    for candidate in owner_rows:
        # publish_state and _event both emit connected; only the state carries
        # owner identity. Never treat the unqualified duplicate as a new owner.
        connected = [r for r in candidate if r.get('event') == 'connected' and r.get('owner_session_id')]
        if connected and all(r.get('owner_session_id') == owner for r in connected):
            if rows is not None:
                raise ValueError('multiple logs match owner; provide only one prefix per owner')
            if any(r['calibration_sha256'] != calibration_sha or
                   r['owner_source_sha256'] != state['owner_source_sha256'] for r in connected):
                raise ValueError('owner log source/calibration mismatch')
            rows = candidate
    if rows is None:
        raise ValueError(f'no exact owner log for {owner}')
    preflight = one_event(rows, 'trajectory_preflight_passed', goal)
    if (preflight['collision_checked_path_sha256'] != binding['effective_path_sha256'] or
            preflight['calibration_sha256'] != calibration_sha):
        raise ValueError('preflight path/calibration mismatch')
    commands = preflight['commanded_samples']
    # A failed run may intentionally lack a complete feedback event. Reject it
    # clearly instead of substituting a different successful owner's trace.
    feedback = one_event(rows, 'trajectory_feedback_complete', goal)['samples']
    count = len(commands)
    samples = feedback[:count]
    if len(samples) != count or any(abs(a['time']-b['time']) > 1e-8 for a,b in zip(samples,commands)):
        raise ValueError('feedback/preflight sample grid mismatch (possibly aborted run)')
    timing_events = [r for r in rows if r.get('event') == 'trajectory_timing' and r.get('goal_id') == goal]
    if len(timing_events) > 1:
        raise ValueError('duplicate trajectory timing events')
    timing = timing_events[0]['timing'] if timing_events else {}
    write_times = timing.get('write_start_monotonic_s')
    encoder_t = times([r['monotonic'] for r in samples], 'feedback timestamps')
    if write_times is not None and len(write_times) != count:
        raise ValueError('write timestamp count does not match preflight')
    command_t = times(write_times, 'write timestamps') if write_times else encoder_t
    condition = conditions.get(directory.name, {})
    output = {'label': directory.name, 'kind': 'bound-owner', 'owner_session_id': owner,
              'owner_trajectory_goal_id': goal, 'calibration_sha256': calibration_sha,
              'owner_source_sha256': state['owner_source_sha256'],
              'effective_path_sha256': binding['effective_path_sha256'],
              'profile': state['profile'], 'profile_registers': state['profile_registers'],
              'sample_period_s': preflight['sample_period_s'],
              'outcome': report.get('success'), 'condition': condition,
              'timing': timing, 'joints': {},
              'physical_acceptance': 'pending human/video assessment; numerical success is insufficient'}
    csv_rows = []
    for joint in artifact['joint_names']:
        c = calibration[joint]
        def degrees(raw):
            return (raw-(c['range_min']+c['range_max'])/2)*STEP_DEG
        actual = [degrees(r['raw'][joint]) for r in samples]
        goals = [degrees(r['goal_raw'][joint]) for r in samples]
        expected = [math.degrees(r['positions'][joint]) for r in commands]
        if max(abs(a-b) for a,b in zip(goals,expected)) > STEP_DEG*1.01:
            raise ValueError(f'{joint}: raw goal differs from effective preflight by >1 encoder count')
        metrics, records = segment_metrics(encoder_t, actual, goals, command_t,
                                          cruise=condition.get('cruise_window_s'), actual_writes=bool(write_times))
        metrics['start_goal_deg'], metrics['end_goal_deg'] = goals[0], goals[-1]
        output['joints'][joint] = metrics
        csv_rows.extend({'run': directory.name, 'phase': 'trajectory', 'joint': joint, **r} for r in records)
    return output, csv_rows


def historical_probe(directory, provenance):
    directory = Path(directory)
    trace = read_json(directory/'trace.json', provenance)
    plan = read_json(directory/'probe-plan.json', provenance)
    result = read_json(directory/'result.json', provenance)
    registers = read_json(directory/'base-registers.json', provenance)
    output = {'label': directory.name, 'kind': 'historical-125hz-probe',
              'calibration_sha256': plan.get('calibration_sha256'), 'result': result,
              'base_registers': registers, 'plan': plan, 'phases': {},
              'timestamp_definition': 'historical command_t/follower_t are measured bus operation midpoints relative to routine start',
              'binding_limitations': ['no owner session or owner source hash in historical schema',
                                      'bus midpoint timestamps do not establish precise command/encoder phase',
                                      'single pose/trial; historical settings cannot establish current behavior']}
    csv_rows = []
    for stage in plan['stages']:
        if not stage['name'].startswith('speed-') or not stage['speed']:
            continue
        ramp = max(.8, stage['speed']*.085)
        selected = [r for r in trace if r['phase'] == stage['name'] and
                    stage['begin']+ramp+.2 <= r['t'] <= stage['begin']+stage['duration']-ramp-.2]
        if len(selected) < 4:
            continue
        ct = [r['command_t'] for r in selected]
        at = [r['follower_t'] for r in selected]
        # Keep a preceding command for coverage and a following command when
        # available. Analysis uses actual write/read timestamps, never phase t.
        all_phase = [r for r in trace if r['phase'] == stage['name']]
        goals = [r for r in all_phase if ct[0] <= r['command_t'] <= at[-1]+.009]
        metrics, records = segment_metrics(at, [r['actual']['shoulder_pan'] for r in selected],
                                          [r['goal']['shoulder_pan'] for r in goals],
                                          [r['command_t'] for r in goals], cruise=(0.,at[-1]-at[0]))
        metrics['speed_deg_s'] = stage['speed']
        metrics['native_acceleration'] = stage['acceleration_register']
        output['phases'][stage['name']] = metrics
        csv_rows.extend({'run': directory.name, 'phase': stage['name'], 'joint': 'shoulder_pan', **r} for r in records)
    return output, csv_rows


def comparisons(runs):
    """Only compare repeat effects after explicit pose/load/path recording."""
    required = ('pose_id', 'load_id', 'path_id', 'profile_id')
    result = []
    for a,b in itertools.combinations(runs,2):
        reasons = []
        if any(not a['condition'].get(k) or not b['condition'].get(k) for k in required):
            reasons.append('missing explicitly recorded pose/load/path/profile conditions')
        elif any(a['condition'][k] != b['condition'][k] for k in required):
            reasons.append('different pose/load/path/profile conditions')
        for k in ('calibration_sha256','owner_source_sha256','profile_registers','sample_period_s'):
            if a[k] != b[k]:
                reasons.append(f'different {k}')
        # Reversals need same endpoint pair for every joint; direction reported separately.
        if any(sorted([a['joints'][j]['start_goal_deg'],a['joints'][j]['end_goal_deg']]) !=
               sorted([b['joints'][j]['start_goal_deg'],b['joints'][j]['end_goal_deg']]) for j in a['joints']):
            reasons.append('different endpoint pair')
        result.append({'runs':[a['label'],b['label']], 'matched': not reasons,
                       'reasons': reasons, 'aggregate_effect': 'withheld; repeat and direction coverage required'})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, action='append', default=[], help='saved bound probe directory')
    parser.add_argument('--owner-log', type=Path, action='append', default=[])
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--historical-probe', type=Path, action='append', default=[])
    parser.add_argument('--conditions', type=Path, help='JSON keyed by run label; never infer matched load/pose')
    parser.add_argument('--output', type=Path, required=True, help='new output directory')
    args = parser.parse_args()
    provenance = []
    conditions = read_json(args.conditions, provenance) if args.conditions else {}
    owner_rows = [read_owner(p,provenance) for p in args.owner_log]
    calibration = read_json(args.calibration,provenance) if args.calibration else None
    if args.run and calibration is None:
        parser.error('--run requires --calibration and matching --owner-log')
    if not args.run and not args.historical_probe:
        parser.error('provide at least one --run or --historical-probe')
    runs, historical, rows, unavailable = [], [], [], []
    calibration_sha = provenance[-1]['sha256'] if args.calibration else None
    for directory in args.run:
        try:
            run, samples = bound_run(directory,owner_rows,calibration,calibration_sha,provenance,conditions)
            runs.append(run); rows.extend(samples)
        except (ValueError, KeyError, FileNotFoundError) as error:
            unavailable.append({'run': str(directory), 'reason': str(error)})
    for directory in args.historical_probe:
        run, samples = historical_probe(directory,provenance)
        historical.append(run); rows.extend(samples)
    output = {'schema': SCHEMA, 'provenance': provenance, 'runs': runs, 'historical': historical,
              'unavailable': unavailable, 'comparisons': comparisons(runs),
              'limitations': ['No automatic tuning, compensation or hardware access.',
                              'No torque saturation inference without synchronized current/load evidence.',
                              'Stick/jump events describe quantized observations, not proven static friction.',
                              'Reported command cadence concerns trajectory ticks; settle cadence is separate.',
                              'Cubic full-path residual is descriptive, not a steady-state band-pass RMS.']}
    output['compensation_candidate'] = {
        'schema': 'offline-velocity-lag-candidate-v1', 'enabled': False,
        'lag_seconds_by_joint': {j: 0.0 for run in runs for j in run['joints']},
        'reason': 'No independently matched whole-trajectory training/validation split supplied; descriptive lag is not a coefficient.',
        'data_required': ['At least two whole training and two whole held-out trajectories in BOTH directions under matched conditions.',
                          'Separate external angle observation with timestamp and uncertainty; tracking and human smoothness acceptance.'],
        'deployment_requirement': 'Export corrected samples first, then run all existing dynamics/collision/effective-path SHA validation; never modify after validation.'}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'analysis.json').write_text(json.dumps(output, indent=2, allow_nan=False)+'\n')
    if rows:
        with (args.output/'samples.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
    print(json.dumps({'output': str(args.output), 'bound_runs': len(runs), 'historical_runs': len(historical),
                      'unavailable': unavailable}, indent=2))
    return 2 if unavailable else 0


if __name__ == '__main__':
    raise SystemExit(main())
