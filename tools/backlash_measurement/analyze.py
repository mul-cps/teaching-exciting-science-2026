"""Offline independent camera/encoder hysteresis; no robot or camera dependencies.

Run with --help. Capture adapters may call append_record with already available
read-only snapshots. This module opens files only; it never connects to hardware.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


SCHEMA = 'so101-camera-backlash-v1'
HASHES = ('model_sha256', 'calibration_sha256', 'mount_sha256', 'source_sha256',
          'collision_sha256', 'controller_sha256')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def number(value, name, positive=False):
    value = float(value)
    require(math.isfinite(value) and (not positive or value > 0), name)
    return value


def rigid(value):
    value = np.asarray(value, dtype=float)
    require(value.shape == (4, 4) and np.isfinite(value).all(), 'invalid transform')
    require(np.allclose(value[3], [0, 0, 0, 1], atol=1e-7), 'invalid homogeneous row')
    r = value[:3, :3]
    require(np.allclose(r.T @ r, np.eye(3), atol=1e-6) and
            abs(np.linalg.det(r)-1) < 1e-6, 'invalid rotation')
    return value


def rotation(axis, degrees):
    a = np.radians(degrees)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a)*cross + (1-math.cos(a))*(cross @ cross)


def rotation_degrees(r):
    return math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2, -1, 1))))


def validate_manifest(m):
    require(m['schema'] == SCHEMA, 'schema mismatch')
    for key in HASHES:
        value = m[key]
        require(isinstance(value, str) and len(value) == 64 and
                all(c in '0123456789abcdef' for c in value), 'invalid '+key)
    for key in ('run_id', 'owner', 'goal_id', 'camera_serial', 'optical_frame',
                'clock_domain', 'load_pose_id', 'uncertainty_source', 'camera_calibration_source'):
        require(isinstance(m[key], str) and bool(m[key].strip()), 'missing '+key)
    require(m['independent_mount_calibration'] is True, 'encoder/FK initialized mounts are circular')
    require(m['pose_source'] == 'rgb_independent', 'depth pose/bias not admitted')
    require(m['camera_fixed'] is True, 'fixed camera required')
    require(len({m['parent']['instance'], m['child']['instance'], m['witness_instance']}) == 3,
            'three independent marker instances required')
    for side in ('parent', 'child'):
        require(m[side]['rigid'] is True and m[side]['moving_finger'] is False,
                'rigid link marker required; finger marker excluded')
        rigid(m[side]['link_from_tag'])
    rigid(m['parent_from_child_zero'])
    axis = np.asarray(m['axis_parent'], dtype=float)
    require(axis.shape == (3,) and np.isfinite(axis).all() and
            abs(np.linalg.norm(axis)-1) < 1e-6, 'unit joint axis required')
    for key in ('degrees_per_count', 'encoder_sign', 'encoder_zero_count'):
        number(m[key], key)
    require(m['degrees_per_count'] > 0 and m['encoder_sign'] in (-1, 1), 'encoder conversion')
    for key in ('mount_uncertainty_deg', 'clock_uncertainty_s', 'max_sync_gap_s',
                'max_speed_deg_s', 'max_camera_drift_deg', 'max_camera_drift_m',
                'max_off_axis_deg', 'max_sigma_deg', 'bin_width_deg',
                'min_visible_motion_deg', 'max_reprojection_px'):
        number(m[key], key, positive=True)
    for key in ('min_frames_per_direction', 'min_reversals', 'min_bins'):
        require(isinstance(m[key], int) and m[key] >= 2, 'invalid '+key)


def validate_record(m, row):
    require(row['kind'] in ('encoder', 'frame'), 'unknown record kind')
    for key in ('run_id', 'owner', 'goal_id', 'load_pose_id', 'clock_domain', *HASHES):
        require(row[key] == m[key], 'provenance changed: '+key)
    number(row['t_s'], 'timestamp')
    if row['kind'] == 'encoder':
        require(type(row['raw_count']) is int, 'raw integer encoder count required')
    else:
        require(row['camera_serial'] == m['camera_serial'] and
                row['optical_frame'] == m['optical_frame'], 'camera identity changed')
        require(isinstance(row['image_sha256'], str) and len(row['image_sha256']) == 64 and
                all(c in '0123456789abcdef' for c in row['image_sha256']), 'image hash required')
        for instance in (m['parent']['instance'], m['child']['instance'], m['witness_instance']):
            obs = row['observations'][instance]
            rigid(obs['camera_from_tag'])
            require(obs['ambiguous'] is False and obs['independent'] is True,
                    'ambiguous or encoder-dependent observation')
            number(obs['sigma_deg'], 'pose sigma', positive=True)
            require(0 <= number(obs['reprojection_rms_px'], 'reprojection') <= m['max_reprojection_px'],
                    'reprojection gate')
            require(np.asarray(obs['corners_px']).shape == (4, 2) and
                    np.isfinite(obs['corners_px']).all(), 'raw corners required')


def append_record(path, manifest, record):
    """Capture API for ROOT's existing read-only snapshots; does not poll anything.

    ROOT supplies exposure/common-clock timestamps, raw corners and independently
    resolved poses. Validation failure raises before appending. Use one writer.
    """
    validate_manifest(manifest)
    validate_record(manifest, record)
    with Path(path).open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, allow_nan=False, separators=(',', ':'))+'\n')


def analyze(m, records):
    validate_manifest(m)
    require(records, 'no synchronized independent observations supplied')
    for row in records:
        validate_record(m, row)
    enc = sorted((r for r in records if r['kind'] == 'encoder'), key=lambda r: r['t_s'])
    frames = sorted((r for r in records if r['kind'] == 'frame'), key=lambda r: r['t_s'])
    require(len(enc) >= 3 and len(frames) >= 4, 'insufficient synchronized observations')
    ts = np.array([r['t_s'] for r in enc])
    require(np.all(np.diff(ts) > 0), 'duplicate encoder timestamps')
    ft = np.array([r['t_s'] for r in frames])
    require(np.all(np.diff(ft) > 0), 'duplicate exposure timestamps')
    counts = np.array([r['raw_count'] for r in enc], dtype=float)
    # Wrapped counts must be unwrapped by the capture adapter with recorded provenance.
    qs = (counts-m['encoder_zero_count'])*m['degrees_per_count']*m['encoder_sign']
    axis = np.asarray(m['axis_parent'])
    zero = rigid(m['parent_from_child_zero'])[:3, :3]
    mount_p = rigid(m['parent']['link_from_tag'])[:3, :3]
    mount_c = rigid(m['child']['link_from_tag'])[:3, :3]
    witness0 = rigid(frames[0]['observations'][m['witness_instance']]['camera_from_tag'])
    output = []
    for row in frames:
        t = row['t_s']
        right = int(np.searchsorted(ts, t, side='right'))
        require(0 < right < len(ts), 'exposure lacks encoder bracket')
        left = right-1
        gap = ts[right]-ts[left]
        require(gap <= m['max_sync_gap_s'], 'encoder synchronization gap')
        speed = (qs[right]-qs[left])/gap
        require(abs(speed) <= m['max_speed_deg_s'], 'not a slow reversal')
        obs = row['observations']
        wp = rigid(obs[m['witness_instance']]['camera_from_tag'])
        drift = rotation_degrees(witness0[:3, :3].T @ wp[:3, :3])
        require(drift <= m['max_camera_drift_deg'] and
                np.linalg.norm(wp[:3, 3]-witness0[:3, 3]) <= m['max_camera_drift_m'],
                'camera/witness drift gate')
        p, c = obs[m['parent']['instance']], obs[m['child']['instance']]
        rp = rigid(p['camera_from_tag'])[:3, :3] @ mount_p.T
        rc = rigid(c['camera_from_tag'])[:3, :3] @ mount_c.T
        relative = rp.T @ rc @ zero.T
        vee = np.array([relative[2, 1]-relative[1, 2], relative[0, 2]-relative[2, 0],
                        relative[1, 0]-relative[0, 1]])/2
        theta = math.degrees(math.atan2(float(axis @ vee), float((np.trace(relative)-1)/2)))
        off_axis = rotation_degrees(rotation(axis, theta).T @ relative)
        require(off_axis <= m['max_off_axis_deg'], 'off-axis motion or mounting error')
        # Conservative additive bound; systematic mount error is never reduced by N.
        sigma = (p['sigma_deg']+c['sigma_deg']+m['mount_uncertainty_deg']+
                 abs(speed)*(m['clock_uncertainty_s']+gap/2)+drift)
        require(sigma <= m['max_sigma_deg'], 'angular uncertainty gate')
        output.append({'t_s': t, 'encoder_deg': float(np.interp(t, ts, qs)),
                       'external_deg': theta, 'sigma_deg': sigma,
                       'direction': int(np.sign(speed)), 'off_axis_deg': off_axis})
    angles = np.degrees(np.unwrap(np.radians([r['external_deg'] for r in output])))
    for row, angle in zip(output, angles):
        row['external_deg'] = float(angle)
    require(np.ptp(angles) >= m['min_visible_motion_deg'], 'no resolved visible pose change')
    for direction in (-1, 1):
        require(sum(r['direction'] == direction for r in output) >= m['min_frames_per_direction'],
                'insufficient bidirectional frames')
    bins = {}
    for row in output:
        if row['direction']:
            key = math.floor(row['encoder_deg']/m['bin_width_deg'])
            bins.setdefault(key, {-1: [], 1: []})[row['direction']].append(row)
    matched = []
    for key, sides in sorted(bins.items()):
        if min(map(len, sides.values())) < 3:
            continue
        # Median external-minus-encoder removes residual encoder position within bin.
        residual = {d: float(np.median([r['external_deg']-r['encoder_deg'] for r in sides[d]]))
                    for d in (-1, 1)}
        bound = sum(max(r['sigma_deg'] for r in sides[d]) for d in (-1, 1)) + m['bin_width_deg']
        width = abs(residual[1]-residual[-1])
        matched.append({'encoder_bin_center_deg': (key+.5)*m['bin_width_deg'],
                        'hysteresis_width_deg': width, 'conservative_bound_deg': bound,
                        'resolved': bool(width > 3*bound)})
    require(len(matched) >= m['min_bins'], 'insufficient matched bidirectional encoder bins')
    reversals = []
    previous_direction, start = 0, None
    for i, row in enumerate(output):
        direction = row['direction']
        if not direction:
            continue
        if previous_direction and direction != previous_direction:
            if start is not None:
                reversals.append({'status': 'censored_before_resolved_motion',
                                  'reversal_t_s': output[start]['t_s']})
            start = max(0, i-1)
        previous_direction = direction
        if start is not None and i > start:
            base = output[start]
            threshold = 3*(base['sigma_deg']+row['sigma_deg'])
            displacement = abs(row['external_deg']-base['external_deg'])
            if displacement > threshold:
                reversals.append({'status': 'resolved_motion_threshold',
                                  'reversal_t_s': base['t_s'],
                                  'encoder_travel_deg': abs(row['encoder_deg']-base['encoder_deg']),
                                  'external_threshold_deg': threshold,
                                  'note': 'detection travel upper bound; includes elastic/dynamic response'})
                start = None
    if start is not None:
        reversals.append({'status': 'censored_end_of_capture', 'reversal_t_s': output[start]['t_s']})
    require(len(reversals) >= m['min_reversals'], 'insufficient reversals')
    return {'schema': SCHEMA, 'run_id': m['run_id'], 'synthetic': m.get('synthetic', False),
            'physical_qualified': False, 'compensation_allowed': False,
            'interpretation': 'load-dependent hysteresis candidates, not isolated gear backlash',
            'matched_bins': matched, 'reversal_detection_travel': reversals,
            'samples': output}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('records', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw_m, raw_r = args.manifest.read_bytes(), args.records.read_bytes()
    try:
        result = analyze(json.loads(raw_m), [json.loads(line) for line in raw_r.splitlines() if line.strip()])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        result = {'schema': SCHEMA, 'physical_qualified': False, 'compensation_allowed': False,
                  'status': 'blocked', 'reason': str(exc)}
    result['input_sha256'] = {'manifest': hashlib.sha256(raw_m).hexdigest(),
                              'records': hashlib.sha256(raw_r).hexdigest()}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'matched_bins')}))
    return 2 if result.get('status') == 'blocked' else 0


if __name__ == '__main__':
    raise SystemExit(main())
