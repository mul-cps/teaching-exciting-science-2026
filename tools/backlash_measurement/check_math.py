"""Bounded offline math/admission regression checks; no hardware imports."""
from copy import deepcopy
import time
import json
import numpy as np
from analyze import analyze, rotation
from synthetic_fixture import fixture


def main():
    manifest, rows = fixture()
    start = time.perf_counter()
    report = analyze(manifest, rows)
    elapsed = time.perf_counter()-start
    json.dumps(report, allow_nan=False)
    widths = [b['hysteresis_width_deg'] for b in report['matched_bins']]
    assert abs(float(np.median(widths))-.8) < .025, widths
    assert report['physical_qualified'] is False and report['synthetic'] is True
    assert len(report['reversal_detection_travel']) == 5
    assert all(r['encoder_travel_deg'] >= .8 for r in report['reversal_detection_travel'])
    failures = 0
    for label in ('circular', 'finger', 'clock', 'uncertainty', 'camera', 'off_axis',
                  'owner', 'ambiguous', 'missing', 'speed', 'hash', 'duplicate'):
        m, r = deepcopy(manifest), deepcopy(rows)
        frame = next(row for row in r if row['kind'] == 'frame')
        if label == 'circular': m['independent_mount_calibration'] = False
        if label == 'finger': m['child']['moving_finger'] = True
        if label == 'clock': frame['t_s'] = -1
        if label == 'uncertainty': frame['observations']['child']['sigma_deg'] = 4
        if label == 'camera':
            second = [row for row in r if row['kind'] == 'frame'][1]
            second['observations']['fixed-witness']['camera_from_tag'][0][3] = .1
        if label == 'off_axis':
            frame['observations']['child']['camera_from_tag'] = np.eye(4).tolist()
            rot = rotation(np.array([1, 0, 0]), 5)
            for i in range(3): frame['observations']['child']['camera_from_tag'][i][:3] = rot[i].tolist()
        if label == 'owner': frame['owner'] = 'other'
        if label == 'ambiguous': frame['observations']['child']['ambiguous'] = True
        if label == 'missing': r = [row for row in r if row['kind'] == 'encoder']
        if label == 'speed': m['max_speed_deg_s'] = .01
        if label == 'hash': frame['model_sha256'] = 'a'*64
        if label == 'duplicate': r.append(deepcopy(frame))
        try:
            analyze(m, r)
        except ValueError:
            failures += 1
        else:
            raise AssertionError('gate failed: '+label)
    print(f'PASS: 0.8 deg synthetic play, 5 reversals, {failures} rejection gates; '
          f'{len(rows)} records analyzed in {elapsed:.3f}s (offline only)')


if __name__ == '__main__':
    main()
