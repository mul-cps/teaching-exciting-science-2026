"""Generate deterministic play-operator math data. NEVER physical evidence."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from analyze import HASHES, SCHEMA, append_record, rotation


def fixture():
    identity = np.eye(4).tolist()
    manifest = dict(schema=SCHEMA, synthetic=True, run_id='SYNTHETIC-MATH-ONLY',
                    owner='synthetic-no-hardware', goal_id='offline-math',
                    camera_serial='SYNTHETIC', optical_frame='synthetic_optical',
                    clock_domain='synthetic_seconds', load_pose_id='fixed-synthetic',
                    uncertainty_source='injected bounds; not a camera accuracy claim',
                    camera_calibration_source='synthetic-pinhole',
                    independent_mount_calibration=True, pose_source='rgb_independent',
                    camera_fixed=True, parent=dict(instance='parent', rigid=True,
                    moving_finger=False, link_from_tag=identity),
                    child=dict(instance='child', rigid=True, moving_finger=False,
                    link_from_tag=identity), witness_instance='fixed-witness',
                    parent_from_child_zero=identity, axis_parent=[0, 0, 1],
                    degrees_per_count=.001, encoder_sign=1, encoder_zero_count=0,
                    mount_uncertainty_deg=.005, clock_uncertainty_s=.001,
                    max_sync_gap_s=.11, max_speed_deg_s=1., max_camera_drift_deg=.2,
                    max_camera_drift_m=.002, max_off_axis_deg=.1, max_sigma_deg=.1,
                    bin_width_deg=.1, min_visible_motion_deg=3., max_reprojection_px=.5,
                    min_frames_per_direction=30, min_reversals=3, min_bins=10)
    for key in HASHES:
        manifest[key] = hashlib.sha256(('synthetic:'+key).encode()).hexdigest()
    common = {k: manifest[k] for k in ('run_id', 'owner', 'goal_id', 'load_pose_id',
                                       'clock_domain', *HASHES)}
    # Three bidirectional cycles with 0.8 degree full play width.
    q = np.concatenate([np.linspace(-3, 3, 301), np.linspace(3, -3, 301)[1:]]*3)
    encoders = [dict(common, kind='encoder', t_s=float(i*.1),
                     raw_count=int(round(x/.001))) for i, x in enumerate(q)]
    rows, external = list(encoders), float(q[0])
    for i in range(len(q)-1):
        at_exposure = float((q[i]+q[i+1])/2)
        external = max(at_exposure-.4, min(at_exposure+.4, external))
        child = np.eye(4)
        child[:3, :3] = rotation(np.array([0, 0, 1]), external)
        observations = {name: dict(camera_from_tag=np.array(pose).tolist(), independent=True,
                                   ambiguous=False, sigma_deg=.005,
                                   reprojection_rms_px=.1,
                                   corners_px=[[10, 10], [30, 10], [30, 30], [10, 30]])
                        for name, pose in (('parent', identity), ('child', child.tolist()),
                                           ('fixed-witness', identity))}
        rows.append(dict(common, kind='frame', t_s=float((i+.5)*.1),
                         camera_serial=manifest['camera_serial'], optical_frame=manifest['optical_frame'],
                         image_sha256=hashlib.sha256(f'synthetic:{i}'.encode()).hexdigest(),
                         observations=observations))
    return manifest, sorted(rows, key=lambda r: r['t_s'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=False)
    manifest, rows = fixture()
    (args.directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    for row in rows:
        append_record(args.directory/'records.jsonl', manifest, row)
    print('SYNTHETIC ONLY:', args.directory)


if __name__ == '__main__':
    main()
