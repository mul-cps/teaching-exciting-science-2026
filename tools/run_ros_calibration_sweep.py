#!/usr/bin/env python3
"""Record a small MoveIt-checked calibration sweep using measured settled poses."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def sweep_poses(amplitude_deg=7.0):
    if not math.isfinite(amplitude_deg) or not 4 <= amplitude_deg <= 8:
        raise ValueError('calibration amplitude must be within 4..8 degrees')
    angle = math.radians(amplitude_deg)
    return [(joint + '-plus', {joint: angle}) for joint in JOINTS] + [
        ('holdout-combination', {'shoulder_lift': angle * .6,
                                 'elbow_flex': -angle * .6,
                                 'wrist_flex': angle * .6}),
        ('return-reference', {})]


def offsets_to_reference(reference, measured, relative):
    result = {joint: float(reference[joint]) + relative.get(joint, 0.)
              - math.radians(float(measured[joint])) for joint in JOINTS}
    if not all(math.isfinite(v) and abs(v) <= .25 for v in result.values()):
        raise ValueError('sweep move exceeds the bounded 0.25 rad displacement')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--state-file', type=Path, default=Path('/run/user/1000/exciting-science/arm-state.json'))
    parser.add_argument('--station-config', required=True)
    parser.add_argument('--amplitude-deg', type=float, default=7.)
    parser.add_argument('--execute', action='store_true', help='execute the bounded physical calibration sweep')
    args = parser.parse_args(argv)
    poses = sweep_poses(args.amplitude_deg)
    if not args.execute:
        print(json.dumps({'execution_requested': False, 'poses_relative_rad': poses}, indent=2))
        return 0
    root = Path(__file__).resolve().parents[1]
    output = args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=False)
    records = []

    def run(command, log_name, timeout):
        with (output / log_name).open('w') as log:
            subprocess.run([sys.executable, *map(str, command)], cwd=root,
                           stdout=log, stderr=subprocess.STDOUT, timeout=timeout, check=True)

    def capture(name):
        run([root / 'tools/capture_ros_robot_registration.py', '--output-directory',
             output / name, '--timeout', '20', '--station-config', args.station_config],
            name + '-capture.log', 30)
        return json.loads((output / name / 'capture.json').read_text())

    def measured():
        state = json.loads(args.state_file.read_text())
        age = time.monotonic() - float(state['monotonic'])
        if not 0 <= age < .5 or state.get('violations'):
            raise RuntimeError('fresh, in-range measured follower state is required')
        return state['pose']

    measured()
    baseline = capture('baseline')
    reference = baseline['joint_positions']
    manifest = {'schema': 1, 'status': 'running', 'reference_bundle': 'baseline',
                'joint_units': 'radians', 'records': records,
                'endpoint_policy': 'record measured settled endpoint; report unmet target separately'}
    try:
        for name, relative in poses:
            offsets = offsets_to_reference(reference, measured(), relative)
            print('MOVE', name, flush=True)
            plan = output / ('move-' + name)
            run([root / 'tools/ros_plan_current.py', '--output-directory', plan,
                 '--joint-offset', json.dumps(offsets), '--velocity-scale', '.4',
                 '--acceleration-scale', '.4', '--timeout', '60', '--calibration', '--execute'],
                name + '-motion.log', 80)
            motion = json.loads((plan / 'metadata.json').read_text())
            if motion['status'] not in ('executed_successfully', 'calibration_measured_endpoint_held'):
                raise RuntimeError('motion did not leave an admitted calibration endpoint')
            snapshot = capture(name)
            records.append({'label': name, 'target_offsets_rad': relative,
                            'motion_status': motion['status'], 'action_result': motion['action_result'],
                            'joint_positions': snapshot['joint_positions'],
                            'role': 'holdout' if name == 'holdout-combination' else
                                    ('repeat' if name == 'return-reference' else 'training')})
            (output / 'sweep.json').write_text(json.dumps(manifest, indent=2))
            print('CAPTURED', name, motion['status'], flush=True)
        manifest['status'] = 'completed_returned_to_reference_request'
    except Exception as error:
        manifest.update(status='stopped_at_last_measured_pose', error=str(error))
        raise
    finally:
        (output / 'sweep.json').write_text(json.dumps(manifest, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
