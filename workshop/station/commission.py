"""Capture a torque-disabled taught arm pose during explicit initial setup.

Capture establishes encoder/identity provenance only. Physical geometry, safe
routes, load transfer and grasp/release success require separate commissioning.
This command never calibrates hardware or marks a station commissioned.
"""
import argparse
from datetime import datetime, timezone

from .arm import LeRobotArm, UNITS, atomic_json
from .config import JOINTS, load_config


def capture_pose(arm, path, name):
    """Explicitly disable torque, validate raw position and atomically save it."""
    if not isinstance(name, str) or not name.strip(): raise ValueError('pose name required')
    try:
        arm.connect()
        arm.stop()
        arm._calibration(); arm._bounds()
        if not arm.bus.is_calibrated: raise ValueError('hardware calibration mismatch')
        torque = {k: arm.bus.read('Torque_Enable', k, num_retry=0) for k in JOINTS}
        if any(torque.values()): raise RuntimeError('capture requires verified torque off')
        raw, position = arm._raw_pose()
        models = {k: arm.bus.ping(k, num_retry=0, raise_on_error=True) for k in JOINTS}
        if any(v != 777 for v in models.values()): raise ValueError('expected six STS3215 motors')
        data = dict(schema=1, kind='measured-home' if name == 'home' else 'measured-station-pose',
                    name=name, arm_id=arm.settings['id'], usb_identity=dict(arm.settings['usb']),
                    port=arm.port, calibration_sha256=arm.settings['calibration']['sha256'],
                    units=UNITS, position=position, raw_position=raw, models=models,
                    torque_at_capture=torque, captured_utc=datetime.now(timezone.utc).isoformat(),
                    geometry_commissioned=False, grasp_commissioned=False)
        # Require a second off readback after capture, before publishing evidence.
        if any(arm.bus.read('Torque_Enable', k, num_retry=0) != 0 for k in JOINTS):
            raise RuntimeError('torque changed during pose capture')
        atomic_json(path, data, backup=True)
    except BaseException as error:
        arm._cleanup(error, close=True)
        raise
    else:
        arm.close()
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--capture-pose', required=True, metavar='NAME')
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    capture_pose(LeRobotArm(load_config(args.config)), args.output, args.capture_pose)
    print('Saved torque-disabled pose; geometry and grasp commissioning remain required')


if __name__ == '__main__': main()
