"""Capture a torque-disabled taught arm pose during explicit initial setup.

Capture establishes encoder/identity provenance only. Physical geometry, safe
routes, load transfer and grasp/release success require separate commissioning.
This command never calibrates hardware or marks a station commissioned.
"""
import argparse
import copy
from pathlib import Path
from datetime import datetime, timezone

from .arm import LeRobotArm, UNITS, atomic_json
from .config import JOINTS, COLORS, load_config, validate_config
from .locking import station_lock, station_lock_path


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


def update_config(arm, path, captured=None, capture_path=None, route=None, color=None):
    """Export validated taught poses/routes without device access or admission.

    Existing commissioned admission is invalidated by any teaching change.
    Absolute calibration/home paths preserve provenance when exporting elsewhere.
    """
    arm._calibration(); arm._bounds()
    config = copy.deepcopy(arm.config)
    settings = config['arm']
    settings['calibration']['path'] = str(Path(settings['calibration']['path']).resolve())
    if settings['home_file']:
        settings['home_file'] = str(Path(settings['home_file']).resolve())
    if captured is not None:
        if captured['calibration_sha256'] != settings['calibration']['sha256'] or captured['arm_id'] != settings['id'] or captured['units'] != UNITS:
            raise ValueError('captured pose identity/calibration/units differ')
        settings['poses'][captured['name']] = arm._pose(captured['position'])
        if captured['name'] == 'home':
            if capture_path is None: raise ValueError('home capture path required')
            settings['home_file'] = str(Path(capture_path).resolve())
    if route is not None:
        name, *sequence = route
        if name not in ('home', 'clearance', 'load_piece', 'pick_and_place') or not sequence:
            raise ValueError('route requires known operation and taught pose names')
        if any(p not in settings['poses'] for p in sequence): raise ValueError('route references missing taught pose')
        for p in sequence: arm._pose(settings['poses'][p])
        for first, second in zip(sequence, sequence[1:]):
            arm._segment(settings['poses'][first], settings['poses'][second])
        if name == 'pick_and_place':
            if color not in COLORS or 'pickup' not in sequence or color not in sequence or sequence.index('pickup') >= sequence.index(color):
                raise ValueError('color route must visit pickup before its color drop')
            settings['trajectories'].setdefault(name, {})[color] = sequence
        else:
            if color is not None: raise ValueError('--color requires pick_and_place')
            endpoint = dict(home='home', clearance='clearance', load_piece='loading')[name]
            if sequence[-1] != endpoint: raise ValueError('route has incorrect endpoint')
            if name == 'load_piece' and 'source' not in sequence: raise ValueError('load_piece must visit source')
            settings['trajectories'][name] = sequence
    config['commissioned'] = False
    config['automatic_start'] = False
    validate_config(config)
    atomic_json(path, config, backup=True)
    arm.config = config; arm.settings = settings
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--lock', type=Path, default=station_lock_path())
    parser.add_argument('--capture-pose', metavar='NAME')
    parser.add_argument('--output', help='measured pose file (required for capture)')
    parser.add_argument('--update-config', metavar='PATH', help='atomically export taught poses/routes; invalidate commissioned admission')
    parser.add_argument('--set-route', nargs='+', metavar='NAME_OR_POSE', help='operation followed by calibrated taught pose names; no device access')
    parser.add_argument('--color', choices=COLORS, help='pick_and_place route color')
    args = parser.parse_args(argv)
    if not args.capture_pose and not args.set_route: parser.error('select --capture-pose or --set-route')
    if args.capture_pose and not args.output: parser.error('--capture-pose requires --output')
    if args.set_route and not args.update_config: parser.error('--set-route requires --update-config')
    if args.color and (not args.set_route or args.set_route[0] != 'pick_and_place'): parser.error('--color requires pick_and_place route')
    # Hold ownership through capture cleanup and configuration export. Route-only
    # teaching also serializes against activation's configuration admission.
    with station_lock(args.lock):
        arm = LeRobotArm(load_config(args.config))
        captured = capture_pose(arm, args.output, args.capture_pose) if args.capture_pose else None
        if args.update_config:
            update_config(arm, args.update_config, captured, args.output, args.set_route, args.color)
    print('Saved taught data; geometry and grasp commissioning remain required')


if __name__ == '__main__': main()
