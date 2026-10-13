"""Station schema and commissioning admission checks (no device I/O)."""
import hashlib
import json
import math
from pathlib import Path

COLORS = ('blue', 'green', 'yellow', 'red')
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
WORKFLOWS = ('arm_loads_sorter', 'arm_unloads_sorter')


def _number(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f'{name} outside permitted range')
    return value


def _positive(value, name):
    if _number(value, name) <= 0:
        raise ValueError(f'{name} must be positive')


def _text(value, name, required=False):
    if not isinstance(value, str) or required and not value.strip():
        raise ValueError(f'{name} must be a string' + (' with a value' if required else ''))


def validate_config(config, physical=False):
    """Validate structural data always; require measured data for physical admission."""
    try:
        if config['workflow'] not in WORKFLOWS:
            raise ValueError('unknown workflow')
        for flag in ('enabled', 'commissioned', 'automatic_start'):
            if type(config[flag]) is not bool:
                raise ValueError(f'{flag} must be boolean')
        if physical and (not config['enabled'] or not config['commissioned']):
            raise ValueError('station needs enabled commissioned configuration')
        if type(config['cycle_count']) is not int or config['cycle_count'] <= 0:
            raise ValueError('cycle_count must be a positive integer')
        for name in ('operation_s', 'stop_s'):
            _positive(config['timeouts'][name], name)
        ev3, arm = config['ev3'], config['arm']
        if ev3['firmware'] not in ('original', 'pybricks'):
            raise ValueError('unknown EV3 firmware')
        ports = ev3['ports']
        if set(ports) != {'feeder', 'sorter', 'touch', 'color'}:
            raise ValueError('EV3 requires feeder, sorter, touch and color ports')
        if any(ports[p] not in ('A', 'B', 'C', 'D') for p in ('feeder', 'sorter')) or any(ports[p] not in ('S1', 'S2', 'S3', 'S4') for p in ('touch', 'color')):
            raise ValueError('invalid EV3 ports')
        if len(set(ports.values())) != 4:
            raise ValueError('duplicate EV3 ports')
        for name in ('feeder', 'sorter'):
            if type(ev3['directions'][name]) is not int or ev3['directions'][name] not in (-1, 1):
                raise ValueError('motor directions must be -1 or 1')
        if set(ev3['destinations']) != set(COLORS):
            raise ValueError('destinations must cover exactly four supported colors')
        for name, value in ev3['destinations'].items():
            _number(value, name, 0, 720)
        for name in ('speed_dps', 'max_travel_deg', 'timeout_s', 'backoff_deg'):
            _positive(ev3['homing'][name], 'homing.' + name)
        home_direction = ev3['feeder'].get('home_direction', 1)
        if type(home_direction) is not int or home_direction not in (-1, 1):
            raise ValueError('feeder.home_direction must be -1 or 1')
        for name in ('speed_dps', 'travel_deg', 'home_speed_dps', 'home_timeout_s', 'home_backoff_deg', 'stall_window_s', 'stall_tolerance_deg'):
            _positive(ev3['feeder'][name], 'feeder.' + name)
        _number(ev3['feeder']['home_travel_deg'], 'feeder.home_travel_deg', 0)
        if physical:
            _positive(ev3['feeder']['home_travel_deg'], 'feeder.home_travel_deg')
        _positive(ev3['tolerance_deg'], 'EV3 tolerance')
        for device in (ev3, arm):
            usb = device['usb']
            for name in ('vendor_id', 'product_id'):
                if type(usb[name]) is not int or not 0 <= usb[name] <= 65535:
                    raise ValueError('USB IDs must be 16 bit integers')
            _text(usb['serial'], 'USB serial', physical and (device is arm or ev3['firmware'] == 'original'))
        _text(arm['id'], 'arm.id', True)
        _text(arm['home_file'], 'arm.home_file', physical)
        _positive(arm['hz'], 'arm.hz')
        _positive(arm['tolerance'], 'arm tolerance')
        calibration = arm['calibration']
        _text(calibration['path'], 'calibration.path', physical)
        _text(calibration['sha256'], 'calibration.sha256', physical)
        digest = calibration['sha256']
        if digest and (len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest)):
            raise ValueError('calibration.sha256 must be a lowercase SHA256')
        if set(arm['limits']) != set(JOINTS):
            raise ValueError('arm limits must cover six joints')
        for joint, bounds in arm['limits'].items():
            low = _number(bounds['min'], joint + '.min')
            high = _number(bounds['max'], joint + '.max')
            if low >= high or joint == 'gripper' and not 0 <= low < high <= 100:
                raise ValueError('invalid joint limits')
            _positive(bounds['speed'], joint + '.speed')
            _positive(bounds['acceleration'], joint + '.acceleration')
            if physical:
                _number(bounds['speed'], joint + '.speed', maximum=25)
                _number(bounds['acceleration'], joint + '.acceleration', maximum=50)
                if joint != 'gripper':
                    _number(low, joint + '.min', -180, 180)
                    _number(high, joint + '.max', -180, 180)
        poses = arm['poses']
        required = {'home', 'clearance', 'loading', 'pickup', 'source', *COLORS}
        if not isinstance(poses, dict) or physical and not required <= set(poses):
            raise ValueError('missing taught poses')
        for name, pose in poses.items():
            if set(pose) != set(JOINTS):
                raise ValueError(f'pose {name} must cover six joints')
            for joint, value in pose.items():
                _number(value, name + '.' + joint, arm['limits'][joint]['min'], arm['limits'][joint]['max'])
        paths = arm['trajectories']
        if not isinstance(paths, dict):
            raise ValueError('trajectories must be an object')
        if physical and not {'home', 'clearance', 'load_piece', 'pick_and_place'} <= set(paths):
            raise ValueError('missing commissioned trajectories')
        for name, path in paths.items():
            candidates = path.values() if name == 'pick_and_place' and isinstance(path, dict) else [path]
            if name == 'pick_and_place' and (not isinstance(path, dict) or not set(path) <= set(COLORS) or physical and set(path) != set(COLORS)):
                raise ValueError('pick_and_place paths require supported colors; physical admission requires all four')
            for sequence in candidates:
                if not isinstance(sequence, list) or not sequence or any(not isinstance(p, str) or p not in poses for p in sequence):
                    raise ValueError('trajectory must reference nonempty taught pose sequence')
        if physical:
            for operation, destination in (('home', 'home'), ('clearance', 'clearance'), ('load_piece', 'loading')):
                if paths[operation][-1] != destination:
                    raise ValueError(f'{operation} trajectory must end at {destination}')
            if 'source' not in paths['load_piece']:
                raise ValueError('load_piece trajectory must visit source before loading')
            for color, sequence in paths['pick_and_place'].items():
                if 'pickup' not in sequence or color not in sequence or sequence.index('pickup') >= sequence.index(color):
                    raise ValueError('pick_and_place must visit pickup before color destination')
        pb = ev3['pybricks']
        for name in ('host', 'user', 'identity_file', 'remote_dir'):
            _text(pb[name], 'pybricks.' + name, physical and ev3['firmware'] == 'pybricks' and name != 'identity_file')
        machine_id = pb.get('expected_machine_id', '')
        _text(machine_id, 'pybricks.expected_machine_id', physical and ev3['firmware'] == 'pybricks')
        if machine_id and (len(machine_id) != 32 or any(c not in '0123456789abcdef' for c in machine_id)):
            raise ValueError('pybricks.expected_machine_id must be lowercase 32 hex')
        if type(pb['port']) is not int or not 1 <= pb['port'] <= 65535:
            raise ValueError('invalid Pybricks port')
        if physical:
            if not config['enabled'] or not config['commissioned']:
                raise ValueError('station needs enabled commissioned configuration')
            if not Path(calibration['path']).is_file() or hashlib.sha256(Path(calibration['path']).read_bytes()).hexdigest() != digest:
                raise ValueError('calibration file missing or hash mismatch')
            if not Path(arm['home_file']).is_file():
                raise ValueError('measured home file missing')
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f'incomplete or malformed station configuration: {error}') from error
    return config


def load_config(path, physical=False):
    """Load JSON, resolving calibration/home paths relative to its configuration."""
    path = Path(path)
    config = json.loads(path.read_text())
    for parent, field in ((config.get('arm', {}).get('calibration', {}), 'path'), (config.get('arm', {}), 'home_file')):
        value = parent.get(field)
        if isinstance(value, str) and value and not Path(value).is_absolute():
            parent[field] = str(path.parent / value)
    return validate_config(config, physical)
