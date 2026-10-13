"""Persist a measured home and plan bounded, rest-to-rest returns to it."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from workshop.choreography import JOINTS, DISPLACEMENT, smooth_state

DEFAULT_HOME = Path(__file__).resolve().parents[1] / 'trajectories/rig01-home.json'
SPEED = dict(zip(JOINTS, (8, 6, 6, 8, 12, 15)))
ACCELERATION = {k: 2*v for k, v in SPEED.items()}
JERK = {k: 8*v for k, v in SPEED.items()}


def load_home(path, arm_id, calibration_sha256):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != 1 or data.get('kind') != 'measured-home':
        raise ValueError('Unsupported home file')
    if data.get('arm_id') != arm_id:
        raise ValueError('Home arm differs')
    if data.get('calibration_sha256') != calibration_sha256:
        raise ValueError('Home calibration differs')
    position = data.get('position', {})
    if data.get('units') != 'body degrees; gripper percent':
        raise ValueError('Home units differ')
    if set(position) != set(JOINTS) or not all(math.isfinite(v) for v in position.values()):
        raise ValueError('Home requires finite positions for all six joints')
    if not 0 <= position['gripper'] <= 100:
        raise ValueError('Home gripper outside calibrated range')
    return dict(position)


def make_home_move(current, home, bounds, hz):
    if not 100 <= hz <= 200:
        raise ValueError('Home control rate must be 100..200 Hz')
    for pose in (current, home):
        if set(pose) != set(JOINTS) or not all(math.isfinite(v) for v in pose.values()):
            raise ValueError('Home move requires finite six-joint positions')
        if any(not bounds[k][0] <= pose[k] <= bounds[k][1] for k in JOINTS):
            raise ValueError('Home move exceeds calibrated bounds')
    if any(abs(current[k]-home[k]) > DISPLACEMENT[k] for k in JOINTS):
        raise ValueError('Current pose outside the bounded home return envelope; no motion started')
    duration = 2.
    for k in JOINTS:
        distance = abs(home[k]-current[k])
        duration = max(duration, 2.1875*distance/SPEED[k],
                       math.sqrt(7.514*distance/ACCELERATION[k]),
                       (54*distance/JERK[k])**(1/3))
    return dict(schema=1, interpolation='septic-rest-to-rest-home', start=dict(current),
                target=dict(home), duration=duration*1.02, hz=hz, speed_limits=dict(SPEED),
                acceleration_limits=dict(ACCELERATION), jerk_limits=dict(JERK))


def home_at(move, elapsed):
    zero = dict.fromkeys(JOINTS, 0.)
    if elapsed >= move['duration']:
        return dict(move['target']), 'safe home'
    return smooth_state(move['start'], move['target'], zero, zero, move['duration'], elapsed/move['duration'])[0], 'safe home'


def validate_home_start(start, home):
    if set(start) != set(JOINTS) or max(abs(start[k]-home[k]) for k in JOINTS) > .1:
        raise ValueError('Replay start differs from the saved home; recapture/replan from this home')


def restart_at(move, plan, elapsed):
    if elapsed < move['duration']:
        return home_at(move, elapsed)
    from workshop.choreography import plan_at
    return plan_at(plan, elapsed-move['duration'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', action='store_true', required=True, help='Save the current pose; never enable torque')
    parser.add_argument('--port', required=True)
    parser.add_argument('--id', default='hs_rig01_follower')
    parser.add_argument('--calibration-dir', required=True, type=Path)
    parser.add_argument('--home', type=Path, default=DEFAULT_HOME)
    args = parser.parse_args()
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from workshop.teleop_capture import normalized_position
    bus = SOFollower(SOFollowerRobotConfig(port=args.port, id=args.id, calibration_dir=args.calibration_dir, use_degrees=True)).bus
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError('Hardware calibration mismatch')
        torque = {k: bus.read('Torque_Enable', k, num_retry=2) for k in JOINTS}
        if any(torque.values()):
            raise RuntimeError('Home capture requires torque off')
        calibration_path = args.calibration_dir / f'{args.id}.json'
        calibration = json.loads(calibration_path.read_text())
        bounds = {k: (normalized_position(calibration[k]['range_min'], calibration[k], k == 'gripper'),
                      normalized_position(calibration[k]['range_max'], calibration[k], k == 'gripper')) for k in JOINTS}
        position = bus.sync_read('Present_Position', num_retry=2)
        make_home_move(position, position, bounds, 125)
        home = dict(schema=1, kind='measured-home', arm_id=args.id, port=args.port,
                    units='body degrees; gripper percent', position=position,
                    calibration_sha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
                    captured_utc=datetime.now(timezone.utc).isoformat(), torque_at_capture=torque,
                    models={k: bus.ping(k, num_retry=2, raise_on_error=True) for k in JOINTS})
        args.home.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.home.with_suffix('.tmp')
        temporary.write_text(json.dumps(home, indent=2)+'\n')
        temporary.replace(args.home)
        print(f'Saved measured home to {args.home}; torque remains off')
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


if __name__ == '__main__':
    main()
