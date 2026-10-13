"""Calibrated SO follower station adapter; no constructor or preflight actuation.

Public API: connect/preflight/home/load_piece/pick_and_place(color)/clear/
wait_clear/stop/close. Motion methods acknowledge encoder arrival. Configuration
is the complete station configuration. Initial calibration and physical pose,
geometry and grasp teaching are separate setup tasks, never blind recalibration.

SDK packet clocks and serial writes are bounded where supported. Serial open,
reconfiguration and native flush can still block: this is not a hard real-time
safety controller and requires the station's independent physical stop path.
"""
from dataclasses import asdict, is_dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from workshop.safe_home import load_home, make_home_move, home_at
from workshop.servo_start import arm_current_pose, startup_caps
from .config import JOINTS, COLORS

UNITS = 'body degrees; gripper percent'


def atomic_json(path, data, backup=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('x') as stream:
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        if backup and path.exists():
            atomic_json(path.with_name(path.name + '.backup-' + uuid.uuid4().hex), json.loads(path.read_text()))
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class LeRobotArm:
    def __init__(self, config, bus=None, abort_event=None, output=None):
        self.config = config
        self.settings = config['arm']
        self.bus = bus
        self.abort_event = abort_event if abort_event is not None else threading.Event()
        self.output = Path(output) if output is not None else Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')) / 'exciting-science/station-arm'
        self.calibration = None
        self.saved = {}
        self.clear_confirmed = False
        self.cleanup_diagnostics = []
        self.port = None

    def _check(self, deadline=None):
        if self.abort_event.is_set(): raise RuntimeError('arm abort requested')
        if deadline is not None and time.monotonic() >= deadline: raise TimeoutError('arm operation deadline exceeded')

    def _calibration(self):
        settings = self.settings
        path = Path(settings['calibration']['path'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != settings['calibration']['sha256']:
            raise ValueError('calibration hash mismatch')
        data = json.loads(path.read_text())
        if set(data) != set(JOINTS): raise ValueError('calibration requires exactly six joint names')
        for i, k in enumerate(JOINTS, 1):
            c = data[k]
            if any(type(c.get(f)) is not int for f in ('id', 'drive_mode', 'homing_offset', 'range_min', 'range_max')):
                raise ValueError('calibration requires integer entries')
            if c['id'] != i or c['drive_mode'] != 0 or not 0 <= c['range_min'] < c['range_max'] <= 4095:
                raise ValueError('invalid calibration ID, drive_mode or raw limits')
        self.calibration = data
        return data

    def connect(self):
        """Select exact persistent USB identity; bus-only connection, torque off."""
        self._calibration()
        if self.bus is None:
            from .discovery import discover_arm
            identity = self.settings['usb']
            if not identity['serial']: raise ValueError('exact arm USB serial required')
            matches = [d for d in discover_arm() if all(d[k] == identity[k] for k in ('vendor_id', 'product_id', 'serial'))]
            if len(matches) != 1 or len(matches[0]['serial_devices']) != 1:
                raise ValueError('arm persistent USB identity absent or ambiguous')
            self.port = matches[0]['serial_devices'][0]['path']
            path = Path(self.settings['calibration']['path'])
            if path.name != self.settings['id'] + '.json': raise ValueError('calibration filename must match arm ID')
            from lerobot.robots.so_follower.so_follower import SOFollower
            from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
            self.bus = SOFollower(SOFollowerRobotConfig(port=self.port, id=self.settings['id'], calibration_dir=path.parent, use_degrees=True)).bus
        if not self.bus.is_connected:
            try:
                self.bus.connect(handshake=False)
                self._timeouts()
                self.stop()
                # The normal connect handshake is deliberately replaced with
                # explicit six-model and firmware readback after timeout setup.
                for k in JOINTS:
                    if self.bus.ping(k, num_retry=0, raise_on_error=True) != 777:
                        raise ValueError('expected STS3215 model 777')
                if hasattr(self.bus, '_assert_same_firmware'):
                    self.bus._assert_same_firmware()
            except BaseException as error:
                self._cleanup(error, close=True)
                raise
        return True

    def _timeouts(self):
        handler = getattr(self.bus, 'port_handler', None)
        if handler is None: return
        serial = getattr(handler, 'ser', None)
        if serial is not None: serial.write_timeout = .05
        handler.getCurrentTime = lambda: time.monotonic() * 1000
        def set_timeout(milliseconds):
            handler.packet_start_time = handler.getCurrentTime()
            handler.packet_timeout = min(50., max(1., milliseconds))
        handler.setPacketTimeout = lambda length: set_timeout(50.)
        handler.setPacketTimeoutMillis = set_timeout

    def _bounds(self):
        bounds = {}
        if not 100 <= self.settings['hz'] <= 200: raise ValueError('arm rate must be 100..200 Hz')
        if set(self.settings['limits']) != set(JOINTS): raise ValueError('six joint limits required')
        for k in JOINTS:
            c, limit = self.calibration[k], self.settings['limits'][k]
            physical = (0., 100.) if k == 'gripper' else tuple((v-(c['range_min']+c['range_max'])/2)*360/4095 for v in (c['range_min'], c['range_max']))
            if any(isinstance(limit[f], bool) or not math.isfinite(limit[f]) for f in ('min', 'max', 'speed', 'acceleration')):
                raise ValueError('finite joint limits required')
            if not 0 < limit['speed'] <= 25 or not 0 < limit['acceleration'] <= 50:
                raise ValueError('unsafe speed/acceleration limit')
            bounds[k] = max(physical[0], limit['min']), min(physical[1], limit['max'])
            if bounds[k][0] >= bounds[k][1]: raise ValueError('empty calibrated/configured intersection')
        self.bounds = bounds

    def _pose(self, pose):
        if set(pose) != set(JOINTS): raise ValueError('pose requires six joints')
        if any(isinstance(pose[k], bool) or not math.isfinite(pose[k]) or not self.bounds[k][0] <= pose[k] <= self.bounds[k][1] for k in JOINTS):
            raise ValueError('pose outside calibrated/configured limits')
        return dict(pose)

    def _raw_pose(self):
        raw = self.bus.sync_read('Present_Position', normalize=False, num_retry=0)
        return self._validate_raw(raw)

    def _validate_raw(self, raw):
        if set(raw) != set(JOINTS): raise ValueError('raw telemetry requires six joints')
        pose = {}
        for k in JOINTS:
            c, value = self.calibration[k], raw[k]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or int(value) != value or not c['range_min'] <= value <= c['range_max']:
                raise ValueError(f'{k} raw position outside calibrated limits')
            pose[k] = (value-c['range_min'])*100/(c['range_max']-c['range_min']) if k == 'gripper' else (value-(c['range_min']+c['range_max'])/2)*360/4095
        return dict(raw), self._pose(pose)

    def _raw_goal(self, pose):
        self._pose(pose)
        result = {}
        for k in JOINTS:
            c = self.calibration[k]
            value = int(pose[k]*(c['range_max']-c['range_min'])/100+c['range_min']) if k == 'gripper' else int(pose[k]*4095/360+(c['range_min']+c['range_max'])/2)
            if not c['range_min'] <= value <= c['range_max']: raise ValueError('raw goal outside calibration')
            result[k] = value
        return result

    def preflight(self):
        """Require calibrated hardware and valid home/poses/routes, never arm."""
        try:
            self._check()
            for value in (self.settings['tolerance'], self.config['timeouts']['operation_s'], self.config['timeouts']['stop_s']):
                if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                    raise ValueError('finite positive tolerance and operation/stop deadlines required')
            self.connect()
            self.stop()
            self._calibration(); self._bounds()
            if not self.bus.is_calibrated: raise ValueError('hardware calibration mismatch')
            cached = getattr(self.bus, 'calibration', None)
            if cached is not None:
                converted = {k: asdict(v) if is_dataclass(v) else v for k, v in cached.items()}
                if converted != self.calibration: raise ValueError('bus calibration differs from validated file')
            for i, k in enumerate(JOINTS, 1):
                if self.bus.motors[k].id != i: raise ValueError('bus joint IDs differ')
            self.home_pose = self._pose(load_home(self.settings['home_file'], self.settings['id'], self.settings['calibration']['sha256']))
            if not {'home', 'clearance', 'loading', 'pickup', 'source', *COLORS} <= set(self.settings['poses']):
                raise ValueError('missing taught station poses')
            for pose in self.settings['poses'].values(): self._pose(pose)
            if self.settings['poses'].get('home') != self.home_pose: raise ValueError('taught home differs from measured home')
            paths = self.settings['trajectories']
            for operation, endpoint in (('home', 'home'), ('clearance', 'clearance'), ('load_piece', 'loading')):
                if not paths.get(operation) or paths[operation][-1] != endpoint: raise ValueError('trajectory endpoint differs')
            if 'source' not in paths['load_piece']: raise ValueError('load route requires source')
            if set(paths.get('pick_and_place', {})) != set(COLORS): raise ValueError('missing color drop route')
            for name in ('home', 'clearance', 'load_piece', 'pick_and_place'):
                for route in paths[name].values() if name == 'pick_and_place' else [paths[name]]:
                    if not route or any(p not in self.settings['poses'] for p in route): raise ValueError('missing taught route pose')
                    for first, second in zip(route, route[1:]): self._segment(self.settings['poses'][first], self.settings['poses'][second])
            return self._raw_pose()[1]
        except BaseException as error:
            self._cleanup(error)
            raise

    def _segment(self, start, target):
        move = make_home_move(start, target, self.bounds, self.settings['hz'])
        duration = move['duration']
        for k in JOINTS:
            distance = abs(target[k]-start[k]); limits = self.settings['limits'][k]
            duration = max(duration, 1.02 * max(2.1875*distance/limits['speed'], math.sqrt(7.514*distance/limits['acceleration']), (54*distance/(8*limits['speed']))**(1/3)))
        move.update(duration=duration, speed_limits={k:self.settings['limits'][k]['speed'] for k in JOINTS}, acceleration_limits={k:self.settings['limits'][k]['acceleration'] for k in JOINTS}, jerk_limits={k:8*self.settings['limits'][k]['speed'] for k in JOINTS})
        return move

    def _run(self, name, color=None):
        deadline = time.monotonic() + self.config['timeouts']['operation_s']
        self.clear_confirmed = False
        trace = []; stem = uuid.uuid4().hex
        try:
            current = self.preflight(); self._check(deadline)
            route = self.settings['trajectories'][name]
            if color is not None:
                route = route[color]
                if 'pickup' not in route or color not in route or route.index('pickup') >= route.index(color):
                    raise ValueError('color route must visit pickup before drop')
            segments = []
            for pose_name in route:
                target = self._pose(self.settings['poses'][pose_name])
                segments.append(self._segment(current, target)); current = target
            if sum(s['duration'] for s in segments) + 1 > deadline-time.monotonic(): raise TimeoutError('planned motion exceeds operation deadline')
            raw, actual = self._raw_pose()
            if max(abs(actual[k]-segments[0]['start'][k]) for k in JOINTS) > self.settings['tolerance']: raise RuntimeError('current pose changed during planning')
            atomic_json(self.output / ('plan-' + stem + '.json'), dict(schema=1, arm_id=self.settings['id'], calibration_sha256=self.settings['calibration']['sha256'], units=UNITS, operation=name, color=color, current_raw=raw, segments=segments))
            self._check(deadline)
            self.saved = {r:{k:self.bus.read(r,k,num_retry=0) for k in JOINTS} for r in ('Acceleration','Goal_Velocity','Goal_Time')}
            velocity = startup_caps(self.bus, {k:self.settings['limits'][k]['speed'] for k in JOINTS}, self.calibration)
            # Feetech register guide: 100 steps/s², 360/4096 degrees/step.
            # Use the slightly larger SDK 360/4095 conversion conservatively.
            # Native register limits do not prove physical acceleration. Reject caps
            # below one unit rather than silently exceeding the configured limit.
            acceleration = {}
            for k in JOINTS:
                angular = self.settings['limits'][k]['acceleration']
                if k == 'gripper': angular *= (self.calibration[k]['range_max']-self.calibration[k]['range_min'])*360/4095/100
                acceleration[k] = min(254, math.floor(angular/(100*360/4095)))
                if acceleration[k] < 1: raise ValueError('acceleration below native representable cap')
            for k in JOINTS:
                phase = self.bus.read('Phase', k, num_retry=0)
                unit = 6*(.0146 if phase & 4 else .732)
                requested = self.settings['limits'][k]['speed']
                if k == 'gripper': requested *= (self.calibration[k]['range_max']-self.calibration[k]['range_min'])*360/4095/100
                if velocity[k]*unit > requested or velocity[k] > 32767:
                    raise ValueError('velocity below native representable cap')
            atomic_json(self.output / ('plan-' + stem + '.json'), dict(schema=1, arm_id=self.settings['id'], calibration_sha256=self.settings['calibration']['sha256'], units=UNITS, operation=name, color=color, current_raw=raw, segments=segments, native_acceleration=acceleration, native_velocity=velocity, native_goal_time=0))
            for k in JOINTS:
                self.bus.write('Goal_Time', k, 0, normalize=False, num_retry=0)
                if self.bus.read('Goal_Time', k, num_retry=0) != 0: raise RuntimeError('Goal_Time readback failed')
            self._check(deadline)
            self._raw_pose()  # raw checks immediately before atomic enable
            adapter = self
            class CheckedStartupBus:
                def __getattr__(self, name): return getattr(adapter.bus, name)
                def read(self, register, joint, **kwargs):
                    adapter._check(deadline)
                    value = adapter.bus.read(register, joint, **kwargs)
                    adapter._check(deadline)
                    return value
                def sync_read(self, register, **kwargs):
                    values = adapter.bus.sync_read(register, **kwargs)
                    if register == 'Present_Position': adapter._validate_raw(values)
                    adapter._check(deadline)
                    return values
            armed = arm_current_pose(CheckedStartupBus(), acceleration, velocity)
            if any(abs(armed[k]-raw[k]) > 1 for k in JOINTS): raise RuntimeError('position changed before atomic arming')
            for move in segments:
                self._execute(move, deadline, trace)
            self._check(deadline)
            return True
        except BaseException as error:
            self._cleanup(error)
            raise
        finally:
            if trace:
                try: atomic_json(self.output / ('trace-' + stem + '.json'), trace)
                except BaseException as error: self.cleanup_diagnostics.append('trace save: ' + str(error))

    def _execute(self, move, deadline, trace):
        start = time.monotonic(); tick = 0; previous = start
        while True:
            scheduled = start + tick / move['hz']
            self._check(deadline)
            self.abort_event.wait(max(0., scheduled-time.monotonic()))
            self._check(deadline)
            now = time.monotonic()
            if now-scheduled > .05 or now-previous > .05: raise RuntimeError('arm control overslip')
            goal, _ = home_at(move, now-start)
            self._raw_pose()
            self._check(deadline)
            self.bus.sync_write('Goal_Position', self._raw_goal(goal), normalize=False, num_retry=0)
            raw, actual = self._raw_pose(); self._check(deadline)
            if time.monotonic()-now > .05: raise RuntimeError('arm transaction overslip')
            if max(abs(actual[k]-goal[k]) for k in JOINTS) > self.settings['tolerance']: raise RuntimeError('arm tracking error')
            trace.append(dict(t=time.monotonic(), goal=goal, raw=raw, actual=actual))
            if now-start >= move['duration']:
                if max(abs(actual[k]-move['target'][k]) for k in JOINTS) > self.settings['tolerance']: raise RuntimeError('arm arrival failed')
                return
            tick += 1; previous = now

    def home(self): return self._run('home')
    def load_piece(self): return self._run('load_piece')
    def pick_and_place(self, color):
        if color not in COLORS: raise ValueError('unknown color')
        return self._run('pick_and_place', color)
    def clear(self): return self._run('clearance')
    def wait_clear(self):
        try:
            self._check()
            _, actual = self._raw_pose()
            if max(abs(actual[k]-self.settings['poses']['clearance'][k]) for k in JOINTS) > self.settings['tolerance']:
                raise RuntimeError('arm clearance not reached')
            self.clear_confirmed = True
            return True
        except BaseException as error:
            self._cleanup(error); raise

    def stop(self):
        """Attempt every joint, independently verify off, then restore RAM only."""
        self.clear_confirmed = False
        if self.bus is None or not self.bus.is_connected: return True
        deadline = time.monotonic() + self.config['timeouts']['stop_s']
        errors = []
        for k in JOINTS:
            try: self.bus.write('Torque_Enable', k, 0, normalize=False, num_retry=0)
            except BaseException as error: errors.append(f'{k} disable: {error}')
        verified = True
        for k in JOINTS:
            try:
                if self.bus.read('Torque_Enable', k, num_retry=0) != 0:
                    verified = False; errors.append(k + ' torque still enabled')
            except BaseException as error:
                verified = False; errors.append(f'{k} torque read: {error}')
        if verified:
            for register, values in self.saved.items():
                for k, value in values.items():
                    try:
                        if self.bus.read(register,k,num_retry=0) != value:
                            self.bus.write(register,k,value,normalize=False,num_retry=0)
                            if self.bus.read(register,k,num_retry=0) != value: raise RuntimeError('restore readback differs')
                    except BaseException as error: errors.append(f'{k} {register} restore: {error}')
            if not errors: self.saved = {}
        for k in JOINTS:
            try:
                if self.bus.read('Torque_Enable',k,num_retry=0) != 0: errors.append(k + ' final torque-off failed')
            except BaseException as error: errors.append(f'{k} final torque read: {error}')
        if time.monotonic() > deadline: errors.append('torque-off cleanup exceeded stop deadline')
        self.cleanup_diagnostics.extend(errors)
        if errors: raise RuntimeError('arm cleanup incomplete: ' + '; '.join(errors))
        return True

    def _cleanup(self, original, close=False):
        try: self.close() if close else self.stop()
        except BaseException as error:
            original.add_note('arm cleanup: ' + str(error))

    def close(self):
        error = None
        try: self.stop()
        except BaseException as failure: error = failure
        finally:
            if self.bus is not None and self.bus.is_connected:
                try: self.bus.disconnect(disable_torque=False)
                except BaseException as failure:
                    handler = getattr(self.bus, 'port_handler', None)
                    if handler is not None:
                        try: handler.closePort()
                        except BaseException as close_error: failure.add_note(str(close_error))
                    if error is None: error = failure
                    else: error.add_note('disconnect: ' + str(failure))
        if error is not None: raise error
        return True
