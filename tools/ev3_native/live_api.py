#!/usr/bin/env pybricks-micropython
"""Persistent localhost EV3 JSON API. Startup brakes only; references are unknown."""
import sys
try:
    import ujson as json
except ImportError:
    import json

PROTOCOL = 'ev3-live-api'
VERSION = 1
MAX_FRAME = 4096
HEARTBEAT_MS = 1000
POWER_SAVE_IDLE_MS = 60000
POWER_SAVE_POLL_MS = 250
MAX_EMPIRICAL_SPEED_HEADROOM_DPS = 30
MAX_ENCODER_SAMPLE_UNCERTAINTY_MS = 20


def native_runtime(implementation):
    name = getattr(implementation, 'name', None)
    version = getattr(implementation, 'version', None)
    if name is None:
        name = implementation[0]
    if version is None:
        version = implementation[1]
    return name == 'pybricks-micropython' and version[0] == 2


def fault(code, message):
    return {'code': code, 'message': message}


def number(value, low, high, name, integer=False):
    if type(value) not in (int, float) or not low <= value <= high or integer and type(value) is not int:
        raise ValueError(name + ' outside permitted range')
    return value


def arguments(args, allowed):
    if any(key not in allowed for key in args):
        raise ValueError('Unexpected operation argument')


class APIError(Exception):
    def __init__(self, code, message):
        self.code, self.message = code, message


class LiveAPI:
    def __init__(self, motors, touch, color, brick, clock, Stop, session,
                 power_save_idle_ms=POWER_SAVE_IDLE_MS):
        self.motors, self.touch, self.color, self.brick = motors, touch, color, brick
        self.clock, self.Stop, self.session = clock, Stop, session
        if type(power_save_idle_ms) is not int or not 1000 <= power_save_idle_ms <= 3600000:
            raise ValueError('power-save idle timeout must be an integer in 1000..3600000 ms')
        self.power_save_idle_ms = power_save_idle_ms
        self.power_mode = 'awake'
        self.last_activity = clock()
        self.references = {'sorter': None, 'feeder': None}
        self.saved = {}
        self.motor_control = {}
        self.speed_samples = {}
        for name, motor in motors.items():
            motor.brake()
            self.saved[name] = motor.control.limits()
            self.motor_control[name] = self.control_parameters(motor)
            self.speed_samples[name] = {'speed_dps': None, 'started_ms': None,
                                        'sampled_ms': None, 'duration_ms': None}
        self.connected = False
        self.protocol_error = False
        self.last_heartbeat = clock()
        self.last_id = 0
        self.cache, self.cache_order = {}, []
        self.job, self.last_job, self.last_error = None, None, None
        self.angles, self.angle_sample_ms, self.angle_read_ms = {}, {}, {}
        for name, motor in motors.items():
            started = clock()
            self.angles[name] = motor.angle()
            self.angle_sample_ms[name] = clock()
            self.angle_read_ms[name] = max(0, self.angle_sample_ms[name] - started)
        self.last_angles = clock()
        self.last_motion_tick = clock() - 20
        self.display_text = ''
        self.last_render_signature = None
        self.last_render_text = None
        self.last_render = -500
        self.last_render_job = None
        self.last_sensors = -500
        self.sensor_touch, self.sensor_color = False, 'unknown'
        self.beep_left, self.beep_active, self.next_beep = 0, False, 0

    def ready(self):
        return {'protocol': PROTOCOL, 'version': VERSION, 'ready': True, 'session': self.session}

    def connect(self):
        self.last_id = 0
        self.cache, self.cache_order = {}, []
        self.connected = True
        self.protocol_error = False
        self.last_heartbeat = self.clock()

    def refresh_angles(self):
        for name, motor in self.motors.items():
            started = self.clock()
            self.angles[name] = motor.angle()
            self.angle_sample_ms[name] = self.clock()
            self.angle_read_ms[name] = max(0, self.angle_sample_ms[name] - started)
        self.last_angles = self.clock()

    def control_parameters(self, motor):
        """Read native controller settings when available; never set them."""
        control = motor.control
        result = {'limits': None, 'target_tolerances': None, 'stall_tolerances': None}
        for key, method_name in (('limits', 'limits'),
                                 ('target_tolerances', 'target_tolerances'),
                                 ('stall_tolerances', 'stall_tolerances')):
            method = getattr(control, method_name, None)
            if method is None:
                continue
            try:
                value = method()
                result[key] = list(value) if type(value) is tuple else value
            except Exception:
                pass
        return result

    def speed_sample(self, motor):
        """Read the existing motor's estimated speed and bracket its call time."""
        method = getattr(motor, 'speed', None)
        if method is None:
            return {'speed_dps': None, 'started_ms': None, 'sampled_ms': None,
                    'duration_ms': None}
        started = self.clock()
        try:
            speed = method()
        except Exception:
            speed = None
        sampled = self.clock()
        return {'speed_dps': speed, 'started_ms': started, 'sampled_ms': sampled,
                'duration_ms': max(0, sampled - started)}

    def brakes(self):
        errors, stopped = [], []
        for name, motor in self.motors.items():
            try:
                motor.brake()
                stopped.append(name)
            except Exception as error:
                errors.append(str(error))
        for name in stopped:
            try:
                self.motors[name].control.limits(*self.saved[name])
                self.motor_control[name] = self.control_parameters(self.motors[name])
            except Exception as error:
                errors.append(str(error))
        if errors:
            self.last_error = fault('brake_failed', '; '.join(errors))
        return errors

    def disconnect(self, reason='disconnect'):
        self.connected = False
        if self.job:
            self.finish('aborted', fault(reason, 'Active job stopped: ' + reason))
        else:
            self.brakes()

    def public_job(self, job):
        if job is None:
            return None
        return {key: value for key, value in job.items() if not key.startswith('_')}

    def status(self):
        motor_control = {}
        for name, motor in self.motors.items():
            if self.power_mode != 'power_save':
                self.speed_samples[name] = self.speed_sample(motor)
            speed = self.speed_samples[name]
            motor_control[name] = dict(self.motor_control[name])
            motor_control[name]['speed_sample'] = dict(speed)
            motor_control[name]['angle_sample_ms'] = self.angle_sample_ms[name]
            motor_control[name]['angle_read_ms'] = self.angle_read_ms[name]
            if self.job and self.job['motor'] == name:
                self.job['speed_sample'] = speed
        return {'session': self.session, 'connected': self.connected, 'busy': self.job is not None,
                'job': self.public_job(self.job), 'last_job': self.public_job(self.last_job),
                'angles': dict(self.angles), 'sampled_ms': self.last_angles,
                'motor_control': motor_control,
                'references': dict(self.references), 'touch': self.sensor_touch,
                'color': self.sensor_color, 'last_error': self.last_error,
                'power_mode': self.power_mode, 'power_save_idle_ms': self.power_save_idle_ms}

    def poll_timeout_ms(self):
        """The sleeping poll still wakes immediately when TCP data arrives."""
        return POWER_SAVE_POLL_MS if self.power_mode == 'power_save' else 20

    def enter_power_save(self):
        if self.job:
            raise APIError('busy', 'Cannot enter power save while a motor job is active')
        if self.power_mode == 'power_save':
            return
        errors = self.brakes()
        self.beep_left, self.beep_active = 0, False
        try:
            self.brick.speaker.set_volume(0, 'Beep')
        except Exception:
            pass
        light = getattr(self.brick, 'light', None)
        if light is not None:
            try:
                light.off()
            except Exception as error:
                self.last_error = fault('power_save_light_error', str(error))
        self.power_mode = 'power_save'
        # The EV3 LCD is reflective. Keep a clear ready message while avoiding
        # redraws; no backlight control is assumed.
        try:
            screen = self.brick.screen
            screen.clear()
            screen.draw_text(0, 0, 'POWER SAVE')
            screen.draw_text(0, 20, 'TCP API ready')
            self.last_render = self.last_angles
            self.last_render_signature = ('POWER SAVE', 'TCP API ready')
            self.last_render_text = self.display_text
            self.last_render_job = None
        except Exception as error:
            self.last_error = fault('power_save_display_error', str(error))
        if errors:
            self.last_error = fault('brake_failed', '; '.join(errors))

    def wake(self):
        if self.power_mode == 'power_save':
            self.power_mode = 'awake'
            now = self.clock()
            self.last_angles = now - 500
            self.last_sensors = now - 500
            self.last_render = now - 500
        self.last_activity = self.clock()

    def protocol_fault(self, request_id, message):
        self.protocol_error = True
        self.disconnect('protocol_error')
        return {'id': request_id, 'ok': False, 'fault': fault('protocol_error', message)}

    def request(self, request):
        if type(request) is not dict or set(request) != set(('id', 'operation', 'args')):
            return self.protocol_fault(None, 'Request requires id, operation, args')
        request_id = request['id']
        if type(request_id) is not int or request_id <= 0:
            return self.protocol_fault(None, 'ID must be a positive integer')
        if type(request['operation']) is not str or type(request['args']) is not dict:
            return self.protocol_fault(request_id, 'Operation must be a string and args an object')
        if request_id in self.cache:
            previous, reply = self.cache[request_id]
            if previous == request:
                if request['operation'] not in ('heartbeat', 'status', 'power_save'):
                    self.wake()
                return json.loads(json.dumps(reply))
            return self.protocol_fault(request_id, 'Changed payload for cached ID')
        if request_id <= self.last_id:
            return self.protocol_fault(request_id, 'Stale request ID')
        self.last_id = request_id
        try:
            result = self.operation(request_id, request['operation'], request['args'])
            reply = {'id': request_id, 'ok': True, 'result': result}
        except APIError as error:
            reply = {'id': request_id, 'ok': False, 'fault': fault(error.code, error.message)}
        except (ValueError, KeyError) as error:
            reply = {'id': request_id, 'ok': False, 'fault': fault('invalid_request', str(error))}
        except Exception as error:
            failure = fault('device_error', str(error))
            if self.job:
                self.finish('fault', failure)
            else:
                self.brakes()
                self.last_error = failure
            reply = {'id': request_id, 'ok': False, 'fault': failure}
        self.cache[request_id] = (json.loads(json.dumps(request)), json.loads(json.dumps(reply)))
        self.cache_order.append(request_id)
        if len(self.cache_order) > 64:
            del self.cache[self.cache_order.pop(0)]
        return reply

    def require_reference(self, name):
        if name not in self.references or self.references[name] is None:
            raise ValueError('Motor reference is unconfirmed: ' + str(name))

    def operation(self, request_id, operation, args):
        if operation in ('heartbeat', 'status', 'stop'):
            arguments(args, ())
            if operation == 'heartbeat':
                self.last_heartbeat = self.clock()
                return {'alive': True, 'session': self.session}
            if operation == 'status':
                return self.status()
            self.wake()
            if self.job:
                errors = self.finish('aborted', fault('stopped', 'Explicit stop'))
            else:
                errors = self.brakes()
            if errors:
                raise APIError('brake_failed', '; '.join(errors))
            return {'stopped': True, 'status': self.status()}
        if operation == 'power_save':
            arguments(args, ('idle_timeout_ms',))
            if self.job:
                raise APIError('busy', 'Cannot enter power save while a motor job is active')
            if 'idle_timeout_ms' in args:
                self.power_save_idle_ms = number(args['idle_timeout_ms'], 1000, 3600000,
                                                  'idle_timeout_ms', True)
            self.enter_power_save()
            return {'power_mode': self.power_mode, 'idle_timeout_ms': self.power_save_idle_ms}
        if operation == 'wake':
            arguments(args, ())
            self.wake()
            return {'power_mode': self.power_mode}
        # Heartbeats and status checks intentionally do not count as user
        # activity, so the persistent host connection cannot prevent sleep.
        self.wake()
        if operation == 'display':
            arguments(args, ('text',))
            text = args['text']
            if type(text) is not str or len(text) > 80 or any(ord(c) < 32 or ord(c) > 126 for c in text):
                raise ValueError('Display text must be printable ASCII, at most 80 characters')
            self.display_text = text
            return {'displayed': text}
        if operation == 'beep':
            arguments(args, ('count',))
            count = number(args.get('count', 1), 1, 3, 'count', True)
            self.schedule_beeps(count)
            return {'scheduled': count}
        if self.job:
            raise ValueError('A motor job is already busy')
        if operation == 'confirm_reference':
            arguments(args, ('name', 'reference', 'angle'))
            name, reference = args['name'], args['reference']
            expected = {'sorter': 'home', 'feeder': 'eject'}
            if name not in expected or reference != expected[name] or args.get('angle', 0) != 0:
                raise ValueError('Reference must be sorter/home or feeder/eject at angle zero')
            if name == 'sorter' and not self.touch.pressed():
                raise ValueError('Sorter home confirmation requires the pressed touch switch')
            self.motors[name].brake()
            self.motors[name].reset_angle(0)
            self.angles[name] = 0
            self.references[name] = reference
            return {'confirmed': True, 'name': name, 'reference': reference, 'angle': 0}
        if operation in ('motor_target', 'eject', 'home_touch', 'shake'):
            self.refresh_angles()
        if operation == 'motor_target':
            arguments(args, ('name', 'target', 'speed', 'acceleration', 'actuation'))
            name = args['name']
            self.require_reference(name)
            maximum = 550 if name == 'sorter' else 209
            target = number(args['target'], 0, maximum, 'target')
            speed = number(args.get('speed', 600), 1, 600, 'speed')
            acceleration = number(args.get('acceleration', 1200 if name == 'sorter' else 2400), 1, 30000, 'acceleration')
            actuation = number(args.get('actuation', 50), 1, 50, 'actuation')
            self.start_job(request_id, operation, name, [(target, 'target')], speed, acceleration, actuation)
        elif operation == 'eject':
            arguments(args, ('cycles',))
            self.require_reference('feeder')
            cycles = number(args.get('cycles', 1), 1, 5, 'cycles', True)
            targets = []
            for index in range(cycles):
                targets.extend([(209, 'load_' + str(index + 1)), (0, 'eject_' + str(index + 1))])
            self.start_job(request_id, operation, 'feeder', targets, 600, 2400, 50)
            self.job['cycles'] = cycles
        elif operation == 'home_touch':
            arguments(args, ())
            pressed = self.touch.pressed()
            self.start_job(request_id, operation, 'sorter', [(self.angles['sorter'] - 550, 'approach')],
                           120, 400, 30, dispatch=not pressed)
            self.job['_deadline'] = self.clock() + 10000
            self.job['_budget'] = 550
            if pressed:
                self.begin_settle()
        elif operation == 'shake':
            arguments(args, ('frequency_hz', 'speed_dps', 'acceleration', 'duration_ms'))
            self.require_reference('sorter')
            frequency = number(args.get('frequency_hz', 12.5), 1, 15, 'frequency_hz')
            speed = number(args.get('speed_dps', 600), 50, 600, 'speed_dps')
            acceleration = number(args.get('acceleration', 30000), 1000, 30000, 'acceleration')
            duration = number(args.get('duration_ms', 1000), 100, 2000, 'duration_ms', True)
            center = self.angles['sorter']
            if not 0 <= center <= 550:
                raise ValueError('Shake center outside sorter range')
            self.start_job(request_id, operation, 'sorter', [(min(550, center + 20), 'shake')], speed, acceleration, 50)
            self.job.update(center_angle=center, requested_frequency_hz=frequency,
                            peak_displacement_deg=0, measured_frequency_hz=None, encoder_reversals=0,
                            setpoint_reversals=0, duration_ms=duration)
            self.job.update(_low=max(0, center - 20), _high=min(550, center + 20),
                            _half_ms=500.0 / frequency, _next_half=self.clock() + 500.0 / frequency,
                            _end=self.clock() + duration, _positive=True, _velocity_sign=0,
                            _first_reverse=None, _last_reverse=None)
        else:
            raise ValueError('Unknown operation')
        return {'accepted': True, 'job': self.public_job(self.job)}

    def start_job(self, request_id, operation, name, targets, speed, acceleration, actuation, dispatch=True):
        now = self.clock()
        motor = self.motors[name]
        motor.brake()
        motor.control.limits(speed=speed, acceleration=acceleration, actuation=actuation)
        self.motor_control[name] = self.control_parameters(motor)
        self.refresh_angles()
        self.speed_samples[name] = self.speed_sample(motor)
        other = 'feeder' if name == 'sorter' else 'sorter'
        self.job = {'id': request_id, 'operation': operation, 'state': 'running', 'phase': '',
                    'started_ms': now, 'motor': name, '_targets': targets, '_index': 0,
                    '_speed': speed, '_other': other, '_other_start': self.angles[other],
                    'native_control': dict(self.motor_control[name]),
                    'speed_sample': dict(self.speed_samples[name])}
        self.last_error = None
        if dispatch:
            self.target_phase()
        else:
            self.job.update(phase='approach', target=targets[0][0], _stage='settling')
        self.schedule_beeps(1)

    def target_phase(self):
        job = self.job
        target, phase = job['_targets'][job['_index']]
        motor = self.motors[job['motor']]
        motor.brake()
        self.refresh_angles()
        start = self.angles[job['motor']]
        job.update(phase=phase, target=target, _stage='moving', _start=start, _last=start,
                   _direction=1 if target > start else -1, _budget=abs(target - start) + 6,
                   _deadline=self.clock() + 6000)
        # Pair the jump-guard baseline with the encoder sample in ``angles``.
        # Do not timestamp it after unrelated setup work or a blocking command.
        job['_last_sample_ms'] = self.angle_sample_ms[job['motor']]
        job['_last_angle_read_ms'] = self.angle_read_ms[job['motor']]
        # Keep a small trace only for an active movement phase. This makes a
        # fault diagnosable without growing state during idle or settling.
        job['_encoder_history'] = [{'angle_deg': start,
                                    'sample_ms': job['_last_sample_ms'],
                                    'event_ms': self.clock()}]
        motor.run_target(job['_speed'], target, then=self.Stop.BRAKE, wait=False)

    def begin_settle(self):
        job = self.job
        motor = self.motors[job['motor']]
        motor.brake()
        angle = self.angles[job['motor']]
        now = self.clock()
        job.update(_stage='settling', _settle_start=now, _min=angle, _max=angle, _quiet=[(now, angle)])

    def finish(self, state, failure=None):
        job = self.job
        errors = self.brakes()
        if not job:
            return errors
        if errors:
            job['cleanup_fault'] = self.last_error
            if failure is None:
                failure = self.last_error
            state = 'fault'
        job['state'] = state
        job['finished_ms'] = self.clock()
        job['elapsed_ms'] = self.clock() - job['started_ms']
        self.last_activity = job['finished_ms']
        if failure:
            job['fault'] = failure
            if '_encoder_diagnostic' in job:
                job['encoder_diagnostics'] = job['_encoder_diagnostic']
            self.last_error = failure
        self.last_job, self.job = job, None
        if state == 'complete':
            self.schedule_beeps(2)
        return errors

    def schedule_beeps(self, count):
        self.beep_left = count
        if not self.beep_active:
            self.next_beep = self.clock()

    def beep_tick(self, now):
        if now < self.next_beep:
            return
        if self.beep_active:
            self.brick.speaker.set_volume(0, 'Beep')
            self.beep_active = False
            self.next_beep = now + 100
        elif self.beep_left:
            self.brick.speaker.set_volume(100, 'Beep')
            self.brick.speaker.beep(frequency=800, duration=-1)
            self.beep_left -= 1
            self.beep_active = True
            self.next_beep = now + 100

    def job_tick(self, now):
        job = self.job
        if not job:
            return
        motor = self.motors[job['motor']]
        angle = self.angles[job['motor']]
        if abs(self.angles[job['_other']] - job['_other_start']) > 3:
            raise ValueError('Other motor drift exceeds 3 degrees')
        home = job['operation'] == 'home_touch'
        if job['motor'] == 'sorter' and not home:
            if not 0 <= angle <= 550:
                raise ValueError('Sorter soft limit exceeded')
            if self.sensor_touch and angle > 20:
                raise ValueError('Unexpected touch above angle 20')
        if job['_stage'] == 'settling':
            job['_min'], job['_max'] = min(job['_min'], angle), max(job['_max'], angle)
            if job['_max'] - job['_min'] > 6:
                raise ValueError('Settling recoil exceeds 6 degrees')
            quiet = job['_quiet']
            quiet.append((now, angle))
            while len(quiet) > 1 and quiet[1][0] <= now - 200:
                quiet.pop(0)
            stable = now - quiet[0][0] >= 200 and max(x[1] for x in quiet) - min(x[1] for x in quiet) <= 1
            if stable:
                if home:
                    if not self.sensor_touch:
                        raise ValueError('Home switch released before quiet zero')
                    motor.reset_angle(0)
                    self.angles['sorter'] = 0
                    self.references['sorter'] = 'home'
                elif job['operation'] != 'shake' and abs(angle - job['target']) > 3:
                    raise ValueError('Settled target error exceeds 3 degrees')
                if not home and job['operation'] != 'shake' and job['_index'] + 1 < len(job['_targets']):
                    job['_index'] += 1
                    self.target_phase()
                else:
                    self.finish('complete')
            elif now - job['_settle_start'] >= 1000:
                raise ValueError('Motor did not settle within 1000 ms')
            return
        step = angle - job['_last']
        # ``now`` is event-processing time. Sensor and peripheral calls can
        # block after ``angles`` was sampled, so only compare encoder samples
        # using the timestamp captured by refresh_angles().
        sample_ms = self.angle_sample_ms[job['motor']]
        elapsed = max(1, sample_ms - job['_last_sample_ms'])
        if job['operation'] == 'shake':
            jump_bound = 20
            native_speed_limit = job['_speed']
            advertised_speed_tolerance = None
            empirical_speed_headroom = 0
            sample_uncertainty_ms = 0
            guard_speed = job['_speed']
        else:
            limits = job['native_control'].get('limits')
            native_speed_limit = abs(job['_speed'])
            if type(limits) in (tuple, list) and limits and type(limits[0]) in (int, float):
                native_speed_limit = min(native_speed_limit, abs(limits[0]))
            tolerances = job['native_control'].get('target_tolerances')
            empirical_speed_headroom = 0
            advertised_speed_tolerance = None
            if (type(tolerances) in (tuple, list) and tolerances and
                    type(tolerances[0]) in (int, float) and tolerances[0] > 0):
                advertised_speed_tolerance = tolerances[0]
                # Pybricks defines this as allowed speed deviation at maneuver
                # completion, not as an in-motion tracking guarantee. Use it
                # only as empirical headroom, capped at 30 dps; missing or
                # invalid settings add zero headroom.
                empirical_speed_headroom = min(tolerances[0],
                                               MAX_EMPIRICAL_SPEED_HEADROOM_DPS)
            # The sample interval may include the previous angle() call time,
            # because that sample could have occurred at its entry. Bound this
            # measured uncertainty to one normal active polling period.
            sample_uncertainty_ms = min(max(0, job['_last_angle_read_ms']),
                                        MAX_ENCODER_SAMPLE_UNCERTAINTY_MS)
            guard_speed = native_speed_limit + empirical_speed_headroom
            elapsed_upper_ms = elapsed + sample_uncertainty_ms
            jump_bound = min(job['_budget'], max(
                20, guard_speed * elapsed_upper_ms / 1000.0 + 6))
        history = job['_encoder_history']
        history.append({'angle_deg': angle, 'sample_ms': sample_ms, 'event_ms': now})
        if len(history) > 16:
            history.pop(0)
        if abs(step) > jump_bound:
            job['_encoder_diagnostic'] = {
                'step_deg': step,
                'elapsed_ms': elapsed,
                'previous_sample_ms': job['_last_sample_ms'],
                'sample_ms': sample_ms,
                'event_ms': now,
                'job_started_ms': job['started_ms'],
                'job_elapsed_ms': now - job['started_ms'],
                'angle_deg': angle,
                'previous_angle_deg': job['_last'],
                'jump_bound_deg': jump_bound,
                'native_speed_limit_dps': native_speed_limit,
                'advertised_speed_tolerance_dps': advertised_speed_tolerance,
                'empirical_speed_headroom_dps': empirical_speed_headroom,
                'guard_speed_dps': guard_speed,
                'sample_uncertainty_ms': sample_uncertainty_ms,
                'previous_angle_read_ms': job['_last_angle_read_ms'],
                'current_angle_read_ms': self.angle_read_ms[job['motor']],
                'native_control': job['native_control'],
                'speed_sample': job.get('speed_sample'),
                'sample_age_ms': now - sample_ms,
                'history': history,
            }
            raise ValueError('Encoder step exceeds bounded motion allowance')
        if job['operation'] == 'shake':
            displacement = abs(angle - job['center_angle'])
            if displacement > 20:
                raise ValueError('Shake displacement exceeds 20 degrees')
            job['peak_displacement_deg'] = max(job['peak_displacement_deg'], displacement)
            sign = 1 if step > 0 else -1 if step < 0 else 0
            if sign and job['_velocity_sign'] and sign != job['_velocity_sign']:
                job['encoder_reversals'] += 1
                job['_last_reverse'] = now
                if job['_first_reverse'] is None:
                    job['_first_reverse'] = now
                elif now > job['_first_reverse']:
                    job['measured_frequency_hz'] = (job['encoder_reversals'] - 1) * 500.0 / (now - job['_first_reverse'])
            if sign:
                job['_velocity_sign'] = sign
            job['_last'] = angle
            job['_last_sample_ms'] = sample_ms
            job['_last_angle_read_ms'] = self.angle_read_ms[job['motor']]
            if now >= job['_end']:
                self.begin_settle()
                return
            if motor.control.stalled() and not motor.control.done():
                raise ValueError('Shake motor stalled')
            if now >= job['_next_half']:
                job['_positive'] = not job['_positive']
                target = job['_high'] if job['_positive'] else job['_low']
                job['target'] = target
                motor.run_target(job['_speed'], target, then=self.Stop.BRAKE, wait=False)
                job['setpoint_reversals'] += 1
                # No catch-up motion burst after a delayed event-loop tick.
                job['_next_half'] = now + job['_half_ms']
            return
        displacement = job['_direction'] * (angle - job['_start'])
        if displacement < -3 or job['_direction'] * step < -3:
            raise ValueError('Motor moved in wrong direction')
        if displacement > job['_budget']:
            raise ValueError('Encoder travel budget exceeded')
        job['_last'] = angle
        job['_last_sample_ms'] = sample_ms
        job['_last_angle_read_ms'] = self.angle_read_ms[job['motor']]
        if now >= job['_deadline']:
            raise ValueError('Native target deadline expired')
        if home and self.sensor_touch:
            self.begin_settle()
        elif motor.control.done():
            if home:
                raise ValueError('Home approach ended without touch')
            if abs(angle - job['target']) > 3:
                raise ValueError('Native target error exceeds 3 degrees')
            self.begin_settle()
        elif motor.control.stalled():
            raise ValueError('Native motor stalled')

    def render(self, now):
        if now - self.last_render < 500:
            return
        job_id = self.job['id'] if self.job else None
        if (self.job and self.job['operation'] == 'shake' and self.last_render_job == job_id
                and self.display_text == self.last_render_text):
            return
        state = 'CONNECTED' if self.connected else 'IDLE'
        phase = self.job['phase'] if self.job else 'IDLE'
        screen = self.brick.screen
        lines = [state + ' ' + phase, self.display_text[:26],
                 'D ' + str(self.angles['sorter']) + ' A ' + str(self.angles['feeder']),
                 'touch ' + str(self.sensor_touch) + ' ' + self.sensor_color[:12],
                 self.last_error['message'][:26] if self.last_error else '']
        signature = tuple(lines)
        if signature == self.last_render_signature:
            return
        screen.clear()
        for index, text in enumerate(lines):
            screen.draw_text(0, index * 20, text)
        self.last_render, self.last_render_job = now, job_id
        self.last_render_signature, self.last_render_text = signature, self.display_text

    def tick(self):
        now = self.clock()
        if self.job and now - self.last_heartbeat >= HEARTBEAT_MS:
            self.disconnect('heartbeat_expired')
        active = self.job is not None
        if (not active and self.power_mode == 'awake' and
                now - self.last_activity >= self.power_save_idle_ms):
            try:
                self.enter_power_save()
            except Exception as error:
                self.last_error = fault('power_save_error', str(error))
            return
        if self.power_mode == 'power_save':
            return
        due = now - self.last_motion_tick >= 20
        sensors_due = now - self.last_sensors >= 500
        try:
            if now - self.last_angles >= (20 if active else 500):
                self.refresh_angles()
            if (active and due and self.job['motor'] == 'sorter') or sensors_due:
                self.sensor_touch = bool(self.touch.pressed())
            if active and due:
                now = self.clock()
                self.last_motion_tick = now
                self.job_tick(now)
        except Exception as error:
            if self.job:
                self.finish('fault', fault('motion_fault', str(error)))
            else:
                self.last_error = fault('telemetry_error', str(error))
        try:
            self.beep_tick(now)
            if sensors_due:
                self.sensor_color = str(self.color.color())
                self.last_sensors = now
            self.render(now)
        except Exception as error:
            self.last_error = fault('peripheral_error', str(error))
            if self.job:
                self.finish('fault', self.last_error)


class Peer:
    def __init__(self, api):
        self.api, self.buffer, self.closed = api, b'', False
        api.connect()

    def close(self, reason='disconnect'):
        if not self.closed:
            self.closed = True
            self.api.disconnect(reason)

    def feed(self, data):
        if self.closed:
            return []
        if not data:
            self.close('eof')
            return []
        self.buffer += data
        replies = []
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            if len(line) > MAX_FRAME:
                replies.append(self.api.protocol_fault(None, 'Frame exceeds 4096 bytes'))
                self.close('protocol_error')
                break
            try:
                request = json.loads(line.decode('ascii'))
            except (ValueError, UnicodeError):
                replies.append(self.api.protocol_fault(None, 'Invalid ASCII JSON frame'))
                self.close('protocol_error')
                break
            replies.append(self.api.request(request))
            if self.api.protocol_error:
                self.close('protocol_error')
                break
        if len(self.buffer) > MAX_FRAME and not self.closed:
            replies.append(self.api.protocol_fault(None, 'Frame exceeds 4096 bytes'))
            self.close('protocol_error')
        return replies


def encoded(value):
    return (json.dumps(value) + '\n').encode('ascii')


def serve(api, socket_module, select_module):
    listener = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM)
    listener.setsockopt(socket_module.SOL_SOCKET, socket_module.SO_REUSEADDR, 1)
    # Unix MicroPython sockets require the native sockaddr buffer returned by
    # getaddrinfo; CPython returns its compatible address tuple here instead.
    address = socket_module.getaddrinfo('127.0.0.1', 8766, socket_module.AF_INET, socket_module.SOCK_STREAM)[0][-1]
    listener.bind(address)
    listener.listen(1)
    listener.setblocking(False)
    poll = select_module.poll()
    poll.register(listener, select_module.POLLIN)
    client, peer, pending = None, None, b''
    close_deadline = None
    print(json.dumps(api.ready()))
    try:
        sys.stdout.flush()
    except OSError as error:
        native = native_runtime(sys.implementation)
        if not native or not error.args or error.args[0] != 22:
            listener.close()
            raise
    try:
        while True:
            events = poll.poll(api.poll_timeout_ms())
            for descriptor, flags in events:
                if descriptor == listener or descriptor == listener.fileno():
                    incoming, address = listener.accept()
                    incoming.setblocking(False)
                    if client is not None:
                        incoming.close()
                    else:
                        client, peer = incoming, Peer(api)
                        pending = encoded(api.ready())
                        close_deadline = None
                        poll.register(client, select_module.POLLIN | select_module.POLLOUT)
                    continue
                if client is None:
                    continue
                try:
                    if flags & select_module.POLLIN and not peer.closed:
                        data = client.recv(MAX_FRAME)
                        for reply in peer.feed(data):
                            pending += encoded(reply)
                        if len(pending) > 65536:
                            peer.close('output_backpressure')
                            pending = b''
                    if flags & select_module.POLLOUT and pending:
                        sent = client.send(pending)
                        if sent:
                            pending = pending[sent:]
                        else:
                            peer.close('send_eof')
                    if flags & (select_module.POLLERR | select_module.POLLHUP):
                        peer.close('socket_error')
                        pending = b''
                except OSError as error:
                    if not error.args or error.args[0] not in (11, 35):
                        peer.close('socket_error')
                        pending = b''
            api.tick()
            if peer and (peer.closed or not api.connected):
                peer.close('disconnect')
                if close_deadline is None:
                    close_deadline = api.clock() + 200
                if not pending or api.clock() >= close_deadline:
                    poll.unregister(client)
                    client.close()
                    client, peer, pending = None, None, b''
                    close_deadline = None
                else:
                    # Motors are already braked; drain only, never admit another request.
                    poll.modify(client, select_module.POLLOUT)
            elif client:
                poll.modify(client, select_module.POLLIN | (select_module.POLLOUT if pending else 0))
    finally:
        api.disconnect('service_exit')
        if client:
            client.close()
        listener.close()


def main():
    if not native_runtime(sys.implementation):
        raise RuntimeError('Requires pybricks-micropython major 2')
    import usocket
    import uselect
    import utime
    from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
    from pybricks.hubs import EV3Brick
    from pybricks.media.ev3dev import Font
    from pybricks.parameters import Port, Stop
    from pybricks.tools import StopWatch
    clock = StopWatch()
    initialized = []
    try:
        sorter = Motor(Port.D)
        initialized.append(sorter)
        sorter.brake()
        feeder = Motor(Port.A)
        initialized.append(feeder)
        feeder.brake()
        brick = EV3Brick()
        brick.screen.set_font(Font(size=12))
        api = LiveAPI({'sorter': sorter, 'feeder': feeder}, TouchSensor(Port.S1), ColorSensor(Port.S3),
                      brick, clock.time, Stop, 'ev3-' + str(utime.time()) + '-' + str(utime.ticks_us()))
        serve(api, usocket, uselect)
    finally:
        for motor in initialized:
            try:
                motor.brake()
            except Exception:
                pass


if __name__ == '__main__':
    main()
