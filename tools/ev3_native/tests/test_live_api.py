import ast
import importlib.util
import json
from pathlib import Path
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'live_api.py'

def load():
    assert MODULE.exists(), 'Persistent live API implementation is missing'
    spec = importlib.util.spec_from_file_location('live_api', MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class Clock:
    def __init__(self): self.ms = 0
    def __call__(self): return self.ms
class Control:
    def __init__(self, motor): self.motor, self.settings = motor, (999, 999, 99)
    def limits(self, *args, **kwargs):
        if args: self.settings = tuple(args)
        if kwargs: self.settings = (kwargs['speed'], kwargs['acceleration'], kwargs['actuation'])
        return self.settings
    def done(self): return self.motor.target is None or abs(self.motor.value - self.motor.target) <= 0.1
    def stalled(self): return self.motor.stall
    def target_tolerances(self): return (10, 2)
    def stall_tolerances(self): return (30, 100)
class Motor:
    def __init__(self):
        self.value, self.target, self.stall, self.commands, self.resets = 0, None, False, [], []
        self.angle_reads, self.speed_reads = 0, 0
        self.control = Control(self)
    def angle(self):
        self.angle_reads += 1
        return int(round(self.value))
    def speed(self):
        self.speed_reads += 1
        return 0
    def brake(self): self.target = None
    def reset_angle(self, value): self.value = value; self.resets.append(value)
    def run_target(self, speed, target, then, wait):
        assert then == 'BRAKE' and wait is False
        self.target = target; self.commands.append((speed, target))
    def advance(self):
        if self.target is not None and not self.stall:
            delta = self.target - self.value
            self.value += min(5, abs(delta)) * (1 if delta > 0 else -1)
class Touch:
    def __init__(self, motor): self.motor, self.override, self.reads = motor, None, 0
    def pressed(self):
        self.reads += 1
        return self.override if self.override is not None else self.motor.angle() <= 0
class Color:
    def __init__(self): self.reads = 0
    def color(self):
        self.reads += 1
        return 'RED'
class Screen:
    def __init__(self): self.frames, self.text = 0, []
    def clear(self): self.frames += 1
    def draw_text(self, x, y, text): self.text.append(text)
class Speaker:
    def __init__(self): self.calls = []
    def beep(self, **kwargs):
        assert kwargs['duration'] < 0
        self.calls.append(('beep', kwargs))
    def set_volume(self, volume, which): self.calls.append(('volume', volume, which))
class Light:
    def __init__(self): self.off_calls = 0
    def off(self): self.off_calls += 1
class Brick:
    def __init__(self): self.screen, self.speaker, self.light = Screen(), Speaker(), Light()
class Stop: BRAKE = 'BRAKE'

class Tests(unittest.TestCase):
    def setUp(self):
        self.lib = load(); self.clock = Clock(); self.motors = {'sorter': Motor(), 'feeder': Motor()}
        self.touch = Touch(self.motors['sorter']); self.brick = Brick()
        self.api = self.lib.LiveAPI(self.motors, self.touch, Color(), self.brick, self.clock, Stop, 'test-session')
        self.api.connect()
        self.id = 0
    def request(self, operation, **args):
        self.id += 1
        return self.api.request({'id': self.id, 'operation': operation, 'args': args})
    def confirm(self, name):
        return self.request('confirm_reference', name=name, reference='home' if name == 'sorter' else 'eject')
    def tick(self, ms=20, heartbeat=True):
        for _ in range(ms // 20):
            self.clock.ms += 20
            for motor in self.motors.values(): motor.advance()
            if heartbeat and self.clock.ms % 400 == 0: self.request('heartbeat')
            self.api.tick()
    def test_new_peer_starts_id_one_without_previous_response_replay(self):
        self.request('display',text='old client')
        self.api.disconnect()
        self.api.connect()
        reply=self.api.request({'id':1,'operation':'status','args':{}})
        self.assertTrue(reply['ok']); self.assertIn('angles',reply['result'])
    def test_initial_pressed_home_does_not_dispatch_native_motion(self):
        self.request('home_touch'); self.tick(300)
        self.assertFalse(self.motors['sorter'].commands)
        self.assertEqual(self.motors['sorter'].resets,[0])
    def test_stop_brake_failure_is_not_reported_stopped(self):
        self.confirm('feeder'); self.request('eject',cycles=1)
        def failed_brake(): raise OSError('cannot brake')
        self.motors['feeder'].brake=failed_brake
        reply=self.request('stop')
        self.assertFalse(reply['ok']); self.assertEqual(reply['fault']['code'],'brake_failed')

    def test_runtime_identity_accepts_native_tuple_and_named_implementation(self):
        self.assertTrue(hasattr(self.lib,'native_runtime'),'Native runtime tuple compatibility helper missing')
        self.assertTrue(self.lib.native_runtime(('pybricks-micropython',(2,0,0))))
        self.assertFalse(self.lib.native_runtime(('micropython',(2,0,0))))
        self.assertFalse(self.lib.native_runtime(('pybricks-micropython',(3,0,0))))

    def test_startup_preserves_origins_and_has_no_motion(self):
        status = self.request('status')['result']
        self.assertFalse(status['busy']); self.assertEqual(status['references'], {'sorter': None, 'feeder': None})
        self.assertTrue(all(not m.commands and not m.resets for m in self.motors.values()))
        self.assertEqual(self.api.ready()['protocol'], 'ev3-live-api')
        self.assertEqual(self.api.ready()['version'], 1)
    def test_idle_auto_power_save_ignores_heartbeat_and_status(self):
        self.api.power_save_idle_ms = 1000
        self.confirm('feeder')
        references = dict(self.api.references)
        self.tick(1000)
        self.assertEqual(self.api.status()['power_mode'], 'power_save')
        self.assertEqual(self.api.status()['references'], references)
        self.assertEqual(self.api.poll_timeout_ms(), self.lib.POWER_SAVE_POLL_MS)
        self.assertEqual(self.api.brick.light.off_calls, 1)
        reads = (sum(m.angle_reads for m in self.motors.values()), self.touch.reads,
                 self.api.color.reads)
        speed_reads = sum(m.speed_reads for m in self.motors.values())
        cached_speeds = json.loads(json.dumps(self.api.speed_samples))
        for _ in range(8):
            self.request('status')
            self.request('heartbeat')
            self.clock.ms += 250
            self.api.tick()
        self.api.disconnect('eof')
        self.request('status')
        self.assertEqual(self.api.status()['power_mode'], 'power_save')
        self.assertEqual((sum(m.angle_reads for m in self.motors.values()), self.touch.reads,
                          self.api.color.reads), reads)
        self.assertEqual(sum(m.speed_reads for m in self.motors.values()), speed_reads)
        self.assertEqual(self.api.speed_samples, cached_speeds)
        self.assertEqual(self.api.brick.screen.text[-2:], ['POWER SAVE', 'TCP API ready'])
        self.request('wake')
        self.request('status')
        self.assertEqual(sum(m.speed_reads for m in self.motors.values()), speed_reads + 2)
    def test_power_save_poll_rate_cuts_idle_tick_wakeups(self):
        active_hz = 1000.0 / self.api.poll_timeout_ms()
        self.request('power_save')
        saved_hz = 1000.0 / self.api.poll_timeout_ms()
        reduction = 100.0 * (1.0 - saved_hz / active_hz)
        self.assertEqual((active_hz, saved_hz), (50.0, 4.0))
        self.assertEqual(reduction, 92.0)
    def test_explicit_sleep_config_and_first_command_wake_keep_references(self):
        self.confirm('feeder')
        result = self.request('power_save', idle_timeout_ms=120000)
        self.assertTrue(result['ok'])
        self.assertEqual(result['result']['idle_timeout_ms'], 120000)
        self.assertEqual(self.api.status()['power_mode'], 'power_save')
        wake = self.request('motor_target', name='feeder', target=100)
        self.assertTrue(wake['ok'])
        self.assertEqual(self.motors['feeder'].speed_reads, 1)
        self.assertEqual(self.api.status()['power_mode'], 'awake')
        self.assertEqual(self.api.status()['references']['feeder'], 'eject')
        self.assertEqual(self.motors['feeder'].target, 100)
    def test_power_save_refuses_active_job_without_interrupting_it(self):
        self.confirm('feeder')
        self.request('motor_target', name='feeder', target=100)
        target = self.motors['feeder'].target
        reply = self.request('power_save')
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['fault']['code'], 'busy')
        self.assertEqual(self.motors['feeder'].target, target)
        self.assertTrue(self.api.status()['busy'])
    def test_wake_operation_and_invalid_foreground_request_wake_immediately(self):
        self.request('power_save')
        self.assertTrue(self.request('wake')['ok'])
        self.assertEqual(self.api.status()['power_mode'], 'awake')
        self.request('power_save')
        self.assertFalse(self.request('display', text='bad\ntext')['ok'])
        self.assertEqual(self.api.status()['power_mode'], 'awake')
    def test_entering_power_save_keeps_timestamps_of_last_real_samples(self):
        self.api.last_angles, self.api.last_sensors = 123, 456
        self.assertTrue(self.request('power_save')['ok'])
        self.assertEqual(self.api.status()['sampled_ms'], 123)
        self.assertEqual(self.api.last_sensors, 456)
    def test_duplicate_exact_request_never_repeats_motion_changed_is_protocol_fault(self):
        self.confirm('feeder')
        self.id += 1; req = {'id': self.id, 'operation': 'motor_target', 'args': {'name':'feeder','target':100}}
        reply = self.api.request(req)
        self.assertTrue(reply['ok']); self.assertEqual(reply, self.api.request(json.loads(json.dumps(req))))
        self.assertEqual(len(self.motors['feeder'].commands), 1)
        changed = dict(req); changed['args'] = {'name':'feeder','target':101}
        self.assertFalse(self.api.request(changed)['ok']); self.assertTrue(self.api.protocol_error)
        self.assertIsNone(self.motors['feeder'].target)
    def test_cache_is_bounded_and_evicted_stale_id_is_rejected(self):
        self.request('status')
        for _ in range(80): self.request('heartbeat')
        reply = self.api.request({'id':1,'operation':'status','args':{}})
        self.assertFalse(reply['ok']); self.assertTrue(self.api.protocol_error)
        self.assertLessEqual(len(self.api.cache), 64)
    def test_references_required_and_targets_admitted_before_native_write(self):
        self.assertFalse(self.request('motor_target', name='sorter', target=132)['ok'])
        self.confirm('sorter')
        for target in (-1, 551, float('nan')):
            self.assertFalse(self.request('motor_target',name='sorter',target=target)['ok'])
        self.assertFalse(self.motors['sorter'].commands)
    def test_finite_job_accepts_status_stop_and_rejects_other_motion_busy(self):
        self.confirm('feeder')
        reply = self.request('motor_target',name='feeder',target=209)
        self.assertTrue(reply['result']['accepted']); self.assertTrue(self.request('status')['result']['busy'])
        self.assertFalse(self.request('eject',cycles=1)['ok'])
        self.assertTrue(self.request('heartbeat')['ok'])
        self.assertTrue(self.request('stop')['result']['stopped'])
        self.assertFalse(self.api.status()['busy']); self.assertEqual(self.api.status()['last_job']['state'],'aborted')
    def test_completed_eject_has_exactly_one_pair_and_restores_limits(self):
        self.confirm('feeder'); self.request('eject',cycles=1); self.tick(2400)
        self.assertEqual([target for speed,target in self.motors['feeder'].commands], [209,0])
        self.assertEqual(self.api.status()['last_job']['state'],'complete')
        self.assertTrue(all(m.control.settings == (999,999,99) for m in self.motors.values()))
    def test_heartbeat_expiry_disconnects_and_brakes_active_job_but_idle_persists(self):
        self.tick(2000,heartbeat=False); self.assertTrue(self.api.connected)
        self.request('heartbeat'); self.confirm('feeder'); self.request('eject',cycles=1)
        self.tick(1000,heartbeat=False)
        self.assertFalse(self.api.connected); self.assertIsNone(self.motors['feeder'].target)
        self.assertEqual(self.api.status()['last_job']['fault']['code'],'heartbeat_expired')
    def test_disconnect_aborts_both_and_keeps_origin(self):
        self.confirm('feeder'); self.request('eject',cycles=1); self.api.disconnect('eof')
        self.assertTrue(all(m.target is None for m in self.motors.values()))
        self.assertEqual(self.api.status()['last_job']['state'],'aborted')
    def test_done_target_error_stall_step_and_other_motor_drift_fault(self):
        for fault in ('stall','done_error','step','other'):
            self.setUp(); self.confirm('feeder'); self.request('motor_target',name='feeder',target=100)
            if fault == 'stall': self.motors['feeder'].stall = True
            if fault == 'done_error': self.motors['feeder'].target = 0
            if fault == 'step': self.motors['feeder'].value = 30
            if fault == 'other': self.motors['sorter'].value = 4
            self.clock.ms += 20
            self.api.tick()
            self.assertEqual(self.api.status()['last_job']['state'],'fault',fault)
            self.assertTrue(all(m.target is None for m in self.motors.values()))
    def test_setup_refreshes_angles_after_limits_and_phase_brake(self):
        self.confirm('feeder')
        motor = self.motors['feeder']
        original_limits = motor.control.limits
        def moving_limits(*args, **kwargs):
            result = original_limits(*args, **kwargs)
            if kwargs: motor.value = 40
            return result
        motor.control.limits = moving_limits
        original_brake = motor.brake
        calls = []
        def moving_brake():
            original_brake()
            calls.append(True)
            if len(calls) == 2: motor.value = 50
        motor.brake = moving_brake
        self.assertTrue(self.request('motor_target', name='feeder', target=100)['ok'])
        self.assertEqual(self.api.job['_start'], 50)
        self.assertEqual(self.api.job['_last'], 50)
        self.assertEqual(self.api.job['_budget'], 56)

    def test_delayed_samples_allow_speed_bounded_motion(self):
        for delay, distance in ((50, 30), (200, 90)):
            self.setUp(); self.confirm('feeder')
            self.request('motor_target', name='feeder', target=209, speed=600)
            self.clock.ms += delay
            self.motors['feeder'].value = distance
            self.api.tick()
            self.assertIsNotNone(self.api.job, (delay, self.api.last_job))
            self.assertEqual(self.api.job['_last_sample_ms'], delay)

    def test_advertised_native_velocity_tolerance_covers_33_degrees_over_211ms(self):
        self.confirm('feeder')
        self.request('motor_target', name='feeder', target=209, speed=120)
        self.clock.ms += 211
        self.motors['feeder'].value = 33
        self.api.tick()
        self.assertIsNotNone(self.api.job)
        self.assertEqual(self.api.job['native_control']['limits'], [120, 2400, 50])
        self.assertEqual(self.api.job['native_control']['target_tolerances'], [10, 2])

    def test_native_tolerance_guard_still_rejects_large_jump_and_reports_settings(self):
        self.confirm('feeder')
        self.request('motor_target', name='feeder', target=209, speed=120)
        self.clock.ms += 211
        self.motors['feeder'].value = 60
        self.api.tick()
        diagnostic = self.api.status()['last_job']['encoder_diagnostics']
        self.assertEqual(self.api.last_job['state'], 'fault')
        self.assertEqual(diagnostic['native_speed_limit_dps'], 120)
        self.assertEqual(diagnostic['advertised_speed_tolerance_dps'], 10)
        self.assertEqual(diagnostic['empirical_speed_headroom_dps'], 10)
        self.assertEqual(diagnostic['guard_speed_dps'], 130)
        self.assertEqual(diagnostic['sample_uncertainty_ms'], 0)
        self.assertEqual(diagnostic['native_control']['stall_tolerances'], [30, 100])
        self.assertIn('speed_dps', diagnostic['speed_sample'])
        status_motor = self.api.status()['motor_control']['feeder']
        self.assertEqual(status_motor['target_tolerances'], [10, 2])
        self.assertIn('duration_ms', status_motor['speed_sample'])

    def test_native_sample_time_uncertainty_is_capped(self):
        self.confirm('feeder')
        motor = self.motors['feeder']
        original_angle = motor.angle
        calls = [0]
        def delayed_baseline_angle():
            calls[0] += 1
            if calls[0] == 3:  # target-phase baseline angle read
                self.clock.ms += 100
            return original_angle()
        motor.angle = delayed_baseline_angle
        self.request('motor_target', name='feeder', target=209, speed=120)
        self.assertEqual(self.api.job['_last_angle_read_ms'], 100)
        self.clock.ms += 211
        motor.value = 38
        self.api.tick()
        diagnostic = self.api.status()['last_job']['encoder_diagnostics']
        self.assertEqual(diagnostic['sample_uncertainty_ms'], 20)
        self.assertEqual(diagnostic['previous_angle_read_ms'], 100)
        self.assertEqual(diagnostic['current_angle_read_ms'], 0)

    def test_native_speed_headroom_cap_and_missing_tolerance_fallback(self):
        self.confirm('feeder')
        motor = self.motors['feeder']
        motor.control.target_tolerances = lambda: (1000, 2)
        self.request('motor_target', name='feeder', target=209, speed=120)
        self.clock.ms += 211
        motor.value = 38
        self.api.tick()
        capped = self.api.status()['last_job']['encoder_diagnostics']
        self.assertEqual(capped['advertised_speed_tolerance_dps'], 1000)
        self.assertEqual(capped['empirical_speed_headroom_dps'], 30)

        self.setUp(); self.confirm('feeder')
        motor = self.motors['feeder']
        motor.control.target_tolerances = None
        self.request('motor_target', name='feeder', target=209, speed=120)
        self.clock.ms += 211
        motor.value = 33
        self.api.tick()
        missing = self.api.status()['last_job']['encoder_diagnostics']
        self.assertIsNone(missing['advertised_speed_tolerance_dps'])
        self.assertEqual(missing['empirical_speed_headroom_dps'], 0)

    def test_sample_clock_includes_blocking_angle_read(self):
        self.confirm('feeder')
        self.request('motor_target', name='feeder', target=209, speed=600)
        motor = self.motors['feeder']
        def delayed_angle():
            self.clock.ms += 180
            return 90
        motor.angle = delayed_angle
        self.clock.ms = 20
        self.api.tick()
        self.assertIsNotNone(self.api.job)
        self.assertEqual(self.api.job['_last_sample_ms'], 200)

    def test_blocking_touch_read_does_not_shorten_encoder_sample_interval(self):
        self.motors['sorter'].value = 100
        self.request('home_touch')
        motor = self.motors['sorter']
        self.clock.ms = 20
        original_pressed = self.touch.pressed
        blocked = [False]
        def delayed_pressed():
            if not blocked[0]:
                blocked[0] = True
                self.clock.ms += 470
                motor.value = 76  # 24 degrees during the blocked sensor read.
            return original_pressed()
        self.touch.pressed = delayed_pressed
        self.api.tick()  # angle is sampled before the blocking touch read
        self.clock.ms += 20
        self.api.tick()  # the next angle sample spans the full 490 ms
        self.assertIsNotNone(self.api.job)
        self.assertEqual(self.api.job['_last_sample_ms'], self.clock.ms)
        self.assertEqual(self.api.job['_last'], 76)

    def test_wake_home_uses_fresh_angle_baseline_across_blocking_native_start(self):
        motor = self.motors['sorter']
        motor.value = 100
        self.request('power_save')
        self.clock.ms += 120000
        original_run_target = motor.run_target
        def delayed_run_target(speed, target, then, wait):
            original_run_target(speed, target, then, wait)
            self.clock.ms += 470
            motor.value = 76  # 24 degrees during native command startup.
        motor.run_target = delayed_run_target
        self.assertTrue(self.request('home_touch')['ok'])
        self.assertEqual(self.api.job['_start'], 100)
        self.api.last_heartbeat = self.clock.ms
        self.api.tick()
        self.assertIsNotNone(self.api.job)
        self.assertEqual(self.api.job['_last'], 76)

    def test_delayed_samples_retain_jump_budget_and_direction_guards(self):
        for target, angle, delay in ((209, 150, 200), (50, 60, 200), (209, -4, 200)):
            self.setUp(); self.confirm('feeder')
            self.request('motor_target', name='feeder', target=target, speed=600)
            self.clock.ms += delay
            self.motors['feeder'].value = angle
            self.api.tick()
            self.assertEqual(self.api.last_job['state'], 'fault', (target, angle))
            self.assertIsNone(self.motors['feeder'].target)

    def test_encoder_fault_exposes_numeric_diagnostics_and_bounded_history(self):
        self.motors['sorter'].value = 100
        self.request('home_touch')
        self.clock.ms = 20
        self.motors['sorter'].value = 60
        self.api.tick()
        failed = self.api.status()['last_job']
        diagnostic = failed['encoder_diagnostics']
        self.assertEqual(failed['state'], 'fault')
        self.assertEqual(diagnostic['step_deg'], -40)
        self.assertEqual(diagnostic['previous_angle_deg'], 100)
        self.assertEqual(diagnostic['angle_deg'], 60)
        self.assertEqual(diagnostic['elapsed_ms'], 20)
        self.assertEqual(diagnostic['jump_bound_deg'], 20)
        self.assertEqual(diagnostic['job_started_ms'], 0)
        self.assertEqual(diagnostic['sample_ms'], 20)
        self.assertLessEqual(len(diagnostic['history']), 16)
        self.assertEqual(diagnostic['history'][-1]['angle_deg'], 60)

    def test_wrong_direction_fault_exposes_bounded_encoder_history(self):
        self.motors['sorter'].value = 100
        self.request('home_touch')
        self.clock.ms = 20
        self.motors['sorter'].value = 104  # Home approaches in the negative direction.
        self.api.tick()
        failed = self.api.status()['last_job']
        diagnostic = failed['encoder_diagnostics']
        self.assertEqual(failed['fault']['message'], 'Motor moved in wrong direction')
        self.assertEqual(diagnostic['fault_reason'], 'wrong_direction')
        self.assertEqual(diagnostic['expected_direction'], -1)
        self.assertEqual(diagnostic['step_deg'], 4)
        self.assertEqual(diagnostic['signed_displacement_deg'], -4)
        self.assertEqual([sample['angle_deg'] for sample in diagnostic['history']], [100, 104])
        self.assertLessEqual(len(diagnostic['history']), 16)
        self.assertIn('speed_dps', diagnostic['native_status'])

    def test_home_touch_uses_gentle_command_speed_with_native_limits(self):
        self.motors['sorter'].value = 100
        self.request('home_touch')
        job = self.api.job
        self.assertEqual(job['_speed'], 50)
        self.assertEqual(self.motors['sorter'].commands[-1], (50, -450))
        self.assertEqual(job['native_control']['limits'], [120, 400, 30])
        self.assertEqual(self.motors['sorter'].control.settings, (120, 400, 30))

    def test_wrong_direction_captures_native_status_before_cleanup(self):
        self.motors['sorter'].value = 100
        motor = self.motors['sorter']
        events = []
        original_brake = motor.brake
        def recorded_brake():
            events.append('brake')
            original_brake()
        motor.brake = recorded_brake
        motor.speed = lambda: (events.append('speed'), 17)[1]
        motor.control.done = lambda: (events.append('done'), False)[1]
        motor.control.stalled = lambda: (events.append('stalled'), False)[1]
        self.request('home_touch')
        events.clear()
        self.clock.ms = 20
        motor.value = 104
        self.api.tick()
        diagnostic = self.api.last_job['encoder_diagnostics']
        sample = diagnostic['native_status']
        self.assertEqual(sample['speed_dps'], 17)
        self.assertFalse(sample['done'])
        self.assertFalse(sample['stalled'])
        self.assertLessEqual(sample['started_ms'], sample['speed_dps_sampled_ms'])
        self.assertLessEqual(sample['speed_dps_sampled_ms'], sample['done_sampled_ms'])
        self.assertLessEqual(sample['done_sampled_ms'], sample['stalled_sampled_ms'])
        self.assertEqual(events[-4:], ['speed', 'done', 'stalled', 'brake'])

    def test_encoder_history_is_allocated_only_for_motion_and_capped_at_16(self):
        self.assertIsNone(self.api.job)
        self.assertFalse(any(key.startswith('_encoder_history') for key in self.api.__dict__))
        self.confirm('feeder')
        self.request('motor_target', name='feeder', target=209, speed=600)
        self.assertEqual(len(self.api.job['_encoder_history']), 1)
        for _ in range(24):
            self.clock.ms += 20
            self.motors['feeder'].value += 1
            self.api.tick()
        self.assertEqual(len(self.api.job['_encoder_history']), 16)

    def test_home_touch_only_resets_after_pressed_and_quiet(self):
        self.motors['sorter'].value = 100
        self.request('home_touch'); self.tick(1500)
        self.assertEqual(self.motors['sorter'].resets,[0])
        self.assertEqual(self.api.status()['references']['sorter'],'home')
        self.assertFalse(self.motors['feeder'].commands)
    def test_home_touch_deadline_never_resets_without_switch(self):
        self.touch.override = False
        self.request('home_touch'); self.tick(4000)
        self.assertEqual(self.api.status()['last_job']['state'],'fault')
        self.assertFalse(self.motors['sorter'].resets)
    def test_shake_uses_finite_targets_and_reports_measured_not_requested_frequency(self):
        self.confirm('sorter'); self.motors['sorter'].value = 132
        self.request('shake',frequency_hz=12.5,speed_dps=600,acceleration=30000,duration_ms=1000)
        self.tick(1400)
        targets=[target for speed,target in self.motors['sorter'].commands]
        self.assertTrue(targets); self.assertTrue(set(targets)<=set([112,152]))
        last=self.api.status()['last_job']; self.assertEqual(last['state'],'complete')
        self.assertIn('measured_frequency_hz',last); self.assertIn('peak_displacement_deg',last)
    def test_shake_does_not_redraw_screen_each_tick(self):
        self.confirm('sorter'); self.motors['sorter'].value=132
        self.request('shake',frequency_hz=12.5,speed_dps=600,acceleration=30000,duration_ms=2000)
        self.tick(1800)
        self.assertLessEqual(self.brick.screen.frames,1)
    def test_display_beeps_and_argument_validation_are_nonblocking(self):
        self.assertTrue(self.request('display',text='Mechanics paused')['ok'])
        self.assertTrue(self.request('beep',count=2)['ok']); self.tick(600)
        self.assertEqual(sum(c[0]=='beep' for c in self.brick.speaker.calls),2)
        self.assertFalse(self.request('beep',count=4)['ok'])
        self.assertFalse(self.request('display',text='bad\ntext')['ok'])
        self.assertFalse(self.request('eval',code='motor.run(999)')['ok'])
    def test_peer_accepts_fragmented_batched_frames_and_ascii_only(self):
        peer=self.lib.Peer(self.api)
        req=json.dumps({'id':1,'operation':'status','args':{}}).encode('ascii')
        self.assertEqual(peer.feed(req[:5]),[])
        replies=peer.feed(req[5:]+b'\n'+json.dumps({'id':2,'operation':'heartbeat','args':{}}).encode('ascii')+b'\n')
        self.assertEqual([r['id'] for r in replies],[1,2]); self.assertFalse(peer.closed)
        peer.feed(b'\xff\n'); self.assertTrue(peer.closed)
    def test_peer_oversized_frame_protocol_error_brakes(self):
        self.confirm('feeder'); self.request('eject',cycles=1)
        peer=self.lib.Peer(self.api); peer.feed(b'x'*4097)
        self.assertTrue(peer.closed); self.assertIsNone(self.motors['feeder'].target)
    def test_real_loopback_protocol_fault_reply_flushes_before_eof(self):
        import socket, select, threading, time, types
        ready, stopping = threading.Event(), threading.Event()
        state = {}
        api = self.api
        api.clock = lambda: int(time.monotonic() * 1000)
        original_tick = api.tick
        def tick():
            if stopping.is_set(): raise RuntimeError('test server exit')
            original_tick()
        api.tick = tick
        class Listener:
            def __init__(self, *args): self.socket = socket.socket(*args)
            def __getattr__(self, key): return getattr(self.socket, key)
            def bind(self, address):
                if not isinstance(address,(bytes,bytearray)):
                    raise TypeError('native usocket requires sockaddr buffer')
                self.socket.bind(('127.0.0.1',0)); state['port']=self.socket.getsockname()[1]
            def listen(self, count): self.socket.listen(count); ready.set()
        module = types.SimpleNamespace(socket=Listener, AF_INET=socket.AF_INET,
                     SOCK_STREAM=socket.SOCK_STREAM, SOL_SOCKET=socket.SOL_SOCKET, SO_REUSEADDR=socket.SO_REUSEADDR,
                     getaddrinfo=lambda host,port,family,kind: [(family,kind,0,'',bytearray(b'linux-sockaddr'))])
        def serve():
            try: self.lib.serve(api,module,select)
            except Exception as error:
                if str(error) != 'test server exit': state['error']=str(error)
        thread=threading.Thread(target=serve); thread.start()
        client=None
        try:
            self.assertTrue(ready.wait(1))
            client=socket.create_connection(('127.0.0.1',state['port']),timeout=1)
            stream=client.makefile('rb')
            self.assertEqual(json.loads(stream.readline())['protocol'],'ev3-live-api')
            client.sendall(b'{"id":1,"operation":"status","args":{}}\n')
            self.assertTrue(json.loads(stream.readline())['ok'])
            client.sendall(b'{"id":2,"operation":"power_save","args":{}}\n')
            self.assertEqual(json.loads(stream.readline())['result']['power_mode'], 'power_save')
            self.assertEqual(api.poll_timeout_ms(), self.lib.POWER_SAVE_POLL_MS)
            client.sendall(b'{"id":3,"operation":"wake","args":{}}\n')
            self.assertEqual(json.loads(stream.readline())['result']['power_mode'], 'awake')
            client.sendall(b'{"id":1,"operation":"display","args":{"text":"changed"}}\n')
            line=stream.readline()
            self.assertTrue(line,'Protocol-fault response was dropped before socket close')
            self.assertEqual(json.loads(line)['fault']['code'],'protocol_error')
            self.assertEqual(stream.readline(),b'')
            stream.close()
        finally:
            if client: client.close()
            stopping.set(); thread.join(1)
        self.assertFalse(thread.is_alive()); self.assertNotIn('error',state)

    def test_status_cli_disconnect_preserves_power_save_and_idle_timer(self):
        import socket, select, threading, time, types
        ready, stopping, disconnected = threading.Event(), threading.Event(), threading.Event()
        state = {}
        api = self.api
        api.clock = lambda: int(time.monotonic() * 1000)
        self.request('power_save')
        idle_since = api.last_activity
        original_disconnect = api.disconnect
        def track_disconnect(reason='disconnect'):
            original_disconnect(reason)
            disconnected.set()
        api.disconnect = track_disconnect
        original_tick = api.tick
        def tick():
            if stopping.is_set(): raise RuntimeError('test server exit')
            original_tick()
        api.tick = tick
        class Listener:
            def __init__(self, *args): self.socket = socket.socket(*args)
            def __getattr__(self, key): return getattr(self.socket, key)
            def bind(self, address):
                if not isinstance(address,(bytes,bytearray)):
                    raise TypeError('native usocket requires sockaddr buffer')
                self.socket.bind(('127.0.0.1',0)); state['port']=self.socket.getsockname()[1]
            def listen(self, count): self.socket.listen(count); ready.set()
        module = types.SimpleNamespace(socket=Listener, AF_INET=socket.AF_INET,
                     SOCK_STREAM=socket.SOCK_STREAM, SOL_SOCKET=socket.SOL_SOCKET, SO_REUSEADDR=socket.SO_REUSEADDR,
                     getaddrinfo=lambda host,port,family,kind: [(family,kind,0,'',bytearray(b'linux-sockaddr'))])
        def serve():
            try: self.lib.serve(api,module,select)
            except Exception as error:
                if str(error) != 'test server exit': state['error']=str(error)
        thread=threading.Thread(target=serve); thread.start()
        try:
            self.assertTrue(ready.wait(1))
            from workshop.station import ev3_live_client
            commands = [
                ['--port', str(state['port']), 'status'],
                ['--port', str(state['port']), 'command', '--json',
                 '{"operation":"power_save","args":{}}'],
                ['--port', str(state['port']), 'command', '--json',
                 '{"operation":"heartbeat","args":{}}'],
            ]
            for index, command in enumerate(commands):
                if index:
                    disconnected.clear()
                self.assertEqual(ev3_live_client.main(command), 0)
                self.assertTrue(disconnected.wait(1))
                self.assertEqual(api.status()['power_mode'], 'power_save')
                self.assertEqual(api.last_activity, idle_since)
        finally:
            stopping.set(); thread.join(1)
        self.assertFalse(thread.is_alive()); self.assertNotIn('error',state)

    def test_python34_syntax_no_blocking_job_waits_or_eval(self):
        tree=ast.parse(MODULE.read_text(),feature_version=(3,4))
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Name): self.assertNotIn(node.func.id,('eval','exec'))
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute): self.assertNotIn(node.func.attr,('run_until_stalled','run_angle'))
        serve = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'serve')
        polls = [node for node in ast.walk(serve) if isinstance(node, ast.Call) and
                 isinstance(node.func, ast.Attribute) and node.func.attr == 'poll' and
                 isinstance(node.func.value, ast.Name) and node.func.value.id == 'poll']
        self.assertEqual(len(polls), 1)
        timeout = polls[0].args[0]
        self.assertIsInstance(timeout, ast.Call)
        self.assertEqual(timeout.func.attr, 'poll_timeout_ms')
        self.assertEqual(timeout.func.value.id, 'api')

if __name__=='__main__': unittest.main()
