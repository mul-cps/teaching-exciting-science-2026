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
class Motor:
    def __init__(self):
        self.value, self.target, self.stall, self.commands, self.resets = 0, None, False, [], []
        self.control = Control(self)
    def angle(self): return int(round(self.value))
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
    def __init__(self, motor): self.motor, self.override = motor, None
    def pressed(self): return self.override if self.override is not None else self.motor.angle() <= 0
class Color:
    def color(self): return 'RED'
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
class Brick:
    def __init__(self): self.screen, self.speaker = Screen(), Speaker()
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

    def test_python34_syntax_no_blocking_job_waits_or_eval(self):
        tree=ast.parse(MODULE.read_text(),feature_version=(3,4))
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Name): self.assertNotIn(node.func.id,('eval','exec'))
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute): self.assertNotIn(node.func.attr,('run_until_stalled','run_angle'))

if __name__=='__main__': unittest.main()
