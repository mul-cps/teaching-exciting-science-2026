"""EV3 MicroPython 2 over pinned SSH, finite JSON requests and motor slices."""
import json
import math
import os
import re
import secrets
import selectors
import shlex
import subprocess
import threading
import time
from copy import deepcopy
from pathlib import Path

from .config import validate_config
from .ev3_motion import EV3Motion, EV3Error, _remaining

PROTOCOL = 'ev3-station'
VERSION = 1
DRIVERS = {'feeder': 'lego-ev3-m-motor', 'sorter': 'lego-ev3-l-motor',
           'touch': 'lego-ev3-touch', 'color': 'lego-ev3-color'}
RUNTIME_SCRIPT = """import sys,ujson
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Stop
from pybricks.tools import StopWatch
print(ujson.dumps({'runtime_name':sys.implementation[0], 'pybricks_version':list(sys.implementation[1]), 'runtime_version':sys.version}))
"""
IDENTITY_SCRIPT = """import glob,json,pathlib,platform
p=pathlib.Path
ports={}
for root in ('/sys/class/tacho-motor/*','/sys/class/lego-sensor/*'):
 for d in glob.glob(root):
  q=p(d); ports[(q/'address').read_text().strip()]=(q/'driver_name').read_text().strip()
model=p('/proc/device-tree/model')
print(json.dumps({'machine_id':p('/etc/machine-id').read_text().strip(),'platform':platform.machine()+' '+p('/proc/cpuinfo').read_text()+' '+(model.read_text() if model.exists() else ''),'ports':ports}))
"""


def validate_identity(identity, ev3):
    if not isinstance(identity, dict) or identity.get('machine_id') != ev3['pybricks'].get('expected_machine_id') or not re.fullmatch('[0-9a-f]{32}', identity.get('machine_id', '')):
        raise EV3Error('remote machine identity mismatch')
    platform = identity.get('platform', '').lower()
    if not any(x in platform for x in ('ev3', 'am1808', 'da850')) or 'arm' not in platform:
        raise EV3Error('remote platform is not EV3 ARM')
    ports = identity.get('ports', {})
    for name, port in ev3['ports'].items():
        address = 'ev3-ports:' + (port if port.startswith('S') else 'out' + port)
        address = address.replace(':S', ':in')
        if ports.get(address) != DRIVERS[name]:
            raise EV3Error(name + ' actual remote port driver mismatch')
    return identity


def validate_runtime(runtime):
    if not isinstance(runtime, dict) or runtime.get('runtime_name') != 'pybricks-micropython':
        raise EV3Error('unsupported Pybricks runtime identity')
    version = runtime.get('pybricks_version')
    if not isinstance(version, (list, tuple)) or len(version) != 5 or any(type(v) is not int or v < 0 for v in version[:3]) or version[0] != 2 or not isinstance(version[3], str) or not version[3] or type(version[4]) is not int or version[4] < 0:
        raise EV3Error('EV3 requires actual Pybricks major version 2')
    text = runtime.get('runtime_version')
    if not isinstance(text, str) or not text.strip():
        raise EV3Error('missing actual MicroPython runtime version')
    return runtime


class SSHTransport:
    """Own a single bounded subprocess; no password or network setup."""
    def __init__(self, config):
        self.config = config
        pb = config['ev3']['pybricks']
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]*', pb['host']) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', pb['user']):
            raise ValueError('invalid SSH host/user')
        if not re.fullmatch(r'/[A-Za-z0-9_./-]+', pb['remote_dir']) or '..' in pb['remote_dir'].split('/'):
            raise ValueError('invalid absolute remote_dir')
        self.argv = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=5', '-p', str(pb['port'])]
        if pb['identity_file']:
            self.argv += ['-i', pb['identity_file']]
        self.argv += [pb['user'] + '@' + pb['host']]
        self.process = None
        self.buffer = bytearray()
        self.selector = None

    def _remote(self, command, deadline, data=None):
        # Capture finite identity/deployment replies in files, avoiding unbounded
        # subprocess PIPE collection from a malfunctioning remote peer.
        import tempfile
        with tempfile.TemporaryFile() as out, tempfile.TemporaryFile() as err:
            result = subprocess.run(self.argv + [command], input=data, stdout=out, stderr=err, timeout=_remaining(deadline), check=False)
            if result.returncode:
                raise EV3Error('SSH identity/deployment failed')
            out.seek(0)
            content = out.read(8193)
            if len(content) > 8192:
                raise EV3Error('oversized SSH identity output')
            return content

    def connect(self, deadline, nonce):
        raw = self._remote('python3 -c ' + shlex.quote(IDENTITY_SCRIPT), deadline)
        try:
            identity = validate_identity(json.loads(raw), self.config['ev3'])
        except (ValueError, TypeError) as error:
            raise EV3Error('malformed remote identity') from error
        try:
            validate_runtime(json.loads(self._remote('brickrun -r -- pybricks-micropython -c ' + shlex.quote(RUNTIME_SCRIPT), deadline)))
        except (ValueError, TypeError) as error:
            raise EV3Error('malformed remote runtime identity') from error
        directory = self.config['ev3']['pybricks']['remote_dir']
        target = directory.rstrip('/') + '/ev3_agent.py'
        source = Path(__file__).parent / 'resources' / 'ev3_agent.py'
        self._remote('mkdir -p -- ' + shlex.quote(directory) + ' && cat > ' + shlex.quote(target), deadline, source.read_bytes())
        settings = json.dumps({'nonce': nonce, 'ports': self.config['ev3']['ports'], 'lease_ms': 500})
        command = ' '.join(shlex.quote(s) for s in ['brickrun', '-r', '--', 'pybricks-micropython', target, settings])
        self.process = subprocess.Popen(self.argv + [command], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.selector = selectors.DefaultSelector()
        for pipe in (self.process.stdout, self.process.stderr, self.process.stdin):
            os.set_blocking(pipe.fileno(), False)
        for pipe in (self.process.stdout, self.process.stderr):
            self.selector.register(pipe, selectors.EVENT_READ)
        return identity

    def send(self, request, deadline):
        packet = (json.dumps(request, separators=(',', ':')) + '\n').encode()
        if len(packet) > 4096:
            raise EV3Error('oversized request')
        offset = 0
        while offset < len(packet):
            _remaining(deadline)
            try:
                written = os.write(self.process.stdin.fileno(), packet[offset:])
                if written <= 0:
                    raise EOFError('SSH write EOF')
                offset += written
            except BlockingIOError:
                self._pump(deadline)

    def _pump(self, deadline):
        for key, _ in self.selector.select(min(.05, _remaining(deadline))):
            data = os.read(key.fileobj.fileno(), 4096)
            if key.fileobj is self.process.stdout:
                if not data:
                    raise EOFError('agent disconnected')
                self.buffer.extend(data)
                if len(self.buffer) > 8192:
                    raise EV3Error('oversized agent output')
            elif not data:
                self.selector.unregister(key.fileobj)
            # Diagnostics drained and discarded; never an unbounded queue.

    def receive(self, deadline):
        while b'\n' not in self.buffer:
            if len(self.buffer) > 4096:
                raise EV3Error('oversized response frame')
            self._pump(deadline)
        raw, _, rest = self.buffer.partition(b'\n')
        self.buffer = bytearray(rest)
        if len(raw) > 4096:
            raise EV3Error('oversized response')
        try:
            return json.loads(raw)
        except (ValueError, UnicodeError) as error:
            raise EV3Error('malformed agent JSON') from error

    def close(self):
        if self.process is not None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=.5)
            self.process.stdout.close()
            self.process.stderr.close()
        if self.selector:
            self.selector.close()


class PybricksEV3(EV3Motion):
    CHUNK_MS = 100
    HEARTBEAT_INTERVAL = 0.1
    RPC_SECONDS = 0.15

    def __init__(self, config, transport=None):
        validate_config(config)
        self.config = deepcopy(config)
        self.ev3 = self.config['ev3']
        if self.ev3['firmware'] != 'pybricks':
            raise ValueError('PybricksEV3 requires pybricks firmware')
        self.injected = transport is not None
        self.transport = transport
        self.ports = self.ev3['ports']
        self.homed = self.faulted = self.preflighted = False
        self.cancel = threading.Event()
        self.stop_generation = self.operation_generation = self.counter = 0
        self.motion_lock = threading.Lock()
        self.protocol_lock = threading.Lock()
        self.identity = None
        self.nonce = secrets.token_hex(16)
        self.connected = False
        self.heartbeat_error = None
        self.heartbeat_stop = threading.Event()
        self.heartbeat_thread = None

    def connect(self):
        if self.faulted or self.identity is not None:
            raise EV3Error('create a new Pybricks backend for a new session')
        deadline = self._deadline()
        if self.transport is None:
            self.transport = SSHTransport(self.config)
        try:
            self.identity = validate_identity(self.transport.connect(deadline, self.nonce), self.ev3)
            ready = self.transport.receive(deadline)
            if not isinstance(ready, dict) or ready.get('protocol') != PROTOCOL or ready.get('version') != VERSION or ready.get('nonce') != self.nonce or ready.get('ready') is not True or ready.get('ports') != self.ports or ready.get('discovered_ports') != self.identity['ports'] or not isinstance(ready.get('runtime_version'), str):
                raise EV3Error('invalid agent ready handshake')
            validate_runtime(ready)
            self.connected = True
            self._start_heartbeat()
            return dict(self.identity)
        except BaseException:
            self.faulted = True
            self.transport.close()
            raise

    def _request(self, operation, args, deadline, guard=None):
        if not self.connected:
            raise EV3Error('Pybricks is not connected')
        if self.heartbeat_error is not None and operation not in ('stop', 'busy'):
            self.check_health()
        deadline = min(deadline, time.monotonic() + self.RPC_SECONDS)
        if not self.protocol_lock.acquire(timeout=_remaining(deadline)):
            raise TimeoutError('agent protocol lock deadline')
        try:
            if guard is not None and not guard():
                raise EV3Error('movement cancelled before agent delivery')
            self.counter += 1
            self.transport.send({'id': self.counter, 'nonce': self.nonce, 'operation': operation, 'args': args}, deadline)
            reply = self.transport.receive(deadline)
            _remaining(deadline)
            if not isinstance(reply, dict) or reply.get('id') != self.counter or type(reply.get('id')) is not int or type(reply.get('ok')) is not bool:
                raise EV3Error('malformed or mismatched agent response')
            if not reply['ok']:
                raise EV3Error('agent fault: ' + str(reply.get('fault', 'unknown')))
            return reply.get('result')
        except BaseException:
            self.faulted = True
            self.homed = False
            raise
        finally:
            self.protocol_lock.release()

    def _start_heartbeat(self):
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop,
                                                  name='ev3-agent-heartbeat', daemon=True)
        self.heartbeat_thread.start()

    def _heartbeat_loop(self):
        while not self.heartbeat_stop.wait(self.HEARTBEAT_INTERVAL):
            try:
                self._request('heartbeat', {}, time.monotonic() + self.RPC_SECONDS)
            except BaseException as error:
                # Preserve the causal failure for the caller. Cancellation is
                # shared with the arm loop by station integration.
                self.heartbeat_error = error
                self.faulted = True
                self.homed = False
                self.cancel.set()
                self.heartbeat_stop.set()
                return

    def check_health(self):
        if self.heartbeat_error is not None:
            raise EV3Error('EV3 agent heartbeat failed: ' + str(self.heartbeat_error)) from self.heartbeat_error
        if self.faulted:
            raise EV3Error('EV3 fault is latched')

    def preflight(self):
        self.preflighted = False
        deadline = self._deadline()
        for name in self.ports:
            self._value(name, deadline)
        self.preflighted = True
        return dict(self.identity)

    def _value(self, name, deadline):
        value = self._request('value', {'name': name}, deadline)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or name == 'touch' and value not in (0, 1):
            self.faulted = True
            self.homed = False
            raise EV3Error('invalid agent sensor value')
        return value

    def _zero(self, name, deadline):
        self._request('zero', {'name': name}, deadline)

    def _busy(self, deadline):
        return self._request('busy', {}, deadline)

    def _stop(self, deadline):
        self._request('stop', {}, deadline)

    def _slice(self, name, direction, speed_dps, deadline):
        ms = min(self.CHUNK_MS, math.floor(_remaining(deadline) * 1000))
        if ms <= 0:
            raise TimeoutError('no time for agent watchdog')
        self._request('feed' if name == 'feeder' else 'slice', {'name': name, 'speed': direction * speed_dps, 'ms': ms}, deadline, guard=lambda: not self._cancelled())
        time.sleep(min(ms / 1000, _remaining(deadline)))

    def close(self):
        self.heartbeat_stop.set()
        thread = self.heartbeat_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.RPC_SECONDS + .2)
        try:
            if self.transport:
                self.transport.close()
        finally:
            self.connected = self.preflighted = self.homed = False
        if thread is not None and thread.is_alive():
            self.faulted = True
            self.cancel.set()
            raise EV3Error('EV3 heartbeat thread did not stop within deadline')
