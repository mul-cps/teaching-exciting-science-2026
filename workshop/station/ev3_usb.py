"""Independent original LEGO firmware direct commands, Linux + pyusb==1.3.1.

Protocol: LEGO Communication Developer Kit pp.24–29; Firmware Developer Kit
pp.9,46–57,90–92,100–101. No ev3-dc code is used. USB transfers and locks are
finite. Native enumeration/resource disposal cannot provide hard real-time
bounds; hardware commissioning and a physical stop remain required.
"""
import math
import re
import struct
import threading
import time
from copy import deepcopy

from .config import validate_config
from .discovery import discover_ev3, select_identity


class EV3Error(RuntimeError):
    """Identity, protocol or motion safety check failed."""


def _positive(value):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError('deadline must be positive and finite')
    return value


def _remaining(deadline):
    seconds = deadline - time.monotonic()
    if seconds <= 0:
        raise TimeoutError('EV3 operation deadline expired')
    return seconds


def _lc(value):
    """Firmware p.9: short constants or signed LC4."""
    value = int(value)
    return bytes([value & 0x3f]) if -32 <= value <= 31 else b'\x83' + struct.pack('<i', value)


class DirectUSB:
    """Injected transport exposes PyUSB write/read(endpoint, ..., timeout=ms)."""
    def __init__(self, transport):
        self.transport = transport
        self.lock = threading.Lock()
        self.counter = 0

    def command(self, operations, globals_size, seconds):
        deadline = time.monotonic() + _positive(seconds)
        if not 0 <= globals_size <= 1023:
            raise ValueError('invalid global memory size')
        if not self.lock.acquire(timeout=_remaining(deadline)):
            raise TimeoutError('EV3 transport lock deadline')
        try:
            self.counter = (self.counter + 1) & 0xffff
            body = struct.pack('<HBH', self.counter, 0, globals_size) + operations
            packet = struct.pack('<H', len(body)) + body
            timeout = max(1, math.ceil(min(_remaining(deadline), 0.1) * 1000))
            written = self.transport.write(0x01, packet, timeout=timeout)
            if written != len(packet):
                raise EV3Error('partial USB write; delivery unknown, never retry')
            while True:
                timeout = max(1, math.ceil(min(_remaining(deadline), 0.1) * 1000))
                reply = bytes(self.transport.read(0x81, 1024, timeout=timeout))
                _remaining(deadline)
                if len(reply) < 5 or struct.unpack('<H', reply[:2])[0] != len(reply) - 2:
                    raise EV3Error('malformed USB reply length')
                if struct.unpack('<H', reply[2:4])[0] != self.counter:
                    continue  # bounded absolute deadline, never resend
                if reply[4] != 0x02:
                    raise EV3Error(f'firmware direct-command error/status {reply[4]:02x}')
                if len(reply) != globals_size + 5:
                    raise EV3Error('incorrect reply global length')
                return reply[5:]
        finally:
            self.lock.release()


class _PyUSBTransport:
    def __init__(self, identity):
        import usb.core
        # No set_configuration, reset, or kernel-driver detach. Match the sysfs
        # identity to the enumerated bus/address before opening a transfer.
        matches = list(usb.core.find(find_all=True, idVendor=identity['vendor_id'], idProduct=identity['product_id']))
        matches = [d for d in matches if d.bus == identity['bus'] and d.address == identity['address']]
        if len(matches) != 1:
            raise EV3Error('selected USB device disappeared or is ambiguous')
        self.device = matches[0]
        self.device.default_timeout = 100  # also bounds descriptor control transfers
        import usb.util
        actual_serial = usb.util.get_string(self.device, self.device.iSerialNumber)
        if actual_serial != identity['serial']:
            raise EV3Error('live USB serial differs from selected sysfs identity')
        self.identity = identity

    def write(self, endpoint, packet, timeout):
        return self.device.write(endpoint, packet, timeout=timeout)

    def read(self, endpoint, size, timeout):
        return self.device.read(endpoint, size, timeout=timeout)

    def close(self):
        import usb.util
        usb.util.dispose_resources(self.device)


class OriginalEV3:
    """Accept the full station configuration, not just its ev3 section.

    connect/preflight perform no motor command. home is explicit and requires
    enabled + commissioned configuration for physical (non-injected) transport.
    Faults latch; stop remains available, but movement never retries after fault.
    """
    CHUNK_MS = 100

    def __init__(self, config, transport=None):
        validate_config(config)
        self.config = deepcopy(config)
        self.ev3 = self.config['ev3']
        if self.ev3['firmware'] != 'original':
            raise ValueError('OriginalEV3 requires original firmware')
        select_identity([self.ev3['usb']], self.ev3['usb'])
        self.transport = transport
        self.injected = transport is not None
        self.protocol = None
        self.identity = None
        self.homed = False
        self.faulted = False
        self.preflighted = False
        self.cancel = threading.Event()
        self.stop_generation = 0
        self.operation_generation = 0
        self.motion_lock = threading.Lock()
        self.ports = self.ev3['ports']

    def connect(self):
        """Select exact device; no homing, reset or output bytecodes."""
        if self.transport is None:
            self.identity = select_identity(discover_ev3(), self.ev3['usb'])
            self.transport = _PyUSBTransport(self.identity)
        else:
            self.identity = select_identity([self.transport.identity], self.ev3['usb'])
        self.protocol = DirectUSB(self.transport)
        return self.identity

    def _deadline(self, stop=False):
        return time.monotonic() + self.config['timeouts']['stop_s' if stop else 'operation_s']

    def _command(self, ops, size, deadline):
        if self.protocol is None:
            raise EV3Error('EV3 is not connected')
        return self.protocol.command(ops, size, _remaining(deadline))

    def _index(self, name):
        port = self.ports[name]
        return 16 + ord(port) - ord('A') if name in ('sorter', 'feeder') else int(port[1]) - 1

    def _mask(self, name):
        return 1 << (self._index(name) - 16)

    def _value(self, name, deadline):
        kind, mode = {'feeder': (8, 0), 'sorter': (7, 0), 'touch': (16, 0), 'color': (29, 2)}[name]
        ops = b'\x99\x1d\x00' + _lc(self._index(name)) + _lc(kind) + _lc(mode) + b'\x01\x60'
        value = struct.unpack('<f', self._command(ops, 4, deadline))[0]
        if not math.isfinite(value):
            raise EV3Error('nonfinite sensor/encoder value')
        if name == 'touch' and value not in (0, 1):
            raise EV3Error('invalid touch value')
        return value

    def preflight(self):
        self.preflighted = False
        deadline = self._deadline()
        for name, kind in (('feeder', 8), ('sorter', 7), ('touch', 16), ('color', 29)):
            reply = self._command(b'\x99\x05\0' + _lc(self._index(name)) + b'\x60\x61', 2, deadline)
            if reply[0] != kind or reply[1] not in (range(6) if name == 'color' else (0,)):
                raise EV3Error(f'{name} port type/mode mismatch')
            self._value(name, deadline)
        for label, subcode in (('firmware_version', 10), ('hardware_version', 9), ('os_version', 3)):
            raw = self._command(b'\x81' + _lc(subcode) + _lc(32) + b'\x60', 32, deadline)
            if b'\0' not in raw:
                raise EV3Error('unterminated firmware identity')
            try:
                text = raw.split(b'\0', 1)[0].decode('ascii')
            except UnicodeDecodeError as error:
                raise EV3Error('invalid firmware identity') from error
            if not text or label != 'hardware_version' and not re.fullmatch(r'V?\d+\.\d+[A-Za-z0-9._-]*', text):
                raise EV3Error('unrecognized original firmware identity')
            self.identity[label] = text
        self.preflighted = True
        return dict(self.identity)

    def read_color(self):
        if not self.preflighted:
            raise EV3Error('preflight required')
        value = self._value('color', self._deadline())
        if value not in (2, 3, 4, 5):
            raise EV3Error('unsupported color classification')
        return {2: 'blue', 3: 'green', 4: 'yellow', 5: 'red'}[value]

    def _stop(self, deadline):
        mask = self._mask('feeder') | self._mask('sorter')
        self._command(b'\xa3\0' + _lc(mask) + b'\x01', 0, deadline)

    def stop(self):
        self.stop_generation += 1
        self._stop(self._deadline(stop=True))

    def wait_stopped(self):
        return self._wait_stopped(self._deadline(stop=True))

    def _wait_stopped(self, deadline):
        mask = self._mask('feeder') | self._mask('sorter')
        while True:
            busy = self._command(b'\xa9\0' + _lc(mask) + b'\x60', 1, deadline)[0]
            if busy == 0:
                return True
            if busy != 1:
                raise EV3Error('invalid motor busy status')
            time.sleep(min(0.01, _remaining(deadline)))

    def _admit(self, deadline, needs_home):
        generation = self.stop_generation
        if not self.motion_lock.acquire(timeout=_remaining(deadline)):
            raise TimeoutError('EV3 motion lock deadline')
        try:
            # State can change while a command waits behind an active move.
            # Check only after acquiring ownership; never reset cancellation.
            if self.faulted:
                raise EV3Error('EV3 fault is latched; create a new backend after inspection')
            if not self.preflighted or needs_home and not self.homed:
                raise EV3Error('preflight and home required')
            if self.cancel.is_set():
                raise EV3Error('EV3 movement cancelled')
            if generation != self.stop_generation:
                raise EV3Error('queued EV3 movement revoked by stop')
            if not self.injected and not (self.config['enabled'] and self.config['commissioned']):
                raise EV3Error('commissioned enabled station required for motion')
            self.operation_generation = generation
        except BaseException:
            self.motion_lock.release()
            raise

    def _cancelled(self):
        return self.cancel.is_set() or self.operation_generation != self.stop_generation

    def _slice(self, name, direction, speed_dps, deadline):
        if self._cancelled():
            raise EV3Error('EV3 movement cancelled')
        # Percent speed is nominal, not calibrated dps. Conservative 20% cap;
        # encoder feedback, not this conversion, determines completion.
        rated = 1560 if name == 'feeder' else 1050
        percent = max(1, min(20, math.ceil(speed_dps * 100 / rated))) * direction
        ms = min(self.CHUNK_MS, math.floor(_remaining(deadline) * 1000))
        if ms <= 0:
            raise TimeoutError('no time remains for motor watchdog')
        ops = b'\xaf\0' + _lc(self._mask(name)) + b'\x81' + struct.pack('b', percent)
        ops += b'\x83' + struct.pack('<i', 0) + b'\x83' + struct.pack('<i', ms) + b'\x83' + struct.pack('<i', 0) + b'\x01'
        self._command(ops, 0, deadline)
        time.sleep(min(ms / 1000, _remaining(deadline)))

    def _track(self, name, direction, speed, deadline, max_travel, target=None, switch=None, stall_home=False):
        start = self._value(name, deadline)
        progress, progress_time = start, time.monotonic()
        released = False
        feeder = self.ev3['feeder']
        while True:
            if self._cancelled():
                raise EV3Error('EV3 movement cancelled')
            current = self._value(name, deadline)
            if abs(current - start) > max_travel:
                raise EV3Error('EV3 maximum encoder travel exceeded')
            if (current - start) * direction < -self.ev3['tolerance_deg']:
                raise EV3Error('encoder moved opposite commissioned direction')
            touch = self._value('touch', deadline) if name == 'sorter' else None
            if name == 'sorter' and switch is None:
                if touch == 0:
                    released = True
                elif released or (target is not None and abs(current - start) >= self.ev3['homing']['backoff_deg']):
                    raise EV3Error('unexpected or stuck sorter limit switch')
            if switch is not None and touch == switch:
                return current
            if target is not None:
                if abs(current - target) <= self.ev3['tolerance_deg']:
                    return current
                if (current - target) * direction > self.ev3['tolerance_deg']:
                    raise EV3Error('EV3 target overshoot')
                if name == 'sorter' and touch == 1 and direction == -self.ev3['directions']['sorter']:
                    raise EV3Error('unexpected sorter limit switch')
            if abs(current - progress) > feeder['stall_tolerance_deg']:
                progress, progress_time = current, time.monotonic()
            elif time.monotonic() - progress_time >= feeder['stall_window_s']:
                if stall_home:
                    return current
                raise EV3Error('EV3 stalled: encoder lacks progress')
            self._slice(name, direction, speed, deadline)

    def _finish(self, error):
        try:
            deadline = self._deadline(stop=True)
            self._stop(deadline)
            self._wait_stopped(deadline)
        except BaseException:
            self.faulted = True
            if error is None:
                raise
        finally:
            self.motion_lock.release()

    def _run(self, callback, needs_home=True):
        deadline = self._deadline()
        self._admit(deadline, needs_home)
        error = None
        try:
            return callback(deadline)
        except BaseException as caught:
            error = caught
            self.faulted = True
            self.homed = False
            raise
        finally:
            self._finish(error)

    def home(self):
        def action(deadline):
            self.homed = False
            homing, feeder = self.ev3['homing'], self.ev3['feeder']
            direction = self.ev3['directions']['sorter']
            end = min(deadline, time.monotonic() + homing['timeout_s'])
            start = self._value('sorter', end)
            # Always move away, even initially pressed; prove released before
            # approaching. No zeroing merely because the initial switch is high.
            self._track('sorter', direction, homing['speed_dps'], end, homing['max_travel_deg'], target=start + direction * homing['backoff_deg'])
            if self._value('touch', end) != 0:
                raise EV3Error('homing switch did not release')
            remaining = homing['max_travel_deg'] - abs(self._value('sorter', end) - start)
            if remaining <= 0:
                raise EV3Error('homing release exhausted travel budget')
            self._track('sorter', -direction, homing['speed_dps'], end, remaining, switch=1)
            self._stop(end)
            self._wait_stopped(end)
            self._command(b'\xb2\0' + _lc(self._mask('sorter')), 0, end)
            if feeder['home_travel_deg'] <= 0:
                raise EV3Error('feeder stall homing travel must be commissioned')
            end = min(deadline, time.monotonic() + feeder['home_timeout_s'])
            fdir = self.ev3['directions']['feeder']
            stalled = self._track('feeder', -fdir, feeder['home_speed_dps'], end, feeder['home_travel_deg'], stall_home=True)
            self._stop(end)
            self._track('feeder', fdir, feeder['home_speed_dps'], end, feeder['home_backoff_deg'] + self.ev3['tolerance_deg'], target=stalled + fdir * feeder['home_backoff_deg'])
            self._stop(end)
            self._wait_stopped(end)
            self._command(b'\xb2\0' + _lc(self._mask('feeder')), 0, end)
            self.homed = True
        return self._run(action, needs_home=False)

    def position(self, color):
        if color not in self.ev3['destinations']:
            raise ValueError('unsupported color destination')
        def action(deadline):
            target = self.ev3['directions']['sorter'] * self.ev3['destinations'][color]
            current = self._value('sorter', deadline)
            direction = 1 if target > current else -1
            return self._track('sorter', direction, self.ev3['homing']['speed_dps'], deadline,
                               self.ev3['homing']['max_travel_deg'], target=target)
        return self._run(action)

    def feed(self):
        def action(deadline):
            fdir, feeder = self.ev3['directions']['feeder'], self.ev3['feeder']
            start = self._value('feeder', deadline)
            return self._track('feeder', fdir, feeder['speed_dps'], deadline,
                               feeder['travel_deg'] + self.ev3['tolerance_deg'], target=start + fdir * feeder['travel_deg'])
        return self._run(action)

    def close(self):
        """Release resources only; caller must stop/wait before closing."""
        if self.transport is not None:
            self.transport.close()
        self.protocol = None
        self.preflighted = self.homed = False
