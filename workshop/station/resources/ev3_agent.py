#!/usr/bin/env pybricks-micropython
"""Standalone EV3 MicroPython 2 agent. No station package or pip dependency.

Only finite <=100 ms run_time commands are accepted; BRAKE ends every slice.
EOF, partial framing, lease expiry and protocol faults stop both motors.
"""
try:
    import ujson as json
    import uselect as select
except ImportError:  # permits deterministic desktop tests
    import json
    import select
import sys
import os

PROTOCOL = 'ev3-station'
VERSION = 1
MAX_LINE = 4096
MAX_CHUNK = 256
EXPIRY_INTERVAL = 32


def flush_output(stream):
    """Flush buffered hosts; native Pybricks 2 stdout writes are unbuffered.

    Its POSIX stream flush calls fsync(), which returns EINVAL on the SSH pipe.
    https://github.com/pybricks/micropython/blob/v1.12/ports/unix/file.c
    Preserve every other I/O failure and EINVAL from unsupported runtimes.
    """
    try:
        stream.flush()
    except OSError as error:
        implementation = sys.implementation
        try:
            native = implementation[0] == 'pybricks-micropython' and implementation[1][0] == 2
        except (TypeError, IndexError):
            native = False
        if not native or not error.args or error.args[0] != 22:
            raise


class NativeChunkReader:
    """Read only bytes already available on the native Linux stdin pipe.

    FIONREAD counts bytes, so readinto avoids text read's UTF-8 character count.
    Pool both buffers and never wait for an unknown-sized remainder of a frame.
    """
    def __init__(self, stdin, ioctl=None):
        if ioctl is None:
            import ufcntl
            ioctl = ufcntl.ioctl
        self.stdin = stdin
        self.ioctl = ioctl
        self.count = bytearray(4)
        self.data = bytearray(MAX_CHUNK)

    def __call__(self):
        self.ioctl(self.stdin.fileno(), 0x541b, self.count)  # Linux FIONREAD
        available = self.count[0] | self.count[1] << 8 | self.count[2] << 16 | self.count[3] << 24
        if available == 0:
            return ''  # Readable EOF/spurious readiness: fail closed, never block.
        requested = min(available, MAX_CHUNK)
        length = self.stdin.readinto(self.data, requested)
        if length is None or length == 0:
            return ''
        if not 0 < length <= requested:
            raise ValueError('invalid native input byte count')
        for index in range(length):
            if self.data[index] >= 128:
                raise ValueError('station JSON transport requires ASCII bytes')
        return bytes(self.data[:length]).decode()


def port_inventory():
    ports = {}
    for root in ('/sys/class/tacho-motor', '/sys/class/lego-sensor'):
        for name in os.listdir(root):
            directory = root + '/' + name
            with open(directory + '/address') as handle:
                address = handle.read().strip()
            with open(directory + '/driver_name') as handle:
                driver = handle.read().strip()
            ports[address] = driver
    return ports


class Agent:
    def __init__(self, feeder, sorter, touch, color, colors, brake, nonce, clock, lease_ms=500):
        self.feeder, self.sorter = feeder, sorter
        self.touch, self.color = touch, color
        self.colors, self.brake = colors, brake
        self.nonce, self.clock = nonce, clock
        self.lease_ms = lease_ms
        self.last_traffic = clock()
        self.last_id = 0
        self.cached_request = self.cached_reply = None
        self.faulted = False
        self.pending = ''
        self.frame_started = None

    def stop(self):
        error = None
        for motor in (self.feeder, self.sorter):
            try:
                motor.brake()
            except Exception as caught:
                error = error or caught
        if error is not None:
            raise error

    def expire(self):
        now = self.clock()
        if now - self.last_traffic >= self.lease_ms or self.frame_started is not None and now - self.frame_started >= self.lease_ms:
            self.faulted = True
            self.stop()
            raise RuntimeError('agent lease/framing expired')

    def character(self, character):
        """Consume one already-polled character; never wait for rest of a line."""
        if character == '':
            self.faulted = True
            self.stop()
            raise EOFError('agent stdin EOF')
        if self.frame_started is None:
            self.frame_started = self.clock()
        if character == '\n':
            line = self.pending
            self.pending = ''
            self.frame_started = None
            try:
                request = json.loads(line)
            except Exception:
                self.faulted = True
                self.stop()
                raise ValueError('malformed JSON frame')
            return self.dispatch(request)
        self.pending += character
        if len(self.pending) > MAX_LINE:
            self.faulted = True
            self.stop()
            raise ValueError('oversized JSON frame')
        return None

    def dispatch(self, request):
        ident = request.get('id') if isinstance(request, dict) else None
        try:
            if not isinstance(request, dict) or request.get('nonce') != self.nonce or type(ident) is not int or ident <= 0:
                raise ValueError('invalid request identity')
            if ident == self.last_id:
                if request != self.cached_request:
                    raise ValueError('duplicate id changed request')
                self.last_traffic = self.clock()
                return self.cached_reply
            if ident < self.last_id:
                raise ValueError('stale request id')
            operation, args = request['operation'], request['args']
            if not isinstance(args, dict):
                raise ValueError('invalid arguments')
            if self.faulted and operation not in ('stop', 'busy'):
                raise RuntimeError('agent fault latched')
            if operation not in ('stop', 'busy'):
                self.expire()
            self.last_traffic = self.clock()
            name = args.get('name')
            motor = self.feeder if name == 'feeder' else self.sorter if name == 'sorter' else None
            if operation in ('slice', 'feed'):
                speed, ms = args.get('speed'), args.get('ms')
                if motor is None or operation == 'feed' and name != 'feeder' or type(ms) is not int or not 1 <= ms <= 100 or type(speed) not in (int, float) or not -200 <= speed <= 200 or speed == 0:
                    raise ValueError('invalid finite motor slice')
                # Never replenish while either previous command is active.
                if not self.feeder.control.done() or not self.sorter.control.done():
                    raise RuntimeError('motor command still active')
                motor.run_time(speed, ms, then=self.brake, wait=False)
                result = None
            elif operation == 'heartbeat':
                if args:
                    raise ValueError('heartbeat requires empty arguments')
                result = None
            elif operation == 'stop':
                self.stop()
                result = None
            elif operation == 'busy':
                result = int(not (self.feeder.control.done() and self.sorter.control.done()))
            elif operation == 'zero' and motor is not None:
                if not motor.control.done():
                    raise RuntimeError('zero requires stopped motor')
                motor.reset_angle(0)
                result = None
            elif operation == 'value':
                if motor is not None:
                    result = motor.angle()
                elif name == 'touch':
                    result = int(self.touch.pressed())
                elif name == 'color':
                    value = self.color.color()
                    result = self.colors.get(value, 0)
                else:
                    raise ValueError('unknown device')
            else:
                raise ValueError('unknown operation')
            reply = {'id': ident, 'ok': True, 'result': result}
        except Exception as error:
            self.faulted = True
            try:
                self.stop()
            except Exception:
                pass
            reply = {'id': ident, 'ok': False, 'fault': str(error)}
        # Monotonic ids retain one response and prevent old feed replay forever.
        if type(ident) is int and ident > self.last_id:
            self.last_id = ident
            self.cached_request = request.copy()
            self.cached_reply = reply
        return reply


def run(agent, poller, stdin, stdout, reader=None):
    if reader is None:
        reader = lambda: stdin.read(1)  # Desktop/injected API compatibility.
    try:
        while True:
            agent.expire()
            events = poller.poll(20)
            for _, event in events:
                if event & (select.POLLHUP | select.POLLERR):
                    raise EOFError('agent pipe closed')
                if event & select.POLLIN:
                    chunk = reader()
                    if chunk == '':
                        agent.character('')  # EOF immediately brakes and raises.
                    if len(chunk) > MAX_CHUNK:
                        raise ValueError('oversized input chunk')
                    for index, character in enumerate(chunk):
                        if index % EXPIRY_INTERVAL == 0:
                            agent.expire()
                        reply = agent.character(character)
                        if reply is not None:
                            stdout.write(json.dumps(reply) + '\n')
                            flush_output(stdout)
    finally:
        agent.stop()


def runtime_identity(implementation, python_version):
    """Read native sys.implementation, not Python language sys.version alone."""
    name, version = implementation[0], implementation[1]
    if name != 'pybricks-micropython' or len(version) != 5 or version[0] != 2:
        raise ValueError('EV3 requires actual Pybricks major version 2')
    if not isinstance(python_version, str) or not python_version.strip():
        raise ValueError('missing actual runtime version')
    return {'runtime_name': name, 'pybricks_version': list(version), 'runtime_version': python_version}


def main():
    from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
    from pybricks.parameters import Port, Color, Stop
    from pybricks.tools import StopWatch
    runtime = runtime_identity(sys.implementation, sys.version)
    settings = json.loads(sys.argv[1])
    nonce, ports = settings['nonce'], settings['ports']
    if not isinstance(nonce, str) or len(nonce) != 32 or settings['lease_ms'] != 500:
        raise ValueError('invalid session settings')
    inventory = port_inventory()
    drivers = {'feeder': 'lego-ev3-m-motor', 'sorter': 'lego-ev3-l-motor', 'touch': 'lego-ev3-touch', 'color': 'lego-ev3-color'}
    if set(ports) != set(drivers):
        raise ValueError('invalid ports')
    for name in ports:
        port = ports[name]
        address = 'ev3-ports:' + ('in' + port[1:] if port.startswith('S') else 'out' + port)
        if inventory.get(address) != drivers[name]:
            raise ValueError('actual port driver mismatch')
    feeder = Motor(getattr(Port, ports['feeder']))
    sorter = Motor(getattr(Port, ports['sorter']))
    watch = StopWatch()
    agent = Agent(feeder, sorter, TouchSensor(getattr(Port, ports['touch'])), ColorSensor(getattr(Port, ports['color'])),
                  {Color.BLUE: 2, Color.GREEN: 3, Color.YELLOW: 4, Color.RED: 5}, Stop.BRAKE, nonce, watch.time)
    try:
        agent.stop()
        reader = NativeChunkReader(sys.stdin)
        poller = select.poll()
        poller.register(sys.stdin, select.POLLIN)
        ready = {'protocol': PROTOCOL, 'version': VERSION, 'nonce': nonce, 'ready': True,
                 'ports': ports, 'discovered_ports': inventory}
        ready.update(runtime)
        sys.stdout.write(json.dumps(ready) + '\n')
        flush_output(sys.stdout)
        run(agent, poller, sys.stdin, sys.stdout, reader=reader)
    finally:
        agent.stop()


if __name__ == '__main__':
    main()
