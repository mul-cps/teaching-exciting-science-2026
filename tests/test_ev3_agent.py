"""Injected transport and MicroPython device tests; no SSH or motor access."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from workshop.station.ev3_pybricks import PybricksEV3, SSHTransport, EV3Error, validate_identity
from workshop.station.resources.ev3_agent import Agent, run


def config():
    c = json.loads(Path('workshop/station/resources/station.example.json').read_text())
    c['ev3']['firmware'] = 'pybricks'
    c['ev3']['pybricks'].update(host='ev3.local', user='robot', remote_dir='/home/robot/station', expected_machine_id='a' * 32)
    c['ev3']['feeder']['home_travel_deg'] = 180
    return c


def identity(c):
    ports = c['ev3']['ports']
    drivers = {'feeder': 'lego-ev3-l-motor', 'sorter': 'lego-ev3-m-motor', 'touch': 'lego-ev3-touch', 'color': 'lego-ev3-color'}
    return {'machine_id': 'a' * 32, 'platform': 'armv5tel LEGO EV3 AM1808',
            'ports': {'ev3-ports:' + ('in' + port[1:] if port.startswith('S') else 'out' + port): drivers[name] for name, port in ports.items()}}


class Transport:
    def __init__(self, c):
        self.config = c
        self.sent = []
        self.closed = False
        self.bad = None
        self.first = True
        self.motor = {'feeder': 0, 'sorter': 0}
        self.agent_identity = identity(c)

    def connect(self, deadline, nonce):
        self.nonce = nonce
        return self.agent_identity

    def receive(self, deadline):
        if self.first:
            self.first = False
            return {'protocol': 'ev3-station', 'version': 1, 'nonce': self.nonce, 'ready': True, 'ports': self.config['ev3']['ports'], 'runtime_version': '2.0', 'discovered_ports': self.agent_identity['ports']}
        if isinstance(self.bad, BaseException):
            raise self.bad
        if self.bad is not None:
            return self.bad
        r = self.sent[-1]
        op, args = r['operation'], r['args']
        if op == 'value':
            name = args['name']
            result = 2 if name == 'color' else int(self.motor['sorter'] <= 0) if name == 'touch' else self.motor[name]
        elif op in ('slice', 'feed'):
            name = args['name']
            self.motor[name] += args['speed'] * args['ms'] / 1000
            if name == 'feeder':
                self.motor[name] = min(self.motor[name], 30)
            result = None
        elif op == 'zero':
            self.motor[args['name']] = 0
            result = None
        else:
            result = 0 if op == 'busy' else None
        return {'id': r['id'], 'ok': True, 'result': result}

    def send(self, request, deadline):
        self.sent.append(request)

    def close(self):
        self.closed = True


def backend():
    c = config()
    wire = Transport(c)
    brick = PybricksEV3(c, wire)
    brick.connect()
    brick.preflight()
    return brick, wire


@pytest.mark.parametrize('reply', [{}, [], {'id': 999, 'ok': True}, {'id': 1, 'ok': 'yes'}, TimeoutError('missing acknowledgement'), EOFError('loss')])
def test_missing_malformed_or_wrong_ack_latches_without_retry(reply):
    brick, wire = backend()
    wire.bad = reply
    before = len(wire.sent)
    with pytest.raises((EV3Error, TimeoutError, EOFError)):
        brick.read_color()
    assert brick.faulted and not brick.homed
    assert len(wire.sent) == before + 1


def test_machine_mismatch_closes_before_handshake():
    c = config()
    wire = Transport(c)
    wire.agent_identity['machine_id'] = 'b' * 32
    brick = PybricksEV3(c, wire)
    with pytest.raises(EV3Error, match='identity'):
        brick.connect()
    assert wire.closed and wire.first and not wire.sent


@pytest.mark.parametrize('change', ['platform', 'ports'])
def test_real_platform_and_ports_required(change):
    c = config()
    value = identity(c)
    value[change] = 'x86 Linux' if change == 'platform' else {}
    with pytest.raises(EV3Error):
        validate_identity(value, c['ev3'])


def test_deployment_failure_never_launches(monkeypatch):
    c = config()
    transport = SSHTransport(c)
    calls = []
    def remote(command, deadline, data=None):
        calls.append(command)
        if len(calls) == 1:
            return json.dumps(identity(c)).encode()
        raise EV3Error('deploy failure')
    monkeypatch.setattr(transport, '_remote', remote)
    with pytest.raises(EV3Error, match='deploy'):
        PybricksEV3(c, transport).connect()
    assert len(calls) == 2 and transport.process is None


def test_pinned_ssh_and_safe_remote_directory():
    transport = SSHTransport(config())
    assert 'StrictHostKeyChecking=yes' in transport.argv
    assert 'BatchMode=yes' in transport.argv
    c = config()
    c['ev3']['pybricks']['remote_dir'] = '/tmp/x; touch /tmp/unsafe'
    with pytest.raises(ValueError):
        SSHTransport(c)


def test_shared_feed_returns_actual_start_and_one_deadline(monkeypatch):
    brick, wire = backend()
    brick.homed = True
    wire.motor['feeder'] = 12
    tracks = []
    def track(name, direction, speed, deadline, max_travel, **kw):
        tracks.append((name, direction, deadline, kw['target']))
        wire.motor[name] = kw['target']
        return kw['target']
    monkeypatch.setattr(brick, '_track', track)
    assert brick.feed() == 12
    assert tracks[0][3] == 192 and tracks[1][3] == 12
    assert tracks[0][2] == tracks[1][2]


def test_sticky_cancel_and_stop_generation(monkeypatch):
    brick, wire = backend()
    brick.homed = True
    brick.cancel.set()
    with pytest.raises(EV3Error, match='cancelled'):
        brick.feed()
    brick.cancel.clear()
    brick.operation_generation = brick.stop_generation
    brick.stop()
    assert brick._cancelled()
    before = len(wire.sent)
    with pytest.raises(EV3Error, match='cancelled'):
        brick._slice('feeder', 1, 30, brick._deadline())
    assert len(wire.sent) == before


class Motor:
    def __init__(self):
        self.calls = []
        self.stops = 0
        self.control = SimpleNamespace(done=lambda: True)
        self.fail_stop = False

    def run_time(self, speed, ms, **kwargs):
        self.calls.append((speed, ms, kwargs))

    def brake(self):
        self.stops += 1
        if self.fail_stop:
            raise OSError('stop failed')

    def angle(self):
        return 12

    def reset_angle(self, angle):
        self.zero = angle


@pytest.fixture
def agent():
    a = Agent(Motor(), Motor(), SimpleNamespace(pressed=lambda: False), SimpleNamespace(color=lambda: 'blue'), {'blue': 2}, 'BRAKE', 'n' * 32, lambda: 0)
    return a


def request(ident=1, operation='feed', **args):
    return {'id': ident, 'nonce': 'n' * 32, 'operation': operation, 'args': args or {'name': 'feeder', 'speed': 30, 'ms': 100}}


def test_duplicate_feed_is_not_executed_twice(agent):
    r = request()
    first = agent.dispatch(r)
    assert first['ok'] and agent.dispatch(r) == first
    assert len(agent.feeder.calls) == 1
    assert agent.feeder.calls[0] == (30, 100, {'then': 'BRAKE', 'wait': False})
    agent.dispatch(request(2, 'value', name='feeder'))
    assert not agent.dispatch(r)['ok']
    assert len(agent.feeder.calls) == 1


def test_changed_duplicate_faults(agent):
    agent.dispatch(request())
    assert not agent.dispatch(request(speed=31, name='feeder', ms=100))['ok']
    assert agent.faulted


@pytest.mark.parametrize('args', [{'name': 'feeder', 'speed': 30, 'ms': 101}, {'name': 'feeder', 'speed': float('nan'), 'ms': 100}, {'name': 'sorter', 'speed': 30, 'ms': 100}])
def test_agent_rejects_unsafe_feed(agent, args):
    assert not agent.dispatch(request(**args))['ok']
    assert not agent.feeder.calls and not agent.sorter.calls
    assert agent.feeder.stops and agent.sorter.stops


def test_stop_attempts_both_when_first_fails(agent):
    agent.feeder.fail_stop = True
    with pytest.raises(OSError):
        agent.stop()
    assert agent.feeder.stops == agent.sorter.stops == 1


def test_eof_partial_frame_stops_both(agent):
    agent.character('{')
    with pytest.raises(EOFError):
        agent.character('')
    assert agent.feeder.stops and agent.sorter.stops


def test_partial_frame_and_idle_expire(agent):
    agent.character('{')
    agent.clock = lambda: 501
    with pytest.raises(RuntimeError, match='expired'):
        agent.expire()
    assert agent.faulted and agent.feeder.stops and agent.sorter.stops


def test_complete_frame_and_oversized_frame(agent):
    reply = None
    for char in json.dumps(request()) + '\n':
        reply = agent.character(char)
    assert reply['ok']
    with pytest.raises(ValueError, match='oversized'):
        for _ in range(4097):
            agent.character('x')


def test_agent_run_eof_finally_stops(agent):
    import io, select
    poller = SimpleNamespace(poll=lambda timeout: [(0, select.POLLIN)])
    with pytest.raises(EOFError):
        run(agent, poller, io.StringIO(''), io.StringIO())
    assert agent.feeder.stops >= 2 and agent.sorter.stops >= 2


def test_shared_homing_releases_switch_zeros_and_stall_backs_off():
    c = config()
    c['ev3']['homing'].update(speed_dps=30, backoff_deg=6, timeout_s=2)
    c['ev3']['feeder'].update(home_speed_dps=30, home_backoff_deg=6, stall_window_s=.02, home_timeout_s=3)
    c['timeouts']['operation_s'] = 4
    wire = Transport(c)
    brick = PybricksEV3(c, wire)
    brick.connect()
    brick.preflight()
    brick.home()
    assert brick.homed and not brick.faulted
    moves = [r for r in wire.sent if r['operation'] in ('feed', 'slice')]
    sorter = [r['args']['speed'] for r in moves if r['args']['name'] == 'sorter']
    feeder = [r['args']['speed'] for r in moves if r['args']['name'] == 'feeder']
    assert sorter[0] > 0 and sorter[-1] < 0
    assert feeder[0] > 0 and feeder[-1] < 0
    assert wire.motor == {'feeder': 0, 'sorter': 0}


def test_homing_switch_stuck_latches_and_stops(monkeypatch):
    brick, wire = backend()
    real_value = brick._value
    monkeypatch.setattr(brick, '_value', lambda name, deadline: 1 if name == 'touch' else real_value(name, deadline))
    with pytest.raises(EV3Error, match='switch'):
        brick.home()
    assert brick.faulted and not brick.homed
    assert any(r['operation'] == 'stop' for r in wire.sent)


def test_homing_total_timeout_latches():
    c = config()
    c['timeouts']['operation_s'] = .005
    wire = Transport(c)
    brick = PybricksEV3(c, wire)
    brick.connect()
    brick.preflight()
    with pytest.raises(TimeoutError):
        brick.home()
    assert brick.faulted and not brick.homed


def test_actual_handshake_inventory_required():
    c = config()
    wire = Transport(c)
    original = wire.receive
    def receive(deadline):
        result = original(deadline)
        result['discovered_ports'] = {}
        return result
    wire.receive = receive
    with pytest.raises(EV3Error, match='handshake'):
        PybricksEV3(c, wire).connect()
    assert wire.closed


def test_expired_agent_stop_remains_available(agent):
    agent.faulted = True
    agent.clock = lambda: 900
    assert agent.dispatch(request(operation='stop'))['ok']
    assert agent.feeder.stops and agent.sorter.stops


def test_agent_busy_prevents_second_slice(agent):
    agent.feeder.control.done = lambda: False
    assert not agent.dispatch(request())['ok']
    assert not agent.feeder.calls


@pytest.mark.parametrize('data,error', [(b'{\n', EV3Error), (b'{"id":1', EOFError), (b'x' * 8193, EV3Error)], ids=['malformed-json', 'partial-eof', 'oversized'])
def test_host_real_pipe_framing_is_bounded(data, error):
    import os, selectors, time
    reader, writer = os.pipe()
    diagnostic_reader, diagnostic_writer = os.pipe()
    stdout = os.fdopen(reader, 'rb', buffering=0)
    stderr = os.fdopen(diagnostic_reader, 'rb', buffering=0)
    transport = SSHTransport(config())
    transport.process = SimpleNamespace(stdout=stdout, stderr=stderr)
    transport.selector = selectors.DefaultSelector()
    for stream in (stdout, stderr):
        os.set_blocking(stream.fileno(), False)
        transport.selector.register(stream, selectors.EVENT_READ)
    # Inject the oversized buffer directly to avoid test-side blocking writes.
    try:
        if len(data) > 4096:
            transport.buffer.extend(data)
        else:
            os.write(writer, data)
            os.close(writer)
            writer = None
        with pytest.raises(error):
            transport.receive(time.monotonic() + .05)
    finally:
        if writer is not None:
            os.close(writer)
        os.close(diagnostic_writer)
        stdout.close()
        stderr.close()
        transport.selector.close()
