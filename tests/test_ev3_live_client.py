"""Persistent wire-level client tests against loopback sockets; no EV3 access."""
import json
import socket
import threading
import time

import pytest

from workshop.station.ev3_live_client import (
    EV3LiveClient, EV3LiveError, EV3LiveFault, EV3LiveProtocolError, main,
)


class Server:
    def __init__(self, handler=None, ready=None, fragment=False):
        self.handler = handler
        self.ready = ready if ready is not None else {
            'protocol': 'ev3-live-api', 'version': 1, 'ready': True, 'session': 'test-session',
        }
        self.fragment = fragment
        self.requests = []
        self.errors = []
        self.closed = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen()
        self.listener.settimeout(.1)
        self.port = self.listener.getsockname()[1]
        self.connection = None
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def send(self, value):
        payload = value if isinstance(value, bytes) else json.dumps(value).encode('ascii') + b'\n'
        if self.fragment:
            for offset in range(0, len(payload), 7): self.connection.sendall(payload[offset:offset + 7])
        else: self.connection.sendall(payload)

    def serve(self):
        try:
            while not self.closed.is_set():
                try: self.connection, _ = self.listener.accept(); break
                except socket.timeout: continue
            if self.connection is None: return
            self.connection.settimeout(.1)
            self.send(self.ready)
            buffer = b''
            while not self.closed.is_set():
                try: data = self.connection.recv(4096)
                except socket.timeout: continue
                if not data: break
                buffer += data
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    request = json.loads(line.decode('ascii'))
                    self.requests.append(request)
                    if self.handler is not None:
                        reply = self.handler(request)
                    else:
                        result = {'busy': False, 'job': None, 'last_job': None, 'last_error': None}
                        reply = {'id': request['id'], 'ok': True, 'result': result}
                    if reply is None: return
                    self.send(reply)
        except (ConnectionError, OSError):
            pass  # A protocol-rejecting client deliberately closes the socket.
        except BaseException as error: self.errors.append(error)
        finally:
            if self.connection is not None: self.connection.close()

    def close(self):
        self.closed.set()
        if self.connection is not None:
            try: self.connection.shutdown(socket.SHUT_RDWR)
            except OSError: pass
        self.listener.close()
        self.thread.join(1)
        assert not self.thread.is_alive()
        assert not self.errors


@pytest.fixture
def server():
    servers = []
    def create(**kwargs):
        instance = Server(**kwargs); servers.append(instance); return instance
    yield create
    for instance in servers: instance.close()


def client_for(server, timeout=.3):
    return EV3LiveClient(port=server.port, timeout=timeout)


def wait_for(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline: pytest.fail('condition deadline expired')
        time.sleep(.01)


def test_fragmented_ready_persistent_calls_and_close_stop(server):
    endpoint = server(fragment=True)
    with client_for(endpoint) as client:
        assert client.session == 'test-session'
        assert client.call('status')['busy'] is False
        assert client.call('display', {'text': 'Hello'})['busy'] is False
    assert [r['operation'] for r in endpoint.requests] == ['status', 'display', 'stop']
    assert [r['id'] for r in endpoint.requests] == [1, 2, 3]
    assert not client.connected
    assert not client._heartbeat_thread.is_alive()


@pytest.mark.parametrize('change', [dict(protocol='bad'), dict(version=2),
                                     dict(version=True), dict(ready=False), dict(session='')])
def test_bad_ready_is_rejected(server, change):
    ready = {'protocol': 'ev3-live-api', 'version': 1, 'ready': True, 'session': 'session'}
    ready.update(change)
    endpoint = server(ready=ready)
    client = client_for(endpoint)
    with pytest.raises(EV3LiveProtocolError): client.connect()
    client.close()
    assert not endpoint.requests


@pytest.mark.parametrize('reply', [b'not-json\n', b'\xff\n', b'[]\n',
                                   b'{"id":999,"ok":true,"result":{}}\n',
                                   b'{"id":true,"ok":true,"result":{}}\n',
                                   b'{"id":1,"ok":1,"result":{}}\n',
                                   b'{"id":1,"ok":true,"result":[]}\n',
                                   b'{"id":1,"ok":false,"fault":"bad"}\n',
                                   b'{"id":1,"ok":true,"result":{"bad":NaN}}\n',
                                   b'x' * 4097])
def test_protocol_error_disconnects_without_retry(server, reply):
    endpoint = server(handler=lambda request: reply)
    client = client_for(endpoint).connect()
    with pytest.raises(EV3LiveProtocolError): client.call('move', {'degrees': 5})
    with pytest.raises(EV3LiveError): client.call('move', {'degrees': 5})
    client.close()
    assert len(endpoint.requests) == 1


def test_remote_fault_is_reported_and_connection_survives_for_stop(server):
    def handler(request):
        if request['operation'] == 'stop': return {'id': request['id'], 'ok': True, 'result': {}}
        return {'id': request['id'], 'ok': False, 'fault': {'code': 'busy', 'message': 'motor busy'}}
    endpoint = server(handler=handler)
    with client_for(endpoint) as client:
        with pytest.raises(EV3LiveFault, match='motor busy') as caught: client.call('move', {})
        assert caught.value.fault['code'] == 'busy'
        assert client.connected
    assert endpoint.requests[-1]['operation'] == 'stop'


def test_heartbeat_runs_and_failure_surfaces(server):
    def handler(request):
        if request['operation'] == 'heartbeat': return None
        return {'id': request['id'], 'ok': True, 'result': {}}
    endpoint = server(handler=handler)
    client = client_for(endpoint).connect()
    wait_for(lambda: client.failure is not None)
    with pytest.raises(EV3LiveError): client.call('status')
    client.close()
    assert [r['operation'] for r in endpoint.requests] == ['heartbeat']


def test_timeout_is_terminal_and_request_is_not_retried(server):
    def handler(request):
        time.sleep(.15)
        return {'id': request['id'], 'ok': True, 'result': {}}
    endpoint = server(handler=handler)
    client = client_for(endpoint, timeout=.04).connect()
    with pytest.raises(EV3LiveError, match='timed out'): client.call('move')
    client.close()
    assert len(endpoint.requests) == 1


def test_concurrent_calls_are_serialized_with_heartbeat(server):
    endpoint = server(fragment=True)
    with client_for(endpoint) as client:
        errors = []
        def request():
            try: client.call('status')
            except Exception as error: errors.append(error)
        threads = [threading.Thread(target=request) for _ in range(10)]
        for thread in threads: thread.start()
        for thread in threads: thread.join(1)
        wait_for(lambda: any(r['operation'] == 'heartbeat' for r in endpoint.requests))
        assert not errors
        assert all(not t.is_alive() for t in threads)
    ids = [r['id'] for r in endpoint.requests]
    assert ids == list(range(1, len(ids) + 1))


def test_oversize_outgoing_request_is_never_sent(server):
    endpoint = server()
    with client_for(endpoint) as client:
        with pytest.raises(ValueError, match='4096'): client.call('display', {'text': 'x' * 4096})
        with pytest.raises(ValueError): client.call('status', [])
        with pytest.raises(ValueError): client.call('', {})
        client.call('status')
    assert [r['operation'] for r in endpoint.requests] == ['status', 'stop']
    assert endpoint.requests[0]['id'] == 1


def test_wait_idle_polls_completion(server):
    polls = []
    def handler(request):
        polls.append(request['operation'])
        return {'id': request['id'], 'ok': True, 'result': {
            'busy': len(polls) < 3, 'last_job': {'state': 'complete'}, 'last_error': None,
        }}
    endpoint = server(handler=handler)
    with client_for(endpoint) as client:
        assert client.wait_idle(timeout=1)['last_job']['state'] == 'complete'
    assert polls[:3] == ['status'] * 3


@pytest.mark.parametrize('status', [
    {'busy': False, 'last_job': {'state': 'fault', 'fault': {'code': 'stall', 'message': 'stalled'}}},
    {'busy': False, 'last_job': {'state': 'aborted'}},
    {'busy': False, 'last_error': {'code': 'lost', 'message': 'lost home'}},
])
def test_wait_idle_reports_job_faults(server, status):
    endpoint = server(handler=lambda r: {'id': r['id'], 'ok': True, 'result': status})
    with client_for(endpoint) as client:
        with pytest.raises(EV3LiveFault): client.wait_idle(timeout=.3)


def test_wait_idle_timeout(server):
    endpoint = server(handler=lambda r: {'id': r['id'], 'ok': True, 'result': {'busy': True}})
    with client_for(endpoint) as client:
        started = time.monotonic()
        with pytest.raises(TimeoutError): client.wait_idle(timeout=.08)
        assert time.monotonic() - started < .3


@pytest.mark.parametrize('status', [dict(busy=0), dict(busy=False, last_job=[]),
                                     dict(busy=False, job='bad'), dict(busy=False, last_error=1)])
def test_malformed_status_is_terminal_protocol_fault(server, status):
    endpoint = server(handler=lambda r: {'id': r['id'], 'ok': True, 'result': status})
    with client_for(endpoint) as client:
        with pytest.raises(EV3LiveProtocolError): client.wait_idle()
        assert not client.connected


def test_slow_partial_reply_has_absolute_deadline(server):
    def handler(request):
        endpoint.connection.sendall(b'{')
        time.sleep(.1)
        return b'"id":1,"ok":true,"result":{}}\n'
    endpoint = server(handler=handler)
    with client_for(endpoint, timeout=.04) as client:
        started = time.monotonic()
        with pytest.raises(EV3LiveError, match='timed out'): client.call('status')
        assert time.monotonic() - started < .15


def test_exact_frame_limit_is_accepted(server):
    def handler(request):
        reply = json.dumps({'id': request['id'], 'ok': True, 'result': {}}).encode('ascii')
        return reply + b' ' * (4096 - len(reply)) + b'\n'
    endpoint = server(handler=handler)
    with client_for(endpoint) as client:
        assert client.call('status') == {}


def test_cli_json_command_wait(server, capsys):
    endpoint = server()
    assert main(['--port', str(endpoint.port), 'command', '--json',
                 '{"operation":"display","args":{"text":"OK"}}', '--wait']) == 0
    assert json.loads(capsys.readouterr().out)['busy'] is False
    assert [r['operation'] for r in endpoint.requests] == ['display', 'status', 'stop']


@pytest.mark.parametrize('kwargs', [{'timeout': 0}, {'timeout': float('nan')},
                                    {'port': 0}, {'port': True}])
def test_invalid_connection_options(kwargs):
    with pytest.raises(ValueError): EV3LiveClient(**kwargs)
