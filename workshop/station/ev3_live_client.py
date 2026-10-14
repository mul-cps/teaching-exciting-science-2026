"""Persistent PC client for the EV3 live API, through an existing SSH forward.

No connection recovery or command retries are performed: after transport failure,
command delivery is unknown. Create a new client only after checking the station.
"""
import argparse
import json
import math
import socket
import sys
import threading
import time

MAX_FRAME = 4096
HEARTBEAT_INTERVAL = .25


class EV3LiveError(RuntimeError):
    """Transport failure or unavailable connection; delivery may be unknown."""


class EV3LiveProtocolError(EV3LiveError):
    """The peer did not follow the live API framing or message contract."""


class EV3LiveFault(EV3LiveError):
    """An explicit native API or completed-job fault."""
    def __init__(self, fault):
        self.fault = fault
        super().__init__(f"{fault.get('code', 'fault')}: {fault.get('message', 'EV3 job failed')}")


def _positive_timeout(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('timeout must be positive and finite')
    return float(value)


def _reject_constant(value):
    raise ValueError('nonfinite JSON constant: ' + value)


class EV3LiveClient:
    """One serialized TCP session, with a background heartbeat until close.

    ``call`` returns the native result object. Remote command rejection raises
    EV3LiveFault without dropping the session; invalid replies and transport or
    heartbeat failures are terminal and visible through ``failure`` and calls.
    """
    def __init__(self, host='127.0.0.1', port=8766, timeout=3):
        if not isinstance(host, str) or not host:
            raise ValueError('host must be a nonempty string')
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('port must be an integer in 1..65535')
        self.host, self.port, self.timeout = host, port, _positive_timeout(timeout)
        self.session = None
        self._lock = threading.RLock()
        self._socket = None
        self._buffer = bytearray()
        self._next_id = 1
        self._failure = None
        self._closing = False
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None

    @property
    def connected(self):
        with self._lock:
            return self._socket is not None and self._failure is None and not self._closing

    @property
    def failure(self):
        with self._lock:
            return self._failure

    def _remaining(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('EV3 live API timed out')
        return remaining

    def _read_frame(self, deadline):
        while True:
            newline = self._buffer.find(b'\n')
            if newline >= 0:
                if newline > MAX_FRAME:
                    raise EV3LiveProtocolError('Reply exceeds 4096 byte frame limit')
                line = bytes(self._buffer[:newline])
                del self._buffer[:newline + 1]
                try:
                    value = json.loads(line.decode('ascii'), parse_constant=_reject_constant)
                except (UnicodeError, ValueError) as error:
                    raise EV3LiveProtocolError('Reply must be ASCII JSON') from error
                if not isinstance(value, dict):
                    raise EV3LiveProtocolError('Reply must be a JSON object')
                return value
            if len(self._buffer) > MAX_FRAME:
                raise EV3LiveProtocolError('Reply exceeds 4096 byte frame limit')
            self._socket.settimeout(self._remaining(deadline))
            data = self._socket.recv(min(1024, MAX_FRAME + 1 - len(self._buffer)))
            if not data:
                raise EV3LiveProtocolError('EV3 connection closed before a complete reply')
            self._buffer.extend(data)

    def _drop_socket(self):
        connection, self._socket = self._socket, None
        if connection is not None:
            try: connection.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            connection.close()
        self._buffer.clear()

    def _fail(self, error):
        if self._failure is None:
            self._failure = error
        self._heartbeat_stop.set()
        self._drop_socket()

    def connect(self):
        with self._lock:
            if self._closing:
                raise EV3LiveError('Client is closed')
            if self._failure is not None:
                raise EV3LiveError('Connection failed; commands are never retried') from self._failure
            if self._socket is not None:
                return self
            try:
                deadline = time.monotonic() + self.timeout
                self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
                ready = self._read_frame(deadline)
                if (ready.get('protocol') != 'ev3-live-api' or type(ready.get('version')) is not int or
                        ready['version'] != 1 or ready.get('ready') is not True or
                        not isinstance(ready.get('session'), str) or not ready['session']):
                    raise EV3LiveProtocolError('Expected ready ev3-live-api version 1 session')
                if self._closing:
                    raise EV3LiveError('Client closed while connecting')
                self.session = ready['session']
                self._heartbeat_thread = threading.Thread(
                    target=self._heartbeat, name='ev3-live-heartbeat', daemon=True)
                self._heartbeat_thread.start()
                return self
            except (OSError, EV3LiveError) as error:
                failure = error if isinstance(error, EV3LiveError) else EV3LiveError(str(error))
                self._fail(failure)
                if failure is error: raise
                raise failure from error

    def _request(self, operation, args, timeout, *, closing=False):
        if not isinstance(operation, str) or not operation:
            raise ValueError('operation must be a nonempty string')
        if args is None: args = {}
        if not isinstance(args, dict):
            raise ValueError('args must be a JSON object')
        request_id = self._next_id
        try:
            payload = json.dumps({'id': request_id, 'operation': operation, 'args': args},
                                 ensure_ascii=True, allow_nan=False, separators=(',', ':')).encode('ascii')
        except (TypeError, ValueError) as error:
            raise ValueError('request must contain JSON values') from error
        if len(payload) > MAX_FRAME:
            raise ValueError('Request exceeds 4096 byte frame limit')
        if self._failure is not None:
            raise EV3LiveError('Connection failed; commands are never retried') from self._failure
        if self._socket is None or (self._closing and not closing):
            raise EV3LiveError('Client is not connected')
        self._next_id += 1
        try:
            deadline = time.monotonic() + timeout
            self._socket.settimeout(self._remaining(deadline))
            self._socket.sendall(payload + b'\n')
            reply = self._read_frame(deadline)
            if type(reply.get('id')) is not int or reply['id'] != request_id:
                raise EV3LiveProtocolError('Reply id differs from outstanding request')
            if type(reply.get('ok')) is not bool:
                raise EV3LiveProtocolError('Reply requires boolean ok')
            if reply['ok']:
                if not isinstance(reply.get('result'), dict):
                    raise EV3LiveProtocolError('Successful reply requires an object result')
                return reply['result']
            fault = reply.get('fault')
            if (not isinstance(fault, dict) or not isinstance(fault.get('code'), str) or
                    not isinstance(fault.get('message'), str)):
                raise EV3LiveProtocolError('Failed reply requires fault code and message')
            raise EV3LiveFault(fault)
        except EV3LiveFault:
            raise
        except (OSError, EV3LiveError) as error:
            failure = error if isinstance(error, EV3LiveError) else EV3LiveError(str(error))
            self._fail(failure)
            if failure is error: raise
            raise failure from error

    def call(self, operation, args=None):
        """Send exactly once and read one matching reply, serialized with heartbeat."""
        with self._lock:
            return self._request(operation, args, self.timeout)

    def _heartbeat(self):
        while not self._heartbeat_stop.wait(HEARTBEAT_INTERVAL):
            with self._lock:
                if self._heartbeat_stop.is_set() or self._closing:
                    return
                try:
                    self._request('heartbeat', {}, self.timeout)
                except EV3LiveError as error:
                    self._fail(error)
                    return

    def wait_idle(self, timeout=15):
        """Poll status until idle, returning status or raising job fault/timeout."""
        deadline = time.monotonic() + _positive_timeout(timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('EV3 job did not become idle before timeout')
            with self._lock:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('EV3 job did not become idle before timeout')
                status = self._request('status', {}, min(self.timeout, remaining))
            invalid_status = type(status.get('busy')) is not bool or any(
                status.get(field) is not None and not isinstance(status[field], dict)
                for field in ('job', 'last_job', 'last_error'))
            if invalid_status:
                error = EV3LiveProtocolError('Status requires boolean busy and object/null job and error fields')
                with self._lock: self._fail(error)
                raise error
            if not status['busy']:
                job = status.get('last_job') or status.get('job') or {}
                fault = status.get('last_error') or job.get('fault')
                if fault:
                    if not isinstance(fault, dict):
                        fault = {'code': 'job_fault', 'message': str(fault)}
                    raise EV3LiveFault(fault)
                if job.get('state') in ('fault', 'aborted'):
                    raise EV3LiveFault({'code': 'job_' + job['state'], 'message': 'EV3 job ' + job['state']})
                return status
            time.sleep(min(.05, max(0, deadline - time.monotonic())))

    def close(self):
        """Best-effort stop, socket close, and heartbeat join; safe to repeat."""
        self._closing = True
        self._heartbeat_stop.set()
        with self._lock:
            if self._socket is not None and self._failure is None:
                try: self._request('stop', {}, self.timeout, closing=True)
                except (EV3LiveError, OSError): pass
            self._drop_socket()
            thread = self._heartbeat_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(self.timeout + .5)

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--timeout', type=float, default=3)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status')
    commands.add_parser('stop')
    display = commands.add_parser('display')
    display.add_argument('--text', required=True)
    beep = commands.add_parser('beep')
    beep.add_argument('--count', type=int, default=1)
    command = commands.add_parser('command')
    command.add_argument('--json', required=True, dest='request_json')
    command.add_argument('--wait', action='store_true')
    args = parser.parse_args(argv)
    try:
        operation, request_args = args.command, {}
        if args.command == 'display': request_args = {'text': args.text}
        elif args.command == 'beep': request_args = {'count': args.count}
        elif args.command == 'command':
            request = json.loads(args.request_json, parse_constant=_reject_constant)
            if not isinstance(request, dict) or set(request) != {'operation', 'args'}:
                raise ValueError('command JSON must contain exactly operation and args')
            operation, request_args = request['operation'], request['args']
        with EV3LiveClient(args.host, args.port, args.timeout) as client:
            result = client.call(operation, request_args)
            if args.command == 'command' and args.wait:
                result = client.wait_idle()
            print(json.dumps(result, sort_keys=True, allow_nan=False))
        return 0
    except (EV3LiveError, ValueError, TimeoutError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
