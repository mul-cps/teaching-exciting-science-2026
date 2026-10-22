"""Fence cold ARM routing and bounded cached recording against hot-path growth."""
import ast
import hashlib
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from workshop.blockly_program import ARM, validate_program
from workshop.blockly_ros import ARM_SERVICES, ROSBackend

ROOT = Path(__file__).resolve().parents[1]


def methods(path, classname):
    cls = next(n for n in ast.parse((ROOT / path).read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == classname)
    return {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}


def digest(node):
    def fields(value):
        if isinstance(value, ast.AST):
            return [type(value).__name__, {key: fields(item) for key, item in ast.iter_fields(value)
                                          if item is not None and item != []}]
        if isinstance(value, list):
            return [fields(item) for item in value]
        return value
    return hashlib.sha256(json.dumps(fields(node), sort_keys=True, separators=(',', ':')).encode()).hexdigest()


@pytest.mark.parametrize('method,expected', [
    ('_status_message', 'e2750f482df0743a3ba8c4946b344050bb02b0165a352e88c1ae0f519ff58155'),
    ('_joints', 'af6b3fe819c9c71a7fcc6408f3ad53bf0250faac3631c95369b261fa860df2cc'),
    ('_refresh_calibration', '5e94d384dc1c932daef76477953a54a9e99fba472cc3869c3b6125755a0eddd3'),
    ('_refresh_owner', 'fb65cbe035d513f853731986eace45093a8ca7f8b95140fac3f94ef97620ffcc'),
    ('sample', 'e5e1fb45c134ec1592410593ab59c2c98b2648b8b9dc29020db4439d221ccda3'),
])
def test_callbacks_and_cached_sampler_remain_baseline(method, expected):
    # Baseline b5d2a06: semantic AST pins reject new callback work/allocations,
    # publishers, subscriptions, driver access, or RPCs in these existing paths.
    assert digest(methods('workshop/blockly_ros.py', 'ROSBackend')[method]) == expected


def test_arm_allowlist_uses_only_existing_operator_rpcs_and_stop():
    assert ARM == {'home', 'moveit', 'teleop_relative', 'teleop_absolute', 'torque_off'}
    assert dict(ARM_SERVICES) == {'home': ('start',), 'moveit': ('moveit',),
                                 'teleop_relative': ('teleop_relative', 'teleop'),
                                 'teleop_absolute': ('teleop_absolute', 'teleop')}
    with pytest.raises(TypeError):
        ARM_SERVICES['home'] = ('other',)
    for command in ARM:
        cancel = threading.Event()
        deadline = time.monotonic() + 10
        calls = []
        backend = ROSBackend.__new__(ROSBackend)
        backend.trigger = lambda *args: calls.append(args)
        backend.stop = lambda: calls.append(('STOP',))
        backend.command({'op': 'arm', 'command': command}, cancel, deadline)
        expected = ([('STOP',)] if command == 'torque_off' else
                    [('/station/operator/' + endpoint, cancel, deadline)
                     for endpoint in ARM_SERVICES[command]])
        assert calls == expected
        calls.clear()
        cancel.set()
        with pytest.raises(InterruptedError):
            backend.command({'op': 'arm', 'command': command}, cancel, deadline)
        assert calls == []
        cancel.clear()
        with pytest.raises(TimeoutError):
            backend.command({'op': 'arm', 'command': command}, cancel, time.monotonic() - 1)
        assert calls == []
    command = methods('workshop/blockly_ros.py', 'ROSBackend')['command']
    arm = next(n for n in command.body if isinstance(n, ast.If)
               and ast.unparse(n.test) == "row['op'] == 'arm'")
    # Every transition shares the runner cancel/deadline; the existing trigger
    # checks both at dispatch, including between the two teleop calls.
    assert {ast.unparse(n.func) for n in ast.walk(arm) if isinstance(n, ast.Call)} == {
        'cancel.is_set', 'InterruptedError', 'time.monotonic', 'TimeoutError',
        'self.stop', 'self.trigger'}
    assert [ast.unparse(n) for n in ast.walk(arm) if isinstance(n, ast.Call)
            and ast.unparse(n.func) == 'self.trigger'] == [
                "self.trigger('/station/operator/' + command, cancel, deadline)"]


def test_recording_loop_is_cached_ten_hz_processing_only():
    command = methods('tools/run_blockly_station.py', 'Application')['_program_command']
    assert {ast.unparse(n.func) for n in ast.walk(command) if isinstance(n, ast.Call)} == {
        'self.backend.command', 'cancel.is_set', 'InterruptedError', 'time.monotonic',
        'TimeoutError', 'check', 'self.backend.sample', 'Recorder', 'RuntimeError',
        'recorder.add', 'cancel.wait', 'min', 'recorder.finish', 'self._save_recording', 'str', 'abs'}
    assert not any(isinstance(n, (ast.Import, ast.ImportFrom, ast.Await, ast.Yield))
                   for n in ast.walk(command))
    loop, = [n for n in ast.walk(command) if isinstance(n, ast.While)]
    assert {ast.unparse(n.func) for n in ast.walk(loop) if isinstance(n, ast.Call)} == {
        'check', 'RuntimeError', 'recorder.add', 'time.monotonic', 'cancel.wait',
        'min', 'self.backend.sample'}
    waits = [n for n in ast.walk(loop) if isinstance(n, ast.Call)
             and ast.unparse(n.func) == 'cancel.wait']
    assert len(waits) == 1 and ast.unparse(waits[0]) == 'cancel.wait(min(0.1, remaining))'
    assert sum(isinstance(n, ast.Call) and ast.unparse(n.func) == 'recorder.add'
               for n in ast.walk(loop)) == 1
    assert not any(isinstance(n, (ast.ListComp, ast.DictComp, ast.SetComp,
                                  ast.Import, ast.ImportFrom, ast.For)) for n in ast.walk(loop))
    # Persistence/timing materialization stays after bounded collection, outside the loop.
    assert not any(isinstance(n, ast.Attribute) and n.attr in {'trigger', 'command', 'identity',
                                                               'publish', 'write', 'dump'}
                   for n in ast.walk(loop))


def test_program_recording_duration_remains_bounded():
    for seconds in (1, 30):
        validate_program([{'op': 'record_stroke', 'name': 'bounded', 'seconds': seconds}])
    for seconds in (0, .99, 30.1, float('inf'), True):
        with pytest.raises(ValueError):
            validate_program([{'op': 'record_stroke', 'name': 'bounded', 'seconds': seconds}])


def test_preflight_uses_names_without_library_payload_or_iteration():
    from tools.run_blockly_station import Application

    class NamesOnly(dict):
        def __getitem__(self, key):
            raise AssertionError('recording payload read')

        def __iter__(self):
            raise AssertionError('recording library traversed')

    app = Application.__new__(Application)
    app.recordings = NamesOnly(known=object())
    app.backend = SimpleNamespace(physical=True)
    app.instructor = True
    app._preflight_program([{'op': 'replay', 'name': 'known', 'speed': .5}] * 100)
    helper = methods('tools/run_blockly_station.py', 'Application')['_preflight_program']
    assert {ast.unparse(n.func) for n in ast.walk(helper) if isinstance(n, ast.Call)} == {
        'set', 'walk', 'ValueError', 'names.add', 'RuntimeError', 'len',
        "(self.directory / (row['name'] + '.json')).exists"}
    for node in ast.walk(helper):
        if isinstance(node, ast.For):
            assert ast.unparse(node.iter) == 'commands'
    run = methods('tools/run_blockly_station.py', 'Application')['run']
    locked = next(n for n in run.body if isinstance(n, ast.With))
    admission = locked.body[1]
    assert ast.unparse(admission) == 'self._preflight_program(program)'
    assert ast.unparse(locked.body[2]) == 'self._check_recordings(program)'
    for node in ast.walk(run):
        if isinstance(node, ast.Call) and ast.unparse(node.func) in {
                'threading.Event', 'threading.Thread', 'secrets.token_hex', 'self.thread.start'}:
            assert node.lineno > admission.end_lineno
