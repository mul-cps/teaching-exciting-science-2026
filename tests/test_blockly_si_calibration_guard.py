"""Persistent endpoint/cost fences for cold Blockly calibration metadata."""
import ast
import hashlib
import json

import pytest

from blockly_si_calibration_benchmark import SOURCE, ROOT, ENDPOINT, load_source, fixture, response


def digest(node):
    # Normalize empty fields consistently across Python 3.12/3.13.
    def fields(value):
        if isinstance(value, ast.AST):
            return [type(value).__name__, {key: fields(item) for key, item in ast.iter_fields(value)
                                           if item is not None and item != []}]
        if isinstance(value, list): return [fields(item) for item in value]
        return value
    return hashlib.sha256(json.dumps(fields(node), sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def test_parser_bounded_local_and_refresh_uses_only_existing_readonly_client():
    tree = ast.parse(SOURCE.read_text())
    parser = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'parse_gripper_calibration_status')
    assert {ast.unparse(n.func) for n in ast.walk(parser) if isinstance(n, ast.Call)} == {
        'RuntimeError', 'ValueError', 'any', 'data.get', 'isinstance', 'json.loads', 'len', 'type'}
    assert not any(isinstance(n, (ast.Import, ast.ImportFrom, ast.While, ast.Await, ast.Yield))
                   for n in ast.walk(parser))
    # Covers the pre-decode8192-character bound, bool/units/proxy/success
    # relation, duplicate-field rejection and controlled recursion rejection.
    # Its one duplicate-key loop is bounded by the cold JSON input size.
    assert digest(parser) == '6a38cf3b584fa240c4d6639ca6ae71a561ebfbb66151e63bdac8bff0eef8c59a'
    refresh = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                   and n.name == '_refresh_calibration')
    assert digest(refresh) == '5e94d384dc1c932daef76477953a54a9e99fba472cc3869c3b6125755a0eddd3'
    # Pins current-future identity, cancellation and original one-second age
    # checks before and after parsing; no retry/progress/publisher was added.


def test_exception_is_one_readonly_endpoint_and_other_backend_code_unchanged():
    cls = next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n, ast.ClassDef))
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    service = methods['_service']
    exceptions = [n for n in service.body if isinstance(n, ast.If) and ast.unparse(n.test) ==
                  "endpoint == '/follower/gripper_calibration_status'"]
    assert len(exceptions) == 1
    exception = exceptions[0]
    assert len(exception.body) == 1 and ast.unparse(exception.body[0]) == (
        'return parse_gripper_calibration_status(result)')
    index = service.body.index(exception)
    assert ast.unparse(service.body[index+1].test) == 'not result.success'
    service.body.remove(exception)
    # Normalize exactly the reviewed strict SI boolean and cache-age additions.
    command = methods['command']
    gate = next(n for n in ast.walk(command) if isinstance(n, ast.If) and ast.unparse(n.test) ==
                "status.get('si_gap_calibrated') is not True")
    gate.test = ast.parse("not status.get('si_gap_calibrated')", mode='eval').body
    status = methods['status']
    new_age = next(n for n in ast.walk(status) if isinstance(n, ast.Compare) and ast.unparse(n) ==
                   "0 <= jaw.get('age_s', 99) <= 2")
    for node in ast.walk(status):
        if isinstance(node, ast.BoolOp):
            node.values[:] = [ast.parse("jaw.get('age_s',99)<2", mode='eval').body
                              if value is new_age else value for value in node.values]
    methods['_refresh_calibration'].body = [ast.Pass()]
    # Reviewed 2026-10-22: baseline b5d2a06 plus cold ARM allowlist dispatch
    # and explicit record_stroke rejection; every other backend method is unchanged.
    # Baseline25dfc5e with only refresh body removed. All action admission,
    # metric conversion, generic success rejection, joint callback, timers,
    # clients/publishers, cancellation, STOP and recovery code stays pinned.
    assert digest(cls) == '44f1ce7bc44412d3c8e55233fc39f5ceca7e9bf252a693a3a297beaa5de15a94'


@pytest.mark.parametrize('name,expected', [
    ('_gripper_calibration_status', 'e712a8731212a9ecce64ae5ef4892eea310e32259442ccd809c22ee93ec9d1de'),
    ('_moveit_gripper_gap', 'aa0ce1bc59f241a8ac6a17faecd011b78bd5a185bf686223cb51547c113ba65c'),
    ('_moveit_gripper_goal_request', '8de2775734fafd6c70238739e040bb1c124f52306e1ca1f1a561124a7b26d94e')])
def test_native_combined_status_and_angular_guards_unchanged(name, expected):
    path = ROOT/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py'
    node = next(n for n in ast.walk(ast.parse(path.read_text()))
                if isinstance(n, ast.FunctionDef) and n.name == name)
    assert digest(node) == expected


def test_negative_trigger_exception_does_not_spread_to_other_endpoints():
    module = load_source(SOURCE.read_text())
    obj = fixture(module, response('si_only'))
    assert obj.trigger(ENDPOINT)['si_gap_calibrated'] is True
    for endpoint in ('/station/operator/moveit', '/station/ev3/eject', '/follower/status', '/station/status'):
        obj.clients[endpoint] = obj.calibration_client
        with pytest.raises(RuntimeError): obj.trigger(endpoint)
