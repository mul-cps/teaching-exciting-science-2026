"""Keep recording admission local and ahead of any program dispatch."""
import ast
from pathlib import Path

from tools.run_blockly_station import Application


SOURCE = Path(__file__).resolve().parents[1] / 'tools/run_blockly_station.py'


def methods():
    application = next(node for node in ast.parse(SOURCE.read_text()).body
                       if isinstance(node, ast.ClassDef) and node.name == 'Application')
    return {node.name: node for node in application.body if isinstance(node, ast.FunctionDef)}


def test_recording_preflight_only_walks_local_names():
    helper = methods()['_check_recordings']
    allowed_names = {'self', 'rows', 'missing', 'set', 'walk', 'commands', 'row',
                     'ValueError', 'sorted'}
    assert {node.id for node in ast.walk(helper) if isinstance(node, ast.Name)} <= allowed_names
    assert not any(isinstance(node, (ast.Import, ast.ImportFrom, ast.Await,
                                    ast.Yield, ast.YieldFrom, ast.While)) for node in ast.walk(helper))
    for node in ast.walk(helper):
        if isinstance(node, ast.Attribute):
            assert ast.unparse(node) in {'self.recordings', 'missing.add', "', '.join"}
            assert isinstance(node.ctx, ast.Load)
        if isinstance(node, ast.Call):
            assert ast.unparse(node.func) in {'set', 'walk', 'missing.add', 'sorted',
                                             'ValueError', "', '.join"}
        if isinstance(node, ast.For):
            assert ast.unparse(node.target) == 'row' and ast.unparse(node.iter) == 'commands'


def test_recording_preflight_precedes_job_event_and_thread_creation():
    run = methods()['run']
    assert ast.unparse(run.body[0]) == 'validate_program(program)'
    locked = next(node for node in run.body if isinstance(node, ast.With))
    assert ast.unparse(locked.items[0].context_expr) == 'self.lock'
    assert [ast.unparse(node) for node in locked.body[:2]] == [
        'self.idle()', 'self._check_recordings(program)']
    admission = locked.body[1]
    calls = [node for node in ast.walk(run) if isinstance(node, ast.Call)]
    assert sum(ast.unparse(node.func) == 'self._check_recordings' for node in calls) == 1
    for node in ast.walk(run):
        if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store):
            assert node.lineno > admission.end_lineno
        if isinstance(node, ast.Call) and ast.unparse(node.func) in {
                'threading.Event', 'threading.Thread', 'secrets.token_hex'}:
            assert node.lineno > admission.end_lineno


def test_recording_preflight_does_not_read_recording_payload_or_backend():
    class NamesOnly(dict):
        def __getitem__(self, key):
            raise AssertionError('recording payload read')

        def __iter__(self):
            raise AssertionError('recording library traversed')

    application = Application.__new__(Application)
    application.recordings = NamesOnly(known=object())
    replay = {'op': 'replay', 'name': 'known', 'speed': .5}
    application._check_recordings([{'op': 'repeat', 'count': 5, 'body': [
        {'op': 'if_color', 'color': 'red', 'then': [replay], 'else': [replay]}]}])
