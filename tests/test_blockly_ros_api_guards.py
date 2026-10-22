"""Static transport cost/safety guards. These tests import no ROS runtime."""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from tools.run_blockly_station import strict_json
from workshop.blockly_ros_api import FIELDS, MAX_JSON_BYTES, ROSProgramAPI

ROOT = Path(__file__).resolve().parents[1]


def test_ros_transport_keeps_exact_bounded_operation_allowlist():
    assert MAX_JSON_BYTES == 64 * 1024
    assert FIELDS == {'status': (), 'run': ('program',), 'record_start': ('name',),
                      'record_stop': (), 'recordings': (), 'stop': (), 'recover': (),
                      'operator': ('command',)}
    tree = ast.parse((ROOT / 'workshop/blockly_ros_api.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    forbidden = {'eval', 'exec', 'compile', 'open', '__import__', 'read_text', 'read_bytes',
                 'write_text', 'write_bytes', 'system', 'popen', 'Popen', 'run', 'call'}
    # Application.run is the one intentional program admission boundary.
    for node in calls:
        name = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, 'attr', '')
        if name == 'run':
            assert ast.unparse(node.func) == 'self.app.run'
        else:
            assert name not in forbidden, ast.unparse(node)
    dispatch = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == 'dispatch')
    bounds = [node for node in ast.walk(dispatch) if isinstance(node, ast.Compare)
              and any(isinstance(part, ast.Name) and part.id == 'MAX_JSON_BYTES' for part in ast.walk(node))]
    assert len(bounds) == 2
    assert all(any(isinstance(part, ast.Call) and isinstance(part.func, ast.Attribute)
                   and part.func.attr == 'encode' for part in ast.walk(bound)) for bound in bounds)
    getters = [node for node in calls if isinstance(node.func, ast.Name) and node.func.id == 'getattr']
    assert len(getters) == 1 and ast.unparse(getters[0]) == 'getattr(self.app, operation)'
    assert any(isinstance(node, ast.Compare) and ast.unparse(node) == 'operation not in FIELDS'
               for node in ast.walk(dispatch))


def test_backend_feedback_callbacks_remain_cache_only():
    tree = ast.parse((ROOT / 'workshop/blockly_ros.py').read_text())
    backend = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'ROSBackend')
    callbacks = [node for node in backend.body if isinstance(node, ast.FunctionDef)
                 and node.name in {'_status_message', '_joints'}]
    assert len(callbacks) == 2
    forbidden = {'dispatch', 'run', 'stop', 'operator', 'recover', '_wait', 'wait', 'sleep',
                 'spin_until_future_complete', 'open', 'read_text', 'write_text', 'Thread'}
    for callback in callbacks:
        for node in ast.walk(callback):
            if isinstance(node, ast.Call):
                name = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, 'attr', '')
                assert name not in forbidden, (callback.name, ast.unparse(node))
    assert not any(isinstance(node, (ast.Import, ast.ImportFrom))
                   and 'blockly_ros_api' in ast.unparse(node) for node in ast.walk(tree))


def endpoint(app):
    api = ROSProgramAPI.__new__(ROSProgramAPI)
    api.app = app; api.strict_json = strict_json; api.closed = False
    api.slots = threading.BoundedSemaphore(2)
    api.state_lock = threading.Lock(); api.active = 0; api.closing = threading.Event()
    return api


def test_executor_capacity_exceeds_nonstop_admission_and_stop_bypasses_it():
    tree = ast.parse((ROOT / 'workshop/blockly_ros_api.py').read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    executor = next(node for node in calls if isinstance(node.func, ast.Name) and node.func.id == 'MultiThreadedExecutor')
    workers = next(keyword.value.value for keyword in executor.keywords if keyword.arg == 'num_threads')
    semaphore = next(node for node in calls if isinstance(node.func, ast.Attribute) and node.func.attr == 'BoundedSemaphore')
    assert semaphore.args[0].value == 2 and workers == 4
    app = SimpleNamespace(stop=lambda: {'stopped': True}, status=lambda: {'ok': True})
    api = endpoint(app)
    assert api.slots.acquire(blocking=False) and api.slots.acquire(blocking=False)
    try:
        with pytest.raises(RuntimeError, match='busy'): api.dispatch('status', '{}')
        assert api.dispatch('stop', '{}') == '{"stopped":true}'
    finally:
        api.slots.release(); api.slots.release()


def test_exact_utf8_request_and_result_byte_boundaries():
    app = SimpleNamespace(status=lambda: {'ok': True})
    api = endpoint(app)
    assert api.dispatch('status', '{}' + ' ' * (MAX_JSON_BYTES - 2)) == '{"ok":true}'
    with pytest.raises(ValueError, match='65536'):
        api.dispatch('status', '{}' + ' ' * (MAX_JSON_BYTES - 1))
    # A two-byte character must consume two request bytes, not one code point.
    body = '{"name":"' + 'é' * (MAX_JSON_BYTES // 2) + '"}'
    with pytest.raises(ValueError, match='65536'): api.dispatch('record_start', body)
    app.status = lambda: {'x': 'a' * (MAX_JSON_BYTES - 8)}
    assert len(api.dispatch('status', '{}').encode()) == MAX_JSON_BYTES
    app.status = lambda: {'x': 'a' * (MAX_JSON_BYTES - 7)}
    with pytest.raises(ValueError, match='result_json'): api.dispatch('status', '{}')


def test_shutdown_is_bounded_and_backend_is_retained_on_cleanup_failure():
    tree = ast.parse((ROOT / 'workshop/blockly_ros_api.py').read_text())
    close = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == 'close')
    executor_stop = next(node for node in ast.walk(close) if isinstance(node, ast.Call)
                         and isinstance(node.func, ast.Attribute) and ast.unparse(node.func) == 'self.executor.shutdown')
    assert {keyword.arg: ast.literal_eval(keyword.value) for keyword in executor_stop.keywords} == {
        'timeout_sec': 2, 'wait_for_threads': False}
    thread_join = next(node for node in ast.walk(close) if isinstance(node, ast.Call)
                       and isinstance(node.func, ast.Attribute) and ast.unparse(node.func) == 'self.thread.join')
    assert {keyword.arg: ast.literal_eval(keyword.value) for keyword in thread_join.keywords} == {'timeout': 2}
    assert any(isinstance(node, ast.Raise) and 'shutdown incomplete' in ast.unparse(node)
               for node in ast.walk(close))
    close_text = ast.unparse(close)
    assert close_text.index('self.closing.set()') < close_text.index('self.app.stop()')
    assert close_text.index('shutdown incomplete') < close_text.index('self.node.destroy_node()')
    dispatch = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == 'dispatch')
    assert "self.app.operator(args['command'], cancel=self.closing)" in ast.unparse(dispatch)
    main_tree = ast.parse((ROOT / 'tools/run_blockly_station.py').read_text())
    main = next(node for node in main_tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
    cleanup = next(node for node in main.body if isinstance(node, ast.Try)).finalbody
    expressions = [ast.unparse(node) for node in cleanup]
    assert expressions[-2:] == ['if ros_api is not None:\n    ros_api.close()', 'app.close()']
