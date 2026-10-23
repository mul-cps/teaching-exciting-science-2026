"""ROS transport fixtures only: no ROS services, devices or context startup."""
import json
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from types import ModuleType, SimpleNamespace

import pytest

from tools.run_blockly_station import Application, strict_json
from workshop.blockly_ros_api import MAX_JSON_BYTES, ROSProgramAPI, SERVICE


@pytest.fixture
def fake_ros(monkeypatch):
    state = SimpleNamespace(shutdown=[], nodes=[], executors=[], fail=False)

    class Context:
        def ok(self): return True

    class Node:
        def __init__(self, name, *, context):
            if state.fail: raise RuntimeError('startup failed')
            self.context = context; self.destroyed = False
            state.nodes.append(self)
        def create_service(self, kind, name, callback, *, callback_group):
            self.service = (kind, name, callback, callback_group)
        def destroy_node(self): self.destroyed = True

    class Executor:
        def __init__(self, *, num_threads, context):
            self.num_threads = num_threads; self.context = context
            self.done = threading.Event(); state.executors.append(self)
        def add_node(self, node): self.node = node
        def spin(self): self.done.wait()
        def shutdown(self, **kwargs): self.done.set()

    def module(name, **values):
        obj = ModuleType(name); obj.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, obj)
    module('rclpy', init=lambda **kw: None, shutdown=lambda **kw: state.shutdown.append(kw))
    module('rclpy.node', Node=Node)
    module('rclpy.context', Context=Context)
    module('rclpy.callback_groups', ReentrantCallbackGroup=type('ReentrantCallbackGroup', (), {}))
    module('rclpy.executors', MultiThreadedExecutor=Executor,
           ExternalShutdownException=type('ExternalShutdownException', (Exception,), {}),
           ShutdownException=type('ShutdownException', (Exception,), {}))
    module('so101_station_workflow', __path__=[])
    module('so101_station_workflow.srv', StationCommand=type('StationCommand', (), {}))
    return state


@pytest.fixture
def api(tmp_path, fake_ros):
    app = Application(tmp_path)
    endpoint = ROSProgramAPI(app, strict_json)
    yield endpoint
    endpoint.close(); app.close()


def command(api, operation, arguments='{}'):
    return api.command(SimpleNamespace(operation=operation, arguments_json=arguments), SimpleNamespace())


def test_shared_application_and_recording_library(api, fake_ros):
    assert fake_ros.nodes[0].service[1] == SERVICE
    assert fake_ros.executors[0].num_threads == 4
    assert json.loads(command(api, 'recordings').result_json) == {'names': ['demo']}
    result = command(api, 'run', json.dumps({'program':[{'op':'log','message':'hello'}],'admission':api.app.status()['admission']}))
    assert result.success, result.message
    api.app.thread.join(2)
    assert json.loads(command(api, 'status').result_json)['job'] == api.app.status()['job']


@pytest.mark.parametrize('operation,arguments', [
    ('__getattribute__', '{}'), ('run', '[]'), ('status', '{"extra":1}'),
    ('run', '{"program":[],"program":[]}'), ('run', '{"program":NaN}'), ('run', '{"program":1e400}'),
    ('status', ' ' * (MAX_JSON_BYTES + 1)), ('operator', '{"command":5}'),
    ('record_start', '{"name":null}'), ('status', '{'),
])
def test_malformed_or_forbidden_requests(api, operation, arguments):
    response = command(api, operation, arguments)
    assert not response.success
    assert response.result_json == '{}'
    assert api.app.thread is None


def test_noninstructor_reuses_application_gate(api):
    api.app.backend.physical = True
    try:
        response = command(api, 'operator', json.dumps({'command':'zero_torque','admission':api.app.status()['admission']}))
        assert not response.success
        assert 'instructor' in response.message
        response = command(api, 'recover',json.dumps({'admission':api.app.status()['admission']}))
        assert not response.success
        assert api.app.operator_busy is False
    finally:
        api.app.backend.physical = False


def test_busy_and_stop_progress_during_waiting_operator(api):
    api.app.backend.physical = True; api.app.instructor = True
    entered = threading.Event(); release = threading.Event(); stopped = threading.Event()
    api.app.backend.instructor = lambda _, **kwargs: (entered.set(), release.wait(2), {'ok': True})[-1]
    api.app.backend.stop = lambda: (stopped.set(), {'stopped': True})[-1]
    try:
        with ThreadPoolExecutor(max_workers=4) as pool:
            pending = pool.submit(command, api, 'operator', json.dumps({'command':'zero_torque','admission':api.app.status()['admission']}))
            assert entered.wait(1)
            busy = command(api, 'run', json.dumps({'program':[{'op':'wait','seconds':0.1}],'admission':api.app.status()['admission']}))
            assert not busy.success and 'active' in busy.message
            stop = pool.submit(command, api, 'stop')
            assert stop.result(timeout=1).success
            assert stopped.is_set() and not pending.done()
            release.set()
            assert not pending.result(timeout=1).success
    finally:
        release.set(); api.app.backend.physical = False


def test_transport_capacity_and_result_bound(api):
    api.slots.acquire(); api.slots.acquire()
    try:
        assert not command(api, 'status').success
        assert command(api, 'stop').success
    finally:
        api.slots.release(); api.slots.release()
    api.app.status = lambda: {'large': 'x' * MAX_JSON_BYTES}
    assert not command(api, 'status').success
    api.app.status = lambda: {'bad': float('inf')}
    assert not command(api, 'status').success


def test_close_preserves_backend_context(tmp_path, fake_ros):
    app = Application(tmp_path)
    context = object()
    app.backend.physical = True; app.backend.node = SimpleNamespace(context=context)
    endpoint = ROSProgramAPI(app, strict_json)
    assert endpoint.app is app
    assert endpoint.node.context is context
    endpoint.close()
    assert not endpoint.thread.is_alive()
    assert fake_ros.shutdown == []
    assert not command(endpoint, 'stop').success
    app.backend.physical = False; app.close()


def test_failed_startup_closes_owned_context(tmp_path, fake_ros):
    app = Application(tmp_path); fake_ros.fail = True
    with pytest.raises(RuntimeError, match='startup failed'):
        ROSProgramAPI(app, strict_json)
    assert len(fake_ros.shutdown) == 1
    app.close()


def test_main_closes_api_before_application_after_http_startup_failure(monkeypatch, tmp_path):
    import tools.run_blockly_station as station
    import workshop.blockly_ros_api as transport
    events = []
    app = SimpleNamespace(close=lambda: events.append('app.close'))
    class Endpoint:
        def __init__(self, supplied, parser):
            assert supplied is app and parser is strict_json
            events.append('api.start')
        def close(self): events.append('api.close')
    monkeypatch.setattr(station, 'Application', lambda *a, **kw: app)
    monkeypatch.setattr(transport, 'ROSProgramAPI', Endpoint)
    def fail(*args, **kwargs): raise OSError('port busy')
    monkeypatch.setattr(station, 'make_server', fail)
    monkeypatch.setattr(sys, 'argv', ['station', '--ros-api', '--recordings', str(tmp_path)])
    with pytest.raises(OSError, match='port busy'):
        station.main()
    assert events == ['api.start', 'api.close', 'app.close']


@pytest.mark.parametrize('body', ['1e400', '-1e400', '{"value":1e400}'])
def test_shared_json_parser_rejects_overflow(body):
    with pytest.raises(ValueError, match='nonfinite'):
        strict_json(body)


def test_active_operator_close_stops_and_preserves_backend_until_callback_finishes(api, fake_ros):
    entered = threading.Event(); release = threading.Event(); stops = []
    api.app.backend.physical = True; api.app.instructor = True
    api.app.backend.instructor = lambda _, **kwargs: (entered.set(), release.wait(2), {'ok': True})[-1]
    api.app.backend.stop = lambda: (stops.append('STOP'), {'stopped': True})[-1]
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(command, api, 'operator', json.dumps({'command':'zero_torque','admission':api.app.status()['admission']}))
            assert entered.wait(1)
            with pytest.raises(RuntimeError, match='retain application ownership'):
                api.close()
            assert stops == ['STOP']
            assert not fake_ros.nodes[0].destroyed
            assert fake_ros.shutdown == []
            assert not command(api, 'stop').success
            release.set()
            assert not pending.result(timeout=1).success
        api.close()
        assert fake_ros.nodes[0].destroyed
    finally:
        release.set(); api.app.backend.physical = False


def test_main_retains_application_on_incomplete_api_close(monkeypatch, tmp_path):
    import tools.run_blockly_station as station
    import workshop.blockly_ros_api as transport
    events = []
    app = SimpleNamespace(close=lambda: events.append('app.close'))
    class Endpoint:
        def __init__(self, *args): pass
        def close(self): raise RuntimeError('program API shutdown incomplete')
    monkeypatch.setattr(station, 'Application', lambda *a, **kw: app)
    monkeypatch.setattr(transport, 'ROSProgramAPI', Endpoint)
    def fail(*args, **kwargs): raise OSError('port busy')
    monkeypatch.setattr(station, 'make_server', fail)
    monkeypatch.setattr(sys, 'argv', ['station', '--ros-api', '--recordings', str(tmp_path)])
    with pytest.raises(RuntimeError, match='shutdown incomplete'):
        station.main()
    assert events == []


def test_operator_cancellation_combines_application_stop_and_transport_shutdown(api):
    from workshop.blockly_ros import ROSBackend
    api.app.backend.physical = True; api.app.instructor = True
    calls = []
    api.app.backend.instructor = lambda command, **kwargs: (calls.append((command, kwargs)), {'ok': True})[-1]
    try:
        api.app.operator('zero_torque')
        assert calls[-1][0]=='zero_torque'
        assert calls[-1][1]['cancel'].local is api.app.control_cancel
        assert calls[-1][1]['cancel'].parent is None
        assert command(api, 'operator', json.dumps({'command':'zero_torque','admission':api.app.status()['admission']})).success
        assert calls[-1][0]=='zero_torque'
        assert calls[-1][1]['cancel'].local is api.app.control_cancel
        assert calls[-1][1]['cancel'].parent is api.closing
        backend = ROSBackend.__new__(ROSBackend)
        backend.trigger = lambda *args, **kwargs: (calls.append((args, kwargs)), {'ok': True})[-1]
        backend.instructor('zero_torque')
        assert 'cancel' not in calls[-1][1]
        backend.instructor('zero_torque', cancel=api.closing)
        assert calls[-1][1]['cancel'] is api.closing
    finally:
        api.app.backend.physical = False


def test_delayed_run_callback_behind_stop_is_rejected(api):
    stamp=api.app.status()['admission']
    entered=threading.Event();release=threading.Event();original=api.app.run_external
    def delayed(program,admission):
        entered.set();assert release.wait(2)
        return original(program,admission)
    api.app.run_external=delayed
    with ThreadPoolExecutor(max_workers=2) as pool:
        late=pool.submit(command,api,'run',json.dumps({'program':[{'op':'log','message':'must not run'}],'admission':stamp}))
        assert entered.wait(2)
        assert command(api,'stop').success
        release.set();response=late.result(2)
    assert not response.success
    assert 'admission' in response.message
    assert api.app.thread is None
    assert 'must not run' not in api.app.backend.events

@pytest.mark.parametrize('operation,arguments',[
    ('operator',{'command':'start'}),('recover',{}),('record_start',{'name':'move'}),('record_stop',{})])
def test_mutating_ros_controls_require_admission(api,operation,arguments):
    api.app.backend.instructor=lambda *args,**kwargs:pytest.fail('unstamped mutation dispatched')
    assert not command(api,operation,json.dumps(arguments)).success
    arguments['admission']=api.app.status()['admission'];command(api,'stop')
    assert not command(api,operation,json.dumps(arguments)).success
