"""Metadata-only notification lifecycle; no cameras, GPU or actuation."""
import sys,time,threading
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_objects'))
from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker


def test_result_notify_is_after_storage_and_outside_worker_lock():
    done=threading.Event();seen=[]
    def notify():
        assert not worker._condition._is_owned()
        seen.append(worker.take_result())
        done.set()
    worker=LatestOnlyAnalysisWorker(lambda item:item+1,on_ready=notify)
    try:
        worker.submit(1);assert done.wait(.5)
        assert seen==[(1,2,None)]
    finally:assert worker.close(.5)


def test_notify_error_is_bounded_latched_and_not_retried():
    done=threading.Event();calls=[]
    def fail():calls.append(1);done.set();raise RuntimeError('notification failed'+'x'*2000)
    worker=LatestOnlyAnalysisWorker(lambda item:item,on_ready=fail)
    try:
        worker.submit(1);assert done.wait(.5)
        deadline=time.monotonic()+.5
        while not worker.notification_error and time.monotonic()<deadline:time.sleep(.001)
        assert 'notification failed' in worker.notification_error
        assert len(worker.notification_error)<=512
        assert worker.take_result()==(1,1,None)
        worker.submit(2)
        deadline=time.monotonic()+.5;result=None
        while result is None and time.monotonic()<deadline:
            result=worker.take_result()
            if result is None:time.sleep(.001)
        assert result==(2,2,None) and calls==[1]
    finally:assert worker.close(.5)


def test_close_prevents_late_notification_after_slow_analysis():
    entered,release,notified=threading.Event(),threading.Event(),threading.Event()
    def analyze(item):entered.set();assert release.wait(1);return item
    worker=LatestOnlyAnalysisWorker(analyze,on_ready=notified.set)
    try:
        worker.submit(1);assert entered.wait(.5)
        assert worker.close(.01) is False
        release.set();assert worker.close(.5)
        assert not notified.is_set() and worker.take_result() is None
    finally:release.set();worker.close(.5)


def test_close_reports_running_notification_before_guard_can_be_destroyed():
    entered,release=threading.Event(),threading.Event()
    def notify():entered.set();assert release.wait(1)
    worker=LatestOnlyAnalysisWorker(lambda item:item,on_ready=notify)
    try:
        worker.submit(1);assert entered.wait(.5)
        start=time.monotonic();assert worker.close(.01) is False
        assert time.monotonic()-start<.1
        release.set();assert worker.close(.5)
    finally:release.set();worker.close(.5)


def test_on_ready_must_be_callable_before_thread_creation():
    with pytest.raises(TypeError,match='on_ready'):LatestOnlyAnalysisWorker(lambda item:item,on_ready=3)


def test_notification_path_has_no_geometry_io_or_per_item_work():
    import ast,inspect,textwrap
    tree=ast.parse(textwrap.dedent(inspect.getsource(LatestOnlyAnalysisWorker._notify_result)))
    assert not any(isinstance(n,(ast.For,ast.While,ast.ListComp,ast.DictComp,ast.SetComp,ast.GeneratorExp)) for n in ast.walk(tree))
    names={n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
    assert not names & {'copy','copy_','cuda','to','numpy','sleep','analyze','read','write','publish','submit'}
    assert 'callback()' in inspect.getsource(LatestOnlyAnalysisWorker._notify_result)


def test_result_guard_keeps_timer_for_cold_faults_and_default_group():
    import ast
    path=Path(__file__).resolve().parents[1]/'ros2/src/so101_station_objects/so101_station_objects/objects.py'
    tree=ast.parse(path.read_text())
    guards=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_guard_condition']
    assert len(guards)==1 and guards[0].args[0].attr=='_consume_analysis'
    assert not guards[0].keywords  # Same default mutually-exclusive group as tracker callbacks.
    timers=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_timer' and isinstance(n.args[0],ast.Constant) and n.args[0].value==.05]
    assert len(timers)==1 and timers[0].args[1].attr=='_consume_analysis_faults'
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_consume_analysis_faults')
    take=next(n for n in ast.walk(method) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='take_result')
    failure=next(n for n in method.body if isinstance(n,ast.If) and isinstance(n.test,ast.Compare) and isinstance(n.test.left,ast.Name) and n.test.left.id=='error')
    assert take in list(ast.walk(failure))  # Only failed wakeup discards; no routine result polling.


@pytest.mark.parametrize('domain,localhost',[('76','1'),('0','1'),('177','0'),('177',None)])
def test_native_fixture_domain_guard_fails_before_any_ros_init(monkeypatch,domain,localhost):
    from test_object_result_delivery_ros import _require_isolated_domain
    monkeypatch.setenv('ROS_DOMAIN_ID',domain)
    if localhost is None:monkeypatch.delenv('ROS_LOCALHOST_ONLY',raising=False)
    else:monkeypatch.setenv('ROS_LOCALHOST_ONLY',localhost)
    with pytest.raises(RuntimeError,match='DDS177 localhost1'):_require_isolated_domain()


def test_native_fixture_guard_precedes_ros_init_static():
    import ast
    path=Path(__file__).with_name('test_object_result_delivery_ros.py')
    tree=ast.parse(path.read_text())
    fixture=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='node')
    calls=[n for n in ast.walk(fixture) if isinstance(n,ast.Call)]
    guard=next(n for n in calls if isinstance(n.func,ast.Name) and n.func.id=='_require_isolated_domain')
    init=next(n for n in calls if isinstance(n.func,ast.Attribute) and n.func.attr=='init')
    assert guard.lineno < init.lineno
