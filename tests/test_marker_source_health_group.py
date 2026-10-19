"""Health-only grouping and isolated real executor concurrency; never live ROS."""
import ast
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import pytest
from test_derived_source_binding_health import prepared,SOURCE

MARKER=SOURCE.with_name('marker_pipeline.py')


def require_isolated_domain():
    assert os.environ.get('ROS_DOMAIN_ID')=='177','requires explicit isolated domain177'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1','requires localhost1'


def test_wrong_domain_fails_before_ros_init(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','76');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    with pytest.raises(AssertionError):require_isolated_domain()


def test_only_health_service_opt_in_existing_intake_group():
    tree=ast.parse(SOURCE.read_text());cls=next(x for x in tree.body if isinstance(x,ast.ClassDef))
    init=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='__init__')
    kwargs=dict(zip([x.arg for x in init.args.kwonlyargs],init.args.kw_defaults))
    assert isinstance(kwargs['health_callback_group'],ast.Constant) and kwargs['health_callback_group'].value is None
    services={call.args[1].value:call for call in ast.walk(init) if isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute) and call.func.attr=='create_service'}
    for name,call in services.items():
        groups=[k.value for k in call.keywords if k.arg=='callback_group']
        assert len(groups)==(1 if name=='~/derived_source_health' else 0)
        if groups:assert isinstance(groups[0],ast.Name) and groups[0].id=='health_callback_group'
    mt=ast.parse(MARKER.read_text())
    derived=next(c for c in ast.walk(mt) if isinstance(c,ast.Call) and isinstance(c.func,ast.Name) and c.func.id=='DerivedAligned')
    names={k.arg:ast.unparse(k.value) for k in derived.keywords}
    assert names['callback_group']==names['health_callback_group']=='self.intake_group'
    assert MARKER.read_text().count('MutuallyExclusiveCallbackGroup()')==1
    assert 'MultiThreadedExecutor(num_threads=2)' in MARKER.read_text()


def test_actual_health_completes_during_default_cv_and_intake_progresses():
    pytest.importorskip('rclpy');require_isolated_domain()
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from std_srvs.srv import Trigger
    import uuid
    rclpy.init(args=[]);node=Node('health_group_'+uuid.uuid4().hex)
    clientnode=Node('health_client_'+uuid.uuid4().hex);executor=MultiThreadedExecutor(num_threads=2)
    release=threading.Event();entered=threading.Event();intake=threading.Event();calls=[];thread=None
    try:
        obj,*_=prepared();group=MutuallyExclusiveCallbackGroup()
        def cv():entered.set();release.wait(.5)
        timer=node.create_timer(.001,cv)
        def health(req,res):calls.append('health');return obj._source_health(req,res)
        def default(req,res):calls.append('default');res.success=True;return res
        prefix='/'+node.get_name()
        node.create_service(Trigger,prefix+'/health',health,callback_group=group)
        node.create_service(Trigger,prefix+'/image',default)
        node.create_service(Trigger,prefix+'/reset',default)
        intake_timer=node.create_timer(.005,lambda:intake.set(),callback_group=group)
        clients=[clientnode.create_client(Trigger,prefix+'/'+name) for name in ('health','image','reset')]
        for n in (node,clientnode):executor.add_node(n)
        thread=threading.Thread(target=executor.spin);thread.start()
        assert entered.wait(1.);held_started=time.monotonic()
        for client in clients:assert client.wait_for_service(timeout_sec=1.)
        futures=[client.call_async(Trigger.Request()) for client in clients]
        deadline=time.monotonic()+.12
        while not futures[0].done() and time.monotonic()<deadline:time.sleep(.001)
        assert futures[0].done() and futures[0].result().success
        assert intake.wait(.02) and not release.is_set()
        assert calls==['health'] and not futures[1].done() and not futures[2].done()
        timer.cancel()
        time.sleep(max(0.,.1-(time.monotonic()-held_started)))
        assert not release.is_set()
        release.set()
        deadline=time.monotonic()+1.
        while not all(f.done() for f in futures) and time.monotonic()<deadline:time.sleep(.001)
        assert all(f.done() for f in futures)
    finally:
        release.set();executor.shutdown(timeout_sec=1.);thread.join(1.) if thread else None
        node.destroy_node();clientnode.destroy_node();rclpy.shutdown()
