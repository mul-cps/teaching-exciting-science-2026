"""STOP/recovery and inherited parent lifetime reject false taught completion."""
import ast
import os
from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace
import pytest
from workshop.station.taught_execution import TaughtSession

ROOT=Path(__file__).resolve().parents[1]


def session(child):
    obj=TaughtSession.__new__(TaughtSession)
    obj.lock=threading.Lock();obj.stop_lock=threading.Lock();obj.child_stop_requested=False;obj.child=child;obj.stopped=False
    obj.shutdown_error=None;obj.terminated=False;obj.lifetime_w=None
    return obj


class Child:
    def __init__(self,*,kill_works=True): self.code=None;self.signals=[];self.kill_works=kill_works
    def poll(self): return self.code
    def send_signal(self,value): self.signals.append(value)
    def wait(self,timeout):
        if self.code is None: raise subprocess.TimeoutExpired('fixture',timeout)
        return self.code
    def kill(self):
        if self.kill_works: self.code=-9


def test_failed_stop_retains_error_until_explicit_proven_terminal_recovery():
    obj=session(Child())
    with pytest.raises(RuntimeError,match='forced down'): obj.stop()
    with pytest.raises(RuntimeError,match='forced down'): obj.stop()
    assert obj.child.poll() is not None
    obj.reconcile_terminated()
    obj.stop()
    assert obj.shutdown_error is None


def test_recovery_cannot_clear_live_unknown_producer():
    obj=session(Child(kill_works=False));obj.shutdown_error='unknown'
    with pytest.raises(RuntimeError,match='still unresolved'): obj.reconcile_terminated()
    assert obj.shutdown_error=='unknown'


def lifetime_function(fd):
    tree=ast.parse((ROOT/'tools/ros_plan_grasp.py').read_text())
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='check_taught_lifetime')
    namespace={'os':os,'args':SimpleNamespace(taught_lifetime_fd=fd),'cancellation_cleanup':False}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),'<lifetime>','exec'),namespace)
    return namespace


def test_parent_eof_before_goal_and_during_phase():
    r,w=os.pipe();os.set_blocking(r,False)
    try:
        ns=lifetime_function(r);check=ns['check_taught_lifetime']
        check() # healthy parent owns writer
        os.close(w);w=None
        with pytest.raises(InterruptedError,match='lifetime ended'): check()
        # Cancellation cleanup must retain ROS context long enough to reach terminal.
        ns['cancellation_cleanup']=True;check()
        ns['cancellation_cleanup']=False
        with pytest.raises(InterruptedError): check()
    finally:
        os.close(r)
        if w is not None:os.close(w)


def test_lifetime_is_checked_before_send_and_inside_result_poll():
    tree=ast.parse((ROOT/'tools/ros_plan_grasp.py').read_text())
    functions={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    assert ast.unparse(functions['send_action_goal'].body[0])=='check_taught_lifetime()'
    loop=functions['spin_future'].body[0]
    assert any(isinstance(n,ast.Call) and ast.unparse(n.func)=='check_taught_lifetime' for n in ast.walk(loop))
    assert any(isinstance(n,ast.Try) and n.finalbody for n in ast.walk(functions['cancel_active_goal']))


def test_observer_bounds_invalid_payloads():
    obj=session(None);obj.status=None;obj.received=0
    for value in ('x'*32769,'[]','{}','{"state":"tracking","objects":[],"grasp_candidates":[{}]}'):
        obj.observe(SimpleNamespace(data=value))
    # A small tracking snapshot is retained; selector still requires actual target schema.
    assert obj.status is not None
    previous=obj.status
    obj.observe(SimpleNamespace(data='x'*32769))
    assert obj.status is previous


def test_taught_stop_sends_native_stop_before_waiting_for_child():
    from workshop.blockly_ros import ROSBackend
    events=[]
    class Producer:
        def request_stop(self): events.append('producer_request')
        def wait_stopped(self): events.append('producer_wait');raise RuntimeError('hung producer')
    obj=ROSBackend.__new__(ROSBackend);obj.taught_session=Producer()
    obj.action_lock=threading.Lock();obj.active_handle=None
    def trigger(endpoint,**kwargs):
        events.append(endpoint)
        if endpoint=='/station/stop': return {'request_id':'exact'}
        return {'stop':{'request_id':'exact','pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    obj.trigger=trigger
    with pytest.raises(RuntimeError,match='producer termination unresolved'): obj._stop()
    assert events==['producer_request','/station/stop','producer_wait','/station/status']


def test_concurrent_stop_requests_signal_child_once():
    child=Child();obj=session(child)
    threads=[threading.Thread(target=obj.request_stop) for _ in range(4)]
    for thread in threads:thread.start()
    for thread in threads:thread.join()
    assert len(child.signals)==1
