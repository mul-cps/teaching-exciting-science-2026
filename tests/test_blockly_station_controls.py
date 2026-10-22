"""Bounded typed station controls and explicit verified recovery in programs."""
import threading
import time
import pytest
from workshop.blockly_program import ProgramRunner, SimulationBackend, validate_program
from workshop.blockly_ros import OPERATOR_COMMANDS, ROSBackend
from tools.run_blockly_station import Application

RECOVERY=frozenset({'reinitialize','verify_recovery'})
SAFE=frozenset({'reinitialize','verify_recovery','emergency_reset','reset','zero_torque',
               'restart_moveit','restart_camera','restart_depth','restart_tags','restart_all','verify_restart'})


def station(command): return {'op':'station','command':command}


@pytest.mark.parametrize('command',(*OPERATOR_COMMANDS,*RECOVERY))
def test_every_station_command_validates(command):
    assert validate_program([station(command)])


@pytest.mark.parametrize('row',[station('shell'),station('recover'),station(True),station([]),
    {'op':'station'}, dict(station('reset'),extra=1)])
def test_station_fields_and_enum_strict(row):
    with pytest.raises(ValueError): validate_program([row])


@pytest.mark.parametrize('command',(*OPERATOR_COMMANDS,*RECOVERY))
def test_station_backend_exact_rpc_mapping_and_context(command):
    backend=ROSBackend.__new__(ROSBackend);calls=[];cancel=threading.Event();deadline=time.monotonic()+60
    backend.trigger=lambda endpoint,event,end:calls.append((endpoint,event,end)) or {'acknowledged':True}
    backend.recover=lambda **context:calls.append(('recover',context)) or {'recovered':True}
    result=backend.command(station(command),cancel,deadline)
    if command in RECOVERY:
        expected=[] if command=='verify_recovery' else [('/station/operator/emergency_reset',cancel,deadline)]
        assert calls==expected+[('recover',{'cancel':cancel,'deadline':deadline})]
        assert result=={'recovered':True}
    else:
        assert calls==[('/station/operator/'+command,cancel,deadline)]


@pytest.mark.parametrize('command',(*OPERATOR_COMMANDS,*RECOVERY))
@pytest.mark.parametrize('failure',['cancel','deadline'])
def test_station_pre_dispatch_context_guards(command,failure):
    backend=ROSBackend.__new__(ROSBackend);calls=[]
    backend.trigger=lambda *args:calls.append(args)
    backend.recover=lambda **kwargs:calls.append(kwargs)
    cancel=threading.Event()
    if failure=='cancel':cancel.set()
    deadline=time.monotonic()+1 if failure=='cancel' else time.monotonic()-1
    with pytest.raises(InterruptedError if failure=='cancel' else TimeoutError):backend.command(station(command),cancel,deadline)
    assert calls==[]


@pytest.mark.parametrize('nested',[False,True])
def test_physical_station_instructor_preflight_across_all_branches(tmp_path,nested):
    app=Application(tmp_path);app.backend.physical=True
    row=station('restart_camera')
    if nested:row={'op':'if_color','color':'red','then':[],'else':[{'op':'repeat','count':1,'body':[row]}]}
    with pytest.raises(RuntimeError,match='instructor'):
        app.run([{'op':'ev3','command':'wake'},row])
    assert app.thread is None and app.backend.events==[]


@pytest.mark.parametrize('command',sorted(SAFE))
def test_latched_safe_flat_program_is_admitted(tmp_path,command):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True;app.recovery_required=True
    app.run([{'op':'log','message':'recovery'}, {'op':'wait','seconds':0},station(command)])
    app.thread.join(2)
    assert app.job['state']=='complete',app.job
    assert app.recovery_required is (command not in RECOVERY)


@pytest.mark.parametrize('rows',[
    [{'op':'log','message':'only'}], [{'op':'stop'}], [station('start')], [station('teleop')],
    [station('moveit')], [station('damping')], [{'op':'arm','command':'torque_off'}],
    [{'op':'jaw','gap_mm':0}], [{'op':'ev3','command':'sleep'}],
    [{'op':'record_stroke','name':'bad','seconds':1}], [{'op':'replay','name':'demo','speed':.5}],
    [{'op':'repeat','count':1,'body':[station('verify_recovery')]}],
    [{'op':'if_color','color':'red','then':[station('verify_recovery')],
      'else':[{'op':'arm','command':'home'}]}],
    [station('verify_recovery'),{'op':'ev3','command':'wake'}]])
def test_latched_program_rejects_motion_or_nested_commands_before_effects(tmp_path,rows):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True;app.recovery_required=True
    with pytest.raises(RuntimeError):app.run(rows)
    assert app.thread is None and app.backend.events==[] and app.recovery_required


@pytest.mark.parametrize('command',sorted(RECOVERY))
@pytest.mark.parametrize('reply',[None,{}, {'recovered':False},{'recovered':1},{'acknowledged':True}])
def test_unconfirmed_recovery_never_clears_latch_or_record_error(tmp_path,command,reply):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True
    app.recovery_required=True;app.record_error='stale recording';app.backend.command=lambda *args:reply
    app.run([station(command)]);app.thread.join(2)
    assert app.job['state']=='failed' and app.recovery_required and app.record_error=='stale recording'
    assert app.backend.events==['STOP']


@pytest.mark.parametrize('command',sorted(RECOVERY))
def test_confirmed_recovery_clears_latch_and_record_error(tmp_path,command):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True
    app.recovery_required=True;app.record_error='old error'
    app.run([station(command)]);app.thread.join(2)
    assert app.job['state']=='complete' and app.recovery_required is False and app.record_error is None


@pytest.mark.parametrize('failure',['cancel','deadline'])
def test_recovery_completion_context_guard_preserves_latch(tmp_path,monkeypatch,failure):
    app=Application(tmp_path);app.backend.physical=True;app.recovery_required=True;app.record_error='old error'
    cancel=threading.Event();clock=[100.];monkeypatch.setattr('tools.run_blockly_station.time.monotonic',lambda:clock[0])
    def complete(*args):
        if failure=='cancel':cancel.set()
        else:clock[0]=102.
        return {'recovered':True}
    app.backend.command=complete
    with pytest.raises(InterruptedError if failure=='cancel' else TimeoutError):
        app._program_command(station('verify_recovery'),cancel,101.)
    assert app.recovery_required and app.record_error=='old error'


def test_unknown_station_rpc_outcome_stops_once_and_latches(tmp_path):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True;calls=[]
    backend=ROSBackend.__new__(ROSBackend)
    def trigger(*args):calls.append(args[0]);raise TimeoutError('delivery unknown')
    backend.trigger=trigger;app.backend.command=backend.command
    app.run([station('restart_camera')]);app.thread.join(2)
    assert calls==['/station/operator/restart_camera']
    assert app.job['state']=='failed' and app.recovery_required and app.backend.events==['STOP']


def test_simulation_station_events_and_recovery_have_no_physical_imports(tmp_path):
    app=Application(tmp_path)
    app.run([station('teleop_relative'),station('reinitialize'),station('verify_recovery')]);app.thread.join(2)
    assert app.job['state']=='complete'
    assert app.backend.events==['station:teleop_relative (simulation)','station:reinitialize (simulation)',
                                'station recovery verified (simulation)','station:verify_recovery (simulation)',
                                'station recovery verified (simulation)']


def recovery_backend():
    backend=ROSBackend.__new__(ROSBackend);backend.action_lock=threading.Lock();backend.active_handle=None
    backend.lock=threading.Lock();backend.pending_mutations=set();backend._identity=lambda _:{}
    owner={'owner_faulted':False,'stop_latched':False,'active_operation':None,'teleop_state':'disabled',
           'torque':dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),0)}
    state={'workflow':{'state':'idle','fault':None},'stop':{'pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    calls=[];backend.stop=lambda:calls.append(('STOP',))
    def trigger(endpoint,**context):
        calls.append((endpoint,context));return owner if endpoint=='/follower/status' else state
    backend.trigger=trigger
    return backend,calls


def test_recovery_native_checks_preserve_program_context():
    backend,calls=recovery_backend();cancel=threading.Event();deadline=time.monotonic()+30
    assert backend.recover(cancel=cancel,deadline=deadline)=={'recovered':True}
    assert calls==[('STOP',),('/follower/status',{'cancel':cancel,'deadline':deadline}),
                   ('/station/status',{'cancel':cancel,'deadline':deadline})]


def test_recovery_pending_mutation_barrier_remains():
    backend,calls=recovery_backend();backend.pending_mutations.add(object())
    with pytest.raises(RuntimeError,match='unresolved'):backend.recover(cancel=threading.Event(),deadline=time.monotonic()+30)
    assert calls==[]


class QueuedFuture:
    """A terminal response can be visible before executor callbacks are scheduled."""
    def __init__(self,result,done=True): self.value=result;self.finished=done;self.callbacks=[]
    def done(self):return self.finished
    def result(self):return self.value
    def add_done_callback(self,callback):self.callbacks.append(callback)


def queued_service_backend():
    import json
    from types import SimpleNamespace
    backend,calls=recovery_backend()
    backend.dispatch_lock=threading.Lock();backend.clients={};backend.Trigger=SimpleNamespace(Request=object)
    owner={'owner_faulted':False,'stop_latched':False,'active_operation':None,'teleop_state':'disabled',
           'torque':dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),0)}
    station_status={'workflow':{'state':'idle','fault':None},
                    'stop':{'request_id':'fake-stop','pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    futures={}
    for endpoint,data in (('/station/operator/emergency_reset',{'acknowledged':True}),
                          ('/follower/status',owner),('/station/status',station_status)):
        future=QueuedFuture(SimpleNamespace(success=True,message=json.dumps(data)));futures[endpoint]=future
        backend.clients[endpoint]=SimpleNamespace(service_is_ready=lambda:True,
                                                  call_async=lambda request,future=future:future)
    backend.trigger=ROSBackend.trigger.__get__(backend)
    return backend,futures


def test_reinitialize_known_response_clears_its_queued_pending_callback():
    backend,futures=queued_service_backend()
    assert backend.command(station('reinitialize'),threading.Event(),time.monotonic()+10)=={'recovered':True}
    future=futures['/station/operator/emergency_reset']
    assert len(future.callbacks)==1 and not backend.pending_mutations
    # The delayed callback remains safe after synchronous terminal cleanup.
    future.callbacks[0](future)
    assert not backend.pending_mutations


def test_unrelated_unresolved_future_still_blocks_reinitialize_and_recovery():
    backend,futures=queued_service_backend();unresolved=QueuedFuture(None,done=False)
    backend.pending_mutations.add(unresolved)
    for operation in (lambda:backend.command(station('reinitialize'),threading.Event(),time.monotonic()+10),
                      lambda:backend.recover(cancel=threading.Event(),deadline=time.monotonic()+10)):
        with pytest.raises(RuntimeError,match='unresolved'):operation()
    assert backend.pending_mutations=={unresolved}
    assert not futures['/station/operator/emergency_reset'].callbacks


@pytest.mark.parametrize('failure',['cancel','timeout'])
def test_unknown_service_outcome_keeps_pending_future(failure,monkeypatch):
    backend,futures=queued_service_backend();cancel=threading.Event();clock=[100.]
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:clock[0])
    future=futures['/station/operator/emergency_reset'];future.finished=False
    def dispatched(request):
        if failure=='cancel':cancel.set()
        else:clock[0]=102.
        return future
    backend.clients['/station/operator/emergency_reset'].call_async=dispatched
    with pytest.raises(InterruptedError if failure=='cancel' else TimeoutError):
        backend.command(station('reinitialize'),cancel,101.)
    assert backend.pending_mutations=={future}
    assert len(future.callbacks)==1
