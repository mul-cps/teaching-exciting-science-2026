"""Verified intentional STOP ends a program without requiring fault recovery."""
import threading
import pytest
from tools.run_blockly_station import Application
from workshop.blockly_program import ProgramRunner, SimulationBackend


def test_verified_stop_has_distinct_result_and_stops_following_commands():
    backend=SimulationBackend()
    result=ProgramRunner(backend).execute([{'op':'stop'},{'op':'log','message':'unreachable'}],threading.Event())
    assert result=={'complete':False,'stopped':True}
    assert backend.events==['STOP']


def test_nested_stop_ends_whole_program_and_all_repeats():
    backend=SimulationBackend()
    result=ProgramRunner(backend).execute([
        {'op':'repeat','count':3,'body':[
            {'op':'if_color','color':'red','then':[{'op':'stop'}],
             'else':[{'op':'log','message':'wrong branch'}]},
            {'op':'log','message':'unreachable repeat'}]},
        {'op':'log','message':'unreachable outer'}],threading.Event())
    assert result=={'complete':False,'stopped':True} and backend.events==['STOP']


def test_verified_physical_stop_allows_subsequent_manual_record_start(tmp_path):
    app=Application(tmp_path); app.backend.physical=True
    def stop():
        app.backend.log('STOP')
        return {'request_id':'correlated-stop','pending':False,'ev3_stopped':True,'arm_disarmed':True}
    app.backend.stop=stop
    app.run([{'op':'stop'}]); app.thread.join(2)
    assert app.job['state']=='stopped'
    assert app.job['result']=={'complete':False,'stopped':True}
    assert app.recovery_required is False
    assert app.backend.events==['STOP']
    try:
        assert app.record_start('after_stop')['recording']=='after_stop'
        assert app.recorder.name=='after_stop'
    finally:
        app.record_cancel.set(); app.record_thread.join(2)


@pytest.mark.parametrize('reply',[None,{}, {'acknowledged':False},{'acknowledged':1},
    {'acknowledged':True},
    *[dict(request_id=request_id,pending=False,ev3_stopped=True,arm_disarmed=True)
      for request_id in ('',None,1)],
    *[dict({'request_id':'stop','pending':False,'ev3_stopped':True,'arm_disarmed':True},**{key:value})
      for key,value in (('pending',True),('pending',0),('ev3_stopped',False),
                        ('ev3_stopped',1),('arm_disarmed',False),('arm_disarmed',1))]])
def test_missing_exact_stop_acknowledgement_fails_and_latches(tmp_path,reply):
    app=Application(tmp_path); app.backend.physical=True
    app.backend.stop=lambda:reply
    app.run([{'op':'stop'},{'op':'log','message':'unreachable'}]); app.thread.join(2)
    assert app.job['state']=='failed' and app.recovery_required is True
    assert 'STOP acknowledgement missing' in app.job['error']
    assert app.backend.events==[]
    with pytest.raises(RuntimeError,match='recover'): app.record_start('blocked')


def test_raised_stop_keeps_failure_barrier(tmp_path):
    app=Application(tmp_path); app.backend.physical=True
    def stop(): raise RuntimeError('unknown STOP delivery')
    app.backend.stop=stop
    app.run([{'op':'stop'}]); app.thread.join(2)
    assert app.job['state']=='failed' and app.recovery_required is True
    assert 'unknown STOP delivery' in app.job['error']


def test_actual_cancellation_remains_failed_and_latched(tmp_path):
    app=Application(tmp_path); app.backend.physical=True
    entered=threading.Event()
    def command(row,cancel,deadline):
        entered.set(); cancel.wait(2)
        if cancel.is_set(): raise InterruptedError('actual cancellation')
    app.backend.command=command
    app.run([{'op':'ev3','command':'wake'}]); assert entered.wait(1)
    app.cancel.set(); app.thread.join(2)
    assert app.job['state']=='failed' and app.recovery_required is True
    assert 'actual cancellation' in app.job['error'] and app.backend.events==['STOP']


@pytest.mark.parametrize('failure',['cancel','deadline'])
def test_stop_checks_concurrent_cancel_or_deadline_before_stopped_result(failure):
    backend=SimulationBackend(); cancel=threading.Event(); clock=[0.]
    def stop():
        backend.log('STOP')
        if failure=='cancel': cancel.set()
        else: clock[0]=181.
        return {'acknowledged':True}
    backend.stop=stop
    with pytest.raises(InterruptedError if failure=='cancel' else TimeoutError):
        ProgramRunner(backend,clock=lambda:clock[0]).execute([{'op':'stop'}],cancel)
    assert backend.events==['STOP','STOP']


@pytest.mark.parametrize('reply',[None,{}, {'acknowledged':False},{'acknowledged':1},
    {'request_id':'stop','pending':False,'ev3_stopped':True,'arm_disarmed':True}])
def test_simulation_requires_exact_boolean_acknowledgement(reply):
    backend=SimulationBackend();backend.stop=lambda:reply
    with pytest.raises(RuntimeError,match='STOP acknowledgement missing'):
        ProgramRunner(backend).execute([{'op':'stop'}],threading.Event())
