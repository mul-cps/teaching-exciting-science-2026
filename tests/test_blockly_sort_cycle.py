import sys
import threading
import time
import types
import pytest
from workshop.blockly_program import validate_program,ProgramRunner,SimulationBackend
from workshop.blockly_ros import ROSBackend,validate_sort_result


def test_cycle_opcode_is_fixed_and_simulation_reports_result():
    assert validate_program([{'op':'sort_cycle'}])
    for extra in ({'cycles':2},{'config':'file'},{'endpoint':'/other'},{'position':[0,0,0]}):
        with pytest.raises(ValueError): validate_program([dict(op='sort_cycle',**extra)])
    backend=SimulationBackend()
    result=ProgramRunner(backend).execute([{'op':'sort_cycle'}],threading.Event())
    assert result['sort_cycles']==[{'completed':1,'colors':{'red':1}}]
    assert backend.events==['sort_cycle: red → commissioned destination (simulation)']


@pytest.mark.parametrize('result',[None,{}, {'completed':0,'colors':{}},
    {'completed':True,'colors':{'red':1}}, {'completed':1,'colors':{'unknown':1}},
    {'completed':1,'colors':{'red':True}}, {'completed':1,'colors':{'red':2}},
    {'completed':1,'colors':{'red':-1}}, {'completed':1,'colors':{}}])
def test_invalid_completion_never_counts_success(result):
    with pytest.raises(RuntimeError): validate_sort_result(result)


def fake_backend(monkeypatch,snapshot):
    class RunCycles:
        class Request:
            cycles=0
    monkeypatch.setitem(sys.modules,'so101_station_workflow.srv',types.SimpleNamespace(RunCycles=RunCycles))
    backend=ROSBackend.__new__(ROSBackend); calls=[]
    def service(endpoint,service_type,request,cancel,deadline):
        calls.append((endpoint,request.cycles)); return {'job_id':'cycle-1'}
    backend._service=service
    backend.trigger=lambda *args,**kwargs:snapshot
    backend.stop=lambda:calls.append(('STOP',None))
    return backend,calls


def test_cycle_waits_for_exact_native_result(monkeypatch):
    backend,calls=fake_backend(monkeypatch,{'workflow':{'job_id':'cycle-1','state':'idle','fault':None,'result':{'completed':1,'colors':{'blue':1}}}})
    result=backend.command({'op':'sort_cycle'},threading.Event(),time.monotonic()+3)
    assert calls==[('/station/run_cycles',1)]
    assert result=={'completed':1,'colors':{'blue':1}}


@pytest.mark.parametrize('workflow',[{'job_id':'other','state':'idle','result':{'completed':1,'colors':{'red':1}}},
    {'job_id':'cycle-1','state':'fault','fault':'pickup unverified','result':None},
    {'job_id':'cycle-1','state':'idle','fault':None,'result':{'completed':0,'colors':{}}}])
def test_failed_wrong_or_incomplete_cycle_stops(monkeypatch,workflow):
    backend,calls=fake_backend(monkeypatch,{'workflow':workflow})
    with pytest.raises(RuntimeError): ProgramRunner(backend).execute([{'op':'sort_cycle'}],threading.Event())
    assert calls[-1]==('STOP',None)


def test_cycle_deadline_or_cancel_stops_without_retry(monkeypatch):
    backend,calls=fake_backend(monkeypatch,{'workflow':{'job_id':'cycle-1','state':'sorting','result':None}})
    with pytest.raises(TimeoutError): backend.command({'op':'sort_cycle'},threading.Event(),time.monotonic()-.1)
    assert calls==[('/station/run_cycles',1)]
    cancel=threading.Event();cancel.set()
    with pytest.raises(InterruptedError): ProgramRunner(backend).execute([{'op':'sort_cycle'}],cancel)
    assert calls[-1]==('STOP',None)


def test_timeout_after_acceptance_requests_stop(monkeypatch):
    backend,calls=fake_backend(monkeypatch,{'workflow':{'job_id':'cycle-1','state':'sorting','result':None}})
    wait=backend._wait_job
    backend._wait_job=lambda job,cancel,deadline:wait(job,cancel,time.monotonic()-.1)
    with pytest.raises(TimeoutError): ProgramRunner(backend).execute([{'op':'sort_cycle'}],threading.Event())
    assert calls==[('/station/run_cycles',1),('STOP',None)]


@pytest.mark.parametrize('workflow',[None,{'state':'sorting','fault':None},
    {'state':'fault','fault':'pickup unverified'},{'state':'idle','fault':'latched'}])
def test_recover_never_clears_active_or_faulted_native_workflow(workflow):
    backend=ROSBackend.__new__(ROSBackend);backend.action_lock=threading.Lock();backend.active_handle=None
    backend.lock=threading.Lock();backend.pending_mutations=set();backend.stop=lambda:None;backend._identity=lambda _:{}
    owner={'torque':dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),0)}
    calls=[]
    def trigger(endpoint):
        calls.append(endpoint)
        return owner if endpoint=='/follower/status' else {'workflow':workflow,'stop':{'pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    backend.trigger=trigger
    with pytest.raises(RuntimeError,match='workflow'): backend.recover()
    assert calls==['/follower/status','/station/status']
