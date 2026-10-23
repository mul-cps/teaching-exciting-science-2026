import json
import threading
import pytest
from workshop import blockly_client as client


class Endpoint:
    def __init__(self, terminal=None):
        self.calls=[]
        self.terminal=terminal or {'id':'exact','state':'complete','result':{'complete':True}}
    def __call__(self, operation, body, timeout):
        self.calls.append((operation,json.loads(body)))
        if operation=='run': return {'id':'exact','state':'running'}
        if operation=='status': return {'physical':False,'job':self.terminal,'admission':{'session':'test-session','epoch':0}}
        if operation=='stop': return {'acknowledged':True,'simulation':True}


def test_readable_builder_and_export_execute_exact_nested_ast():
    program=client.Program().log('Grüße').repeat(2,client.Program().wait(0)).if_color('red',client.Program().stop(),client.Program().jaw(12)).program
    scope={'__name__':'export_test'}
    exec(compile(client.export_python(program),'student.py','exec'),scope)
    assert scope['program']==program
    endpoint=Endpoint()
    assert client.run_program(program,transport=endpoint,physical=False)=={'complete':True}
    assert endpoint.calls[0]==('status',{})
    assert endpoint.calls[1]==('run',{'program':program,'admission':{'session':'test-session','epoch':0}})


@pytest.mark.parametrize('terminal',[
    {'id':'wrong','state':'complete','result':{'complete':True}},
    {'id':'exact','state':'complete','result':{}},
    {'id':'exact','state':'failed','error':'collision'},
    {'id':'exact','state':'stopped','result':{'complete':False}},
])
def test_uncertain_or_failed_job_stops_once_and_never_retries_run(terminal):
    endpoint=Endpoint(terminal)
    with pytest.raises(RuntimeError): client.run_program([{'op':'log','message':'hello'}],transport=endpoint,physical=False)
    assert [op for op,_ in endpoint.calls].count('run')==1
    assert [op for op,_ in endpoint.calls].count('stop')==1


def test_explicit_stop_terminal_result_is_successful_stopped_outcome():
    endpoint=Endpoint({'id':'exact','state':'stopped','result':{'complete':False,'stopped':True}})
    assert client.run_program([{'op':'stop'}],transport=endpoint,physical=False)['stopped'] is True


def test_unknown_run_delivery_requires_stop_and_no_resubmission():
    endpoint=Endpoint()
    def transport(op,body,timeout):
        if op=='run':
            endpoint.calls.append((op,json.loads(body)))
            raise TimeoutError('unknown delivery')
        return endpoint(op,body,timeout)
    with pytest.raises(TimeoutError): client.run_program([{'op':'stop'}],transport=transport,physical=False)
    assert [op for op,_ in endpoint.calls].count('run')==1
    assert [op for op,_ in endpoint.calls].count('stop')==1


def test_wrong_endpoint_mode_refuses_before_mutation():
    endpoint=Endpoint()
    with pytest.raises(RuntimeError,match='mode'): client.run_program([{'op':'stop'}],transport=endpoint,physical=True)
    assert [op for op,_ in endpoint.calls]==['status']


def test_cancel_after_dispatch_requires_verified_stop():
    endpoint=Endpoint(); cancel=threading.Event()
    def transport(op,body,timeout):
        result=endpoint(op,body,timeout)
        if op=='run': cancel.set()
        return result
    with pytest.raises(InterruptedError): client.run_program([{'op':'stop'}],transport=transport,physical=False,cancel=cancel)
    assert endpoint.calls[-1][0]=='stop'


def test_offline_simulation_never_opens_ros_or_wrapper(monkeypatch):
    monkeypatch.setattr(client,'ROSProgramTransport',lambda:pytest.fail('ROS accessed'))
    monkeypatch.setattr(client,'_wrapper_run',lambda *args:pytest.fail('wrapper accessed'))
    result=client.simulate_program(client.Program().ev3('eject').log('done'))
    assert result['complete'] is True
    assert result['simulation'] is True
    assert result['events']==['ev3:eject','done']


@pytest.mark.parametrize('interval',[0,-1,float('nan'),float('inf'),True])
def test_invalid_poll_interval_rejected_before_transport(interval):
    endpoint=Endpoint()
    with pytest.raises(ValueError): client.run_program([{'op':'stop'}],transport=endpoint,physical=False,poll_interval=interval)
    assert endpoint.calls==[]


def test_safe_explicit_launcher_prefix_has_no_shell(monkeypatch):
    monkeypatch.setenv('STATION_PROGRAM_LAUNCHER','["~/.local/bin/station-fwmini4", "ros-vulkan", "python"]')
    assert client._launcher_prefix()[0].startswith('/')
    assert client._launcher_prefix()[1:]==['ros-vulkan','python']
    monkeypatch.setenv('STATION_PROGRAM_LAUNCHER','"shell command"')
    with pytest.raises(ValueError): client._launcher_prefix()


@pytest.mark.parametrize('stop_reply',[
    {'pending':False,'ev3_stopped':True,'arm_disarmed':True},
    {'request_id':'request','pending':True,'ev3_stopped':True,'arm_disarmed':True},
    {'request_id':'request','pending':False,'ev3_stopped':False,'arm_disarmed':True},
])
def test_physical_uncertainty_does_not_accept_partial_stop(stop_reply):
    endpoint=Endpoint({'id':'wrong','state':'complete','result':{'complete':True}})
    def transport(operation,body,timeout):
        result=endpoint(operation,body,timeout)
        if operation=='status': result['physical']=True
        if operation=='stop': return stop_reply
        return result
    with pytest.raises(RuntimeError,match='STOP unverified'):
        client.run_program([{'op':'stop'}],transport=transport,physical=True)
    assert [op for op,_ in endpoint.calls].count('stop')==1
