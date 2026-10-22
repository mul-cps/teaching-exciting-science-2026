"""Deterministic native EV3 terminal classification without ROS or devices."""
import threading
import pytest
from workshop.station.ev3_job import run_ev3_operation


def status(*,busy=False,state='complete',job_id=7,session='same',fault=None,cleanup=None):
    job={'id':job_id,'state':state}
    if fault is not None:job['fault']=fault
    if cleanup is not None:job['cleanup_fault']=cleanup
    return {'session':session,'busy':busy,'job':job if busy else None,
            'last_job':None if busy else job,'last_error':fault}


class Proxy:
    session='same'
    def __init__(self,observations,on_status=None):self.observations=iter(observations);self.calls=[];self.on_status=on_status
    def call(self,operation):
        self.calls.append(operation)
        if operation=='home_touch':return {'accepted':True,'job':{'id':7,'state':'running'}}
        if operation=='status':
            if self.on_status:self.on_status()
            return next(self.observations)
        if operation=='stop':return {'stopped':True}
        return {'power_mode':'awake'}


def run(proxy,cancel=None):
    events=[];cancel=cancel or threading.Event()
    value=run_ev3_operation(proxy,cancel,'home_touch',events.append,{'request_id':'STOP-1'},sleep=lambda _:None)
    return value,events


def test_accepted_matching_job_completes_and_persists_bounded_evidence():
    observed=status();proxy=Proxy([status(busy=True,state='running'),observed])
    result,events=run(proxy)
    assert result is observed and proxy.calls==['wake','home_touch','status','status']
    assert [event['event'] for event in events]==['ev3_job_admitted','ev3_job_terminal']
    for event in events:
        assert event['expected_native_job_id']==7 and event['expected_session']=='same'
        assert event['stop_request_id']=='STOP-1'
        assert isinstance(event['time_s'],float) and isinstance(event['monotonic_s'],float)
    assert events[-1]['observed_native_job_id']==7 and events[-1]['native_state']=='complete'


def test_stop_during_status_rpc_is_explicit_interruption_and_not_success():
    cancel=threading.Event();observed=status(state='aborted',fault={'code':'stopped','message':'Explicit stop'})
    proxy=Proxy([observed],on_status=cancel.set);events=[]
    with pytest.raises(InterruptedError,match='interrupted by station STOP'):
        run_ev3_operation(proxy,cancel,'home_touch',events.append,{},sleep=lambda _:None)
    assert proxy.calls==['wake','home_touch','status']
    assert [event['event'] for event in events]==['ev3_job_admitted','ev3_job_failed']
    assert events[-1]['native_fault']['code']=='stopped' and events[-1]['native_state']=='aborted'


@pytest.mark.parametrize('fault',[
    {'code':'stall','message':'Feeder stalled'}, {'code':'disconnect','message':'Connection lost'}])
def test_real_native_fault_is_not_masked_by_simultaneous_cancel(fault):
    cancel=threading.Event();proxy=Proxy([status(state='fault',fault=fault)],on_status=cancel.set)
    with pytest.raises(RuntimeError,match=fault['code']):run(proxy,cancel)


def test_cleanup_fault_precedes_stopped_interruption():
    cancel=threading.Event();proxy=Proxy([status(state='fault',fault={'code':'stopped','message':'Explicit stop'},
                      cleanup={'code':'brake_failed','message':'Feeder brake failed'})],on_status=cancel.set)
    with pytest.raises(RuntimeError,match='cleanup.*brake_failed'):run(proxy,cancel)


@pytest.mark.parametrize('change',[{'session':'other'},{'job_id':8},{'job_id':True}])
def test_terminal_identity_change_remains_fault_even_when_cancelled(change):
    cancel=threading.Event();proxy=Proxy([status(state='aborted',fault={'code':'stopped','message':'Explicit stop'},**change)],on_status=cancel.set)
    with pytest.raises(RuntimeError,match='session|identity'):run(proxy,cancel)


@pytest.mark.parametrize('observed',[
    {'session':'same','busy':False,'job':None,'last_job':None,'last_error':None},
    {'session':'same','busy':0,'last_job':{'id':7,'state':'complete'}},
    {'session':'same','busy':True,'job':{'id':8,'state':'running'}},
    status(state='aborted'),status(state='unknown'),status(fault={'code':'stopped','message':'Explicit stop'}),
    dict(status(),last_error='bad'),dict(status(),last_job=[]),status(cleanup='bad')])
def test_malformed_or_unknown_terminal_never_becomes_cancel_or_success(observed):
    cancel=threading.Event();proxy=Proxy([observed],on_status=cancel.set)
    with pytest.raises(RuntimeError):run(proxy,cancel)


def test_cancel_after_active_status_stops_and_reports_interruption():
    cancel=threading.Event();proxy=Proxy([status(busy=True,state='running')],on_status=cancel.set)
    with pytest.raises(InterruptedError,match='cancelled'):run(proxy,cancel)
    assert proxy.calls[-1]=='stop'


def test_cancelled_active_status_cleanup_failure_stays_fault():
    cancel=threading.Event();proxy=Proxy([status(busy=True,state='running')],on_status=cancel.set)
    original=proxy.call
    proxy.call=lambda operation: {'stopped':False} if operation=='stop' else original(operation)
    with pytest.raises(RuntimeError,match='stop acknowledgement'):run(proxy,cancel)


def test_cancel_during_admission_stops_before_any_status_rpc():
    cancel=threading.Event();proxy=Proxy([]);original=proxy.call
    def call(operation):
        result=original(operation)
        if operation=='home_touch':cancel.set()
        return result
    proxy.call=call
    with pytest.raises(InterruptedError,match='cancelled'):
        run(proxy,cancel)
    assert proxy.calls==['wake','home_touch','stop']


@pytest.mark.parametrize('reason',['cancel','deadline'])
def test_poll_sleep_preemption_stops_before_another_status_rpc(reason):
    cancel=threading.Event();clock=[0.]
    proxy=Proxy([status(busy=True,state='running')])
    def sleep(_):
        if reason=='cancel':cancel.set()
        else:clock[0]=21.
    with pytest.raises(InterruptedError if reason=='cancel' else TimeoutError):
        run_ev3_operation(proxy,cancel,'home_touch',lambda _:None,{},clock=lambda:clock[0],sleep=sleep)
    assert proxy.calls==['wake','home_touch','status','stop']


def test_terminal_native_fault_message_and_evidence_are_bounded():
    fault={'code':'stall','message':'mechanical issue '+'x'*10000};proxy=Proxy([status(state='fault',fault=fault)]);events=[]
    with pytest.raises(RuntimeError) as caught:
        run_ev3_operation(proxy,threading.Event(),'home_touch',events.append,{},sleep=lambda _:None)
    assert 'stall' in str(caught.value) and 'inspect' in str(caught.value)
    assert len(str(caught.value))<500 and len(events[-1]['native_fault']['message'])<=240
    assert 'status' not in str(caught.value)


def test_real_job_fault_precedes_stopped_last_error_and_is_preserved_in_evidence():
    cancel=threading.Event()
    observed=status(state='fault',fault={'code':'stall','message':'Feeder stalled'})
    observed['last_error']={'code':'stopped','message':'Explicit stop'}
    proxy=Proxy([observed],on_status=cancel.set);events=[]
    with pytest.raises(RuntimeError,match='stall'):
        run_ev3_operation(proxy,cancel,'home_touch',events.append,{},sleep=lambda _:None)
    assert events[-1]['native_fault']['code']=='stall'


def test_stop_interruption_remains_latched_until_explicit_workflow_reset(tmp_path):
    import json
    from workshop.station.ros_workflow import StationWorkflow
    workflow=StationWorkflow();path=tmp_path/'events.jsonl'
    def record(event):
        workflow.record(event)
        with path.open('a') as stream:stream.write(json.dumps(event)+'\n')
    def work(cancel):
        observed=status(state='aborted',fault={'code':'stopped','message':'Explicit stop'})
        proxy=Proxy([observed],on_status=cancel.set)
        return run_ev3_operation(proxy,cancel,'home_touch',record,{'request_id':'STOP-test'},sleep=lambda _:None)
    workflow.start('home_touch',work);workflow.thread.join(2)
    assert workflow.state=='fault' and 'interrupted by station STOP' in workflow.fault
    with pytest.raises(RuntimeError,match='fault latched'):workflow.start('home_touch',work)
    events=[json.loads(row) for row in path.read_text().splitlines()]
    assert [event['event'] for event in events]==['ev3_job_admitted','ev3_job_failed']
    assert events[-1]['expected_native_job_id']==events[-1]['observed_native_job_id']==7
    assert events[-1]['expected_session']==events[-1]['observed_session']=='same'
    assert events[-1]['stop_request_id']=='STOP-test'
    workflow.reset();assert workflow.fault is None
