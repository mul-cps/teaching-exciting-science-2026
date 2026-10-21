"""Transport-injected station caller: no ROS init, EV3 or hardware."""
import inspect
import json
import threading
from types import SimpleNamespace

import pytest
from workshop.station.ros_arm_prepare import prepare_arm_once
from tools.ros_station_action import parse_args


class Trigger:
    class Request:
        pass


class Future:
    def __init__(self, result=None):
        self.value = result
        self.cancelled = False
    def done(self):return self.value is not None or self.cancelled
    def result(self):return self.value
    def cancel(self):self.cancelled = True


def harness(reply, *, available=True, stopped=True, hook=lambda: None,
            log_failure=None, destroy_failure=None, state_factory=dict):
    clock = SimpleNamespace(value=1.)
    cancel = threading.Event()
    state = state_factory(pending=False)
    events = []
    calls = []
    future = Future(reply)
    class Client:
        def __init__(self, name):self.name = name
        def wait_for_service(self, timeout_sec):
            clock.value += timeout_sec
            return available or self.name == '/follower/stop'
        def call_async(self, request):
            calls.append(self.name)
            if self.name == '/follower/prepare':return future
            hook()
            return Future(SimpleNamespace(success=stopped, message='verified disarm'))
        def remove_pending_request(self, result):assert result is future
    def spin_once(timeout_sec):
        clock.value += timeout_sec
        hook()
    def destroy(client):
        if client.name==destroy_failure:raise RuntimeError('client disposal failed')
    def record(event):
        events.append(event)
        if event['event']==log_failure:raise OSError('event disk full')
    worker = SimpleNamespace(create_client=lambda kind,name:Client(name),
        destroy_client=destroy,
        _station_executor=SimpleNamespace(spin_once=spin_once))
    def invoke():
        return prepare_arm_once(worker,cancel,state,record,trigger_type=Trigger,
                                clock=lambda:clock.value)
    return invoke,cancel,state,calls,events,future


def reply(success=True, **changes):
    value = dict(schema=1,kind='arm_prepare',state='ready',
                 home_completed_this_session=True,owner_started_monotonic=.5)
    value.update(changes)
    return SimpleNamespace(success=success,message=json.dumps(value))


def test_success_is_one_call_and_never_uses_ev3():
    invoke,cancel,state,calls,events,future=harness(reply())
    assert invoke()['state']=='ready'
    assert calls==['/follower/prepare']
    assert events[-1]['event']=='arm_prepare_complete'
    assert not state['pending']


@pytest.mark.parametrize('case',['cancel','unavailable','explicit_rejection'])
def test_known_pre_delivery_rejection_does_not_stop(case):
    response=reply(False,state='rejected',command_delivery='not_sent',error='not admitted',
                   phases=[],stop_verified=None,home_completed_this_session=False)
    invoke,cancel,state,calls,events,future=harness(response,available=case!='unavailable')
    if case=='cancel':cancel.set()
    with pytest.raises((RuntimeError,InterruptedError)):invoke()
    assert '/follower/stop' not in calls
    assert not state['pending']


@pytest.mark.parametrize('case',['failed_after_home','plain_false','malformed_success','bad_generation'])
def test_false_or_unknown_after_dispatch_requires_verified_stop(case):
    response=reply(False,state='failed',command_delivery='may_have_been_sent',
                   phases=['recover_camera','home'],error='readiness rejected after home')
    if case=='plain_false':response=SimpleNamespace(success=False,message='home failed')
    if case=='malformed_success':response=SimpleNamespace(success=True,message='{}')
    if case=='bad_generation':response=reply(owner_started_monotonic=True)
    invoke,cancel,state,calls,events,future=harness(response)
    with pytest.raises(RuntimeError):invoke()
    assert calls==['/follower/prepare','/follower/stop']
    assert state.get('arm_disarmed') is not True
    assert events[-1]['event']=='arm_prepare_stop_verified'


def test_cancel_of_inflight_trigger_stops_once_and_removes_pending_future():
    holder={}
    invoke,cancel,state,calls,events,future=harness(None,hook=lambda:holder['cancel'].set())
    holder['cancel']=cancel
    with pytest.raises(InterruptedError):invoke()
    assert calls==['/follower/prepare','/follower/stop']
    assert future.cancelled and events[-1]['event']=='arm_prepare_stop_verified'


def test_unverified_stop_remains_pending_and_cannot_be_success():
    invoke,cancel,state,calls,events,future=harness(reply(False,state='failed'),stopped=False)
    with pytest.raises(RuntimeError) as caught:invoke()
    assert calls==['/follower/prepare','/follower/stop']
    assert state['pending'] and state['arm_disarmed'] is False
    assert 'unresolved' in caught.value.__notes__[0]


def test_existing_station_stop_is_observed_without_second_correlation():
    holder={}
    def hook():
        state=holder['state']
        if not state['pending']:
            state.update(pending=True,request_id='current-stop',arm_disarmed=False)
        else:state['arm_disarmed']=True
    invoke,cancel,state,calls,events,future=harness(None,hook=hook)
    holder['state']=state
    with pytest.raises(InterruptedError):invoke()
    assert calls==['/follower/prepare']
    assert state['request_id']=='current-stop' and state['arm_disarmed']


def test_same_caller_is_inside_current_job_and_quick_routes_are_explicit():
    import tools.ros_station_api as api
    source=inspect.getsource(api.main)
    assert source.count('prepare_arm_once(worker, cancel, stop_state,')==2
    helper=inspect.getsource(prepare_arm_once)
    assert 'workflow.start' not in helper and 'proxy.call' not in helper
    assert parse_args(['arm-prepare']).command=='arm-prepare'
    assert parse_args(['arm-stop']).command=='arm-stop'


@pytest.mark.parametrize('event',['arm_prepare_request','arm_prepare_complete',
    'arm_prepare_stop_requested','arm_prepare_stop_verified'])
def test_logging_failure_cannot_skip_stop_or_undo_verified_ack(event):
    response=reply() if event in {'arm_prepare_request','arm_prepare_complete'} else reply(False,state='failed')
    invoke,cancel,state,calls,events,future=harness(response,log_failure=event)
    with pytest.raises((RuntimeError,OSError)):invoke()
    if event=='arm_prepare_request':
        assert calls==[] and not state['pending']
    else:
        assert calls==['/follower/prepare','/follower/stop']
        assert state.get('arm_disarmed') is not True and not state['pending']


@pytest.mark.parametrize('client',['/follower/prepare','/follower/stop'])
def test_client_disposal_error_does_not_undo_or_skip_physical_stop(client):
    response=reply() if client=='/follower/prepare' else reply(False,state='failed',error='original failure')
    invoke,cancel,state,calls,events,future=harness(response,destroy_failure=client)
    with pytest.raises(RuntimeError) as caught:invoke()
    assert calls==['/follower/prepare','/follower/stop']
    assert state.get('arm_disarmed') is not True and not state['pending']
    if client=='/follower/stop':
        assert str(caught.value)=='original failure'
        assert 'stop verified' in caught.value.__notes__[0]


def test_station_stop_announces_correlation_then_dispatches_arm_before_ev3_wait():
    from tools.ros_station_api import request_station_stop
    state=dict(pending=False,arm_disarmed=True,request_id='old')
    events=[]
    def cancel():
        assert state['pending'] and state['arm_disarmed'] is False
        assert state['request_id']!='old'
        events.append('cancel')
    class StopFuture:
        def add_done_callback(self, callback):callback(self)
        def result(self):return SimpleNamespace(success=True,message='actual disarm')
    def arm(request):events.append('arm');return StopFuture()
    def ev3(operation):
        assert events==['cancel','arm']
        assert state['arm_disarmed'] and state['pending']
        events.append('ev3');return {'stopped':True}
    result=request_station_stop(SimpleNamespace(stop=cancel,job_id='same-job'),state,
        SimpleNamespace(call=ev3),SimpleNamespace(service_is_ready=lambda:True,call_async=arm),
        lambda event:None,Trigger)
    assert events==['cancel','arm','ev3']
    assert not state['pending'] and result['job_id']=='same-job'
    assert result['request_id']==state['request_id'] and result['request_id']!='old'


def test_private_stop_ack_cannot_verify_new_external_station_correlation():
    holder={}
    def install_external():
        holder['state'].update(request_id='new-external',pending=True,arm_disarmed=False)
    invoke,cancel,state,calls,events,future=harness(reply(False,state='failed'),hook=install_external)
    holder['state']=state
    with pytest.raises(RuntimeError) as caught:invoke()
    assert state['request_id']=='new-external'
    assert state['pending'] and state['arm_disarmed'] is False
    assert not any(event['event']=='arm_prepare_stop_verified' for event in events)
    assert 'unresolved' in caught.value.__notes__[0]


def test_private_verified_cleanup_never_mutates_station_disarm_ack():
    class ReplaceAtFinalComparison(dict):
        reads=0
        def get(self,key,default=None):
            value=super().get(key,default)
            if key=='request_id':
                self.reads+=1
                if self.reads==2:
                    # Return the previous identity to the final comparison,
                    # then install a newer station request before any ACK write.
                    self.update(request_id='new-external',pending=True,arm_disarmed=False)
            return value
        def __setitem__(self,key,value):
            assert key!='arm_disarmed' or value is not True
            super().__setitem__(key,value)
    invoke,cancel,state,calls,events,future=harness(reply(False,state='failed'),
        state_factory=ReplaceAtFinalComparison)
    with pytest.raises(RuntimeError):invoke()
    assert state['request_id']=='new-external'
    assert state['pending'] and state['arm_disarmed'] is False
    assert events[-1]['event']=='arm_prepare_stop_verified' # local cleanup only


@pytest.mark.parametrize('verified',[False,True])
@pytest.mark.parametrize('note',['arm_prepare_stop_verified','/follower/stop'])
def test_same_job_cleanup_notes_preserve_verified_outcome_and_other_cycle_barriers(verified,note):
    from tools.ros_station_api import retain_cleanup_barrier
    invoke,cancel,state,calls,events,future=harness(reply(False,state='failed'),stopped=verified,
        log_failure=note if not note.startswith('/') else None,
        destroy_failure=note if note.startswith('/') else None)
    with pytest.raises(RuntimeError) as caught:invoke()
    retain_cleanup_barrier(caught.value,state)
    assert state['pending'] is not verified
    assert getattr(caught.value,'arm_prepare_stop_verified',False) is verified
    assert state.get('arm_disarmed') is not True
    later=RuntimeError('later cycle cleanup uncertain')
    later.add_note('missing cycle stop acknowledgement')
    retain_cleanup_barrier(later,state)
    assert state['pending']


@pytest.mark.parametrize('command,endpoint',[('arm-prepare','/station/arm/prepare'),
    ('arm-stop','/follower/stop')])
@pytest.mark.parametrize('unknown',[False,True])
def test_quick_action_aliases_send_zero_argument_trigger(monkeypatch,command,endpoint,capsys,unknown):
    import sys
    import tools.ros_station_action as action
    calls=[]
    clock=[0.]
    class Node:
        def __init__(self,name):pass
        def create_client(self,kind,name):
            assert kind is Trigger
            def send(request):
                assert vars(request)=={}
                calls.append(name)
                return Future(None if unknown else SimpleNamespace(success=True,message='accepted'))
            return SimpleNamespace(wait_for_service=lambda **kwargs:True,call_async=send)
        def destroy_node(self):pass
    def spin(node,timeout_sec):clock[0]+=1.
    monkeypatch.setattr(action.time,'monotonic',lambda:clock[0])
    monkeypatch.setitem(sys.modules,'rclpy',SimpleNamespace(init=lambda:None,shutdown=lambda:None,spin_once=spin))
    monkeypatch.setitem(sys.modules,'rclpy.node',SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules,'std_srvs.srv',SimpleNamespace(Trigger=Trigger))
    monkeypatch.setitem(sys.modules,'so101_station_workflow.srv',SimpleNamespace(**{
        name:Trigger for name in ('RunCycles','ConfirmReference','SelectDestination','SelectPlacementRegion')}))
    if unknown:
        with pytest.raises(TimeoutError,match='unknown; inspect status before retrying'):
            action.main([command])
        assert clock[0]==(12. if command=='arm-stop' else 5.)
    else:
        assert action.main([command])==0
        assert json.loads(capsys.readouterr().out)['success'] is True
    assert calls==[endpoint]
