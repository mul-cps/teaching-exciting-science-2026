"""Calibration admission through actual ROSBackend methods, without ROS startup."""
import json
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace as NS
import pytest
from workshop.blockly_ros import ROSBackend


def response(si=True, angular=False, **changes):
    data=dict(schema='so101-gripper-calibration-status-v1',si_gap_calibrated=si,
              moveit_joint_calibrated=angular,si_position_units='metres',
              moveit_position_units='radians',visual_proxy_only=not angular)
    data.update(changes)
    return NS(success=si and angular,message=json.dumps(data))


class Client:
    def __init__(self, reply): self.reply=reply; self.futures=[]; self.removed=[]
    def service_is_ready(self): return True
    def call_async(self, request):
        future=Future(); self.futures.append(future)
        if self.reply is not None: future.set_result(self.reply)
        return future
    def remove_pending_request(self, future): self.removed.append(future)


def backend(reply):
    b=ROSBackend.__new__(ROSBackend)
    b.lock=threading.Lock(); b.dispatch_lock=threading.Lock(); b.pending_mutations=set(); b.cache={}
    b.Trigger=NS(Request=lambda:object()); b.Gripper=NS(Goal=lambda:NS(command=NS()))
    b.jaw=object(); b.clients={'/follower/gripper_calibration_status':Client(reply)}
    b.calibration_client=b.clients['/follower/gripper_calibration_status']
    b.calibration_pending=None; b.calibration_pending_at=0.
    goals=[]
    b._action=lambda client,goal,cancel,deadline: goals.append(goal) or NS(reached_goal=True,stalled=False)
    return b,goals


@pytest.mark.parametrize('angular',[False,True])
def test_si_dispatches_one_goal_in_metres(angular):
    b,goals=backend(response(angular=angular))
    b.command({'op':'jaw','gap_mm':30},threading.Event(),time.monotonic()+3)
    assert len(goals)==1
    assert goals[0].command.position==.030
    assert goals[0].command.max_effort==0.


def test_both_false_status_inspectable_but_no_jaw():
    b,goals=backend(response(False,False))
    assert b.trigger('/follower/gripper_calibration_status')['si_gap_calibrated'] is False
    with pytest.raises(RuntimeError): b.command({'op':'jaw','gap_mm':30},threading.Event(),time.monotonic()+3)
    assert goals==[]


BAD=[response(schema='bad'),response(si_position_units='mm'),response(moveit_position_units='degrees'),
     response(si_gap_calibrated='true'),response(moveit_joint_calibrated='false'),
     response(visual_proxy_only=False),response(reason=3),response(reason='x'*8193),
     NS(success=True,message=response().message),NS(success=False,message='x'*8193)]
@pytest.mark.parametrize('reply',BAD)
def test_invalid_status_rejects_command_and_cache(reply):
    b,goals=backend(reply)
    with pytest.raises(RuntimeError): b.command({'op':'jaw','gap_mm':30},threading.Event(),time.monotonic()+3)
    b._refresh_calibration()
    assert 'gripper_calibration' not in b.cache
    assert goals==[]


def test_generic_mutating_trigger_false_still_rejected():
    b,_=backend(response())
    b.clients['/station/operator/moveit']=Client(NS(success=False,message='rejected'))
    with pytest.raises(RuntimeError,match='rejected'): b.trigger('/station/operator/moveit')


@pytest.mark.parametrize('case',['expired','replaced','cancelled'])
def test_late_or_cancelled_callback_cannot_refresh_cache(monkeypatch,case):
    now=[100.]; monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:now[0])
    b,_=backend(None); b._refresh_calibration(); old=b.calibration_pending
    if case=='expired': now[0]+=1.01
    elif case=='replaced':
        now[0]+=.2; b.calibration_pending=Future()
    else:
        old.cancel()
        assert 'gripper_calibration' not in b.cache
        return
    old.set_result(response())
    assert 'gripper_calibration' not in b.cache


@pytest.mark.parametrize('age,si,allowed',[(0,True,True),(2,True,True),(-.1,True,False),(2.01,True,False),(0,'true',False)])
def test_capability_uses_bounded_age_and_exact_bool(monkeypatch,age,si,allowed):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    b,_=backend(response()); b.cache['gripper_calibration']=(100.-age,{'si_gap_calibrated':si})
    assert b.status()['capabilities']['jaw'] is allowed


def test_cancel_before_command_never_dispatches():
    b,goals=backend(response()); cancel=threading.Event(); cancel.set()
    with pytest.raises(InterruptedError): b.command({'op':'jaw','gap_mm':30},cancel,time.monotonic()+3)
    assert goals==[]

@pytest.mark.parametrize('message',[
    '['*1500+'0'+']'*1500,
    response().message[:-1]+',"si_gap_calibrated":true}',
    response().message[:-1]+',"reason":"first","reason":"second"}',
])
def test_nested_or_duplicate_json_controlled_rejection(message):
    b,goals=backend(NS(success=False,message=message))
    with pytest.raises((RuntimeError,ValueError)):
        b.command({'op':'jaw','gap_mm':30},threading.Event(),time.monotonic()+3)
    b._refresh_calibration()
    assert 'gripper_calibration' not in b.cache
    assert goals==[]
