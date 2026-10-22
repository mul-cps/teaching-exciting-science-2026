"""Explicit inspected EV3 fault reset uses the existing global station service."""
import threading
import time
import pytest
from tools.run_blockly_station import Application
from workshop.blockly_program import validate_program
from workshop.blockly_ros import EV3_SERVICES,ROSBackend


def reset():return {'op':'ev3','command':'reset_fault'}


def test_reset_fault_enum_and_existing_global_service():
    validate_program([reset()])
    assert EV3_SERVICES['reset_fault']=='/station/reset_fault'
    backend=ROSBackend.__new__(ROSBackend);calls=[];cancel=threading.Event();deadline=time.monotonic()+30
    backend.trigger=lambda endpoint,event,end:calls.append((endpoint,event,end)) or {'acknowledged':True}
    backend._wait_job=lambda *args:pytest.fail('fault reset without job must not invent a motion job')
    backend.command(reset(),cancel,deadline)
    assert len(calls)==1 and calls[0][0]=='/station/reset_fault' and calls[0][1] is cancel
    assert time.monotonic()<calls[0][2]<=deadline


@pytest.mark.parametrize('nested',[False,True])
def test_physical_reset_fault_requires_instructor_before_any_effect(tmp_path,nested):
    app=Application(tmp_path);app.backend.physical=True
    row=reset()
    if nested:row={'op':'if_color','color':'red','then':[],
                  'else':[{'op':'repeat','count':1,'body':[row]}]}
    with pytest.raises(RuntimeError,match='instructor'):app.run([{'op':'ev3','command':'wake'},row])
    assert app.thread is None and app.backend.events==[]


def test_instructor_physical_reset_is_admitted(tmp_path):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True
    app.run([reset()]);app.thread.join(2)
    assert app.job['state']=='complete' and app.backend.events==['ev3:reset_fault']


def test_simulation_explicit_reset_remains_offline(tmp_path):
    app=Application(tmp_path);app.run([reset()]);app.thread.join(2)
    assert app.job['state']=='complete' and app.backend.events==['ev3:reset_fault']


def test_existing_ev3_commands_still_allow_noninstructor_physical_programs(tmp_path):
    app=Application(tmp_path);app.backend.physical=True
    app.run([{'op':'ev3','command':'wake'}]);app.thread.join(2)
    assert app.job['state']=='complete' and app.backend.events==['ev3:wake']


def test_fault_reset_is_not_implicitly_added_to_station_recovery(tmp_path):
    app=Application(tmp_path);app.backend.physical=True;app.instructor=True;app.recovery_required=True
    app.run([{'op':'station','command':'verify_recovery'}]);app.thread.join(2)
    assert app.job['state']=='complete'
    assert not any('reset_fault' in event for event in app.backend.events)
