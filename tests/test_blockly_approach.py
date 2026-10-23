import threading
import time
import pytest
from workshop.blockly_program import validate_program
from workshop.blockly_recording import Recorder
from workshop.blockly_ros import ROSBackend
from tools.run_blockly_station import Application

IDENTITY={'arm_id':'robot','calibration_sha256':'a'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64,'owner_generation':2.,'source_sha256':'b'*64}


def backend():
    recorder=Recorder('stroke',{**IDENTITY,'owner_generation':1.})
    recorder.add([.2]*5,.1,0.); recorder.add([.21]*5,.1,.1)
    obj=ROSBackend.__new__(ROSBackend); obj.recordings={'stroke':recorder.finish()}
    obj.identity=lambda:dict(IDENTITY)
    state={'identity':dict(IDENTITY),'positions':[0.]*5,'jaw':.1,'stamp':1.}
    obj.sample=lambda:dict(state)
    calls=[]
    def plan(target,sample,speed,cancel,deadline):
        calls.append(('plan',target,list(sample['positions']),speed))
        return [{'time':0.,'positions':sample['positions']},{'time':1.,'positions':target}]
    def execute(points,cancel,deadline,identity=None,jaw=None):
        calls.append(('execute',points)); state.update(positions=points[-1]['positions'],stamp=2.)
    obj._plan_recording_start=plan; obj._execute_recording_trajectory=execute
    return obj,state,calls


def test_approach_is_strict_and_uses_planner_then_measured_arrival():
    row={'op':'approach_recording','name':'stroke','speed':.4}; validate_program([row])
    obj,state,calls=backend()
    obj.command(row,threading.Event(),time.monotonic()+5)
    assert calls[0]==('plan',[.2]*5,[0.]*5,.4)
    assert calls[1][0]=='execute'
    assert state['positions']==[.2]*5
    for bad in ({**row,'speed':True},{**row,'speed':1.1},{**row,'coordinates':[]}):
        with pytest.raises(ValueError): validate_program([bad])


@pytest.mark.parametrize('fault',['planner','arrival','jaw','owner','stale'])
def test_approach_failure_never_claims_arrival(fault):
    obj,state,calls=backend()
    if fault=='planner':
        def failed(*args): raise RuntimeError('no collision-free path')
        obj._plan_recording_start=failed
    else:
        execute=obj._execute_recording_trajectory
        def changed(*args,**kwargs):
            execute(*args,**kwargs)
            if fault=='arrival': state['positions']=[0.]*5
            if fault=='jaw': state['jaw']=.4
            if fault=='owner': state['identity']={**IDENTITY,'owner_generation':3.}
            if fault=='stale': state['stamp']=1.
        obj._execute_recording_trajectory=changed
    with pytest.raises(RuntimeError): obj.command({'op':'approach_recording','name':'stroke','speed':.4},threading.Event(),time.monotonic()+5)
    if fault=='planner': assert calls==[]


def test_replay_still_requires_close_start_after_restart():
    obj,state,calls=backend()
    with pytest.raises(RuntimeError,match='recorded start'): obj.command({'op':'replay','name':'stroke','speed':.4},threading.Event(),time.monotonic()+5)
    assert calls==[]


def test_loaded_recordings_need_live_revalidation(tmp_path):
    import json
    obj,_,_=backend()
    (tmp_path/'stroke.json').write_text(json.dumps(obj.recordings['stroke']))
    app=Application(tmp_path)
    assert app.status()['recording_info']['stroke']['state']=='revalidation_required'
    obj.status=lambda:{'physical':True}
    app.backend=obj
    app._revalidate_recordings([{'op':'approach_recording','name':'stroke','speed':.4}])
    assert app.status()['recording_info']['stroke']['state']=='usable'
    obj.identity=lambda:{**IDENTITY,'calibration_sha256':'c'*64}
    with pytest.raises(ValueError): app._revalidate_recordings([{'op':'replay','name':'stroke','speed':.4}])
    assert app.recording_info['stroke']['state']=='invalid'


def planner_backend(monkeypatch,*,error=1,joints=None,times=(0.,2.),positions=None):
    """Message stand-ins only; planner request/retiming/admission are real code."""
    import sys
    import types
    from workshop.blockly_recording import JOINTS
    class Message:
        def __init__(self): pass
    class State:
        def __init__(self): self.joint_state=types.SimpleNamespace(header=types.SimpleNamespace())
    class Constraints:
        def __init__(self): self.joint_constraints=[]
    msg=types.ModuleType('moveit_msgs.msg')
    msg.Constraints=Constraints; msg.JointConstraint=Message; msg.MotionPlanRequest=Message; msg.RobotState=State
    srv=types.ModuleType('moveit_msgs.srv'); srv.GetMotionPlan=types.SimpleNamespace(Request=Message)
    monkeypatch.setitem(sys.modules,'moveit_msgs.msg',msg); monkeypatch.setitem(sys.modules,'moveit_msgs.srv',srv)
    names=list(JOINTS) if joints is None else joints
    values=positions or [[0.]*5,[.2]*5]
    points=[types.SimpleNamespace(positions=value,time_from_start=types.SimpleNamespace(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))) for value,stamp in zip(values,times)]
    response=types.SimpleNamespace(motion_plan_response=types.SimpleNamespace(error_code=types.SimpleNamespace(val=error),
        trajectory=types.SimpleNamespace(joint_trajectory=types.SimpleNamespace(joint_names=names,points=points))))
    class Future:
        def done(self): return True
        def result(self): return response
    calls=[]
    class Client:
        def service_is_ready(self): return True
        def call_async(self,request): calls.append(request); return Future()
    obj=ROSBackend.__new__(ROSBackend); obj.lock=threading.Lock(); obj.clients={'/follower/plan_kinematic_path':Client()}
    obj.node=types.SimpleNamespace(get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:'now')))
    return obj,calls


def test_actual_planner_request_holds_jaw_and_retimes_from_current_pose(monkeypatch):
    from workshop.blockly_recording import JOINTS
    obj,calls=planner_backend(monkeypatch)
    sample={'positions':[0.]*5,'jaw':.3}
    result=obj._plan_recording_start([.2]*5,sample,.4,threading.Event(),time.monotonic()+5)
    assert result[-1]['positions']==[.2]*5
    assert len(calls)==1
    request=calls[0].motion_plan_request
    assert request.start_state.joint_state.name==list(JOINTS)+['gripper']
    assert request.start_state.joint_state.position==[0.]*5+[.3]
    assert [(c.joint_name,c.position) for c in request.goal_constraints[0].joint_constraints]==list(zip(JOINTS,[.2]*5))
    assert request.max_velocity_scaling_factor==.4
    assert request.start_state.is_diff is False


@pytest.mark.parametrize('options',[
    {'error':-1}, {'joints':['wrong']*5}, {'times':(1.,0.)},
    {'positions':[[0.]*5,[float('nan')]*5]}, {'positions':[[0.]*5,[.1]*5]},
    {'positions':[[0.]*5,[.2]*4]}, {'positions':[[0.]*5,[10.]*5]},
])
def test_actual_planner_rejects_failure_and_malformed_paths(monkeypatch,options):
    obj,calls=planner_backend(monkeypatch,**options)
    with pytest.raises((RuntimeError,ValueError)):
        obj._plan_recording_start([.2]*5,{'positions':[0.]*5,'jaw':.3},.4,threading.Event(),time.monotonic()+5)
    assert len(calls)==1
