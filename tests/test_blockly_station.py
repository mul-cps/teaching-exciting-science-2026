import threading
import pytest
from workshop.blockly_program import validate_program, ProgramRunner, SimulationBackend
from workshop.blockly_recording import Recorder


def test_nested_program_and_limits():
    assert validate_program([{'op':'repeat','count':2,'body':[{'op':'wait','seconds':1}]}])
    for bad in ([{'op':'shell','command':'x'}], [{'op':'repeat','count':6,'body':[]}],
                [{'op':'wait','seconds':float('nan')}], [{'op':'wait','seconds':1,'extra':1}],
                [{'op':'replay','name':'../x','speed':.5}]):
        with pytest.raises(ValueError): validate_program(bad)


def test_program_run_and_failure_stop():
    backend=SimulationBackend()
    runner=ProgramRunner(backend)
    runner.execute([{'op':'if_color','color':'red','then':[{'op':'ev3','command':'eject'}], 'else':[]}], threading.Event())
    assert backend.events == ['ev3:eject']
    with pytest.raises(RuntimeError):
        runner.execute([{'op':'replay','name':'missing','speed':.5}], threading.Event())
    assert backend.events[-1] == 'STOP'


def test_cancel_stops():
    backend=SimulationBackend(); cancel=threading.Event(); cancel.set()
    with pytest.raises(InterruptedError): ProgramRunner(backend).execute([{'op':'wait','seconds':1}],cancel)
    assert backend.events == ['STOP']


def test_recording_fixed_jaw_and_gap():
    r=Recorder('transfer', {'arm_id':'arm','calibration_sha256':'a'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}, clock=lambda:0)
    r.add([0]*5,.2,0); r.add([.1]*5,.2,.1)
    assert r.finish()['points'][1]['time'] == .1
    for jaw,stamp in ((.3,.2),(.2,1)):
        with pytest.raises(ValueError): r.add([0]*5,jaw,stamp)


def test_exact_job_terminal_and_error():
    from workshop.blockly_ros import terminal_job
    assert terminal_job({'workflow':{'job_id':'a','state':'idle','result':{}}},'a')
    assert not terminal_job({'workflow':{'job_id':'a','state':'working','result':None}},'a')
    with pytest.raises(RuntimeError): terminal_job({'workflow':{'job_id':'b'}},'a')
    with pytest.raises(RuntimeError): terminal_job({'workflow':{'job_id':'a','state':'fault','fault':'stuck'}},'a')


def test_replay_speed_identity_and_invalid_samples():
    from workshop.blockly_recording import replay_points
    identity={'arm_id':'a','calibration_sha256':'a'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}
    r=Recorder('stroke',identity); r.add([0]*5,0,0); r.add([.1]*5,0,.1)
    recording=r.finish()
    assert replay_points(recording,identity,.5)[1]['time']==.2
    with pytest.raises(ValueError): replay_points(recording,{'arm_id':'b'},.5)
    recording['points'][1]['positions']=[float('nan')]*5
    with pytest.raises(ValueError): replay_points(recording,identity,.5)


def test_server_auth_and_simulation(tmp_path):
    from tools.run_blockly_station import Application, make_server
    import urllib.request, urllib.error, json
    app=Application(tmp_path); server=make_server(app,0)
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    root=f'http://127.0.0.1:{server.server_port}'
    try:
        with pytest.raises(urllib.error.HTTPError): urllib.request.urlopen(root+'/api/status')
        request=urllib.request.Request(root+'/api/program/run',data=json.dumps({'program':[{'op':'log','message':'hello'}],'admission':app.status()['admission']}).encode(),headers={'X-Station-Token':app.token})
        assert json.load(urllib.request.urlopen(request))['id']
        request=urllib.request.Request(root+'/api/program/stop',data=b'{}',headers={'X-Station-Token':app.token,'Origin':'https://evil.invalid'})
        with pytest.raises(urllib.error.HTTPError): urllib.request.urlopen(request)
        with urllib.request.urlopen(root+'/') as response: assert app.token.encode() in response.read()
    finally: server.shutdown(); server.server_close(); app.close()


def test_adversarial_types_bounded_expansion_and_depth():
    for bad in ([{'op':{}}],[{'op':'ev3','command':[]}],[{'op':'if_color','color':{},'then':[],'else':[]}],
                [{'op':'repeat','count':5,'body':[{'op':'repeat','count':5,'body':[{'op':'wait','seconds':0}]*5}]}]):
        with pytest.raises(ValueError): validate_program(bad)
    deep=[{'op':'wait','seconds':0}]
    for _ in range(6): deep=[{'op':'repeat','count':1,'body':deep}]
    with pytest.raises(ValueError): validate_program(deep)


def test_recording_errors_stay_latched():
    r=Recorder('stroke',{'arm_id':'a','calibration_sha256':'b'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64})
    r.add([0]*5,0,0)
    with pytest.raises(ValueError): r.add([0]*5,.2,.1)
    with pytest.raises(ValueError): r.finish()


def test_strict_json_and_physical_unknown_barrier(tmp_path):
    from tools.run_blockly_station import strict_json, Application
    for text in ('{"x":1,"x":2}','{"x":NaN}','{"x":Infinity}'):
        with pytest.raises(ValueError): strict_json(text)
    app=Application(tmp_path)
    class Unknown(SimulationBackend):
        physical=True
        def command(self,*_): raise TimeoutError('unknown delivery')
        def stop(self): raise TimeoutError('STOP unknown')
    app.backend=Unknown()
    app.run([{'op':'ev3','command':'eject'}]); app.thread.join(2)
    assert app.recovery_required
    with pytest.raises(RuntimeError): app.run([{'op':'wait','seconds':0}])


@pytest.mark.parametrize('tail', [
    [{'op':'replay','name':'missing','speed':.5}],
    [{'op':'repeat','count':2,'body':[{'op':'if_color','color':'red',
       'then':[], 'else':[{'op':'replay','name':'missing','speed':.5}]}]}],
    [{'op':'if_color','color':'blue',
      'then':[{'op':'repeat','count':2,'body':[{'op':'replay','name':'missing','speed':.5}]}],
      'else':[]}],
])
def test_application_rejects_missing_replay_before_dispatch(tmp_path, tail):
    from tools.run_blockly_station import Application
    app=Application(tmp_path)
    class PhysicalLike(SimulationBackend):
        physical=True
        def status(self): raise AssertionError('static admission must not query backend status')
        def color(self): raise AssertionError('static admission must not query backend color')
    app.backend=PhysicalLike(app.recordings)
    cancel=app.cancel; cancel.set()
    job=app.job; thread=app.thread
    try:
        with pytest.raises(ValueError, match='missing'):
            app.run([{'op':'ev3','command':'eject'}]+tail)
    finally:
        if app.thread: app.thread.join(2)
    assert app.backend.events == []
    assert app.cancel is cancel and app.cancel.is_set()
    assert app.job is job and app.job == {'state':'idle'}
    assert app.thread is thread
    assert not app.recovery_required


def test_application_allows_known_replays_in_nested_branches(tmp_path):
    from tools.run_blockly_station import Application
    app=Application(tmp_path)
    class PhysicalLike(SimulationBackend):
        physical=True
        def identity(self): return {'arm_id':'physical-fixture','calibration_sha256':'a'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}
    recorder=Recorder('known',PhysicalLike().identity(),origin='physical')
    recorder.add([0.]*5,0.,0.); recorder.add([0.]*5,0.,.1)
    app.recordings['known']=recorder.finish()
    app.backend=PhysicalLike(app.recordings)
    replay={'op':'replay','name':'known','speed':.5}
    app.run([{'op':'ev3','command':'eject'},
             {'op':'repeat','count':2,'body':[{'op':'if_color','color':'red',
              'then':[replay], 'else':[replay]}]}])
    app.thread.join(2)
    assert app.job['state'] == 'complete'
    assert app.backend.events == ['ev3:eject','replay:known','replay:known']
    assert not app.recovery_required


def test_ready_client_never_dispatches_after_cancel():
    from workshop.blockly_ros import ROSBackend
    backend=ROSBackend.__new__(ROSBackend); backend.lock=threading.Lock(); backend.dispatch_lock=threading.Lock(); backend.pending_mutations=set()
    class Client:
        def service_is_ready(self): return True
        def call_async(self,_): raise AssertionError('late mutation dispatched')
    backend.clients={'/test':Client()}
    class Trigger:
        class Request: pass
    backend.Trigger=Trigger
    cancel=threading.Event(); cancel.set()
    with pytest.raises(InterruptedError): backend.trigger('/test',cancel)


def test_no_direct_hardware_or_unbounded_code_execution():
    import ast
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    for relative in ('workshop/blockly_program.py','workshop/blockly_ros.py','workshop/blockly_recording.py','tools/run_blockly_station.py'):
        tree=ast.parse((root/relative).read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Name):
                assert node.func.id not in {'eval','exec','compile','__import__'}
            if isinstance(node,(ast.Import,ast.ImportFrom)):
                names=[a.name for a in node.names] if isinstance(node,ast.Import) else [node.module or '']
                assert not any(n.split('.')[0] in {'serial','lerobot','scservo_sdk','subprocess'} for n in names)


def test_control_preflight_has_one_origin_header(tmp_path):
    import http.client
    from tools.run_blockly_station import Application,make_server
    app=Application(tmp_path); server=make_server(app,0,control_only=True,frontend_port=8877)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        client=http.client.HTTPConnection('127.0.0.1',server.server_port)
        client.request('OPTIONS','/api/program/stop',headers={'Origin':'http://127.0.0.1:8877'})
        response=client.getresponse(); assert response.status==204
        assert len([value for key,value in response.getheaders() if key.lower()=='access-control-allow-origin'])==1
        client.close()
    finally: server.shutdown();server.server_close();app.close()


def test_recovery_requires_six_real_torque_off_values():
    from workshop.blockly_ros import ROSBackend
    backend=ROSBackend.__new__(ROSBackend);backend.lock=threading.Lock();backend.pending_mutations=set();backend.action_lock=threading.Lock();backend.active_handle=None
    backend.stop=lambda:None;backend._identity=lambda _:{}
    state={'owner_faulted':False,'stop_latched':False,'active_operation':None,'teleop_state':'disabled'}
    backend.trigger=lambda endpoint: state if endpoint=='/follower/status' else {'workflow':{'state':'idle','fault':None},'stop':{'pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    for torque in (None,{},dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),1)):
        state['torque']=torque
        with pytest.raises(RuntimeError): backend.recover()
    state['torque']=dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),0)
    assert backend.recover()['recovered']


def test_action_cancel_failure_does_not_skip_station_stop():
    from workshop.blockly_ros import ROSBackend
    backend=ROSBackend.__new__(ROSBackend);backend.action_lock=threading.Lock()
    class Handle:
        def cancel_goal_async(self): raise RuntimeError('action transport down')
    backend.active_handle=Handle();calls=[]
    def trigger(endpoint,**_):
        calls.append(endpoint)
        if endpoint=='/station/stop': return {'request_id':'fresh-stop'}
        return {'stop':{'request_id':'fresh-stop','pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    backend.trigger=trigger
    assert backend._stop()['action_cancel_error']=='action transport down'
    assert calls==['/station/stop','/station/status']


def test_unknown_mutation_blocks_second_but_stop_pending_is_removed():
    from workshop.blockly_ros import ROSBackend
    from concurrent.futures import Future
    backend=ROSBackend.__new__(ROSBackend);backend.lock=threading.Lock();backend.dispatch_lock=threading.Lock();backend.pending_mutations=set()
    class Client:
        calls=0;removed=0
        def service_is_ready(self): return True
        def call_async(self,_): self.calls+=1;return Future()
        def remove_pending_request(self,_): self.removed+=1
    client=Client();backend.clients={'/mutation':client,'/station/stop':client}
    class Trigger:
        class Request: pass
    backend.Trigger=Trigger
    def timeout(*_): raise TimeoutError('unknown delivery')
    backend._wait=timeout
    with pytest.raises(TimeoutError): backend.trigger('/mutation')
    assert len(backend.pending_mutations)==1
    with pytest.raises(RuntimeError): backend.trigger('/mutation')
    with pytest.raises(TimeoutError): backend.trigger('/station/stop')
    assert len(backend.pending_mutations)==1 and client.calls==2 and client.removed==1


def test_repeated_stop_coalesces_pending_action_cancellation():
    from workshop.blockly_ros import ROSBackend
    from concurrent.futures import Future
    backend=ROSBackend.__new__(ROSBackend);backend.action_lock=threading.Lock();backend.lock=threading.Lock();backend.pending_mutations=set();backend.pending_cancel=None
    class Handle:
        calls=0
        def cancel_goal_async(self): self.calls+=1;return Future()
    handle=Handle();backend.active_handle=handle
    for _ in range(10): backend._cancel_handle(handle)
    assert handle.calls==1 and len(backend.pending_mutations)==1
    backend._clear_handle(handle)
    assert not backend.pending_mutations and backend.active_handle is None


def test_ev3_motion_requires_workflow_job_id():
    from workshop.blockly_ros import ROSBackend
    import time
    backend=ROSBackend.__new__(ROSBackend);backend.trigger=lambda *args,**kwargs:{'accepted':True}
    for command in ('home','left_end','right_end','eject'):
        with pytest.raises(RuntimeError): backend.command({'op':'ev3','command':command},threading.Event(),time.monotonic()+2)


def test_replay_cohort_cannot_change_between_identity_and_sample(monkeypatch):
    from workshop.blockly_ros import ROSBackend
    import sys,types,time
    monkeypatch.setitem(sys.modules,'trajectory_msgs.msg',types.SimpleNamespace(JointTrajectoryPoint=object))
    monkeypatch.setitem(sys.modules,'builtin_interfaces.msg',types.SimpleNamespace(Duration=object))
    identity={'arm_id':'a','calibration_sha256':'b'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64,'owner_generation':1}
    r=Recorder('stroke',identity);r.add([0]*5,0,0);r.add([0]*5,0,.1)
    backend=ROSBackend.__new__(ROSBackend);backend.recordings={'stroke':r.finish()};backend.identity=lambda:identity
    backend.sample=lambda:{'identity':dict(identity,owner_generation=2),'positions':[0]*5,'jaw':0}
    with pytest.raises(RuntimeError,match='cohort changed'):
        backend.command({'op':'replay','name':'stroke','speed':.5},threading.Event(),time.monotonic()+2)
