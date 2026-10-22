import threading
import time
import pytest
from workshop.blockly_program import ProgramRunner, SimulationBackend, validate_program
from workshop.blockly_ros import ROSBackend
from tools.run_blockly_station import Application


@pytest.mark.parametrize('row', [
    {'op':'arm','command':'restart_all'}, {'op':'arm','command':[]},
    {'op':'arm','command':'home','extra':1},
    *[{'op':'record_stroke','name':'stroke','seconds':v} for v in (True,0,31,float('nan'),'1')],
    {'op':'record_stroke','name':'../stroke','seconds':1},
    {'op':'record_stroke','name':'stroke','seconds':1,'extra':1}])
def test_strict_commands(row):
    with pytest.raises(ValueError): validate_program([row])


def record(name='stroke'): return {'op':'record_stroke','name':name,'seconds':1}


def test_allowlist_and_bounds():
    for command in ('home','moveit','teleop_relative','teleop_absolute','torque_off'):
        validate_program([{'op':'arm','command':command}])
    validate_program([record(),dict(record(),seconds=30)])


@pytest.mark.parametrize('program', [
    [record(),record()], [record('demo')],
    [{'op':'repeat','count':2,'body':[record()]}],
    [record(),{'op':'replay','name':'demo','speed':.5}]])
def test_record_preflight_no_effects(tmp_path,program):
    app=Application(tmp_path)
    with pytest.raises(ValueError): app.run(program)
    assert app.thread is None and app.backend.events==[]


def test_nested_arm_requires_instructor_before_any_effect(tmp_path):
    app=Application(tmp_path); app.backend.physical=True
    program=[{'op':'ev3','command':'wake'}, {'op':'if_color','color':'red','then':[],
        'else':[{'op':'arm','command':'home'}]}]
    with pytest.raises(RuntimeError,match='instructor'): app.run(program)
    assert app.thread is None and app.backend.events==[]


def test_record_library_limit_all_branches(tmp_path):
    app=Application(tmp_path); app.recordings.update({f'x{i}':{} for i in range(62)})
    with pytest.raises(ValueError,match='64'):
        app.run([{'op':'if_color','color':'red','then':[record('one')],'else':[record('two')]}])
    assert app.thread is None


@pytest.mark.parametrize(('command','expected'), [
    ('home',('start',)), ('moveit',('moveit',)),
    ('teleop_relative',('teleop_relative','teleop')),
    ('teleop_absolute',('teleop_absolute','teleop'))])
def test_arm_rpc_order_context(command,expected):
    backend=ROSBackend.__new__(ROSBackend); calls=[]; cancel=threading.Event(); deadline=time.monotonic()+60
    backend.trigger=lambda endpoint,event,end: calls.append((endpoint,event,end)) or {}
    backend.command({'op':'arm','command':command},cancel,deadline)
    assert [call[0] for call in calls]==['/station/operator/'+x for x in expected]
    assert all(event is cancel and end==deadline for _,event,end in calls)


def test_arm_failure_stops_without_retry():
    backend=ROSBackend.__new__(ROSBackend); calls=[]
    def trigger(endpoint,cancel,deadline):
        calls.append(endpoint); raise TimeoutError('unknown outcome')
    backend.trigger=trigger; backend.stop=lambda: calls.append('STOP')
    with pytest.raises(TimeoutError):
        ProgramRunner(backend).execute([{'op':'arm','command':'teleop_relative'}],threading.Event())
    assert calls==['/station/operator/teleop_relative','STOP']


def test_torque_off_whole_stop():
    backend=ROSBackend.__new__(ROSBackend); calls=[]; backend.stop=lambda: calls.append('STOP')
    backend.command({'op':'arm','command':'torque_off'},threading.Event(),time.monotonic()+1)
    assert calls==['STOP']


@pytest.mark.parametrize('backend', [SimulationBackend(),ROSBackend.__new__(ROSBackend)])
def test_record_requires_application(backend):
    with pytest.raises(ValueError,match='Application'):
        backend.command(record(),threading.Event(),time.monotonic()+2)


class FastCancel:
    def __init__(self,clock,cancel_after=None): self.clock=clock; self.cancel_after=cancel_after
    def is_set(self): return self.cancel_after is not None and self.clock[0]>=self.cancel_after
    def wait(self,seconds): self.clock[0]+=seconds; return self.is_set()


def recording_app(tmp_path,monkeypatch,mutate=None):
    app=Application(tmp_path); clock=[100.]; monkeypatch.setattr('tools.run_blockly_station.time.monotonic',lambda:clock[0])
    backend=app.backend; backend.physical=True; count=[0]
    def sample():
        count[0]+=1
        result={'positions':[.1]*5,'jaw':.2,'stamp':clock[0], 'identity':backend.identity()}
        if mutate: mutate(count[0],result)
        return result
    backend.sample=sample
    return app,clock


def test_record_success_and_fixed_jaw(tmp_path,monkeypatch):
    app,clock=recording_app(tmp_path,monkeypatch)
    app._program_command(record(),FastCancel(clock),103.)
    assert app.recorder is None
    assert app.recordings['stroke']['jaw_radians']==.2
    assert len(app.recordings['stroke']['points'])>=10
    assert (tmp_path/'stroke.json').exists()


@pytest.mark.parametrize('failure',['cancel','deadline','stale','jaw','cohort'])
def test_record_failure_no_partial_file(tmp_path,monkeypatch,failure):
    def mutate(index,sample):
        if index>2:
            if failure=='stale': sample['stamp']=100.
            if failure=='jaw': sample['jaw']=.4
            if failure=='cohort': sample['identity']={'arm_id':'changed'}
    app,clock=recording_app(tmp_path,monkeypatch,mutate)
    cancel=FastCancel(clock,100.3 if failure=='cancel' else None)
    with pytest.raises((RuntimeError,ValueError,InterruptedError,TimeoutError)):
        app._program_command(record(),cancel,100.3 if failure=='deadline' else 103.)
    assert app.recorder is None and 'stroke' not in app.recordings
    assert list(tmp_path.iterdir())==[]


def test_program_record_blocks_manual_stop(tmp_path,monkeypatch):
    app,clock=recording_app(tmp_path,monkeypatch)
    original=app.backend.sample; checked=[]
    def sample():
        if app.recorder:
            with pytest.raises(RuntimeError,match='program'): app.record_stop()
            with pytest.raises(RuntimeError): app.record_start('other')
            checked.append(True)
        return original()
    app.backend.sample=sample
    app._program_command(record(),FastCancel(clock),103.)
    assert checked


def test_simulation_record_and_later_replay_memory_only(tmp_path,monkeypatch):
    app=Application(tmp_path); clock=[100.]
    monkeypatch.setattr('tools.run_blockly_station.time.monotonic',lambda:clock[0])
    app._program_command(record(),FastCancel(clock),103.)
    ProgramRunner(app.backend).execute([{'op':'replay','name':'stroke','speed':.5}],threading.Event())
    assert 'stroke' in app.recordings and list(tmp_path.iterdir())==[]
    assert app.backend.events[-1]=='replay:stroke'


def test_duplicate_record_names_across_branches(tmp_path):
    app=Application(tmp_path)
    with pytest.raises(ValueError,match='unique'):
        app.run([{'op':'if_color','color':'red','then':[record()],'else':[record()]}])
    assert app.thread is None


def test_record_preserves_unloaded_file(tmp_path,monkeypatch):
    app,clock=recording_app(tmp_path,monkeypatch)
    path=tmp_path/'stroke.json'; path.write_text('preserve')
    with pytest.raises(FileExistsError): app._program_command(record(),FastCancel(clock),103.)
    assert path.read_text()=='preserve' and 'stroke' not in app.recordings
    assert app.recorder is None


@pytest.mark.parametrize('failure',['owner_stale','joint_stale','cohort','jaw','cancel'])
def test_final_sample_must_be_fresh_exact_and_uncancelled(tmp_path,monkeypatch,failure):
    app,clock=recording_app(tmp_path,monkeypatch); original=app.backend.sample
    cancel=FastCancel(clock)
    def sample():
        if clock[0]>=101.:
            # The terminal collection sample is valid; invalidate the final admission sample.
            calls[0]+=1
            if calls[0]>=2:
                if failure in ('owner_stale','joint_stale'): raise RuntimeError(failure)
                if failure=='jaw':
                    result=original(); result['jaw']=.4; return result
                if failure=='cancel': cancel.cancel_after=clock[0]
                if failure=='cohort':
                    result=original(); result['identity']={'arm_id':'changed'}; return result
        return original()
    calls=[0]; app.backend.sample=sample
    with pytest.raises((RuntimeError,InterruptedError)):
        app._program_command(record(),cancel,103.)
    assert list(tmp_path.iterdir())==[] and 'stroke' not in app.recordings and app.recorder is None


def test_program_failure_stops_and_latches_recovery(tmp_path,monkeypatch):
    app=Application(tmp_path); app.backend.physical=True; app.instructor=True
    app.backend.sample=lambda: (_ for _ in ()).throw(RuntimeError('feedback stale'))
    app.run([record()]); app.thread.join(2)
    assert app.job['state']=='failed' and app.recovery_required
    assert app.backend.events==['STOP'] and app.recorder is None
    assert list(tmp_path.iterdir())==[]


def test_program_record_blocks_other_active_controls(tmp_path,monkeypatch):
    app=Application(tmp_path); app.instructor=True
    started=threading.Event(); release=threading.Event(); original=app.backend.sample
    def sample():
        started.set(); release.wait(2)
        raise RuntimeError('bounded test termination')
    app.backend.sample=sample; app.backend.physical=True
    app.run([record()]); assert started.wait(1)
    try:
        for call in (lambda:app.run([{'op':'log','message':'x'}]),lambda:app.record_start('other'),
                     lambda:app.operator('home')):
            with pytest.raises(RuntimeError,match='program still active'): call()
    finally: release.set(); app.thread.join(2)


def test_arm_preserves_cancellation_between_mode_rpcs():
    backend=ROSBackend.__new__(ROSBackend); calls=[]; cancel=threading.Event()
    def trigger(endpoint,event,deadline):
        if event.is_set(): raise InterruptedError('cancelled')
        calls.append(endpoint); event.set()
    backend.trigger=trigger; backend.stop=lambda: calls.append('STOP')
    with pytest.raises(InterruptedError):
        ProgramRunner(backend).execute([{'op':'arm','command':'teleop_absolute'}],cancel)
    assert calls==['/station/operator/teleop_absolute','STOP']


def test_arm_deadline_stops_before_any_rpc():
    backend=ROSBackend.__new__(ROSBackend); calls=[]
    backend.trigger=lambda *args: calls.append(args)
    backend.stop=lambda: calls.append('STOP')
    with pytest.raises(TimeoutError):
        ProgramRunner(backend,clock=lambda:0).execute([{'op':'arm','command':'home'}],threading.Event())
    assert calls==['STOP']


def test_overfull_simulation_library_allows_existing_programs(tmp_path):
    app=Application(tmp_path)
    app.recordings.update({f'old{i}':{} for i in range(64)})
    app.run([{'op':'replay','name':'demo','speed':.5}, {'op':'log','message':'existing'}])
    app.thread.join(2)
    assert app.job['state']=='complete'
    assert app.backend.events==['replay:demo','existing']
    with pytest.raises(ValueError,match='64'): app.run([record()])
