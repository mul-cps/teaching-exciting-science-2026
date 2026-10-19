"""Raw held-body jaw commissioning uses only ID6 writes and honest terminal proof."""
import copy
import pytest
from test_arm_session import make_session
from workshop.station.arm_session import ArmPlanPreflightError, JOINTS


def held(tmp_path):
    session,arm,clock=make_session(tmp_path);session.initialize();session._ensure_profile()
    arm.bus.data['Torque_Enable']=dict.fromkeys(JOINTS,1)
    session.publish_state(event='sample');arm.bus.writes.clear();arm.bus.goal_history.clear()
    command=dict(command='commission_gripper_held_step',delta_counts=8,
        expected_start_raw=dict(arm.bus.data['Present_Position']),
        owner_session_id=session._owner_session_id,command_id='held-jaw-test')
    return session,arm.bus,clock,command


def test_held_jaw_success_only_jaw_writes_restores_profile(tmp_path):
    s,b,c,cmd=held(tmp_path);before=copy.deepcopy(b.data)
    reply=s.handle(cmd)
    assert reply['ok'] is True and reply['qualified'] is False
    assert reply['state']['event']=='commission_gripper_held_complete'
    assert set(b.data['Torque_Enable'].values())=={1}
    assert b.data['Present_Position']['gripper']==2008
    assert not b.goal_history and all(j=='gripper' for _,j,_ in b.writes)
    assert [v for r,j,v in b.writes if r=='Goal_Position']==[2008]
    assert not any(r=='Torque_Enable' for r,j,v in b.writes)
    for r in ('Acceleration','Goal_Velocity','Torque_Limit'):assert b.data[r]==before[r]
    for j in JOINTS[:-1]:assert b.data['Goal_Position'][j]==before['Goal_Position'][j]
    assert c.value<2


def test_held_jaw_stable_timeout_holds_measured_jaw_and_latches(tmp_path):
    s,b,c,cmd=held(tmp_path);before=copy.deepcopy(b.data);b.blocked_gripper=True
    reply=s.handle(cmd)
    assert reply['ok'] is False and reply['status']=='failed_held_jaw'
    assert reply['hold']['schema']=='failed-held-jaw-v1'
    assert reply['hold']['body_anchor_raw']==cmd['expected_start_raw']
    assert reply['target_reached'] is False and reply['qualified'] is False
    assert set(b.data['Torque_Enable'].values())=={1}
    assert [v for r,j,v in b.writes if r=='Goal_Position']==[2008,2000]
    assert all(j=='gripper' for _,j,_ in b.writes) and not b.goal_history
    for r in ('Acceleration','Goal_Velocity','Torque_Limit'):assert b.data[r]==before[r]
    with pytest.raises(ArmPlanPreflightError):s.handle(cmd)
    s.handle({'command':'stop'});assert set(b.data['Torque_Enable'].values())=={0}


@pytest.mark.parametrize('fault',['alloff','mixed','stale_start','session','bounds','extra','delta'])
def test_held_jaw_bad_admission_no_writes(tmp_path,fault):
    s,b,c,cmd=held(tmp_path)
    if fault=='alloff':b.data['Torque_Enable']=dict.fromkeys(JOINTS,0)
    if fault=='mixed':b.data['Torque_Enable']['gripper']=0
    if fault=='stale_start':cmd['expected_start_raw']['gripper']+=1
    if fault=='session':cmd['owner_session_id']='other'
    if fault=='bounds':b.data['Present_Position']['wrist_roll']=4095;b.data['Goal_Position']['wrist_roll']=4095;cmd['expected_start_raw']['wrist_roll']=4095
    if fault=='extra':cmd['torque_limit']=100
    if fault=='delta':cmd['delta_counts']=17
    with pytest.raises(ArmPlanPreflightError):s.handle(cmd)
    assert not b.writes and not b.goal_history


@pytest.mark.parametrize('fault',['drift','torque','goal','slow','read','stop'])
def test_held_jaw_health_failure_retains_global_shutdown(tmp_path,fault):
    s,b,c,cmd=held(tmp_path);write=b.write
    def inject(reg,j,value,**kw):
        write(reg,j,value,**kw)
        if reg=='Goal_Position':
            if fault=='drift':b.stalled_joints.add('shoulder_pan');b.data['Present_Position']['shoulder_pan']+=3
            if fault=='torque':b.data['Torque_Enable']['shoulder_pan']=0
            if fault=='goal':b.data['Goal_Position']['shoulder_pan']+=1
            if fault=='slow':b.present_read_delays.append(.11)
            if fault=='read':b.sync_read_failures['Present_Position']=10
            if fault=='stop':s._cancel_requested=True;s._cancel_reason='busy_stop'
    b.write=inject
    with pytest.raises(Exception):s.handle(cmd)
    assert set(b.data['Torque_Enable'].values())=={0}
    assert s.saved_ram is None
    assert len([1 for r,j,v in b.writes if r=='Goal_Position'])==1


def test_held_jaw_fault_anchor_does_not_ratchet(tmp_path):
    s,b,c,cmd=held(tmp_path);b.blocked_gripper=True;s.handle(cmd)
    anchor=copy.deepcopy(s._failed_endpoint_hold);b.stalled_joints.add('shoulder_pan');b.data['Present_Position']['shoulder_pan']+=2
    s.handle({'command':'cancel_hold'});assert s._failed_endpoint_hold==anchor
    b.data['Present_Position']['shoulder_pan']+=1
    with pytest.raises(RuntimeError,match='body drift'):s.handle({'command':'cancel_hold'})


@pytest.mark.parametrize('fault',['setup_deadline','source','goal_readback','restore','ram_write','terminal_stop'])
def test_held_jaw_setup_restore_and_terminal_faults_never_claim_hold(tmp_path,monkeypatch,fault):
    from workshop.station import arm_session as module
    s,b,c,cmd=held(tmp_path);write=b.write;intent=s._event;goal_count=0
    def event(name,**kw):
        intent(name,**kw)
        if name=='component_write_intent' and kw.get('register')=='Goal_Position' and not goal_count:
            if fault=='setup_deadline':c.sleep(2.01)
        if name=='component_profile_readback' and kw.get('raw_value')==1 and fault=='source':
            monkeypatch.setattr(module,'owner_source_sha256',lambda path:{})
    def intercept(reg,j,v,**kw):
        nonlocal goal_count
        write(reg,j,v,**kw)
        if reg=='Goal_Position':
            goal_count+=1
            if fault=='goal_readback':b.data[reg][j]=v+1
        if fault=='ram_write' and reg=='Acceleration' and v==1:raise OSError('partial RAM write')
        if fault=='restore' and reg=='Goal_Velocity' and v==220:raise OSError('partial RAM restore')
    b.write=intercept;s._event=event
    record=s._record
    def terminal(value):
        record(value)
        if fault=='terminal_stop' and value.get('event')=='commission_gripper_held_complete':
            s._cancel_requested=True;s._cancel_reason='busy_stop'
    s._record=terminal
    with pytest.raises(Exception):s.handle(cmd)
    assert s._failed_endpoint_hold is None
    assert set(b.data['Torque_Enable'].values())=={0} and s.saved_ram is None
    if fault in {'setup_deadline','source'}:assert goal_count==0


def test_held_jaw_post_install_cancel_keeps_distinct_failure(tmp_path):
    s,b,c,cmd=held(tmp_path);b.blocked_gripper=True;record=s._record
    def terminal(value):
        record(value)
        if value.get('event')=='commission_gripper_held_failed':
            s._cancel_requested=True;s._cancel_reason='trajectory_cancel_hold'
    s._record=terminal
    reply=s.handle(cmd)
    assert reply['status']=='failed_held_jaw' and reply['state']['event']=='read_state'
    assert set(b.data['Torque_Enable'].values())=={1}
    assert [v for r,j,v in b.writes if r=='Goal_Position']==[2008,2000]


def test_held_jaw_caller_alias_raw_bounds():
    from tools.ros_station_action import parse_args
    assert parse_args(['gripper-held-step','-16']).delta_counts==-16
    with pytest.raises(SystemExit):parse_args(['gripper-held-step','17'])
    with pytest.raises(SystemExit):parse_args(['gripper-held-step','0'])


def test_held_jaw_two_second_budget_includes_entry_hash_and_reads(tmp_path,monkeypatch):
    from workshop.station import arm_session as module
    s,b,c,cmd=held(tmp_path);original=module.owner_source_sha256;first=True
    def slow_entry(path):
        nonlocal first
        if first:c.sleep(2.01);first=False
        return original(path)
    monkeypatch.setattr(module,'owner_source_sha256',slow_entry)
    with pytest.raises((TimeoutError,RuntimeError)):
        s.handle(cmd)
    assert not any(r=='Goal_Position' for r,j,v in b.writes)


@pytest.mark.parametrize('value',[True,1.0])
def test_held_jaw_scalar_profile_readback_requires_exact_int(tmp_path,value):
    s,b,c,cmd=held(tmp_path);read=b.read
    def malformed(reg,j,**kw):
        v=read(reg,j,**kw)
        return value if reg=='Acceleration' and v==1 else v
    b.read=malformed
    with pytest.raises(RuntimeError,match='readback differs'):s.handle(cmd)
    assert not any(r=='Goal_Position' for r,j,v in b.writes)
    assert set(b.data['Torque_Enable'].values())=={0}


def test_held_jaw_entry_reads_cannot_reset_two_second_budget(tmp_path):
    s,b,c,cmd=held(tmp_path);read=s._read;injected=False
    def delayed(reg):
        nonlocal injected
        v=read(reg)
        if reg=='Torque_Limit' and not injected:c.sleep(2.01);injected=True
        return v
    s._read=delayed
    with pytest.raises((RuntimeError,TimeoutError)):s.handle(cmd)
    assert not any(r=='Goal_Position' for r,j,v in b.writes)


def test_held_sampler_sdk_workload_and_id6_write_guard(tmp_path):
    import ast,inspect,textwrap
    from workshop.station.arm_session import ArmSession
    tree=ast.parse(textwrap.dedent(inspect.getsource(ArmSession.commission_gripper_step)))
    sample=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='sample')
    reads=[n.args[0].value for n in ast.walk(sample) if isinstance(n,ast.Call) and
        ast.unparse(n.func)=='self._read']
    assert reads==['Present_Position','Torque_Enable','Goal_Position']
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls & {'self.bus.sync_write','self._ensure_profile','self._run_leg','self._enable_current_goal'}
    writer=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='write')
    sdk=[n for n in ast.walk(writer) if isinstance(n,ast.Call) and
        ast.unparse(n.func) in {'self.bus.write','self.bus.read'}]
    assert len(sdk)==2
    for call in sdk:
        assert call.args[1].value=='gripper'
        assert {kw.arg:ast.literal_eval(kw.value) for kw in call.keywords}=={'normalize':False,'num_retry':0}
    # The inherited calibration filesystem guard is explicit per sample, not a zero-I/O claim.
    assert 'self._calibration_guard' in {ast.unparse(n.func) for n in ast.walk(sample) if isinstance(n,ast.Call)}
    s,b,c,cmd=held(tmp_path);read=s._read;event=s._event;inventory=[];samples=[]
    def traced(reg):inventory.append(reg);return read(reg)
    def observed(name,**kw):
        event(name,**kw)
        if name=='component_raw_observation':samples.append(inventory[-3:])
    s._read=traced;s._event=observed;s.handle(cmd)
    assert len(samples)>=5 and all(x==reads for x in samples)
