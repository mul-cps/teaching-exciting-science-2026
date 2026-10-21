"""Bounded offline contract regression for Rig01's encoder coordinate domain."""
import ast
import copy
import inspect
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from workshop.station import ros_trajectory as motion
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_control'))
from so101_station_control.model_offsets import apply_joint_zero_offsets, model_position_limits


def fixture():
    # Frozen raw endpoints from Rig01 calibration and owner observation, not
    # claims of additional mechanical travel (in particular no wrist ±270°).
    endpoints = [(745,3470), (1063,3240), (848,3055), (820,3171), (0,4095), (1872,3366)]
    names = (*motion.ARM_JOINTS, 'gripper')
    calibration = {j: dict(range_min=a, range_max=b) for j,(a,b) in zip(names,endpoints)}
    configured = [(-90,90),(-95.6923076923077,95.6923076923077),
                  (-96.82986737710912,96.82986737710912),(-94.99984017946129,94.99984017946129),(-90,90),(0,100)]
    settings = dict(id='hs_rig01_follower', calibration=dict(sha256='a'*64),
        limits={j:dict(min=a,max=b) for j,(a,b) in zip(names,configured)})
    fw_low = dict(zip(names,[745,1063,848,820,0,1872]))
    fw_high = dict(zip(names,[3470,3240,3055,3171,4095,3366]))
    lo,hi = motion.admitted_raw_limits(calibration,settings['limits'],fw_low,fw_high)
    sources = {'arm_session':'b'*64,'ros_trajectory':'c'*64}
    state = dict(monotonic=10.,torque_monotonic=10.,arm_id=settings['id'],
        calibration_sha256='a'*64,owner_source_sha256=sources,observe_only=True,
        torque=dict.fromkeys(names,0),raw={j:(lo[j]+hi[j])//2 for j in names},
        arm_bounds=dict(schema='arm-bounds-observation-v1',position_monotonic=10.,classification='normal',
            admitted_min=lo,admitted_max=hi,firmware_min=fw_low,firmware_max=fw_high))
    contract=motion.freeze_joint_limit_contract(settings,calibration,state,sources,now=10.1)
    return settings,calibration,sources,state,contract


@pytest.mark.parametrize('degrees',[90.15384615384616,93.67032967032966])
def test_existing_normal_elbow_range_can_plan_without_clamping(degrees):
    settings,calibration,sources,state,contract=fixture()
    limits=motion.validate_joint_limit_contract(contract,settings,calibration,sources)
    assert contract['admitted_min']['elbow_flex']==851
    assert contract['admitted_max']['elbow_flex']==3052
    start=dict.fromkeys(motion.ARM_JOINTS,0.)
    start['elbow_flex']=math.radians(degrees)
    state['raw']['elbow_flex']=round(1951.5+start['elbow_flex']*4095/(2*math.pi))
    motion.check_joint_limit_state(contract,state)
    points=[dict(time=0.,positions=list(start.values())),dict(time=10.,positions=[start[j]-.01 for j in start])]
    prepared=motion.prepare_trajectory(motion.ARM_JOINTS,points,start_radians=start,
        max_velocity=.125,max_acceleration=.2,position_limits=limits)
    assert prepared.samples[0].positions[2]==start['elbow_flex']
    # Portable fallback remains bounded; hardware evidence is required to widen.
    if degrees>math.degrees(motion.POSITION_LIMITS_RAD['elbow_flex'][1]):
        with pytest.raises(ValueError):
            motion.prepare_trajectory(motion.ARM_JOINTS,points,start_radians=start,max_velocity=.125,max_acceleration=.2)


def test_true_outside_admitted_refused_even_if_classification_claims_normal():
    _,_,_,state,contract=fixture()
    state['raw']['elbow_flex']=3053
    with pytest.raises(ValueError,match='outside admitted'):
        motion.check_joint_limit_state(contract,state)


@pytest.mark.parametrize('fault',['source','calibration','settings','bounds','stale','torque','active'])
def test_export_rejects_stale_or_mismatched_evidence(fault):
    settings,calibration,sources,state,_=fixture()
    if fault=='source': state['owner_source_sha256']={}
    if fault=='calibration': state['calibration_sha256']='f'*64
    if fault=='settings': settings['limits']['elbow_flex']['max']=90
    if fault=='bounds': state['arm_bounds']['admitted_min']=dict(state['arm_bounds']['admitted_min'],elbow_flex=1)
    if fault=='stale': state['monotonic']=1.
    if fault=='torque': state['torque']['elbow_flex']=1
    if fault=='active': state['trajectory_goal_id']='active'
    with pytest.raises(ValueError):
        motion.freeze_joint_limit_contract(settings,calibration,state,sources,now=10.1)


def test_model_offset_does_not_shift_encoder_limits_and_other_joints_remain_bounded():
    _,calibration,_,_,contract=fixture()
    limits=contract['position_limits_rad']
    xml='<robot name="test">'+''.join(f'<joint name="{j}" type="revolute"><origin rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-1.57079632679" upper="1.57079632679" velocity="1" effort="1"/></joint>' for j in motion.ARM_JOINTS)+'</robot>'
    shifted=apply_joint_zero_offsets(xml,{'elbow_flex':8})
    model=model_position_limits(shifted,limits)
    assert model_position_limits(model)=={j:tuple(v) for j,v in limits.items()}
    assert ET.fromstring(model).find("joint[@name='elbow_flex']/origin").get('rpy') != '0 0 0'
    for j in motion.ARM_JOINTS:
        assert -math.pi<limits[j][0]<limits[j][1]<math.pi
    assert math.degrees(limits['wrist_roll'][1]-limits['wrist_roll'][0])<181


def test_runtime_contract_comparison_has_no_io_hashes_or_conversion():
    tree=ast.parse(inspect.getsource(motion.check_joint_limit_state))
    calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id for n in ast.walk(tree)
        if isinstance(n,ast.Call) and isinstance(n.func,(ast.Attribute,ast.Name))}
    assert not calls & {'read_bytes','read_text','open','sha256','encoder_position_limits','admitted_raw_limits'}


def test_read_only_export_accepts_healthy_idle_holding_owner_without_disarm():
    settings,calibration,sources,state,_=fixture()
    state['observe_only']=False
    state['torque']=dict.fromkeys(state['torque'],1)
    motion.freeze_joint_limit_contract(settings,calibration,state,sources,now=10.1)


def test_actual_wrist_3022_normal_state_uses_calibrated_model_instead_of_ninety_degrees():
    settings,calibration,sources,state,contract=fixture()
    state['raw']['wrist_flex']=3022
    motion.check_joint_limit_state(contract,state)
    limits=motion.validate_joint_limit_contract(contract,settings,calibration,sources)
    q=(3022-1995.5)*2*math.pi/4095
    assert math.degrees(q)==pytest.approx(90.24175824175825)
    assert math.pi/2<q<limits['wrist_flex'][1]
    start=dict.fromkeys(motion.ARM_JOINTS,0.);start['wrist_flex']=q
    points=[dict(time=0.,positions=list(start.values())),dict(time=2.,positions=list(start.values()))]
    assert motion.prepare_trajectory(motion.ARM_JOINTS,points,start_radians=start,
        max_velocity=.125,max_acceleration=.2,position_limits=limits).samples[0].positions[3]==q


def test_owner_independently_refuses_contract_cohort_mismatch_before_writes(tmp_path):
    from test_arm_session import make_session
    session,arm,clock=make_session(tmp_path)
    session.initialize()
    state=session.publish_state()
    contract=motion.freeze_joint_limit_contract(arm.settings,arm.calibration,state,
        session._owner_source_sha256,now=state["monotonic"]+.001)
    contract['owner_source_sha256']={}
    before=len(arm.bus.writes)
    with pytest.raises(ValueError,match='cohort mismatch'):
        session.execute_trajectory('bad-contract',motion.ARM_JOINTS,[],
            expected_path_sha256='f'*64,joint_limit_contract=contract)
    assert len(arm.bus.writes)==before
    session.close_log()


def test_bootstrap_exports_new_passive_generation_and_exits_before_planner(monkeypatch,tmp_path):
    import hashlib
    import json
    import time
    from tools import freeze_ros_joint_limits as exporter
    from workshop.station.recovery_types import owner_source_sha256
    settings,calibration,_,state,_=fixture()
    calibration_path=tmp_path/'calibration.json'
    calibration_path.write_text(json.dumps(calibration))
    settings['calibration']=dict(path=str(calibration_path),sha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest())
    config=tmp_path/'station.json';config.write_text(json.dumps({'arm':settings}))
    statefile=tmp_path/'state.json';output=tmp_path/'limits.json'
    events=[]
    class Pipe:
        def close(self): events.append('closed')
    class Process:
        stdin=Pipe()
        def __init__(self,command,**kwargs):
            assert '--observe-only' in command
            state.update(monotonic=time.monotonic(),torque_monotonic=time.monotonic(),
                calibration_sha256=settings['calibration']['sha256'],owner_session_id='new-passive-owner',
                owner_source_sha256=owner_source_sha256(Path(exporter.__file__).resolve().parents[1]/'workshop/station'))
            state['arm_bounds']['position_monotonic']=state['monotonic']
            statefile.write_text(json.dumps(state))
        def poll(self): return None
        def wait(self,timeout): events.append('exited');return 0
    def launch(command,**kwargs):
        assert events==['closed','exited']
        assert f'joint_limit_contract:={output}' in command
        assert output.is_file()
        events.append('planner');return 0
    monkeypatch.setattr(exporter.subprocess,'Popen',Process)
    monkeypatch.setattr(exporter.subprocess,'call',launch)
    monkeypatch.setattr(sys,'argv',['freeze', '--config',str(config),'--state-file',str(statefile),
        '--output',str(output),'--bootstrap-owner','--lock',str(tmp_path/'lock'),
        '--log-dir',str(tmp_path/'logs'),'--launch','--','enable_hardware:=true'])
    with pytest.raises(SystemExit) as exit_info: exporter.main()
    assert exit_info.value.code==0
    assert events==['closed','exited','planner']


def test_bridge_and_owner_sampling_calls_pass_explicit_position_limits():
    root=Path(__file__).resolve().parents[1]
    bridge=ast.parse((root/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py').read_text())
    calls=[n for n in ast.walk(bridge) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr in {'prepare_trajectory','prepare_collinear_quintic'}]
    assert len(calls)==2
    assert all('position_limits' in {k.arg for k in call.keywords} for call in calls)
    owner=ast.parse((root/'workshop/station/arm_session.py').read_text())
    calls=[n for n in ast.walk(owner) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='prepare_trajectory']
    assert len(calls)==1 and 'position_limits' in {k.arg for k in calls[0].keywords}


def test_contract_settings_resolve_paths_relative_to_config_like_owner(tmp_path):
    settings=dict(calibration=dict(path='../calibration/arm.json'),home_file='home.json',limits={})
    resolved=motion.resolve_joint_limit_settings(settings,tmp_path/'config/station.json')
    assert resolved['calibration']['path']==str(tmp_path/'calibration/arm.json')
    assert resolved['home_file']==str(tmp_path/'config/home.json')
    assert settings['home_file']=='home.json'
