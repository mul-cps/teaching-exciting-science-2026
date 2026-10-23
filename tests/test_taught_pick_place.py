"""Named skills fail closed before any physical execution."""
import json
import threading
import time
import pytest
from workshop.blockly_program import ProgramRunner, SimulationBackend, validate_program
from workshop.blockly_skills import SkillLibrary, validate_skill_result


def test_named_ast_only():
    row={'op':'pick_place_recording','name':'fixture_left','speed':.5}
    assert validate_program([row])==[row]
    with pytest.raises(ValueError): validate_program([{**row,'path':'/tmp/a'}])
    with pytest.raises(ValueError): validate_program([{**row,'speed':float('nan')}])


def test_missing_manifest_rejected(tmp_path):
    library=SkillLibrary(tmp_path)
    with pytest.raises(ValueError,match='not found'): library.load('missing',physical=True)


@pytest.mark.parametrize('field',['pickup_verified','lift_verified','release_verified'])
def test_incomplete_evidence_rejected(field):
    result={'state':'pick_place_complete','track_id':'red-0001','pickup_verified':True,'lift_verified':True,'release_verified':True}
    result[field]=False
    with pytest.raises(RuntimeError): validate_skill_result(result,'red-0001')


def test_wrong_track_rejected():
    with pytest.raises(RuntimeError): validate_skill_result({'state':'pick_place_complete','track_id':'red-0002','pickup_verified':True,'lift_verified':True,'release_verified':True},'red-0001')


def test_simulation_explicit_assumptions():
    backend=SimulationBackend()
    result=ProgramRunner(backend).execute([{'op':'pick_place_recording','name':'demo','speed':.5}],threading.Event())
    skill=result['taught_skills'][0]
    assert skill['simulation'] is True and skill['physical_evidence'] is False
    assert skill['pickup_verified'] is False
    assert backend.state.brick=='place' and backend.state.grasp_verified is False


def test_runner_rejects_fake_physical_result():
    class Backend:
        physical=True
        def command(self,*args): return {'state':'pick_place_complete'}
        def stop(self): self.stopped=True
    backend=Backend()
    with pytest.raises(RuntimeError): ProgramRunner(backend).execute([{'op':'pick_place_recording','name':'demo','speed':.5}],threading.Event())
    assert backend.stopped


def fixture_skill(tmp_path,*,origin='physical',qualified=True):
    import hashlib
    from workshop.blockly_recording import Recorder
    identity={'arm_id':'test-arm','calibration_sha256':'b'*64,'source_sha256':'c'*64,'owner_generation':1.}
    def save(filename,value):
        path=tmp_path/filename; path.write_text(json.dumps(value))
        return {'path':filename,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    artifacts={key:save(key+'.json',{}) for key in ('jaw_calibration','tcp_calibration','camera_arm_calibration')}
    config={key:entry['path'] for key,entry in artifacts.items()}
    config['effective_model_sha256']='d'*64
    artifacts['joint_calibration']=save('joint_calibration.json',{'schema':'so101-gripper-joint-calibration-v1','qualified':True,'arm_id':identity['arm_id'],'calibration_sha256':identity['calibration_sha256']})
    artifacts['execution_config']=save('execution.json',config)
    identity.update(jaw_calibration_sha256=artifacts['jaw_calibration']['sha256'],joint_calibration_sha256=artifacts['joint_calibration']['sha256'])
    phases={}
    for phase in ('approach','lift','transfer','return'):
        recording=Recorder(phase,identity,origin=origin)
        recording.add([0.]*5,0.,0.);recording.add([0.]*5,0.,.1)
        phases[phase]=save(phase+'.stroke',recording.finish())
    data={'schema':'so101-taught-pick-place-v1','name':'fixture','origin':origin,'qualified':qualified,
          'qualification':{'instructor':'fixture only','evidence_sha256':'e'*64},
          'binding':{**{k:v for k,v in identity.items() if k!='owner_generation'},'effective_model_sha256':'d'*64},
          'frame_id':'base_link','tip_link':'gripper_frame_link',
          'fixture':{'id':'test','provenance_sha256':'f'*64},
          'checkpoints':{k:[0.,0.,0.,0.,0.,0.,1.] for k in ('pickup','drop','approach_start','return')},
          'artifacts':artifacts,'phases':phases}
    save('fixture.json',data)
    return data,identity


@pytest.mark.parametrize('mutation,match',[
    (lambda d:d['phases'].pop('lift'),'four taught'),
    (lambda d:d.update(origin='synthetic_simulation'),'synthetic'),
    (lambda d:d.update(qualified=False),'uncommissioned'),
    (lambda d:d['phases']['lift'].update(path='../escape'),'outside'),
    (lambda d:d.update(path='/tmp/a'),'schema'),
])
def test_manifest_admission_guards(tmp_path,mutation,match):
    data,identity=fixture_skill(tmp_path);mutation(data)
    (tmp_path/'fixture.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match=match): SkillLibrary(tmp_path).load('fixture',physical=True,identity=identity)


def test_restart_revalidates_but_changed_source_reteaches(tmp_path):
    data,identity=fixture_skill(tmp_path)
    library=SkillLibrary(tmp_path)
    assert library.status()['skills']['fixture']['state']=='revalidation_required'
    assert library.load('fixture',physical=True,identity={**identity,'owner_generation':2.})
    for field in ('arm_id','calibration_sha256','source_sha256'):
        with pytest.raises(ValueError,match='changed'):
            library.load('fixture',physical=True,identity={**identity,field:'changed'})


def test_hash_changed_rejected(tmp_path):
    fixture_skill(tmp_path)
    (tmp_path/'jaw_calibration.json').write_text('{} ')
    with pytest.raises(ValueError,match='hash changed'): SkillLibrary(tmp_path).load('fixture',physical=True)


def test_synthetic_recording_hidden_in_physical_manifest_rejected(tmp_path):
    import hashlib
    data,identity=fixture_skill(tmp_path)
    path=tmp_path/'lift.stroke';stroke=json.loads(path.read_text());stroke['origin']='synthetic_simulation';path.write_text(json.dumps(stroke))
    data['phases']['lift']['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    (tmp_path/'fixture.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='synthetic recording'): SkillLibrary(tmp_path).load('fixture',physical=True)


def test_phase_start_jaw_and_cohort_gates(tmp_path):
    from workshop.station.taught_phases import admit_phase,require_phase_cohort
    from workshop.blockly_recording import JOINTS
    _,identity=fixture_skill(tmp_path)
    recording=SkillLibrary(tmp_path).load('fixture')['recordings']['lift']
    measured={**dict.fromkeys(JOINTS,0.),'gripper':0.}
    assert admit_phase(recording,identity,measured,.5)
    with pytest.raises(RuntimeError,match='two degrees'): admit_phase(recording,identity,{**measured,JOINTS[0]:.04},.5)
    with pytest.raises(RuntimeError,match='fixed jaw'): admit_phase(recording,identity,{**measured,'gripper':.02},.5)
    with pytest.raises(RuntimeError,match='cohort'): require_phase_cohort({**identity,'owner_generation':2.},identity,measured)
    with pytest.raises(RuntimeError,match='jaw changed'): require_phase_cohort(identity,identity,{**measured,'gripper':.02},0.)


def test_cancel_before_skill_dispatch_stops():
    backend=SimulationBackend();cancel=threading.Event();cancel.set()
    with pytest.raises(InterruptedError): ProgramRunner(backend).execute([{'op':'pick_place_recording','name':'demo','speed':.5}],cancel)
    assert backend.state.placed_count==0 and backend.state.segment=='stopped'


def test_client_named_builder_and_export():
    from workshop.blockly_client import Program,export_python
    rows=Program().pick_place_recording('fixture',.3).program
    assert rows==[{'op':'pick_place_recording','name':'fixture','speed':.3}]
    compile(export_python(rows,physical=True),'<export>','exec')


def test_application_missing_physical_library_before_job(tmp_path):
    from tools.run_blockly_station import Application
    app=Application(tmp_path)
    app.backend.physical=True
    try:
        with pytest.raises(ValueError,match='not found'): app.run([{'op':'pick_place_recording','name':'fixture','speed':.5}])
        assert app.job['state']=='idle' and app.thread is None
    finally: app.close()


def test_status_catalog_memory_only_and_defensive(tmp_path,monkeypatch):
    fixture_skill(tmp_path);library=SkillLibrary(tmp_path)
    def forbidden(*args,**kwargs): raise AssertionError('status performed disk IO')
    monkeypatch.setattr(type(tmp_path),'glob',forbidden)
    monkeypatch.setattr(type(tmp_path),'read_bytes',forbidden)
    state=library.status();state['skills'].clear()
    assert 'fixture' in library.status()['skills']


def test_catalog_discovery_capped_before_sort(tmp_path,monkeypatch):
    yielded=[]
    def candidates(self,pattern):
        for index in range(1000):
            yielded.append(index)
            yield tmp_path/(f'file{index}.json')
    monkeypatch.setattr(type(tmp_path),'glob',candidates)
    library=SkillLibrary(tmp_path)
    assert len(yielded)==64 and len(library.status()['skills'])==64


def test_loaded_calibration_status_requires_exact_digests(tmp_path):
    from workshop.blockly_skills import require_taught_calibration_status
    data,_=fixture_skill(tmp_path)
    status={'si_gap_calibrated':True,'moveit_joint_calibrated':True,'visual_proxy_only':False,
            'jaw_calibration_sha256':data['artifacts']['jaw_calibration']['sha256'],
            'joint_calibration_sha256':data['artifacts']['joint_calibration']['sha256']}
    require_taught_calibration_status(status,data)
    for field,value in [('joint_calibration_sha256','0'*64),('jaw_calibration_sha256',None),('moveit_joint_calibrated',False),('visual_proxy_only',True)]:
        with pytest.raises(RuntimeError): require_taught_calibration_status({**status,field:value},data)


def test_helper_only_emits_uncommissioned_draft(tmp_path):
    from workshop.station.taught_commission import draft
    data,identity=fixture_skill(tmp_path)
    evidence=tmp_path/'evidence.txt';evidence.write_text('synthetic fixture only')
    path=draft(tmp_path,'draft',execution_config=tmp_path/'execution.json',joint_calibration=tmp_path/'joint_calibration.json',
        phases={phase:tmp_path/entry['path'] for phase,entry in data['phases'].items()},checkpoints=data['checkpoints'],
        fixture_id='test',fixture_evidence=evidence,instructor='fixture',qualification_evidence=evidence,frame_id='base_link',tip_link='gripper_frame_link')
    assert json.loads(path.read_text())['qualified'] is False
    with pytest.raises(ValueError,match='uncommissioned'): SkillLibrary(tmp_path).load('draft',physical=True)


@pytest.mark.parametrize('field',['jaw_calibration_sha256','joint_calibration_sha256'])
def test_taught_phase_calibration_binding_must_match_pinned_artifact(tmp_path,field):
    data,identity=fixture_skill(tmp_path)
    data['binding'][field]='0'*64
    (tmp_path/'fixture.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='pinned artifact'):
        SkillLibrary(tmp_path).load('fixture',physical=True)
