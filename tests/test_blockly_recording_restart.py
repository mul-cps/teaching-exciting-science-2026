import copy
import pytest
from workshop.blockly_recording import Recorder, replay_points

IDENTITY={'arm_id':'robot','calibration_sha256':'a'*64,'owner_generation':1.,'source_sha256':'b'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}


def stroke(origin='physical'):
    recorder=Recorder('stroke',IDENTITY,origin=origin)
    recorder.add([0.]*5,0.,1.); recorder.add([.01]*5,0.,1.1)
    return recorder.finish()


def test_restart_same_robot_requires_live_binding_but_not_old_owner_generation():
    source=stroke(); current={**IDENTITY,'owner_generation':2.}
    assert source['schema']=='so101-blockly-stroke-v2'
    assert source['capture_cohort']['owner_generation']==1.
    assert replay_points(source,current,physical=True)
    assert source['identity']==IDENTITY


@pytest.mark.parametrize('field,value',[('arm_id','other'),('calibration_sha256','c'*64),('source_sha256','c'*64)])
def test_changed_durable_identity_or_model_source_rejected(field,value):
    with pytest.raises(ValueError): replay_points(stroke(),{**IDENTITY,field:value},physical=True)


def test_synthetic_origin_cannot_be_relabelled_by_matching_physical_identity():
    with pytest.raises(ValueError,match='synthetic'): replay_points(stroke('synthetic_simulation'),IDENTITY,physical=True)


def test_schema_binding_and_identity_cannot_disagree():
    source=stroke(); source['binding']['arm_id']='another'
    with pytest.raises(ValueError): replay_points(source,IDENTITY)


def test_legacy_identity_migrates_conservatively_and_preserves_source_hash():
    source=stroke(); source['schema']='so101-blockly-stroke-v1'
    for field in ('binding','capture_cohort','origin'): source.pop(field)
    assert replay_points(source,{**IDENTITY,'owner_generation':8.},physical=True)
    with pytest.raises(ValueError): replay_points(source,{**IDENTITY,'source_sha256':'e'*64},physical=True)


@pytest.mark.parametrize('field',['jaw_calibration_sha256','joint_calibration_sha256'])
def test_changed_loaded_calibration_after_restart_requires_reteaching(field):
    with pytest.raises(ValueError):
        replay_points(stroke(),{**IDENTITY,'owner_generation':2.,field:'e'*64},physical=True)


@pytest.mark.parametrize('field',['jaw_calibration_sha256','joint_calibration_sha256'])
@pytest.mark.parametrize('value',[None,'','unknown','A'*64,'g'*64,False])
def test_physical_capture_requires_qualified_loaded_calibration_digests(field,value):
    identity={**IDENTITY,field:value}
    with pytest.raises(ValueError,match='calibration'):
        Recorder('stroke',identity,origin='physical')


@pytest.mark.parametrize('schema',['so101-blockly-stroke-v1','so101-blockly-stroke-v2'])
def test_old_physical_recording_without_loaded_calibration_binding_requires_reteach(schema):
    source=stroke();source['schema']=schema
    for field in ('jaw_calibration_sha256','joint_calibration_sha256'):
        source['identity'].pop(field);source['binding'].pop(field)
    current={key:value for key,value in IDENTITY.items() if key not in ('jaw_calibration_sha256','joint_calibration_sha256')}
    with pytest.raises(ValueError,match='calibration'):
        replay_points(source,current,physical=True)
