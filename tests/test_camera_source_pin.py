"""Camera-only pins are distinct from physical registration source cohorts."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_perception'),str(ROOT/'tests')]
from test_derived_registration import fixture,endpoints
from test_aligned_cohort import messages as base_messages

def messages(value):
    result=base_messages(value)
    for message in result.values():message.distortion_model="plumb_bob"
    return result


def api():
    path=ROOT/'ros2/src/so101_station_perception/so101_station_perception/camera_source_pin.py'
    assert path.exists(),'camera-only source pin helper missing'
    from so101_station_perception import camera_source_pin
    return camera_source_pin


def advanced(value):
    from copy import deepcopy
    result=deepcopy(value);result['sequence']+=1
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):result['original'][key]+=1
    for key in ('aligned_stamp_ns','cloud_stamp_ns'):result['outputs'][key]+=1
    return result


def feed(core,value):
    for kind in ('depth','color'):
        info=messages(value)[kind+'_info']
        core.info(kind,info,value['source'][kind+'_info_publisher_gid'],now_ns=10_120_000_000,now_mono=100.12)
    core.lineage(json.dumps(value),value['outputs']['status_publisher_gid'],now_ns=10_120_000_000,now_mono=100.12)


def prepared():
    module=api();core=module.CameraPins();value=fixture();feed(core,value)
    with pytest.raises(ValueError,match='two advancing'):core.select(now_ns=10_120_000_000,now_mono=100.12)
    value=advanced(value);feed(core,value)
    selected=core.select(now_ns=10_120_000_000,now_mono=100.12)
    matrix=[[float(i==j) for j in range(4)] for i in range(4)]
    for i in range(3):matrix[i][3]=value['calibration']['translation_m'][i]
    from so101_station_perception.derived_lineage import native_snapshot
    identity=dict(serial_number=value['source']['serial_number'],source_snapshot=native_snapshot(value),
        request_started_monotonic=100.,confirmed_monotonic=100.1,confirmed_ros_ns=10_100_000_000)
    return module,core,value,selected,matrix,identity


def test_two_advancing_pairs_exact_info_and_distinct_artifact():
    module,core,value,selected,matrix,identity=prepared()
    graph=endpoints(value)
    core.verify(selected,lookup=graph.__getitem__,color_from_depth=matrix,identity=identity,
                now_ns=10_120_000_000,now_mono=100.12)
    artifact=module.artifact(selected,identity,source_hashes={'helper':'a'*64})
    assert artifact['schema']=='so101-camera-source-v1' and artifact['qualified'] is False
    assert artifact['derived_source']['pin']==core.ledger.pin
    assert not set(artifact)&{'model_sha256','owner_session_id','world_from_base','fit_eligibility_at_capture'}
    assert artifact['camera_evidence']['lineage']['received_monotonic']==100.12
    assert artifact['camera_evidence']['lineage']['publisher_gid']==value['outputs']['status_publisher_gid']


def test_no_old_statement_fallback_and_info_duplicates_do_not_renew():
    module,core,value,selected,matrix,identity=prepared()
    core.info('depth',messages(value)['depth_info'],value['source']['depth_info_publisher_gid'],now_ns=10_120_000_000,now_mono=100.13)
    assert core.infos['depth'][-1][2]==100.12
    newer=advanced(value);core.lineage(json.dumps(newer),newer['outputs']['status_publisher_gid'],now_ns=10_120_000_000,now_mono=100.13)
    with pytest.raises(ValueError,match='exact native'):core.select(now_ns=10_120_000_000,now_mono=100.13)


def test_generation_change_after_device_rpc_latches_and_rpc_expiry_rejects():
    module,core,value,selected,matrix,identity=prepared();graph=endpoints(value)
    with pytest.raises(ValueError,match='lease'):
        core.verify(selected,lookup=graph.__getitem__,color_from_depth=matrix,identity=identity,now_ns=10_120_000_000,now_mono=101.01)
    graph['/camera/color/camera_info'][0].endpoint_gid=b'\xaa'*16
    with pytest.raises(ValueError):core.verify(selected,lookup=graph.__getitem__,color_from_depth=matrix,identity=identity,now_ns=10_120_000_000,now_mono=100.12)
    assert core.ledger.invalid_reason


def test_exclusive_complete_json_publication_and_no_partial_artifact(tmp_path):
    module=api();output=tmp_path/'source.json';evidence=tmp_path/'source.evidence.json'
    module.publish_exclusive({output:{'complete':True},evidence:{'complete':True}},check=lambda:None)
    assert json.loads(output.read_text())=={'complete':True}
    with pytest.raises(FileExistsError):module.publish_exclusive({output:{'bad':True},evidence:{'bad':True}},check=lambda:None)
    assert json.loads(output.read_text())=={'complete':True}
    other=tmp_path/'other.json'
    def failed():raise ValueError('expired after serialization')
    with pytest.raises(ValueError):module.publish_exclusive({other:{'complete':True}},check=failed)
    assert not other.exists()


def test_publication_final_lease_failure_rolls_back_only_own_links(tmp_path):
    module=api();output=tmp_path/'source.json';sidecar=tmp_path/'source.evidence.json';calls=[]
    def expire_after_publish():
        calls.append(True)
        if len(calls)==2:
            assert json.loads(output.read_text())=={'complete':True}
            raise ValueError('original lease expired before successful command exit')
    with pytest.raises(ValueError):module.publish_exclusive({sidecar:{'evidence':True},output:{'complete':True}},check=expire_after_publish)
    assert not output.exists() and not sidecar.exists()
    assert not list(tmp_path.iterdir())


def test_replayed_exposures_do_not_satisfy_two_pairs_and_buffers_bounded():
    module=api();core=module.CameraPins();value=fixture();feed(core,value);feed(core,value)
    assert core.advancing_pairs==1
    with pytest.raises(ValueError,match='two advancing'):core.select(now_ns=10_120_000_000,now_mono=100.12)
    for _ in range(20):value=advanced(value);feed(core,value)
    assert len(core.infos['depth'])==4 and len(core.ledger.entries)==8
    replay=advanced(value);replay['original']['color_stamp_ns']-=1;replay['original']['pair_delta_ns']=abs(replay['original']['depth_stamp_ns']-replay['original']['color_stamp_ns'])
    with pytest.raises(ValueError):feed(core,replay)
    assert core.ledger.invalid_reason


def test_independent_rpc_endpoint_snapshot_must_match_all_four_gids():
    module,core,value,selected,matrix,identity=prepared();graph=endpoints(value)
    identity['source_snapshot']['endpoints']['color_info']['gid']='a'*32
    with pytest.raises(ValueError,match='generation/serial'):
        core.verify(selected,lookup=graph.__getitem__,color_from_depth=matrix,identity=identity,now_ns=10_120_000_000,now_mono=100.12)
    assert core.ledger.invalid_reason


def test_info_future_and_noninteger_exposure_never_advance_observations():
    module=api();core=module.CameraPins();value=fixture();info=messages(value)['color_info'];info.header.stamp.sec+=1000
    with pytest.raises(ValueError,match='future'):core.info('color',info,value['source']['color_info_publisher_gid'],now_ns=10_120_000_000,now_mono=100.12)
    assert not core.infos['color']
    info.header.stamp.sec=10.;
    with pytest.raises(ValueError,match='timestamp'):core.info('color',info,value['source']['color_info_publisher_gid'],now_ns=10_120_000_000,now_mono=100.12)


def test_unsupported_unbounded_native_info_not_retained():
    module=api();core=module.CameraPins();value=fixture();info=messages(value)['depth_info'];info.d=[0.]*100
    with pytest.raises(ValueError,match='bounded'):core.info('depth',info,value['source']['depth_info_publisher_gid'],now_ns=10_120_000_000,now_mono=100.12)
    assert not core.infos['depth']


def test_no_overwrite_race_preserves_foreign_artifact_and_rolls_back_evidence(tmp_path,monkeypatch):
    module=api();artifact=tmp_path/'source.json';evidence=tmp_path/'source.evidence.json';original=module.os.link
    def raced_link(source,destination):
        if destination==artifact:
            artifact.write_text('{"foreign":true}')
            raise FileExistsError(artifact)
        return original(source,destination)
    monkeypatch.setattr(module.os,'link',raced_link)
    with pytest.raises(FileExistsError):module.publish_exclusive({evidence:{'complete':True},artifact:{'complete':True}},check=lambda:None)
    assert json.loads(artifact.read_text())=={'foreign':True} and not evidence.exists()


def test_metadata_only_intake_and_hashing_outside_callbacks_static_guard():
    import ast
    path=ROOT/'ros2/src/so101_station_perception/so101_station_perception/camera_source.py'
    text=path.read_text();tree=ast.parse(text)
    assert 'PointCloud2' not in text and 'imgmsg_to_cv2' not in text and 'from sensor_msgs.msg import Image' not in text
    init=next(fn for fn in ast.walk(tree) if isinstance(fn,ast.FunctionDef) and fn.name=='__init__')
    calls=[n for n in ast.walk(init) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_subscription']
    assert len(calls)==3
    assert {ast.unparse(call.args[0]) for call in calls}=={'String','CameraInfo'}
    for name in ('_lineage','_info','_depth_info','_color_info'):
        fn=next(fn for fn in ast.walk(tree) if isinstance(fn,ast.FunctionDef) and fn.name==name)
        source=ast.get_source_segment(text,fn)
        assert not any(s in source for s in ('read_bytes','open(','.copy(', 'imgmsg_to_cv2','DeviceInfo.Request'))
