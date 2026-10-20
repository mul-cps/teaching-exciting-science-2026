"""Bounded metadata only; no ROS init, camera, GPU or actuation."""
import json
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))


def test_worker_opt_in_preserves_legacy_take_api():
    from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker
    done = threading.Event()
    payload = object()
    worker = LatestOnlyAnalysisWorker(lambda item: item, on_ready=done.set, diagnostics=True)
    try:
        worker.submit(payload)
        assert done.wait(1)
        result = worker.take_result()
        assert result == (payload, payload, None)
        assert worker.take_result_with_diagnostics() is None
    finally:
        assert worker.close(1)


def test_worker_timing_envelope_keeps_report_identity():
    from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker
    done = threading.Event()
    payload, report = object(), {'same': object()}
    worker = LatestOnlyAnalysisWorker(lambda item: report, on_ready=done.set, diagnostics=True)
    try:
        worker.submit(payload)
        assert done.wait(1)
        result, timing = worker.take_result_with_diagnostics()
        assert result[0] is payload and result[1] is report and result[2] is None
        assert timing['sequence'] == 1
        times = [timing[k] for k in ('submit', 'start', 'analysis_enter', 'analysis_return', 'store_enter')]
        assert times == sorted(times)
        assert worker.take_result() is None
    finally:
        assert worker.close(1)


def test_worker_disabled_does_not_create_timing_envelopes():
    from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker
    worker = LatestOnlyAnalysisWorker(lambda item: item)
    try:
        with pytest.raises(ValueError, match='disabled'):
            worker.take_result_with_diagnostics()
        assert worker.diagnostic_losses is None
    finally:
        assert worker.close(1)


def sample():
    from so101_station_objects.timing_diagnostic import TimingRecorder
    recorder = TimingRecorder()
    row = recorder.begin(None, consumer_epoch='')
    return recorder, row


def test_ring_batch_size_and_loss_accounting():
    recorder, row = sample()
    for _ in range(70):
        row = recorder.begin(None)
        recorder.finish(row, outcome=0, site=1, error='unicode😀' * 100)
    raw = recorder.drain()
    assert len(raw.encode()) <= 8192
    message = json.loads(raw)
    assert len(message['rows']) <= 4
    assert message['counts']['observed'] == 70
    assert message['counts']['ring_dropped'] == 6
    assert message['locally_drained_without_known_loss'] is False
    assert len(recorder.rows) <= 64
    assert len(message['latest_rejections']) == 5


def test_diagnostic_failure_never_escapes():
    recorder, row = sample()
    row['clocks'][0] = float('nan')
    recorder.finish(row, outcome=0, site=1)
    message = json.loads(recorder.drain())
    assert message['counts']['invalid'] == 1 and not message['rows']


def test_deadlines_preserve_original_clocks_and_metadata_only():
    recorder, row = sample()
    assert len(row['clocks']) == 22
    assert not any(key in row for key in ('rgb', 'depth', 'token', 'report'))
    recorder.clock(row, 0, 10.)
    recorder.finish(row, outcome=1, site=0)
    result = json.loads(recorder.drain())['rows'][0]
    assert result[12][0] == 10.


def test_worker_diagnostic_flag_strict_bool():
    from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker
    for bad in (1, 'false', None):
        with pytest.raises(TypeError, match='bool'):
            LatestOnlyAnalysisWorker(lambda x: x, diagnostics=bad)


def test_five_source_check_sites_and_final_publication_order_static():
    import ast
    tree=ast.parse((ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py').read_text())
    publish=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_publish_report')
    final=next(n for n in publish.body if isinstance(n,ast.If) and ast.unparse(n.test)=='derived_token is not None')
    following=publish.body[publish.body.index(final)+1:]
    assert [ast.unparse(n.value.func) for n in following[:4]]==['self.detections_pub.publish','self.markers_pub.publish','self.grasps_pub.publish','self.contacts_pub.publish']
    assert ast.unparse(following[-2]).startswith('TimingRecorder.clock(timing, 13)')
    assert ast.unparse(following[-1])=='if timing is not None:\n    finish_node(self, 1)'
    methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    checks=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='check' and ast.unparse(n.func.value)=='self.derived']
    assert len(checks)==5
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in {'read','write','open','lookup_transform','get_publishers_info_by_topic'} for n in ast.walk(methods['_publish_timing']))


def test_maximum_encoded_row_and_four_row_packet():
    recorder,_=sample()
    for _ in range(4):
        row=recorder.begin(None, consumer_epoch='f'*32)
        row.update(generation='f'*32,sequence=(1<<63)-1,headers=[(1<<63)-1]*4,
                   source=[1.7976931348623157e308]*4,identity=[1.7976931348623157e308]*3+[(1<<63)-1],
                   lineage=1.7976931348623157e308,consumer=[1.7976931348623157e308]*4,
                   clocks=[1.7976931348623157e308]*22,analyze_calls=2)
        recorder.finish(row,outcome=0,site=1,error='\x00'*100)
    raw=recorder.drain();v=json.loads(raw)
    assert len(raw.encode())<=8192 and len(v['rows'])==4
    assert all(len(json.dumps(row,separators=(',',':'),allow_nan=False).encode())<=1536 for row in v['rows'])


def test_one_terminal_per_taken_result_and_shutdown_loss():
    recorder,row=sample()
    recorder.finish(row,outcome=0,site=1);recorder.finish(row,outcome=1,site=0)
    assert recorder.counts['observed']==1
    recorder.close()
    assert not recorder.rows and recorder.counts['shutdown_lost']==1


def test_metadata_snapshot_does_not_keep_token_or_image_references():
    import weakref,gc
    from types import SimpleNamespace as NS
    recorder,_=sample()
    images=[NS() for _ in range(4)]
    # NS is not weak-referenceable: use an ordinary payload object instead.
    class Payload:pass
    images=[Payload() for _ in range(4)];refs=[weakref.ref(x) for x in images]
    token={'statement':{'producer_generation':'a'*32,'sequence':4,
        'original':{k+'_stamp_ns':123 for k in ('depth','color','depth_info','color_info')},
        'health':{'source_receipts_monotonic':{k:1. for k in ('depth','color','depth_info','color_info')}},
        'identity':dict(request_started_monotonic=1.,confirmed_monotonic=1.1,expires_monotonic=2.,confirmed_ros_ns=123)},
        'lineage_receipt':1.1,'records':{k:(image,'gid',1.1,123) for k,image in zip(('color','aligned','color_info','depth_info'),images)}}
    row=recorder.begin(token,consumer_epoch='b'*32)
    recorder.finish(row,outcome=0,site=1)
    del token,images;gc.collect()
    assert all(ref() is None for ref in refs)


def test_counter_overflow_and_context_malformed_never_claim_complete():
    recorder,_=sample()
    recorder.sequence=(1<<63)-1
    row=recorder.begin(None,consumer_epoch='😀'*32)
    recorder.finish(row,outcome=0,site=1)
    body=json.loads(recorder.drain())
    assert body['counter_overflow'] and not body['locally_drained_without_known_loss']
    assert body['counts']['invalid']==1 and body['rows']==[]


def test_malformed_timing_is_diagnostic_error_not_analysis_error():
    from so101_station_objects.timing_diagnostic import worker_timing,count_analyze
    recorder,row=sample()
    worker_timing(row,{})
    count_analyze(row)
    recorder.finish(row,outcome=0,site=1)
    assert json.loads(recorder.drain())['counts']['invalid']==1


def test_legacy_publication_ast_and_pure_algorithms_unchanged():
    import ast,hashlib
    source=ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py'
    tree=ast.parse(source.read_text())
    publish=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_publish_report')
    names={'timing','TimingRecorder','timing_site','finish_node'}
    publish.body=[stmt for stmt in publish.body if not any(isinstance(n,ast.Name) and n.id in names for n in ast.walk(stmt))]
    digest=stable_ast_digest
    assert digest(publish)==BASE_PUBLISH_AST
    actual={node.name:digest(node) for node in tree.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name!='ColoredObjectsNode'}
    assert actual==BASE_ALGORITHM_AST



def stable_ast_digest(node):
    # ast.dump omits empty fields by default on newer Python: pin explicit fields.
    import ast,hashlib,json
    def canonical(value):
        if isinstance(value,ast.AST):
            fields=list(ast.iter_fields(value))
            if isinstance(value,(ast.FunctionDef,ast.ClassDef,ast.AsyncFunctionDef)) and not any(key=='type_params' for key,_ in fields):
                fields.append(('type_params',[]))
            return [type(value).__name__,[[key,canonical(item)] for key,item in fields]]
        if isinstance(value,list):return [canonical(item) for item in value]
        return value
    return hashlib.sha256(json.dumps(canonical(node),separators=(',',':')).encode()).hexdigest()

def test_stable_ast_digest_normalizes_missing_empty_type_params():
    import ast,hashlib,json
    tree=ast.parse('class Example:\n    def method(self):\n        async def nested():\n            pass\n')
    definitions=[node for node in ast.walk(tree) if isinstance(node,(ast.FunctionDef,ast.ClassDef,ast.AsyncFunctionDef))]
    for node in definitions:
        if 'type_params' not in node._fields:
            node._fields=(*node._fields,'type_params')
        node.type_params=[]
    def explicit_fields(value):
        if isinstance(value,ast.AST):return [type(value).__name__,[[key,explicit_fields(item)] for key,item in ast.iter_fields(value)]]
        if isinstance(value,list):return [explicit_fields(item) for item in value]
        return value
    expected=hashlib.sha256(json.dumps(explicit_fields(tree),separators=(',',':')).encode()).hexdigest()
    assert stable_ast_digest(tree)==expected
    for node in definitions:
        del node.type_params
    assert stable_ast_digest(tree)==expected


# Pinned a25b769 canonical explicit-field AST (runtime independent).
BASE_PUBLISH_AST='2a76b9fef3bdf776dc35c8c300c67174f62fc5c68e64a844f4e90281a4e093f0'
BASE_ALGORITHM_AST={'_repository_root': 'f5174ac378828ede48ac13740557f517582f4a738cd60761b09ccee7dc552883', 'SupportPlaneUnavailable': '9f8cb4b65c809f1a6264e36e374507b7438b3e8fb9fa2c06caf5a671550c2199', 'cached_marker_should_clear': 'a63bf4295c1151178a8d3fe9883015e1761072fab7d03eea610c42e0681d2271', 'camera_points': '40ce33a856322e0c63b28c5df03ba0a4ba2c19b6b316a3bd7cde100a0e8234b2', 'transform_points': '23a8cb392e862294d4c1c51014d55e0914680b7f0e39cc5543e62ee8d4f7c8fc', '_orient_plane_up': '8949bf69d9a4c3b492ad774a6f1161905f32fa022d8810353c83b3cd3dcf5f7a', '_fit_world_support': 'aab601bae933565be6315d373c3950da75d76b085e7788735e4475945fa1ebe9', '_plane_world_to_camera': '4c7b56da64cecfaecc79f751fbfb8ea0ba09e08d25001b155b29b7ace8a2e9a3', '_split_elongated_contours': '069b489d5083ff785ed23347bd1fd430f095266ea2ecb01af67e20cc1785d6cb', '_component_geometry': '4f2cfab98246075a1f757c70a479783942603f5737b7a778132fe96b4672941e', '_validate_roi': 'e7f4c4de75966d05436198920110522af70ee69fc250f7a68361ee1cbca46d78', '_inside_roi': '01a4f8686643f7e9d04e89955ddade4e71d48cb8712d48d59407797398e4c835', '_visible_component_geometry': '0fe7f5f91251caf37033e7e5a8d2eff6ad69993ff8243d0fe228617a75ddf987', 'analyze_rgbd': 'e2e97a25dc66c1a012d960ddcfcf8ae9c541576294f1477b9f0253e3ebf6b1b0', 'StableObjectTracker': '5218994e8fe5916adf32aa94b867c5f4cb832f9a5a2b2efc18f46b26cb1f8a2d', 'matrix_quaternion': '8e16e20d77de783642cee6e5f3d92739ddf7bcd4e20c5bd6691b10460ad4f59f', 'grasp_pose_rotation': '71f42501db9532668e71d874d81b2046653e359952dadda8b811f883b7f47e58', 'transform_from_msg': '67abb03d6f5c578d69e2059bc6d3dd2374156314a44e6fec8fb63b7735ed0e58', 'elevated_pose_copy': 'c243c3a8b535c85c2ee07c04b5e2dd0d02ac9641bc3292af2752866d07262b42', 'pregrasp_position': '889828e808d180bb6e52b5f1d8b99c02a4aa0efef6a19e286f5ca6b56c2fb1c1', 'main': '82c03626b39bbb0f2bf3113e86d06ccd8c1937b79c68b2b5ef7e25b9d161abe3'}


def test_worker_shutdown_counts_active_and_pending_without_new_callbacks():
    from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker
    entered,release=threading.Event(),threading.Event()
    def analyze(item):
        entered.set();assert release.wait(1)
        return item
    worker=LatestOnlyAnalysisWorker(analyze,diagnostics=True)
    try:
        worker.submit(1);assert entered.wait(.5)
        assert worker.submit(2) is False
        assert worker.submit(3) is True
        assert worker.pending_replacements==1
        assert worker.close(.001) is False
        assert worker.diagnostic_losses['shutdown']==2
        release.set();assert worker.close(.5)
        assert worker.diagnostic_losses['shutdown']==2 and worker.take_result() is None
    finally:
        release.set();assert worker.close(1)


def test_leaf_launch_uses_typed_default_false_flag():
    import ast
    tree=ast.parse((ROOT/'ros2/src/so101_station_objects/launch/objects.launch.py').read_text())
    declarations=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument' and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value=='analysis_timing_diagnostics']
    assert len(declarations)==1
    assert next(k.value.value for k in declarations[0].keywords if k.arg=='default_value')=='false'
    parameters=[value for n in ast.walk(tree) if isinstance(n,ast.Dict) for key,value in zip(n.keys,n.values) if isinstance(key,ast.Constant) and key.value=='analysis_timing_diagnostics']
    assert len(parameters)==1
    assert isinstance(parameters[0].func,ast.Name) and parameters[0].func.id=='ParameterValue'
    assert next(k.value.id for k in parameters[0].keywords if k.arg=='value_type')=='bool'


def test_recorder_hot_path_only_fixed_metadata_no_image_io_or_sync():
    import ast,inspect
    from so101_station_objects.timing_diagnostic import TimingRecorder
    tree=ast.parse(inspect.getsource(TimingRecorder))
    forbidden={'open','read','write','copy','numpy','cuda','synchronize','lookup_transform','get_publishers_info_by_topic','publish','sleep'}
    assert not any(isinstance(n,ast.Attribute) and n.attr in forbidden for n in ast.walk(tree))
    assert not any(isinstance(n,ast.Name) and n.id in {'cv2','np','rclpy','torch'} for n in ast.walk(tree))


def test_cold_fault_never_misattributed_to_pending_acquisition():
    from types import SimpleNamespace as NS
    from so101_station_objects.timing_diagnostic import site,finish_node
    recorder,row=sample();node=NS(timing_recorder=recorder,_timing_current=row)
    site(row,15);finish_node(node,0,'unassociated newest fault')
    assert recorder.counts['observed']==0 and not row.get('_finished')
    site(row,3);finish_node(node,0,'actual result rejection')
    assert recorder.counts['observed']==1 and recorder.rows[0][2]==3


def test_local_buffer_field_never_claims_full_acquisition_coverage():
    from types import SimpleNamespace as NS
    from so101_station_objects.timing_diagnostic import site,finish_node
    recorder,row=sample()  # Begun but unfinished; worker state is intentionally unknown.
    packet=json.loads(recorder.drain())
    assert 'coverage_complete' not in packet
    assert packet['locally_drained_without_known_loss'] is True
    assert 'not active-job/acquisition/transport completeness' in packet['coverage_scope']
    node=NS(timing_recorder=recorder,_timing_current=row)
    site(row,15);finish_node(node,0,'unassociated fault')
    packet=json.loads(recorder.drain())
    assert 'coverage_complete' not in packet and packet['rows']==[]
    assert packet['locally_drained_without_known_loss'] is True
