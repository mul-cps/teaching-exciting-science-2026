"""Versioned historical evidence never grants current source or motion authority."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest
from test_derived_registration import fixture
from test_perception_recovery_core import proof
from so101_station_perception.derived_lineage import statement_pin
from so101_station_perception.admission_proof import pin_digest

ROOT=Path(__file__).resolve().parents[1]
CONSUMERS=('/so101_charuco_world','/so101_colored_objects')


def api():
    path=ROOT/'ros2/src/so101_station_perception/so101_station_perception/completion_event.py'
    assert path.exists(),'completion event core not implemented'
    spec=importlib.util.spec_from_file_location('so101_station_perception.completion_event',path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def inputs():
    pin=statement_pin(fixture())
    context=dict(boot_id='11111111-2222-3333-4444-555555555555',monotonic_clock_domain='CLOCK_MONOTONIC',
        coordinator_instance_id='1'*32,job_id='2'*32,accepted_reset_id='3'*32,
        producer_generation=pin['producer_generation'],source_pin_sha256=pin_digest(pin),
        artifact_sha256='4'*64,baseline_sha256='5'*64,
        job_started_monotonic=99.,job_deadline_monotonic=139.,job_start_ros_ns=9_000_000_000)
    return context


def witness(index=0,consumer=0):
    v=fixture();v['sequence']=index+1
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):
        v['original'][key]+=index*200_000_000
    v['health']['source_receipts_monotonic']=dict.fromkeys(v['health']['source_receipts_monotonic'],100.1+index*.2)
    v['health']['source_expiry_monotonic']=100.35+index*.2
    p=proof(v);p['consumer_epoch']=('e' if consumer==0 else 'f')*32
    p['checked_ros_ns']=10_100_000_000+index*200_000_000
    p['original_receipt_expiry_monotonic']=100.3+index*.2
    return api().record_witness(p,v,request_mono=100.12+index*.2,
        observed_done_mono=100.13+index*.2,take_mono=100.14+index*.2,
        consumed_mono=100.15+index*.2,consumed_ros_ns=10_150_000_000+index*200_000_000,
        rpc_deadline=100.4+index*.2,lineage_receipt=100.11+index*.2,lineage_gid='6'*32)


def event():
    return api().prepare_completion_event(inputs(),{
        c:dict(old_epoch='b'*32,new_epoch=('e' if i==0 else 'f')*32,
            witnesses=[witness(0,i),witness(1,i)]) for i,c in enumerate(CONSUMERS)},
        checked_mono=100.4,checked_ros_ns=10_400_000_000)


def verify(value,**kwargs):
    return api().validate_completion_event(value,inputs(),received_mono=kwargs.get('mono',100.6),
        received_ros_ns=kwargs.get('ns',10_600_000_000),job_deadline=139.)


def response_for(e):
    consumers={c:dict(latest=deepcopy(v['witnesses'][-1]['proof']),witnesses=[deepcopy(w['proof']) for w in v['witnesses']]) for c,v in e['consumers'].items()}
    return dict(schema='so101-perception-recovery-v1',state='completed',job_id=e['job_id'],accepted_reset_id=e['accepted_reset_id'],
        producer_generation=e['producer_generation'],source_pin_sha256=e['source_pin_sha256'],artifact_sha256=e['artifact_sha256'],
        artifact='/trusted/'+e['job_id']+'.json',deadline_monotonic=e['job_deadline_monotonic'],consumers=consumers,
        operation_success=True,ready_now=False,qualified=False,execution_enabled=False,completion_event=e,
        current_readiness=dict(schema='so101-current-source-readiness-v1',ready=False,checked_monotonic=100.5,checked_ros_ns=10_500_000_000,
            reason='expired',original_receipt_valid_until_monotonic=None,original_identity_valid_until_monotonic=None,
            original_local_lineage_valid_until_monotonic=None,effective_valid_until_monotonic=None,original_header_valid_until_ros_ns=None,
            qualified=False,execution_enabled=False))


def test_frozen_legacy_negative_cannot_be_retrofitted():
    raw=(ROOT/'tests/fixtures/completion_event_legacy_negative.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()=='fee440e7ffba7f9825db8f72c02b770e1ed6c993fc2da046fb8a5b7224fd574a'
    f=json.loads(raw);row=f['raw_rows'][1];v=json.loads(row['raw'])
    assert v['operation_success'] is True and v['ready_now'] is False
    expiry=v['consumers'][CONSUMERS[0]]['latest']['original_receipt_expiry_monotonic']
    assert row['request_monotonic']<expiry<row['done_monotonic']<row['take_monotonic']
    assert f['expected']['must_not_relabel_original_negative'] is True
    with pytest.raises(ValueError):verify(v.get('completion_event'))


def test_history_valid_at_consumption_is_not_current_at_late_receipt():
    e=event();v=verify(e)
    assert v['historical_completion_valid'] is True and v['observed_within_job_budget'] is True
    assert v['current_readiness'] is False and v['qualified'] is False and v['execution_enabled'] is False
    assert e['consumers'][CONSUMERS[0]]['witnesses'][0]['proof']['original_receipt_expiry_monotonic']<e['event_checked_monotonic']
    assert verify(e,mono=140.,ns=50_000_000_000)['observed_within_job_budget'] is False


@pytest.mark.parametrize('field,bad',[('boot_id','9'*36),('monotonic_clock_domain','OTHER'),('job_id','a'*32),('source_pin_sha256','a'*64)])
def test_wrong_context_never_becomes_past_only(field,bad):
    e=event();e[field]=bad
    with pytest.raises(ValueError):verify(e)


@pytest.mark.parametrize('kind',['nan','bool','future','rpc_order','receipt_future','identity_future','header','source_expiry','gid'])
def test_witness_original_clock_contradictions_reject(kind):
    m=api();v=fixture();p=proof(v);kw=dict(request_mono=100.12,observed_done_mono=100.13,take_mono=100.14,
        consumed_mono=100.15,consumed_ros_ns=10_150_000_000,rpc_deadline=100.4,lineage_receipt=100.11,lineage_gid='6'*32)
    if kind=='nan':kw['consumed_mono']=float('nan')
    elif kind=='bool':kw['consumed_ros_ns']=True
    elif kind=='future':p['checked_ros_ns']=11_000_000_000
    elif kind=='rpc_order':kw['take_mono']=100.1
    elif kind=='receipt_future':kw['lineage_receipt']=100.2
    elif kind=='identity_future':v['identity']['request_started_monotonic']=100.2
    elif kind=='header':p['original_header_expiry_ros_ns']+=1
    elif kind=='source_expiry':v['health']['source_expiry_monotonic']+=.1
    elif kind=='gid':kw['lineage_gid']='a'*32
    with pytest.raises(ValueError):m.record_witness(p,v,**kw)


@pytest.mark.parametrize('kind',['duplicate','reordered','missing','health_schema','expired_latest','future_event','late_job','nonfinite','oversize'])
def test_event_negative_matrix(kind):
    e=event();w=e['consumers'][CONSUMERS[0]]['witnesses']
    if kind=='duplicate':w[1]=deepcopy(w[0])
    elif kind=='reordered':w.reverse()
    elif kind=='missing':w.pop()
    elif kind=='health_schema':w[0]['proof']['schema']='so101-derived-source-binding-health-v1'
    elif kind=='expired_latest':e['event_checked_monotonic']=100.51
    elif kind=='future_event':e['event_checked_ros_ns']=20_000_000_000
    elif kind=='late_job':e['event_checked_monotonic']=140.
    elif kind=='nonfinite':w[0]['consumed_monotonic']=float('inf')
    elif kind=='oversize':e['scope']='x'*9000
    # Rehash to prove semantic validation, not just checksum, rejects contradictions.
    with pytest.raises(ValueError):
        e['event_sha256']=api().event_digest(e)
        verify(e)


def test_event_immutable_and_digest_covers_all_originals():
    e=event();before=json.dumps(e,sort_keys=True);verify(e);assert json.dumps(e,sort_keys=True)==before
    changed=deepcopy(e);changed['consumers'][CONSUMERS[0]]['witnesses'][0]['proof']['original_receipt_expiry_monotonic']+=.001
    with pytest.raises(ValueError):verify(changed)
    assert len(api().canonical_json(e))<=8192


def test_current_readiness_independent_effective_original_limits():
    m=api();r=dict(schema='so101-current-source-readiness-v1',ready=True,checked_monotonic=100.2,checked_ros_ns=10_200_000_000,
        reason=None,original_receipt_valid_until_monotonic=100.4,original_identity_valid_until_monotonic=100.3,
        original_local_lineage_valid_until_monotonic=100.35,effective_valid_until_monotonic=100.3,
        original_header_valid_until_ros_ns=10_500_000_000,qualified=False,execution_enabled=False)
    assert m.validate_current_readiness(r,take_mono=100.25,take_ros_ns=10_250_000_000)
    assert not m.validate_current_readiness(r,take_mono=100.31,take_ros_ns=10_310_000_000)
    r['checked_monotonic']=100.5
    with pytest.raises(ValueError):m.validate_current_readiness(r,take_mono=100.31,take_ros_ns=10_310_000_000)


def test_offline_new_verifier_rejects_legacy_negative_and_accepts_separate_history(tmp_path,capsys):
    import importlib.util
    spec=importlib.util.spec_from_file_location('verify_event',ROOT/'tools/verify_completion_event.py')
    tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    e=event();context=tmp_path/'context.json';response=tmp_path/'response.json'
    context.write_text(json.dumps(inputs()))
    r=response_for(e)
    response.write_text(json.dumps(r));args=['--response',str(response),'--expected-context',str(context),'--take-monotonic','100.6','--take-ros-ns','10600000000']
    assert tool.main(args)==0;out=json.loads(capsys.readouterr().out)
    assert out['historical_completion_valid'] and not out['current_readiness'] and not out['observation_qualified']
    old=json.loads((ROOT/'tests/fixtures/completion_event_legacy_negative.json').read_text())
    response.write_text(old['raw_rows'][1]['raw']);assert tool.main(args)==2
    assert json.loads(capsys.readouterr().out)['historical_completion_valid'] is False


def test_core_static_has_no_clock_io_ros_or_unbounded_witness_work():
    import ast
    path=ROOT/'ros2/src/so101_station_perception/so101_station_perception/completion_event.py'
    tree=ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node,(ast.Import,ast.ImportFrom)):
            names=[x.name for x in node.names]
            assert not any(x.split('.')[0] in ('rclpy','torch','numpy','socket','threading','time','os','subprocess') for x in names)
        if isinstance(node,ast.Call):
            assert not (isinstance(node.func,ast.Name) and node.func.id in ('open','eval','exec'))
    assert "len(c['witnesses'])==2" in path.read_text()


def test_exact_event_8192_boundary_and_one_byte_over_cap():
    m=api();selected=None
    for digits in range(10,60):
        e=event();offset=10**digits;ctx=inputs();ctx['job_start_ros_ns']+=offset
        for key in ('job_start_ros_ns','event_checked_ros_ns','latest_header_valid_until_ros_ns'):e[key]+=offset
        for c in e['consumers'].values():
            for w in c['witnesses']:
                w['consumed_ros_ns']+=offset
                w['original']={k:v+offset for k,v in w['original'].items()}
                for k in ('depth_stamp_ns','color_stamp_ns','checked_ros_ns','original_header_expiry_ros_ns'):w['proof'][k]+=offset
                w['proof']['scope']=''
        left=8192-len(m.canonical_json(e))
        if 0<=left<1024:selected=(e,ctx,left);break
    assert selected is not None
    e,ctx,left=selected
    for c in e['consumers'].values():
        for w in c['witnesses']:
            n=min(left,256);w['proof']['scope']='x'*n;left-=n
    assert left==0
    e['event_sha256']=m.event_digest(e);assert len(m.canonical_json(e))==8192
    assert m.validate_completion_event(e,ctx,received_mono=100.6,received_ros_ns=e['event_checked_ros_ns']+1,job_deadline=139.)['historical_completion_valid']
    target=next(w for c in e['consumers'].values() for w in c['witnesses'] if len(w['proof']['scope'])<256)
    target['proof']['scope']+='x';e['event_sha256']=m.event_digest(e)
    assert len(m.canonical_json(e))==8193
    with pytest.raises(ValueError,match='exceeds8192'):m.validate_completion_event(e,ctx,received_mono=100.6,received_ros_ns=e['event_checked_ros_ns']+1,job_deadline=139.)


@pytest.mark.parametrize('key,bad',[('original_receipt_valid_until_monotonic',-1.),('original_identity_valid_until_monotonic',False),
    ('original_local_lineage_valid_until_monotonic',float('inf')),('effective_valid_until_monotonic','<invalid>'),('original_header_valid_until_ros_ns',-1)])
def test_false_readiness_checks_known_malformed_fields_before_negative(key,bad):
    r=response_for(event())['current_readiness'];r[key]=bad
    with pytest.raises(ValueError):api().validate_current_readiness(r,take_mono=100.6,take_ros_ns=10_600_000_000)


@pytest.mark.parametrize('key,bad',[('accepted_reset_id','a'*32),('producer_generation','b'*32),('source_pin_sha256','b'*64),
    ('artifact_sha256','b'*64),('deadline_monotonic',140.),('artifact','/other/job.json')])
def test_full_status_event_context_binding_rejects_all_outer_mismatches(key,bad):
    r=response_for(event());r[key]=bad
    with pytest.raises(ValueError):api().validate_status_event(r,inputs(),received_mono=100.6,received_ros_ns=10_600_000_000)
