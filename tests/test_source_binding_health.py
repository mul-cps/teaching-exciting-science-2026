"""Pure source-binding validators: no image admission, ROS init or services."""
from copy import deepcopy
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception'))
from so101_station_perception.source_binding_health import (SCHEMA,HEADERS,RECEIPTS,binding_context,binding_coverage,SourceBindingProgress)
from so101_station_perception.supervisor_health import HealthNotCurrent
from so101_station_perception.derived_lineage import statement_pin
from so101_station_perception.admission_proof import pin_digest
from test_derived_registration import fixture


def binding_fixture(value=None,*,lineage_receipt=100.11,checked=100.12,checked_ns=10_120_000_000):
    value=value or fixture();pin=statement_pin(value)
    deadlines={'consumer_lineage':lineage_receipt+.25,
        **{'source_'+k:v+.25 for k,v in value['health']['source_receipts_monotonic'].items()},
        'consumer_depth_info':100.105+.25,'consumer_color_info':100.106+.25}
    headers={k:value['original'][k] for k in HEADERS};receipt=min(deadlines.values());identity=value['identity']['expires_monotonic']
    return dict(schema=SCHEMA,state='ready',binding_invalidated=False,image_admitted=False,processing_progress=False,qualified=False,execution_enabled=False,
        consumer_epoch='d'*32,source_pin_sha256=pin_digest(pin),producer_generation=pin['producer_generation'],
        sequence=value['sequence'],depth_stamp_ns=headers['depth_stamp_ns'],color_stamp_ns=headers['color_stamp_ns'],
        original_header_stamps_ns=headers,original_header_expiry_ros_ns=min(headers.values())+500_000_000,
        receipt_deadlines_monotonic=deadlines,original_receipt_expiry_monotonic=receipt,
        original_identity_expiry_monotonic=identity,effective_health_expiry_monotonic=min(receipt,identity),
        checked_ros_ns=checked_ns,checked_monotonic=checked,camera_info_validation='independent_native_info',
        identity_validation='producer_attested_device_info_no_new_rpc',optical_tf_validation='local_exact_depth_timestamp')


def coverage(proof,value=None,**changes):
    value=value or fixture();options=dict(lineage_receipt=100.11,now=100.122,ns=10_122_000_000);options.update(changes)
    return binding_coverage(proof,value,statement_pin(value),100.121,**options)


def test_valid_distinct_binding_never_claims_images_or_processing():
    proof=binding_fixture();assert coverage(proof)==100.35
    assert proof['image_admitted'] is False and proof['processing_progress'] is False


@pytest.mark.parametrize('key,value',[('schema','so101-derived-admission-status-v1'),('image_admitted',True),('processing_progress',True),
    ('binding_invalidated',True),('binding_invalidated',None),('binding_invalidated',0),('qualified',True),('execution_enabled',True),('checked_monotonic',100.2),('checked_ros_ns',11_000_000_000),
    ('original_identity_expiry_monotonic',102.),('effective_health_expiry_monotonic',100.5),
    ('original_header_expiry_ros_ns',10_600_000_000),('camera_info_validation','producer_lineage_attested_not_independent'),
    ('identity_validation','independent_camera_rpc'),('optical_tf_validation','latest_transform'),('original_receipt_expiry_monotonic',True)])
def test_invalid_or_future_binding_is_fatal_even_when_old(key,value):
    proof=binding_fixture();proof[key]=value
    with pytest.raises(ValueError) as error:coverage(proof,now=100.4,ns=10_400_000_000)
    assert not isinstance(error.value,HealthNotCurrent)


@pytest.mark.parametrize('mutation',[lambda p:p['receipt_deadlines_monotonic'].pop('consumer_depth_info'),
    lambda p:p['receipt_deadlines_monotonic'].update(source_depth=100.4),
    lambda p:p['receipt_deadlines_monotonic'].update(consumer_color_info=float('nan')),
    lambda p:p['original_header_stamps_ns'].update(depth_info_stamp_ns=10_010_000_000)])
def test_all_original_deadlines_headers_required_before_past_expiry(mutation):
    proof=binding_fixture();mutation(proof)
    with pytest.raises(ValueError) as error:coverage(proof,now=100.4,ns=10_400_000_000)
    assert not isinstance(error.value,HealthNotCurrent)


def test_known_past_original_expiry_is_unavailable_and_does_not_modify_proof():
    proof=binding_fixture();before=deepcopy(proof)
    with pytest.raises(HealthNotCurrent):coverage(proof,now=100.351,ns=10_351_000_000)
    assert proof==before


def test_progress_requires_two_independent_original_metadata_pairs_and_never_renews_duplicates():
    value=fixture();pin=statement_pin(value);proof=binding_fixture(value);progress=SourceBindingProgress(pin)
    progress.observe(proof,value,now_ns=10_122_000_000,now_mono=100.122,response_receipt=100.121,lineage_receipt=100.11)
    assert not progress.complete
    duplicate=deepcopy(proof);duplicate.update(checked_monotonic=100.13,checked_ros_ns=10_130_000_000)
    progress.observe(duplicate,value,now_ns=10_132_000_000,now_mono=100.132,response_receipt=100.131,lineage_receipt=100.11)
    assert progress.response_receipt==100.121 and progress.current==proof
    changed=deepcopy(proof);changed['receipt_deadlines_monotonic']['consumer_lineage']+=.001
    with pytest.raises(ValueError):progress.observe(changed,value,now_ns=10_132_000_000,now_mono=100.132,response_receipt=100.131,lineage_receipt=100.11)
    second=deepcopy(value);second['sequence']=2
    for key in HEADERS:second['original'][key]+=10_000_000
    for key in ('cloud_stamp_ns','aligned_stamp_ns'):second['outputs'][key]=second['original']['depth_stamp_ns']
    second['health']['source_receipts_monotonic']={k:v+.01 for k,v in second['health']['source_receipts_monotonic'].items()}
    second['health']['source_expiry_monotonic']=min(second['health']['source_receipts_monotonic'].values())+.25
    next_proof=binding_fixture(second,lineage_receipt=100.12,checked=100.14,checked_ns=10_140_000_000)
    progress.observe(next_proof,second,now_ns=10_142_000_000,now_mono=100.142,response_receipt=100.141,lineage_receipt=100.12)
    assert progress.complete and progress.check(second,now_ns=10_143_000_000,now_mono=100.143)
    assert progress.current['image_admitted'] is False


def test_exact_recorded_metadata_dds_contract_at_original_client_take_clocks():
    import json
    record=json.loads((Path(__file__).parent/'fixtures/source_binding_health_recorded.json').read_text())
    value=record['matching_lineage'];proof=record['response']
    end=binding_coverage(proof,value,statement_pin(value),record['response_received_monotonic'],
        lineage_receipt=record['consumer_lineage_receipt_monotonic'],now=record['response_received_monotonic'],ns=record['response_received_ros_ns'])
    assert end>record['response_received_monotonic']
    assert proof['image_admitted'] is False and proof['processing_progress'] is False
