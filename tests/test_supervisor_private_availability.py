"""Pure policy/metadata tests; no ROS or hardware."""
from copy import deepcopy
import importlib
import pytest
from so101_station_perception.supervisor_policy import SupervisorPolicy
from so101_station_perception.admission_proof import pin_digest
from so101_station_perception.derived_lineage import statement_pin
from test_derived_registration import fixture


def policy():
    p=SupervisorPolicy();p.clocks(100.,1000.);p.enable(100.);return p


def test_distinct_private_intent_deduplicates_before_collection_and_shares_quota():
    p=policy();fault={'fault_id':'a'*32}
    assert hasattr(p,'begin_private_repair'),'distinct private repair intent missing'
    p.begin_private_repair('b'*32,fault,100.)
    assert p.state=='private_repair_preflight' and list(p.handled)==['a'*32] and not p.attempts
    assert p.events[-1]['event']=='private_availability_repair_requested' and p.events[-1]['monotonic']==100.
    p.reserve_private_repair('b'*32,100.2,1000.2)
    assert p.job==dict(state='dispatch_unknown',reserved_monotonic=100.2,origin='private_availability_repair',repair_id='b'*32,outage_id='a'*32)
    p.accepted('c'*32,140.3,100.3)
    restored=SupervisorPolicy(p.journal());assert restored.job==p.job and restored.latched and not restored.enabled
    p.completed(101.);assert p.state=='warming' and not p.job and list(p.handled)==['a'*32] and len(p.attempts)==1
    with pytest.raises(ValueError):p.begin_private_repair('d'*32,fault,101.)


@pytest.mark.parametrize('field,value',[('origin','unknown'),('repair_id','bad'),('outage_id','bad'),('extra',1)])
def test_strict_new_origin_rejects_unknown_or_malformed_fields(field,value):
    p=policy();job=dict(state='dispatch_unknown',reserved_monotonic=100.,origin='private_availability_repair',repair_id='b'*32,outage_id='a'*32)
    history=p.journal();history['inflight']=job;job[field]=value
    with pytest.raises(ValueError):SupervisorPolicy(history)


def test_binding_unavailable_is_negative_metadata_only_not_health():
    value=fixture();pin=statement_pin(value)
    module=importlib.import_module('so101_station_perception.private_availability')
    proof=dict(schema='so101-derived-source-binding-health-v1',state='unavailable',image_admitted=False,processing_progress=False,
        qualified=False,execution_enabled=False,binding_invalidated=False,consumer_epoch='a'*32,producer_generation=pin['producer_generation'],
        source_pin_sha256=pin_digest(pin),camera_info_validation='independent_native_info',
        identity_validation='producer_attested_device_info_no_new_rpc',optical_tf_validation='local_exact_depth_timestamp')
    assert module.binding_metadata(proof,pin,'a'*32,100.,now=100.1,ns=10_000_000_000) is False
    for change in ({'binding_invalidated':True},{'consumer_epoch':'b'*32},{'producer_generation':'b'*32},{'qualified':True},
                   {'checked_monotonic':101.},{'schema':'so101-derived-admission-status-v1'}):
        with pytest.raises(ValueError):module.binding_metadata(dict(proof,**change),pin,'a'*32,100.,now=100.1,ns=10_000_000_000)


def test_private_budget_and_fault_dedupe_shared_with_original_route():
    p=policy();p.begin_private_repair('b'*32,{'fault_id':'a'*32},100.)
    with pytest.raises(ValueError):p.reserve_private_repair('c'*32,100.1,1000.1)
    p.reserve_private_repair('b'*32,100.1,1000.1);p.accepted('d'*32,140.,100.2);p.completed(100.3)
    p.state='armed'
    with pytest.raises(ValueError):p.outage({'fault_id':'a'*32},100.4)
    p.state='warming';p.begin_private_repair('c'*32,{'fault_id':'d'*32},101.)
    with pytest.raises(ValueError):p.reserve_private_repair('c'*32,101.,1001.)
    assert len(p.attempts)==1 and list(p.handled)==['a'*32,'d'*32]


@pytest.mark.parametrize('change',[lambda v:v.update(checked_monotonic=101.),lambda v:v.update(checked_ros_ns=11_000_000_000),
    lambda v:v['original_header_stamps_ns'].update(depth_stamp_ns=11_000_000_000),
    lambda v:v['receipt_deadlines_monotonic'].update(source_depth=101.),lambda v:v.update(effective_health_expiry_monotonic=102.),
    lambda v:v.update(original_header_expiry_ros_ns=11_000_000_000)])
def test_past_numeric_proof_never_hides_future_or_contradictory_evidence(change):
    from so101_station_perception.private_availability import binding_metadata
    value=fixture();pin=statement_pin(value)
    # Use independent explicit bounded numeric evidence, already PAST at decision.
    headers={k:9_000_000_000 for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')}
    from so101_station_perception.source_binding_health import RECEIPTS
    p=dict(schema='so101-derived-source-binding-health-v1',state='unavailable',image_admitted=False,processing_progress=False,
        qualified=False,execution_enabled=False,binding_invalidated=False,consumer_epoch='a'*32,producer_generation=pin['producer_generation'],
        source_pin_sha256=pin_digest(pin),camera_info_validation='independent_native_info',identity_validation='producer_attested_device_info_no_new_rpc',
        optical_tf_validation='local_exact_depth_timestamp',sequence=1,checked_monotonic=99.,checked_ros_ns=9_100_000_000,
        depth_stamp_ns=9_000_000_000,color_stamp_ns=9_000_000_000,original_header_stamps_ns=headers,
        original_header_expiry_ros_ns=9_500_000_000,receipt_deadlines_monotonic={k:99.2 for k in RECEIPTS},
        original_receipt_expiry_monotonic=99.2,original_identity_expiry_monotonic=99.8,effective_health_expiry_monotonic=99.2)
    assert binding_metadata(p,pin,'a'*32,100.,now=100.1,ns=10_000_000_000) is False
    change(p)
    with pytest.raises(ValueError):binding_metadata(p,pin,'a'*32,100.,now=100.1,ns=10_000_000_000)
