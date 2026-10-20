"""Artifact-only deterministic actual-method reproduction, no ROS init or IO."""
from copy import deepcopy
from types import SimpleNamespace as NS
from pathlib import Path
import json
import pytest
from test_derived_registration import fixture
from so101_station_perception import perception_supervisor as m
from so101_station_perception.derived_lineage import statement_pin,validate_statement
from so101_station_perception.admission_proof import pin_digest


@pytest.mark.parametrize('mode',['source_expired','identity_expired','future','context','all_expired'])
def test_expired_original_sources_in_older_retained_row_do_not_hide_later_current_candidate(monkeypatch,mode):
    old=fixture();current=deepcopy(old)
    for v in (old,current):v['producer_generation']='b'*32
    current['sequence']=2
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):
        current['original'][key]+=200_000_000
    current['outputs']['cloud_stamp_ns']=current['outputs']['aligned_stamp_ns']=current['original']['depth_stamp_ns']
    current['health']['source_receipts_monotonic']=dict.fromkeys(m.TOPICS,100.3)
    current['health'].update(source_expiry_monotonic=100.55,observation_ros_ns=10_300_000_000)
    if mode=='identity_expired':old['identity'].update(request_started_monotonic=99.3,confirmed_monotonic=99.31,expires_monotonic=100.3)
    if mode=='identity_expired':old['health']['source_expiry_monotonic']=100.3
    # Both ORIGINAL records had passed real intake at these clocks.
    validate_statement(old,now_ns=10_120_000_000,now_mono=100.12,receipt_mono=100.12)
    validate_statement(current,now_ns=10_320_000_000,now_mono=100.32,receipt_mono=100.32)
    if mode=='future':
        old['health']['source_receipts_monotonic']['depth']=100.5
    if mode=='context':old['calibration']['sha256']='0'*64
    pin=statement_pin(current);job='c'*32;digest='d'*64;reset='e'*32
    path=Path('/synthetic-owned-artifacts')/(job+'.json')
    value=dict(operation_success=True,state='completed',producer_generation='b'*32,accepted_reset_id=reset,
        artifact=str(path),artifact_sha256=digest,source_pin_sha256=pin_digest(pin),consumers={})
    for consumer in m.CONSUMERS:
        value['consumers'][consumer]={'latest':dict(schema='so101-derived-admission-status-v1',state='ready',qualified=False,execution_enabled=False,
            consumer_epoch='f'*32,producer_generation='b'*32,source_pin_sha256=pin_digest(pin))}
    saved=[]
    node=NS(pin=statement_pin(fixture()),original_pin=statement_pin(fixture()),expected_generation='b'*32,accepted_reset_id=reset,
        recovery_directory=path.parent,policy=NS(job={'job_id':job,'deadline':140.},completed=lambda _:saved.append('completed')),
        epochs={c:'a'*32 for c in m.CONSUMERS},raw_lineages=[(json.dumps(old),'6'*32,100.2),(json.dumps(current),'6'*32,100.32)],
        _baseline_unchanged=lambda:None,_graph=lambda:None,get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_400_000_000)),
        pre_outage=NS(clear=lambda:None),reconciliation=None,_persist=lambda:saved.append('persist'))
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:100.4))
    monkeypatch.setattr(m,'read_artifact',lambda _:({'derived_source':{'pin':pin}},digest))
    if mode=='all_expired':node.raw_lineages=node.raw_lineages[:1]
    if mode in ('future','context','all_expired'):
        with pytest.raises(m.AdoptionPending if mode=='all_expired' else ValueError):m.PerceptionSupervisor._adopt_completed(node,value)
        assert not saved and node.pin!=pin
        return
    m.PerceptionSupervisor._adopt_completed(node,value)
    assert saved==['completed','persist'] and node.pin==pin
    assert [r[0]['sequence'] for r in node.ledger.entries]==[2]
    assert node.ledger.entries[0][2]==100.32  # ORIGINAL local receipt, not decision100.4.


def test_bounded_cold_adoption_metadata_microbenchmark(monkeypatch):
    import os,time,statistics
    original=m.PerceptionSupervisor._adopt_completed;samples=[]
    def measure(node,value):
        node.raw_lineages=node.raw_lineages[:1]*7+node.raw_lineages[-1:]
        began=time.perf_counter_ns();result=original(node,value)
        samples.append((time.perf_counter_ns()-began)/1000.)
        return result
    monkeypatch.setattr(m.PerceptionSupervisor,'_adopt_completed',measure)
    for _ in range(110):
        test_expired_original_sources_in_older_retained_row_do_not_hide_later_current_candidate(monkeypatch,'source_expired')
    values=sorted(samples[10:])
    result=dict(scope='actual cold _adopt_completed with8retainedrows7past/1current, real validation/deepcopy/ledger; fakegraph/artifactread/journal; no ROSinit/RPC/SDK/GPU',
        samples=len(values),mean_us=statistics.mean(values),p95_us=values[94],max_us=max(values),raw_rows=8)
    if os.environ.get('ADOPTION_HISTORY_BENCHMARK_OUTPUT'):
        Path(os.environ['ADOPTION_HISTORY_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')


def test_monitor_waits_only_for_current_lineage_then_adopts_same_owned_job(monkeypatch):
    clock=[100.];calls=[];job=dict(state='accepted',job_id='c'*32,deadline=102.)
    node=NS(policy=NS(job=job),pin={'producer_generation':'a'*32},expected_generation=None,accepted_reset_id=None,health_error=None)
    node._take=lambda _: (True,dict(job_id='c'*32,state='completed',producer_generation='b'*32,accepted_reset_id='e'*32),clock[0])
    def adopt(value):
        calls.append(value['job_id'])
        if len(calls)==1:raise m.AdoptionPending('waiting current lineage')
    node._adopt_completed=adopt
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    m.PerceptionSupervisor._monitor_job(node)
    assert node.policy.job is job and node.health_error=='waiting current lineage'
    clock[0]=100.2;m.PerceptionSupervisor._monitor_job(node)
    assert calls==['c'*32,'c'*32] and node.policy.job is job
    clock[0]=102.01
    with pytest.raises(ValueError,match='absolute deadline'):m.PerceptionSupervisor._monitor_job(node)
    assert len(calls)==2


def test_adoption_wait_does_not_swallow_unknown_or_deadline_crossing(monkeypatch):
    clock=[100.];job=dict(state='accepted',job_id='c'*32,deadline=102.)
    node=NS(policy=NS(job=job),pin={'producer_generation':'a'*32},expected_generation=None,accepted_reset_id=None,health_error=None)
    node._take=lambda _: (True,dict(job_id='c'*32,state='completed',producer_generation='b'*32,accepted_reset_id='e'*32),clock[0])
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    def invalid(value):raise ValueError('context contradiction')
    node._adopt_completed=invalid
    with pytest.raises(ValueError,match='context contradiction'):m.PerceptionSupervisor._monitor_job(node)
    def slow_pending(value):clock[0]=102.01;raise m.AdoptionPending('past only')
    node._adopt_completed=slow_pending
    with pytest.raises(ValueError,match='absolute deadline'):m.PerceptionSupervisor._monitor_job(node)
    assert node.policy.job is job
