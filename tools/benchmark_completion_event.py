#!/usr/bin/env python3
"""CPU-only fixed4-witness whole JSON evidence boundary, no ROS/graph/FS in loop."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_perception'),str(ROOT/'tests')]
from so101_station_perception.completion_event import record_witness,prepare_completion_event,validate_status_event,CONSUMERS
from so101_station_perception.derived_lineage import statement_pin,validate_statement
from so101_station_perception.admission_proof import pin_digest
from so101_station_perception.recovery_core import bounded_json
from test_derived_registration import fixture


def fixtures(maximum=False):
    base=1_000_000_000. if maximum else 100.;nsbase=1_792_000_000_000_000_000 if maximum else 10_000_000_000
    pin=statement_pin(fixture());items=[]
    for consumer in range(2):
        for i in range(2):
            v=fixture();v['sequence']=i+1
            for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):v['original'][k]=nsbase+i*200_000_000
            v['original']['pair_delta_ns']=0
            v['outputs']['cloud_stamp_ns']=v['outputs']['aligned_stamp_ns']=nsbase+i*200_000_000
            receipt=base+.1+i*.2
            v['health'].update(source_receipts_monotonic=dict.fromkeys(v['health']['source_receipts_monotonic'],receipt),source_expiry_monotonic=receipt+.25,observation_ros_ns=nsbase+100_000_000+i*200_000_000)
            v['identity'].update(request_started_monotonic=base,confirmed_monotonic=base+.01,confirmed_ros_ns=nsbase-1,expires_monotonic=base+1.)
            p=dict(schema='so101-derived-admission-status-v1',qualified=False,execution_enabled=False,state='ready',
                scope='x'*(256 if maximum else 68),consumer_epoch=('e' if consumer==0 else 'f')*32,
                source_pin_sha256=pin_digest(pin),producer_generation=pin['producer_generation'],sequence=i+1,
                depth_stamp_ns=v['original']['depth_stamp_ns'],color_stamp_ns=v['original']['color_stamp_ns'],
                checked_ros_ns=nsbase+100_000_000+i*200_000_000,original_receipt_expiry_monotonic=receipt+.2,
                original_header_expiry_ros_ns=nsbase+500_000_000+i*200_000_000)
            kw=dict(request_mono=base+.12+i*.2,observed_done_mono=base+.13+i*.2,take_mono=base+.14+i*.2,
                consumed_mono=base+.15+i*.2,consumed_ros_ns=nsbase+150_000_000+i*200_000_000,
                rpc_deadline=base+.4+i*.2,lineage_receipt=base+.11+i*.2,lineage_gid='6'*32)
            validate_statement(v,now_ns=kw['consumed_ros_ns'],now_mono=kw['consumed_mono'],receipt_mono=kw['lineage_receipt'])
            items.append((CONSUMERS[consumer],p,v,kw))
    context=dict(boot_id='11111111-2222-3333-4444-555555555555',monotonic_clock_domain='CLOCK_MONOTONIC',
        coordinator_instance_id='1'*32,job_id='2'*32,accepted_reset_id='3'*32,producer_generation=pin['producer_generation'],
        source_pin_sha256=pin_digest(pin),artifact_sha256='4'*64,baseline_sha256='5'*64,
        job_started_monotonic=base-1.,job_deadline_monotonic=base+39.,job_start_ros_ns=nsbase-1_000_000_000)
    return items,context,base,nsbase


def whole_boundary(f,legacy_template=None):
    items,context,base,nsbase=f;records={c:dict(old_epoch='b'*32,new_epoch=('e' if i==0 else 'f')*32,witnesses=[]) for i,c in enumerate(CONSUMERS)}
    for consumer,p,v,kw in items:records[consumer]['witnesses'].append(record_witness(p,v,**kw))
    event=prepare_completion_event(context,records,checked_mono=base+.4,checked_ros_ns=nsbase+400_000_000)
    legacy=dict(schema='so101-perception-recovery-v1',state='completed',operation_success=True,ready_now=False,
        qualified=False,execution_enabled=False,job_id=context['job_id'],accepted_reset_id=context['accepted_reset_id'],
        producer_generation=context['producer_generation'],source_pin_sha256=context['source_pin_sha256'],artifact_sha256=context['artifact_sha256'],
        artifact='/trusted/'+context['job_id']+'.json',deadline_monotonic=context['job_deadline_monotonic'],consumers={c:dict(latest=r['witnesses'][-1]['proof'],witnesses=[w['proof'] for w in r['witnesses']]) for c,r in records.items()})
    if legacy_template is not None:
        template=deepcopy(legacy_template)
        for key in ('completion_event','completion_event_unavailable','current_readiness'):template.pop(key,None)
        template.update(legacy);legacy=template
    status=deepcopy(legacy);status['completion_event']=deepcopy(event)
    status['completion_event_unavailable']=None
    status['current_readiness']=dict(schema='so101-current-source-readiness-v1',ready=False,checked_monotonic=base+.6,
        checked_ros_ns=nsbase+600_000_000,reason='original deadlines expired',original_receipt_valid_until_monotonic=base+.5,
        original_identity_valid_until_monotonic=base+1.,original_local_lineage_valid_until_monotonic=base+.56,
        effective_valid_until_monotonic=base+.5,original_header_valid_until_ros_ns=nsbase+700_000_000,qualified=False,execution_enabled=False)
    raw=bounded_json(status);parsed=json.loads(raw)
    validate_status_event(parsed,context,received_mono=base+.6,received_ros_ns=nsbase+600_000_000)
    return len(raw),len(bounded_json(event))


def main():
    p=argparse.ArgumentParser();p.add_argument('--samples',type=int,default=1000);p.add_argument('--output',required=True);p.add_argument('--dds-evidence');a=p.parse_args()
    if not 10<=a.samples<=10000:raise ValueError('bounded samples10..10000 required')
    legacy_template=json.loads(Path(a.dds_evidence).read_text())['original']['completed'] if a.dds_evidence else None
    report=dict(scope='CPU fixed4witness metadata; record/hash/event/deepcopy/whole responseJSON/parse/verify; NO ROS graph/callback/native/process/FS timing',actual_dds_legacy_template=bool(legacy_template),samples=a.samples,warmup=3,cases={})
    for label,maximum in [('typical',False),('large4witness_stress_vector',True)]:
        f=fixtures(maximum)
        for _ in range(3):whole_boundary(f,legacy_template)
        times=[];cpu=time.process_time_ns()
        for _ in range(a.samples):
            start=time.perf_counter_ns();sizes=whole_boundary(f,legacy_template);times.append((time.perf_counter_ns()-start)/1000.)
        report['cases'][label]=dict(mean_us=statistics.mean(times),p95_us=sorted(times)[int(.95*(len(times)-1))],max_us=max(times),
            process_cpu_ms=(time.process_time_ns()-cpu)/1e6,status_bytes=sizes[0],event_bytes=sizes[1])
    Path(a.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
