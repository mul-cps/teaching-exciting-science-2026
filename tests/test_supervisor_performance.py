"""Bounded CPU metadata benchmark; synthetic native Node, no executor/payload."""
from copy import deepcopy
import json
import os
import statistics
import time
from types import SimpleNamespace as NS
from pathlib import Path
import pytest
assert os.environ.get('ROS_DOMAIN_ID')=='177','Use fixed177 test launcher'
from test_perception_supervisor_ros import supervisor
from so101_station_perception import perception_supervisor as module
from so101_station_perception.derived_lineage import validate_graph
from test_derived_registration import endpoints
from test_source_binding_health import binding_fixture


def test_whole_metadata_health_status_and_transition_journal(supervisor,monkeypatch):
    node,value=supervisor;graph=endpoints(value)
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:100.12,time=time.time))
    monkeypatch.setattr(node,'get_clock',lambda:NS(now=lambda:NS(nanoseconds=10_120_000_000)))
    node.producer=({},None,100.11)
    monkeypatch.setattr(node,'_producer_now',lambda:dict(state='ready',fault=None,producer_generation=node.pin['producer_generation']))
    monkeypatch.setattr(node,'get_publishers_info_by_topic',graph.__getitem__)
    for sequence in range(1,9):
        value['sequence']=sequence
        for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][key]+=1_000_000
        value['outputs']['cloud_stamp_ns']=value['original']['depth_stamp_ns'];value['outputs']['aligned_stamp_ns']=value['original']['depth_stamp_ns']
        node.ledger.add(json.dumps(value),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.11)
    for sequence in range(128):node.policy.event('own_job_completed',100.,job_id='f'*32)
    proof=binding_fixture(value,checked=100.11,checked_ns=10_120_000_000)
    monkeypatch.setattr(node,'_take',lambda _: (True,deepcopy(proof),100.11))
    monkeypatch.setattr(node,'_request',lambda *a:None)
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=value['original'][k+'_info_stamp_ns'],receipt=100.1) for k in ('depth','color')) for _ in range(2)],
        selected=tuple((100.1,value['original'][k+'_info_stamp_ns']) for k in ('depth','color')),request=100.,received=100.01)
    def native_final():node._native_final()
    def health():node._current_health()
    def completion_guard_metadata():node._current_health(start_requests=False)
    def status():module.bounded_json(node._snapshot())
    def context():module.same_auto_context(node.original_pin,node.pin)
    # Whole finite journal serialization/fsync/rename/readback; no ROS RPC.
    def journal():node.journal.write(node.policy.journal(),consumer_epochs=node.epochs,adopted=None)
    def measure(call,count):
        for _ in range(10):call()
        values=[]
        for _ in range(count):
            start=time.perf_counter_ns();call();values.append((time.perf_counter_ns()-start)/1000.)
        values.sort();return dict(samples=count,mean_us=statistics.mean(values),p95_us=values[int(.95*(count-1))],max_us=max(values))
    result=dict(scope='CPU synthetic metadata only; fake graph lookup, no DDS/GPU/payload; journal real temp filesystem',
        bounded_sizes=dict(ledger=8,events=128,event_job_id_chars=32),health_validation=measure(health,1000),completion_guard_metadata=measure(completion_guard_metadata,1000),final_original_native_witness=measure(native_final,1000),status_snapshot_json=measure(status,1000),
        immutable_context_comparison=measure(context,1000),cold_journal_transition=measure(journal,20))
    if os.environ.get('SUPERVISOR_BENCHMARK_OUTPUT'):
        path=Path(os.environ['SUPERVISOR_BENCHMARK_OUTPUT']);path.open('x').write(json.dumps(result,indent=2)+'\n')
    assert set(node.rpc)==set() and node.policy.job is None
