"""Whole cold final gate with real graph validation/read-artifact and fake lookup."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import json,statistics,time
from test_supervisor_consumer_reconciliation_ros import reconciler
from test_supervisor_private_availability_ros import begin,proof
from test_derived_registration import endpoints
from so101_station_perception import perception_supervisor as m


def test_whole_private_final_gate_metadata_and_tempfs(reconciler,monkeypatch):
    node,clock=reconciler;status=begin(node,monkeypatch);clock[0]=100.2
    node.repair_bindings={c:(proof(node),100.1) for c in m.CONSUMERS}
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=10_100_000_000+offset,receipt=100.1) for _ in range(2)) for offset in (0,10_000_000)],
        selected=((100.1,10_110_000_000),)*2,request=100.05,received=100.1)
    records=endpoints(node.baseline['camera_evidence']['lineage']['statement']);gid='9'*32
    records[m.PRODUCER_STATUS]=[NS(node_namespace='/',node_name='so101_rocm_rgbd',endpoint_gid=bytes.fromhex(gid))]
    lookups=[0]
    def lookup(topic):lookups[0]+=1;return records[topic]
    monkeypatch.setattr(node,'get_publishers_info_by_topic',lookup)
    for name in ('_graph','_baseline_unchanged','_producer_now'):
        monkeypatch.setattr(node,name,getattr(m.PerceptionSupervisor,name).__get__(node))
    node.producer=(status,gid,100.1);node.repair_coordinator_receipt=100.1
    def measure(call,n):
        for _ in range(3):call()
        samples=[];before=lookups[0]
        for _ in range(n):
            start=time.perf_counter_ns();call();samples.append((time.perf_counter_ns()-start)/1000.)
        return dict(samples=n,mean_us=statistics.mean(samples),p95_us=sorted(samples)[int(.95*(n-1))],max_us=max(samples),graph_lookups_per_call=(lookups[0]-before)/n)
    result=dict(scope='CPU whole final private gate: actual source artifact read/hash+7GID validation+producer endpoint+binding/native scalar guards, fixed synthetic lookup/clock; no RPC/GPU/device',
        final_gate=measure(node._repair_final,1000),status=measure(lambda:m.bounded_json(node._snapshot()),1000),
        journal=measure(lambda:node.journal.write(node.policy.journal(),consumer_epochs=node.epochs,adopted=node.adopted),20))
    assert result['final_gate']['graph_lookups_per_call']==8
    assert not node.rpc and not node.policy.job
    if os.environ.get('PRIVATE_REPAIR_BENCHMARK_OUTPUT'):Path(os.environ['PRIVATE_REPAIR_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')
