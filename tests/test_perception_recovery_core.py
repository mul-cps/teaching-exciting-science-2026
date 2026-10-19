"""No ROS, GPU, pixels or devices: recovery metadata invariants."""
import json
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest

from so101_station_perception.recovery_core import (Budget, NativePairs, ConsumerProgress,
    stable_pin, bounded_json)
from so101_station_perception.derived_lineage import statement_pin
from so101_station_perception.admission_proof import pin_digest
from test_derived_registration import fixture


def info(stamp):
    return NS(header=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='optical'),
              width=8,height=6,k=[4.,0.,2.,0.,4.,2.,0.,0.,1.],d=[0.]*5,distortion_model='plumb_bob')


def test_native_two_advancing_pairs_after_start_and_duplicates_never_renew():
    pairs=NativePairs(start_ns=10_000_000_000,start_mono=100.)
    graph={'node':'/camera','endpoints':{k:{'node':'/camera','gid':('%02x'%(i+1))*16}
           for i,k in enumerate(('depth','color','depth_info','color_info'))}}
    for n in range(2):
        stamp=10_010_000_000+n*20_000_000
        for kind in ('depth','color'):
            pairs.add(kind,info(stamp),graph['endpoints'][kind+'_info']['gid'],now_ns=stamp+1_000_000,now_mono=100.01+n*.02)
        ready=pairs.select(graph,now_ns=stamp+2_000_000,now_mono=100.02+n*.02)
        assert ready is (n==1)
    for kind in ('depth','color'):
        pairs.add(kind,info(stamp),graph['endpoints'][kind+'_info']['gid'],now_ns=stamp+3_000_000,now_mono=100.24)
    assert not pairs.select(graph,now_ns=10_300_000_000,now_mono=100.30)
    assert pairs.infos['depth'][-1]['receipt']==100.03


def test_budget_is_absolute_and_rpc_budget_cannot_restart_job_clock():
    b=Budget(100.)
    assert b.rpc_deadline(139.)==140.
    b.check(139.9)
    with pytest.raises(ValueError):b.check(140.01)
    with pytest.raises(ValueError):b.check(99.)


def test_stable_pin_allows_only_runtime_endpoint_generations():
    a=statement_pin(fixture());b=deepcopy(a)
    b['producer_generation']='f'*32;b['source_generation']='e'*64
    b['source']['depth_publisher_gid']='1'*32;b['outputs']['aligned_publisher_gid']='2'*32
    assert stable_pin(a)==stable_pin(b)
    b['implementation']['producer_node_sha256']='3'*64
    assert stable_pin(a)!=stable_pin(b)


def proof(value,mono=100.1):
    o=value['original']
    return dict(schema='so101-derived-admission-status-v1',qualified=False,execution_enabled=False,
        state='ready',consumer_epoch='e'*32,source_pin_sha256=pin_digest(statement_pin(value)),
        producer_generation=value['producer_generation'],sequence=value['sequence'],
        depth_stamp_ns=o['depth_stamp_ns'],color_stamp_ns=o['color_stamp_ns'],
        checked_ros_ns=10_100_000_000,original_receipt_expiry_monotonic=100.2,
        original_header_expiry_ros_ns=min(o[k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'))+500_000_000)


def test_consumer_replay_wrong_epoch_pin_generation_and_original_expiry():
    v=fixture();p=proof(v)
    c=ConsumerProgress('a'*32,statement_pin(v),start_ns=9_000_000_000)
    c.observe(p,v,now_ns=10_110_000_000,now_mono=100.11)
    c.observe(p,v,now_ns=10_120_000_000,now_mono=100.12)
    assert not c.complete
    changed=deepcopy(p);changed['original_receipt_expiry_monotonic']+=.001
    with pytest.raises(ValueError):c.observe(changed,v,now_ns=10_120_000_000,now_mono=100.12)
    for key,bad in [('consumer_epoch','a'*32),('producer_generation','f'*32),('source_pin_sha256','b'*64)]:
        changed=deepcopy(p);changed[key]=bad
        with pytest.raises(ValueError):c.observe(changed,v,now_ns=10_120_000_000,now_mono=100.12)
    with pytest.raises(ValueError):c.check(v,now_ns=10_300_000_000,now_mono=100.3)


def test_bounded_status_json_rejects_nonfinite_and_unbounded():
    with pytest.raises(ValueError):bounded_json({'bad':float('nan')})
    with pytest.raises(ValueError):bounded_json({'bad':'x'*20000})


def test_native_future_metadata_cannot_poison_highwater_and_graph_swap_latches():
    n=NativePairs(start_ns=10_000_000_000,start_mono=100.)
    with pytest.raises(ValueError):n.add('depth',info(10_999_000_000),'1'*32,now_ns=10_100_000_000,now_mono=100.1)
    assert n.high['depth']==0
    assert n.add('depth',info(10_010_000_000),'1'*32,now_ns=10_020_000_000,now_mono=100.02)
    n.graph={'node':'/camera','endpoints':{'depth_info':{'gid':'2'*32}}}
    with pytest.raises(ValueError):n.add('depth',info(10_030_000_000),'3'*32,now_ns=10_040_000_000,now_mono=100.04)
    assert n.invalid


def test_latest_consumer_clear_cannot_preserve_old_readiness():
    v=fixture();c=ConsumerProgress('b'*32,statement_pin(v),start_ns=9_000_000_000)
    p=proof(v);c.observe(p,v,now_ns=10_110_000_000,now_mono=100.11)
    v2=deepcopy(v);v2['sequence']+=1
    for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):v2['original'][k]+=20_000_000
    c.observe(proof(v2),v2,now_ns=10_120_000_000,now_mono=100.12)
    assert c.complete
    c.clear_current();assert not c.complete
    with pytest.raises(ValueError):c.check(v2,now_ns=10_120_000_000,now_mono=100.12)
