"""Exact immutable RGB/aligned-depth cohorts without decoding image bytes."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import json

import pytest
from test_derived_registration import fixture

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'ros2/src/so101_station_perception';sys.path.insert(0,str(PACKAGE))


def api():
    assert (PACKAGE/'so101_station_perception/aligned_cohort.py').exists(),'exact aligned admission missing'
    from so101_station_perception.aligned_cohort import AlignedCohorts
    return AlignedCohorts


class Storage:
    def __init__(self,n):self.n=n
    def __len__(self):return self.n


def messages(v=None):
    v=v or fixture();result={}
    for stream,kind in [('color','color'),('aligned','color'),('color_info','color'),('depth_info','depth')]:
        c=v['calibration'];o=v['original'];ns=o['depth_stamp_ns' if stream=='aligned' else stream+'_stamp_ns']
        width,height=c[kind+'_size'];fx,fy,cx,cy=c[kind+'_K']
        result[stream]=NS(header=NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id=o[kind+'_frame']),
            width=width,height=height,k=[fx,0.,cx,0.,fy,cy,0.,0.,1.],d=[0.]*5)
        if stream in ('color','aligned'):
            n=3 if stream=='color' else 2
            result[stream].encoding='rgb8' if stream=='color' else '16UC1'
            result[stream].step=width*n;result[stream].is_bigendian=False;result[stream].data=Storage(width*height*n)
    return result


def gate():
    from so101_station_perception.derived_lineage import validate_statement
    v=fixture();pin=validate_statement(v,now_ns=10_120_000_000,now_mono=100.12,receipt_mono=100.12)
    return api()(pin),v


def feed(g,v,items=None):
    items=items or messages(v)
    for kind,msg in items.items():
        gid=v['outputs']['aligned_publisher_gid'] if kind=='aligned' else v['source'][kind+'_publisher_gid']
        g.receive(kind,msg,bytes.fromhex(gid),now_ns=10_120_000_000,now_mono=100.12)
    g.lineage(json.dumps(v),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    return items


def test_out_of_order_arrivals_select_exact_original_refs_once_no_copies():
    g,v=gate();items=messages(v)
    g.lineage(json.dumps(v),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    assert g.take(now_ns=10_120_000_000,now_mono=100.12) is None
    feed(g,v,items)
    token=g.take(now_ns=10_120_000_000,now_mono=100.12)
    assert token['color'] is items['color'] and token['aligned'] is items['aligned']
    assert token['color_info'] is items['color_info'] and token['lineage_receipt']==100.12
    assert g.take(now_ns=10_120_000_000,now_mono=100.13) is None


def test_nearby_rgb_is_not_exact_rgb_and_never_substituted():
    g,v=gate();items=messages(v);items['color'].header.stamp.nanosec+=1
    feed(g,v,items)
    assert g.take(now_ns=10_120_000_000,now_mono=100.12) is None


def test_changed_stream_generation_latches_and_clears():
    g,v=gate();items=feed(g,v)
    with pytest.raises(ValueError):g.receive('aligned',items['aligned'],b'\xff'*16,now_ns=10_120_000_000,now_mono=100.12)
    assert g.ledger.invalid_reason
    with pytest.raises(ValueError):g.take(now_ns=10_120_000_000,now_mono=100.12)


def test_raw_buffers_bounded_and_future_stamp_cannot_poison_current():
    g,v=gate();items=messages(v)
    future=messages(v)['color'];future.header.stamp.sec+=3600
    with pytest.raises(ValueError):g.receive('color',future,bytes.fromhex('2'.zfill(32)),now_ns=10_120_000_000,now_mono=100.12)
    feed(g,v,items);assert g.take(now_ns=10_120_000_000,now_mono=100.12)
    for i in range(10):
        msg=messages(v)['color'];msg.header.stamp.nanosec+=i+1
        g.receive('color',msg,bytes.fromhex(v['source']['color_publisher_gid']),now_ns=10_120_000_000,now_mono=100.12)
    assert len(g.streams['color'])==4
    g.close()
    with pytest.raises(ValueError):g.take(now_ns=10_120_000_000,now_mono=100.12)


def test_latest_statement_never_falls_back_to_old_complete_and_receipts_not_renewed():
    from copy import deepcopy
    g,v=gate();items=feed(g,v)
    newer=deepcopy(v);newer['sequence']+=1
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):
        newer['original'][key]+=1
    newer['outputs']['cloud_stamp_ns']+=1;newer['outputs']['aligned_stamp_ns']+=1
    g.lineage(json.dumps(newer),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.13)
    assert g.take(now_ns=10_120_000_000,now_mono=100.13) is None
    assert g.streams['color'][0][2]==100.12


def test_final_check_expiry_graph_change_and_reset_reject_old_token():
    from test_derived_registration import endpoints
    g,v=gate();feed(g,v);token=g.take(now_ns=10_120_000_000,now_mono=100.12)
    graph=endpoints(v);lookup=lambda topic:graph[topic]
    matrix=[[float(i==j) for j in range(4)] for i in range(4)]
    for i in range(3):matrix[i][3]=v['calibration']['translation_m'][i]
    assert g.check(token,now_ns=10_120_000_000,now_mono=100.12,lookup=lookup,color_from_depth=matrix)
    with pytest.raises(ValueError):g.check(token,now_ns=10_400_000_000,now_mono=100.4,lookup=lookup,color_from_depth=matrix)
    replacement=api()(g.ledger.pin)
    with pytest.raises(ValueError):replacement.check(token,now_ns=10_120_000_000,now_mono=100.12,lookup=lookup,color_from_depth=matrix)
    graph[v['outputs']['aligned_topic']][0].endpoint_gid=b'\xee'*16
    with pytest.raises(ValueError):g.check(token,now_ns=10_120_000_000,now_mono=100.12,lookup=lookup,color_from_depth=matrix)
    assert g.ledger.invalid_reason and not any(g.streams.values())


def test_duplicates_do_not_renew_references_and_retained_bytes_are_bounded():
    g,v=gate();items=feed(g,v);initial=g.retained_bytes()
    feed(g,v,messages(v))
    assert g.retained_bytes()==initial
    assert all(len(items)==1 for items in g.streams.values())
    assert all(items[0][2]==100.12 for items in g.streams.values())
