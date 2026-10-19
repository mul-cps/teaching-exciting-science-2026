"""Bounded unavailable metadata snapshots, without ROS initialization."""
from collections import deque
import json
import pytest
from test_derived_source_binding_health import prepared,status


@pytest.mark.parametrize('stamps,target,expected',[
    ([],10,'empty'),([11,12,13,14],15,'not_yet'),([11,12,13,14],10,'older_than_oldest'),
    ([11,13,14,15],12,'interior_missing'),([11,12,13,14],12,'present')])
def test_exact_target_and_original_receipt_bounds(stamps,target,expected):
    obj,value,_,_,_,_=prepared();obj.core.ledger.entries[-1][0]['original']['depth_info_stamp_ns']=target
    records=obj.core.streams['depth_info'];records.clear()
    for i,stamp in enumerate(stamps):records.append((object(),'gid',100.+i*.001,stamp))
    before=list(records);d=obj._info_diagnostics();s=d['streams']['depth_info']
    assert s['classification']==expected and s['target_stamp_ns']==target
    assert s['retained_count']==len(stamps)<=4 and list(records)==before
    assert s['oldest_stamp_ns']==(min(stamps) if stamps else None)
    assert s['newest_stamp_ns']==(max(stamps) if stamps else None)
    assert s['first_unique_receipt_min_monotonic']==(100. if stamps else None)
    assert s['first_unique_receipt_max_monotonic']==(100.+(len(stamps)-1)*.001 if stamps else None)
    assert len(json.dumps(d))<1600


def test_unavailable_response_adds_diagnostics_without_state_or_receipt_mutation():
    obj,value,_,_,_,_=prepared();obj.core.streams['depth_info'].clear()
    before=list(obj.core.streams['color_info']);proof=obj.last_proof;emitted=obj.core.emitted
    ready,result=status(obj);d=result['native_info_diagnostics']
    assert not ready and not result['binding_invalidated']
    assert d['latest_sequence']==value['sequence'] and d['consumer_epoch']==obj.consumer_epoch
    assert d['streams']['depth_info']['classification']=='empty'
    assert d['streams']['color_info']['classification']=='present'
    assert list(obj.core.streams['color_info'])==before and obj.last_proof is proof and obj.core.emitted==emitted
    assert status(obj)[1]['native_info_diagnostics']==d


def test_ready_schema_unchanged_and_no_lineage_never_invents_target():
    obj,_,_,_,_,_=prepared()
    assert 'native_info_diagnostics' not in status(obj)[1]
    obj.core.ledger.entries.clear();result=status(obj)[1]['native_info_diagnostics']
    assert result['latest_sequence'] is None
    assert all(s['target_stamp_ns'] is None and s['classification']=='target_unavailable' for s in result['streams'].values())
