"""Selective unadmitted early native Info discard, using real core metadata."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import json
import uuid

import pytest
from test_aligned_cohort import messages, gate, feed
from test_derived_source_binding_health import prepared, intake_method, status, SOURCE
from so101_station_perception.derived_lineage import canonical_gid, SCHEMA
from so101_station_perception.aligned_cohort import AlignedCohorts
from so101_station_perception.admission_proof import pin_digest

KINDS=('depth_info','color_info')


def setup(metadata=True):
    obj,value,items,graph,clocks,scope=prepared();intake_method(obj,scope)
    scope['canonical_gid']=canonical_gid
    offers=[];obj.metadata_offer=offers.append if metadata else None
    obj._early_info_drops=dict.fromkeys(KINDS,0)
    obj._early_info_last_age_ns=dict.fromkeys(KINDS,None)
    obj.drain=Mock();obj.last_take=42.
    obj.core,value=gate();feed(obj.core,value)
    obj.core.emitted=0
    return obj,value,clocks,scope,offers


def message(value,clocks,kind,offset=6_840_000):
    msg=messages(value)[kind];ns=clocks['ns']+offset
    msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
    return msg


def snapshot(obj):
    return (dict((k,list(v)) for k,v in obj.core.streams.items()),
            list(obj.core.ledger.entries),deepcopy(obj.core.last),obj.core.emitted,
            obj.last_take,obj.last_proof,obj.core.epoch,obj.fault)


@pytest.mark.parametrize('kind',KINDS)
@pytest.mark.parametrize('gid_format',['bytes','string'])
def test_early_pinned_info_discard_preserves_exact_cohort_pending_offer_and_original_receipts(kind,gid_format):
    obj,value,clocks,_,offers=setup();before=snapshot(obj);_,health=status(obj)
    gid=value['source'][kind+'_publisher_gid'];gid=bytes.fromhex(gid) if gid_format=='bytes' else gid
    obj._intake(kind,message(value,clocks,kind),gid)
    assert snapshot(obj)==before and offers==[]
    obj.drain.assert_not_called()
    assert obj._early_info_drops[kind]==1
    assert obj._early_info_last_age_ns[kind]==-6_840_000
    ready,after=status(obj);assert ready
    for key in ('receipt_deadlines_monotonic','effective_health_expiry_monotonic','original_header_stamps_ns'):
        assert after[key]==health[key]
    diag=after['early_native_info_diagnostics']
    assert diag['consumer_epoch']==obj.consumer_epoch and diag['enabled']
    assert diag['input_admitted'] is diag['processing_progress'] is diag['qualified'] is False
    assert diag['streams'][kind]=={'discarded_unadmitted_count':1,'last_header_age_ns':-6_840_000}
    assert obj.core.take(now_ns=clocks['ns'],now_mono=clocks['mono']) is not None


@pytest.mark.parametrize('case',['wrong_gid','missing_gid','stale','future_image','K','D','frame','profile','bool_sec','bad_nsec'])
def test_nonmatching_or_invalid_inputs_keep_original_failure_path(case):
    obj,value,clocks,_,offers=setup();kind='color' if case=='future_image' else 'depth_info'
    msg=message(value,clocks,kind,offset=-500_000_001 if case=='stale' else 1)
    gid=bytes.fromhex(value['source'][kind+'_publisher_gid'])
    if case=='wrong_gid':gid=b'\xee'*16
    elif case=='missing_gid':gid=b''
    elif case=='K':msg.k[0]+=1
    elif case=='D':msg.d[0]=.1
    elif case=='frame':msg.header.frame_id='wrong'
    elif case=='profile':msg.width+=1
    elif case=='bool_sec':msg.header.stamp.sec=True
    elif case=='bad_nsec':msg.header.stamp.nanosec=10**9
    obj._intake(kind,msg,gid)
    assert obj.fault and offers==[None]
    assert not any(obj.core.streams.values())
    assert obj._early_info_drops==dict.fromkeys(KINDS,0)
    obj.drain.assert_not_called()


@pytest.mark.parametrize('period',[0.,.2])
def test_legacy_marker_future_info_still_fails(period):
    obj,value,clocks,_,_=setup(metadata=False);obj.period=period
    obj._intake('depth_info',message(value,clocks,'depth_info'),bytes.fromhex(value['source']['depth_info_publisher_gid']))
    assert obj.fault and not any(obj.core.streams.values())
    assert obj._early_info_drops==dict.fromkeys(KINDS,0)


def test_no_old_cohort_fallback_after_newest_lineage_and_discard():
    obj,value,clocks,_,_=setup();newer=deepcopy(value);newer['sequence']+=1
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):newer['original'][key]+=1
    newer['outputs']['cloud_stamp_ns']+=1;newer['outputs']['aligned_stamp_ns']+=1
    _,gid,receipt=obj.core.ledger.entries[-1];obj.core.ledger.entries.append((newer,gid,receipt))
    before=snapshot(obj)
    obj._intake('depth_info',message(value,clocks,'depth_info'),bytes.fromhex(value['source']['depth_info_publisher_gid']))
    assert snapshot(obj)==before
    assert obj.core.take(now_ns=clocks['ns'],now_mono=clocks['mono']) is None
    assert not status(obj)[0]


def test_bounded_diagnostics_are_cold_snapshots_and_reset_with_consumer_epoch(tmp_path):
    obj,value,clocks,scope,offers=setup();kind='color_info'
    obj._early_info_drops[kind]=2**31-1
    obj._intake(kind,message(value,clocks,kind),bytes.fromhex(value['source'][kind+'_publisher_gid']))
    assert obj._early_info_drops[kind]==2**31-1
    before=snapshot(obj);_,health=status(obj);health['early_native_info_diagnostics']['streams'][kind]['discarded_unadmitted_count']=0
    assert snapshot(obj)==before and obj._early_info_drops[kind]==2**31-1
    path=tmp_path/'pin.json';path.write_text(json.dumps({'derived_source':{'schema':SCHEMA,'qualified':False,'pin':obj.core.ledger.pin}}))
    scope.update(Path=Path,AlignedCohorts=AlignedCohorts,uuid=uuid,pin_digest=pin_digest,SCHEMA=SCHEMA)
    tree=ast.parse(SOURCE.read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='reload')
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(SOURCE),'exec'),scope)
    epoch=obj.consumer_epoch;obj.node.get_parameter=lambda _:NS(value=str(path))
    scope['reload'](obj)
    assert obj.consumer_epoch!=epoch and obj.last_proof is None and offers==[None]
    assert obj._early_info_drops==dict.fromkeys(KINDS,0)
    assert obj._early_info_last_age_ns==dict.fromkeys(KINDS,None)
    ready,health=status(obj);assert not ready
    assert health['early_native_info_diagnostics']['consumer_epoch']==obj.consumer_epoch


@pytest.mark.parametrize('case',['closed_core','latched_core','duplicate_bad_K','missing_stamp','negative_sec','oversized_sec','float_sec'])
def test_skip_never_hides_closed_or_invalid_native_metadata(case):
    obj,value,clocks,_,offers=setup();kind='depth_info';msg=message(value,clocks,kind)
    if case=='closed_core':obj.core.close()
    elif case=='latched_core':obj.core.ledger.invalid_reason='existing contradiction'
    elif case=='duplicate_bad_K':
        msg=messages(value)[kind];msg.k[0]+=1
    elif case=='missing_stamp':del msg.header.stamp
    elif case=='negative_sec':msg.header.stamp.sec=-1
    elif case=='oversized_sec':msg.header.stamp.sec=2**31
    elif case=='float_sec':msg.header.stamp.sec=10.5
    obj._intake(kind,msg,bytes.fromhex(value['source'][kind+'_publisher_gid']))
    assert obj.fault and offers==[None] and obj._early_info_drops[kind]==0


def test_skip_never_renews_an_already_expired_original_cohort():
    obj,value,clocks,_,_=setup();clocks['mono']=100.4;before=snapshot(obj)
    obj._intake('depth_info',message(value,clocks,'depth_info'),bytes.fromhex(value['source']['depth_info_publisher_gid']))
    assert snapshot(obj)==before
    assert not status(obj)[0]
    with pytest.raises(ValueError):obj.core.take(now_ns=clocks['ns'],now_mono=clocks['mono'])
