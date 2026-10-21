"""Source-only metadata health; pure production method replay, no ROS/EGL init."""
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from threading import RLock
from types import SimpleNamespace as NS

import numpy as np
import pytest
from test_aligned_cohort import gate, feed, messages
from test_derived_registration import endpoints
from so101_station_perception.derived_lineage import (require, finite, statement_pin,
    validate_statement, validate_graph, validate_camera)

SOURCE=Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception/so101_station_perception/derived_aligned.py'


def prepared():
    tree=ast.parse(SOURCE.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    needed={'_source_health','_check_source_binding','_info_diagnostics'}
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in needed]
    assert len(methods)==len(needed),'source-only health API missing'
    cls.bases=[];cls.body=methods
    clocks={'mono':100.12,'ns':10_120_000_000}
    scope=dict(json=json,np=np,math=__import__('math'),time=NS(monotonic=lambda:clocks['mono']),
        Time=lambda **kwargs:NS(**kwargs),require=require,finite=finite,statement_pin=statement_pin,
        validate_statement=validate_statement,validate_graph=validate_graph,validate_camera=validate_camera,
        quaternion_matrix=lambda *args:np.eye(3))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(SOURCE),'exec'),scope)
    obj=scope['DerivedAligned']();core,value=gate();items=feed(core,value)
    # No RGB or aligned-image record is needed or read by this API.
    core.streams['color'].clear();core.streams['aligned'].clear()
    obj.core=core;obj.lock=RLock();obj.closed=False;obj.consumer_epoch='original-epoch'
    obj.source_pin_sha256='b'*64;obj.last_proof=object();core.emitted=123
    graph=endpoints(value)
    obj.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=clocks['ns'])),get_publishers_info_by_topic=graph.__getitem__)
    xyz=value['calibration']['translation_m']
    obj.buffer=NS(lookup_transform=lambda *args:NS(transform=NS(translation=NS(x=xyz[0],y=xyz[1],z=xyz[2]),rotation=NS(x=0.,y=0.,z=0.,w=1.))))
    return obj,value,items,graph,clocks,scope


def status(obj):
    response=obj._source_health(None,NS(success=False,message=''))
    return response.success,json.loads(response.message)


def test_source_ready_without_image_admission_or_processing_and_original_deadline_components():
    obj,value,_,_,clocks,_=prepared();proof=obj.last_proof;original=list(obj.core.ledger.entries)
    ready,result=status(obj)
    assert ready and result['schema']=='so101-derived-source-binding-health-v1'
    assert result['binding_invalidated'] is False
    for key in ('image_admitted','processing_progress','qualified','execution_enabled'):assert result[key] is False
    assert result['camera_info_validation']=='independent_native_info'
    assert result['original_receipt_expiry_monotonic']==pytest.approx(100.35)
    assert result['original_identity_expiry_monotonic']==101.
    assert result['effective_health_expiry_monotonic']==pytest.approx(100.35)
    assert len(result['receipt_deadlines_monotonic'])==7
    assert result['original_header_expiry_ros_ns']==10_500_000_000
    assert obj.last_proof is proof and obj.core.emitted==123 and list(obj.core.ledger.entries)==original
    clocks.update(mono=100.15,ns=10_150_000_000)
    _,again=status(obj)
    for field in ('original_receipt_expiry_monotonic','original_identity_expiry_monotonic',
                  'effective_health_expiry_monotonic','original_header_expiry_ros_ns','receipt_deadlines_monotonic'):
        assert again[field]==result[field]
    assert again['checked_monotonic']>result['checked_monotonic']


@pytest.mark.parametrize('kind',['depth_info','color_info'])
def test_requires_exact_actual_info_and_never_uses_nearest(kind):
    obj,_,_,_,_,_=prepared();obj.core.streams[kind].clear()
    assert status(obj)[0] is False
    assert obj.core.ledger.invalid_reason is None
    assert status(obj)[1]['binding_invalidated'] is False


@pytest.mark.parametrize('ns,mono',[(10_600_000_000,100.12),(10_120_000_000,100.36),
                                   (9_999_999_999,100.12),(10_120_000_000,100.01)])
def test_expired_or_future_source_is_unavailable_without_freshness_renewal(ns,mono):
    obj,_,_,_,clocks,_=prepared();clocks.update(ns=ns,mono=mono)
    assert not status(obj)[0]
    assert obj.core.ledger.invalid_reason is None
    assert status(obj)[1]['binding_invalidated'] is False


def test_stale_actual_info_receipt_rejects_even_if_lineage_receipt_fresh():
    obj,_,_,_,_,_=prepared();records=obj.core.streams['depth_info'];message,gid,_,stamp=records[0]
    records[0]=(message,gid,99.,stamp)
    assert not status(obj)[0] and obj.core.ledger.invalid_reason is None


@pytest.mark.parametrize('mutation',['graph','K','D','TF'])
def test_structural_contradiction_latches_until_explicit_reset(mutation):
    obj,value,items,graph,_,_=prepared()
    if mutation=='graph':graph[value['outputs']['aligned_topic']][0].endpoint_gid=b'\xee'*16
    elif mutation=='K':items['color_info'].k[0]+=1
    elif mutation=='D':items['color_info'].d[0]=.1
    else:obj.buffer.lookup_transform=lambda *args:NS(transform=NS(translation=NS(x=1.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    assert not status(obj)[0] and obj.core.ledger.invalid_reason
    assert status(obj)[1]['binding_invalidated'] is True
    assert not status(obj)[0]


@pytest.mark.parametrize('change',['clear','newer','close','epoch'])
def test_serialization_boundary_rechecks_newest_epoch_and_closed_state(change):
    obj,_,_,_,_,scope=prepared();original_dump=json.dumps
    def dump(value,**kwargs):
        if value.get('state')=='ready':
            if change=='clear':obj.core.ledger.entries.clear()
            elif change=='newer':obj.core.ledger.entries.append(deepcopy(obj.core.ledger.entries[-1]))
            elif change=='close':obj.closed=True
            else:obj.consumer_epoch='new-epoch'
        return original_dump(value,**kwargs)
    scope['json']=NS(dumps=dump)
    success,result=status(obj)
    assert not success
    if change=='epoch':assert result['binding_invalidated'] is True


def test_latest_missing_info_does_not_fallback_to_older_complete_cohort():
    obj,_,_,_,_,_=prepared();value,gid,receipt=obj.core.ledger.entries[-1];newer=deepcopy(value)
    newer['sequence']+=1
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):newer['original'][key]+=1
    newer['outputs']['cloud_stamp_ns']+=1;newer['outputs']['aligned_stamp_ns']+=1
    obj.core.ledger.entries.append((newer,gid,receipt))
    assert not status(obj)[0]


def test_existing_selected_image_status_method_is_exactly_unchanged():
    tree=ast.parse(SOURCE.read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_admission_status')
    assert hashlib.sha256(ast.get_source_segment(SOURCE.read_text(),fn).encode()).hexdigest()=='144301fae34488f4151aac542f06b8ce7bee3d27ab2eb31ad6f3c5c188a625ef'


def intake_method(obj,scope):
    tree=ast.parse(SOURCE.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    names={'_validate_received_info','_intake','_invalidate_offers'}
    for method in [n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]:
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(SOURCE),'exec'),scope)
        setattr(obj,method.name,scope[method.name].__get__(obj))
    scope['struct']=__import__('struct')
    obj.period=.2;obj.fault=None;obj.metadata_offer=None
    return obj


@pytest.mark.parametrize('kind,key',[('depth_info','K'),('color_info','D'),('depth_info','profile'),('color_info','frame')])
def test_duplicate_actual_info_contradictions_rejected_before_core_duplicate_discard(kind,key):
    obj,value,_,_,_,scope=prepared();intake_method(obj,scope)
    duplicate=messages(value)[kind]
    if key=='K':duplicate.k[0]+=1
    elif key=='D':duplicate.d[0]=.1
    elif key=='profile':duplicate.width+=1
    else:duplicate.header.frame_id='wrong_frame'
    obj._intake(kind,duplicate,bytes.fromhex(value['source'][kind+'_publisher_gid']))
    assert obj.core.ledger.invalid_reason and obj.fault
    assert not status(obj)[0]


def test_duplicate_valid_info_keeps_first_receipt_and_new_health_does_not_touch_images():
    obj,value,_,_,clocks,scope=prepared();intake_method(obj,scope)
    original=obj.core.streams['color_info'][0]
    clocks.update(ns=10_150_000_000,mono=100.15)
    obj._intake('color_info',messages(value)['color_info'],bytes.fromhex(value['source']['color_info_publisher_gid']))
    assert obj.core.streams['color_info'][0] is original
    assert len(obj.core.streams['color_info'])==1 and original[2]==100.12
    assert status(obj)[0]
    assert not obj.core.streams['color'] and not obj.core.streams['aligned']


def test_cold_api_and_info_guard_have_bounded_metadata_only_work():
    tree=ast.parse(SOURCE.read_text());methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    for name in ('_check_source_binding','_source_health','_validate_received_info'):
        text=ast.unparse(methods[name])
        assert not any(x in text for x in ('self.core.take(','.pop_fault(','.last_proof',"streams['color']", "streams['aligned']",'.data','create_subscription','create_publisher'))
        assert not any(isinstance(n,ast.While) for n in ast.walk(methods[name]))
    text=ast.unparse(methods['_validate_received_info'])
    assert not any(x in text for x in ('lookup_transform','validate_graph','np.','Image('))
    assert "('depth_info', 'color_info')" in ast.unparse(methods['_intake'])
    assert "'~/derived_source_health'" in ast.unparse(methods['__init__'])



def test_new_actual_dds_fixture_fails_wrong_domain_before_ros_init(monkeypatch):
    # ROOT comes directly from the known source location rather than importing ROS.
    path=Path(__file__).with_name('test_source_binding_health_ros.py')
    tree=ast.parse(path.read_text());guard=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='require_isolated_domain')
    scope={'os':__import__('os')};exec(compile(ast.Module(body=[guard],type_ignores=[]),str(path),'exec'),scope)
    monkeypatch.setenv('ROS_DOMAIN_ID','76');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    with pytest.raises(AssertionError):scope['require_isolated_domain']()
    fixture=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name.startswith('test_actual'))
    text=ast.unparse(fixture)
    assert text.index('require_isolated_domain()')<text.index('import rclpy')<text.index('rclpy.init(')


def test_consumer_lineage_receipt_uses_250ms_not_generic_500ms_metadata_age():
    obj,_,_,_,_,_=prepared();value,gid,_=obj.core.ledger.entries[-1]
    # Deliberately adversarial historical receipt: generic statement check permits
    # a 500ms local metadata age, but this API promises the original 250ms lease.
    obj.core.ledger.entries[-1]=(value,gid,99.8)
    success,result=status(obj)
    assert not success and result['binding_invalidated'] is False


@pytest.mark.parametrize('field',['k','d'])
def test_oversize_native_metadata_is_rejected_without_iteration(field):
    class Oversize:
        def __len__(self):return 1_000_000
        def __iter__(self):pytest.fail('oversize Info metadata must never be iterated')
    obj,value,_,_,_,scope=prepared();intake_method(obj,scope)
    msg=messages(value)['color_info'];setattr(msg,field,Oversize())
    obj._intake('color_info',msg,bytes.fromhex(value['source']['color_info_publisher_gid']))
    assert obj.core.ledger.invalid_reason and obj.fault


@pytest.mark.parametrize('field',['k','d'])
def test_oversize_cached_native_metadata_is_rejected_before_shared_validator(field):
    class Oversize:
        def __len__(self):return 1_000_000
        def __iter__(self):pytest.fail('oversize cached Info metadata must never be iterated')
    obj,_,items,_,_,_=prepared();setattr(items['color_info'],field,Oversize())
    success,result=status(obj)
    assert not success and result['binding_invalidated'] is True
