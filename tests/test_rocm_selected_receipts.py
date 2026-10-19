"""Selected acquisition provenance, independently of stream activity."""
from collections import deque
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
from test_ros_rocm_rgbd import module, delayed_info_fixture


def test_latest_stream_activity_cannot_rewrite_selected_receipts():
    fake,offers,records,_,_=delayed_info_fixture()
    selected=dict(fake.last_receipt)
    fake.last_receipt={k:v+.01 for k,v in selected.items()}
    module.ROCmRGBD._admit(fake)
    assert offers[0]['source_received_monotonic']==selected


@pytest.mark.parametrize('stream',list(module.TOPICS))
@pytest.mark.parametrize('age',[.250001,-.000001])
def test_expired_or_future_selected_record_rejected_before_any_work(monkeypatch,stream,age):
    fake,offers,records,_,_=delayed_info_fixture()
    now=100.;monkeypatch.setattr(module.time,'monotonic',lambda:now)
    fake.identity['request_started_monotonic']=99.9
    fake.last_receipt={k:now for k in module.TOPICS}
    for k,items in fake.buffers.items():
        record=items[-1];items[-1]=(*record[:3],now-(age if k==stream else .01))
    monkeypatch.setattr(module,'image_array',lambda *a:pytest.fail('decoded expired selected input'))
    module.ROCmRGBD._admit(fake)
    assert offers==records==[]


def test_duplicate_only_updates_activity_not_first_unique_receipt(monkeypatch):
    fake,_,_,_,_=delayed_info_fixture();stream='color'
    stamp,gid,message=fake.buffers[stream][-1][:3]
    fake.buffers={k:deque(maxlen=4) for k in module.TOPICS}
    fake.invalid=None;fake.closed=False;fake.received={k:0 for k in module.TOPICS}
    fake.unique={k:0 for k in module.TOPICS};fake.last_header={};fake.last_receipt={}
    fake._invalidate=lambda error:pytest.fail(str(error))
    clock=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    module.ROCmRGBD._receive(fake,stream,message,{'publisher_gid':bytes.fromhex(gid)})
    original=fake.buffers[stream][-1]
    clock[0]+=.1
    module.ROCmRGBD._receive(fake,stream,deepcopy(message),{'publisher_gid':bytes.fromhex(gid)})
    assert fake.buffers[stream][-1] is original
    assert original[3]==100. and fake.last_receipt[stream]==100.1
    assert fake.unique[stream]==1 and len(fake.buffers[stream])==1


def test_newer_unselected_rgb_and_info_keep_exact_nearest_receipts():
    fake,offers,_,_,stamp=delayed_info_fixture()
    selected=dict(fake.last_receipt)
    for stream in ('color','depth_info','color_info'):
        old=fake.buffers[stream][-1];message=deepcopy(old[2]);new_stamp=stamp+30_000_000
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(new_stamp,10**9)
        fake.buffers[stream].append((new_stamp,old[1],message,selected[stream]+.02))
        fake.last_receipt[stream]=selected[stream]+.02
    module.ROCmRGBD._admit(fake)
    assert offers[0]['source_received_monotonic']==selected
    assert offers[0]['color_stamp_ns']==stamp
    assert offers[0]['color_info_stamp_ns']==stamp


def test_selected_receipt_exact_250ms_boundary_is_preserved(monkeypatch):
    fake,offers,_,_,_=delayed_info_fixture();monkeypatch.setattr(module.time,'monotonic',lambda:100.)
    fake.identity['request_started_monotonic']=99.9
    for items in fake.buffers.values():items[-1]=(*items[-1][:3],99.75)
    module.ROCmRGBD._admit(fake)
    assert len(offers)==1 and set(offers[0]['source_received_monotonic'].values())=={99.75}


def test_receipt_path_stays_bounded_metadata_only():
    import ast,inspect,textwrap
    tree=ast.parse(textwrap.dedent(inspect.getsource(module.ROCmRGBD._admit)))
    assert not any(isinstance(n,ast.Attribute) and n.attr=='last_receipt' for n in ast.walk(tree))
    source=inspect.getsource(module.ROCmRGBD._receive)
    assert '(stamp, gid, message, receipt)' in source
    for forbidden in ('read_bytes','open','synchronize','cpu','tolist'):
        assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr==forbidden for n in ast.walk(tree))
