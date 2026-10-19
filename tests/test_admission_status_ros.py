"""Cold consumer status never retains pixels or renews original receipts."""
import json
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from test_derived_aligned_ros import setup,send,next_cohort


def status(adapter):
    response=adapter._admission_status(None,NS())
    return response,json.loads(response.message)


def test_status_waits_for_checked_admission_and_reset_changes_epoch(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    response,waiting=status(a)
    assert not response.success and waiting['qualified'] is False
    epoch=waiting['consumer_epoch']
    send(a,v)
    assert not status(a)[0].success
    a.check(calls[0][3])
    response,proof=status(a)
    assert response.success and proof['sequence']==v['sequence']
    assert proof['consumer_epoch']==epoch
    assert len(proof['source_pin_sha256'])==64
    assert proof['producer_generation']==v['producer_generation']
    assert proof['execution_enabled'] is False
    assert len(response.message)<4096
    a._reset(None,NS())
    response,waiting=status(a)
    assert not response.success and waiting['consumer_epoch']!=epoch
    assert waiting['source_pin_sha256']==proof['source_pin_sha256']


def test_repeated_status_does_not_extend_original_expiry(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    send(a,v);a.check(calls[0][3])
    first=status(a)[1]
    clock[:]=[100.20,10_200_000_000]
    response,later=status(a)
    assert response.success
    assert first['original_receipt_expiry_monotonic']==later['original_receipt_expiry_monotonic']
    assert first['depth_stamp_ns']==later['depth_stamp_ns']
    clock[:]=[100.4,10_400_000_000]
    assert not status(a)[0].success


def test_cleared_latest_and_changed_actual_graph_revoke_status(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    send(a,v);a.check(calls[0][3]);a.core.ledger.entries.clear()
    response,proof=status(a)
    assert not response.success and 'newest' in proof['reason']
    send(a,next_cohort(v,1));a.check(calls[-1][3])
    graph=a.node.get_publishers_info_by_topic
    graph(v['outputs']['aligned_topic'])[0].endpoint_gid=b'\xee'*16
    assert not status(a)[0].success


def test_serialization_delay_is_checked_before_success(tmp_path,monkeypatch):
    import so101_station_perception.derived_aligned as module
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    send(a,v);a.check(calls[0][3]);original=module.json.dumps
    def slow(*args,**kwargs):
        result=original(*args,**kwargs)
        clock[0]+=.251;clock[1]+=251_000_000
        return result
    monkeypatch.setattr(module.json,'dumps',slow)
    response,result=status(a)
    assert not response.success and result['state']=='unavailable'


# Independent review regressions, retained in the repository.
def test_graph_swap_during_materialization_is_rejected(tmp_path,monkeypatch):
    import so101_station_perception.derived_aligned as module
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    send(a,v);a.check(calls[0][3]);original=json.dumps
    def change_graph(*args,**kwargs):
        result=original(*args,**kwargs)
        if args and isinstance(args[0],dict) and args[0].get('state')=='ready':
            a.node.get_publishers_info_by_topic(v['outputs']['aligned_topic'])[0].endpoint_gid=b'\xee'*16
        return result
    monkeypatch.setattr(module.json,'dumps',change_graph)
    response=a._admission_status(None,NS())
    assert not response.success


def test_observed_generation_failure_revokes_old_proof(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    send(a,v);a.check(calls[0][3])
    endpoint=a.node.get_publishers_info_by_topic(v['outputs']['aligned_topic'])[0]
    original=endpoint.endpoint_gid;endpoint.endpoint_gid=b'\xee'*16
    assert not a._admission_status(None,NS()).success
    endpoint.endpoint_gid=original
    assert not a._admission_status(None,NS()).success
