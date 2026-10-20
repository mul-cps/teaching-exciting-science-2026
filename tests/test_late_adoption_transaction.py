"""Cold fixed-file transaction tests. No ROS, devices, reset or timing lease changes."""
from copy import deepcopy
import time,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception'))
import pytest
from so101_station_perception.supervisor_journal import SupervisorJournal
from so101_station_perception.supervisor_policy import SupervisorPolicy
from so101_station_perception.late_adoption import SCHEMA,bounded_read,digest,write_marker,validate_marker


def transaction(tmp_path):
    journal=SupervisorJournal(tmp_path,'a'*64);policy=SupervisorPolicy();now=time.monotonic();utc=time.time()
    policy.clocks(now,utc);policy.begin_reconciliation('b'*32,now);policy.reserve_reconciliation('b'*32,now,utc)
    policy.accepted('c'*32,now+40.,now);policy.fail('original adoption expired')
    journal.write(policy.journal(),consumer_epochs={'/so101_charuco_world':'d'*32,'/so101_colored_objects':'e'*32})
    original=bounded_read(journal.path);candidate=deepcopy(original)
    candidate['policy'].update(inflight=None,latched=None)
    candidate['consumer_epochs']={'/so101_charuco_world':'f'*32,'/so101_colored_objects':'1'*32}
    candidate['adopted']={'job_id':'c'*32,'artifact_sha256':'2'*64,'producer_generation':'3'*32}
    marker=dict(schema=SCHEMA,boot_id=journal.boot,baseline_sha256='a'*64,adoption_id='4'*32,owned_job=original['policy']['inflight'],
        original_journal=original,original_journal_sha256=digest(original),candidate_journal=candidate,candidate_journal_sha256=digest(candidate),
        prospective_artifact='/fixed/c.json',prospective_artifact_sha256='2'*64,prospective_pin_sha256='5'*64,
        entry_monotonic=now,entry_ros_ns=1000000000,deadline_monotonic=now+2.,phase='prepared')
    return journal,marker


@pytest.mark.parametrize('state',['original','candidate','committed'])
def test_marker_restart_never_silently_adopts_or_clears_owned_job_or_quota(tmp_path,state):
    journal,marker=transaction(tmp_path)
    if state!='original':
        candidate=marker['candidate_journal'];journal.write(candidate['policy'],consumer_epochs=candidate['consumer_epochs'],adopted=candidate['adopted'])
    if state=='committed':marker['phase']='committed_local_success'
    write_marker(journal,marker);journal.close()
    second=SupervisorJournal(tmp_path,'a'*64)
    try:
        assert second.loaded['policy']['inflight']==marker['owned_job']
        assert second.loaded['policy']['attempts']==marker['original_journal']['policy']['attempts']
        assert second.loaded['policy']['latched'] and second.loaded['adopted']==marker['original_journal']['adopted']
        assert not second.late_marker_conflict
    finally:second.close()


def test_restart_after_later_legitimate_activity_preserves_latest_accounting_and_rejects_old_reconstruction(tmp_path):
    journal,marker=transaction(tmp_path);write_marker(journal,marker)
    latest=deepcopy(marker['candidate_journal']);latest['policy']['attempts'].append(latest['policy']['last_utc'])
    latest['policy']['handled'].append('6'*32)
    latest['policy']['inflight']=deepcopy(marker['owned_job']);latest['policy']['inflight']['job_id']='7'*32
    journal.write(latest['policy'],consumer_epochs=latest['consumer_epochs'],adopted=latest['adopted']);journal.close()
    second=SupervisorJournal(tmp_path,'a'*64)
    try:
        assert second.late_marker_conflict
        assert second.loaded['policy']['attempts']==latest['policy']['attempts']
        assert second.loaded['policy']['handled']==latest['policy']['handled']
        assert second.loaded['policy']['inflight']==latest['policy']['inflight']
        assert second.loaded['adopted']==latest['adopted']
    finally:second.close()


@pytest.mark.parametrize('change',[lambda m:m.pop('owned_job'),lambda m:m.update(schema='unknown'),
    lambda m:m.update(original_journal_sha256='0'*64),lambda m:m.update(candidate_journal_sha256='0'*64),
    lambda m:m.update(deadline_monotonic=True),lambda m:m['candidate_journal']['policy'].update(attempts=[])])
def test_malformed_marker_is_unknown_never_absent_and_preserves_latest_journal(tmp_path,change):
    journal,marker=transaction(tmp_path);change(marker)
    with pytest.raises((ValueError,KeyError)):validate_marker(marker,journal)
    journal.late_marker_path.write_text(json.dumps(marker));original=bounded_read(journal.path);journal.close()
    second=SupervisorJournal(tmp_path,'a'*64)
    try:
        assert second.late_marker_conflict and second.loaded['policy']['latched']
        assert second.loaded['policy']['attempts']==original['policy']['attempts']
        assert second.loaded['policy']['inflight']==original['policy']['inflight']
    finally:second.close()


def test_compensation_carries_actual_observed_clock_pair_and_descendant_never_grants_authority(tmp_path):
    from so101_station_perception.late_adoption import compensation_snapshot,original_descendant
    journal,marker=transaction(tmp_path);original=marker['original_journal']['policy']
    actual=deepcopy(marker['candidate_journal']);actual['policy']['last_mono']+=.1;actual['policy']['last_utc']+=.1
    # Pin exact candidate with its actually observed pair.
    marker['candidate_journal']=actual;marker['candidate_journal_sha256']=digest(actual)
    pair=(actual['policy']['last_mono']+.2,actual['policy']['last_utc']+.2)
    restored=compensation_snapshot(marker,actual,*pair)
    assert (restored['policy']['last_mono'],restored['policy']['last_utc'])==pair
    assert original_descendant(marker,restored) and restored['policy']['inflight']==marker['owned_job']
    assert restored['policy']['attempts']==original['attempts']
    restored['policy']['latched']=None;assert not original_descendant(marker,restored)
    restored['policy']['latched']='UNKNOWN';restored['policy']['last_utc']+=3.
    assert not original_descendant(marker,restored)
    journal.close()


def native_buffer():
    from types import SimpleNamespace as NS
    from collections import deque
    records={k:deque([dict(stamp=1_030_000_000,receipt=10.03,gid=k),dict(stamp=1_060_000_000,receipt=10.06,gid=k)],maxlen=4) for k in ('depth','color')}
    graph={'endpoints':{k+'_info':{'gid':k} for k in records}}
    return NS(start_ns=1_000_000_000,start_mono=10.,invalid=None,graph=None,infos=records),graph


def test_late_buffer_selects_two_original_acquisitions_despite_reordered_storage():
    from so101_station_perception.late_adoption import buffered_native_pairs
    native,graph=native_buffer();native.infos['depth'].reverse()
    selected=buffered_native_pairs(native,graph,now_ns=1_120_000_000,now_mono=10.12)
    assert [p[0]['receipt'] for p in selected]==[10.03,10.06]
    assert all(a['stamp']<b['stamp'] for a,b in zip(*selected))


@pytest.mark.parametrize('mutation',['one','duplicate','old','preentry','mismatch'])
def test_late_buffer_unavailable_never_fabricates_second_pair(mutation):
    from so101_station_perception.late_adoption import buffered_native_pairs
    native,graph=native_buffer()
    if mutation=='one':native.infos['color'].pop()
    if mutation=='duplicate':native.infos['color'][1]=deepcopy(native.infos['color'][0])
    if mutation=='old':
        for k in native.infos:native.infos[k][0]['receipt']=9.8
    if mutation=='preentry':
        for k in native.infos:native.infos[k][0]['stamp']=999_999_999
    if mutation=='mismatch':native.infos['color'][0]['stamp']=1_900_000_000
    if mutation=='mismatch':
        with pytest.raises(ValueError,match='future'):buffered_native_pairs(native,graph,now_ns=1_120_000_000,now_mono=10.12)
    else:assert buffered_native_pairs(native,graph,now_ns=1_120_000_000,now_mono=10.12) is None


@pytest.mark.parametrize('mutation',['gid','future_receipt','future_header','duplicate_renewal','graph','invalid','oversize'])
def test_late_buffer_contradiction_is_fatal(mutation):
    from so101_station_perception.late_adoption import buffered_native_pairs
    native,graph=native_buffer()
    if mutation=='gid':native.infos['depth'][0]['gid']='replacement'
    if mutation=='future_receipt':native.infos['depth'][0]['receipt']=11.
    if mutation=='future_header':native.infos['depth'][0]['stamp']=2_000_000_000
    if mutation=='duplicate_renewal':native.infos['depth'][1]['stamp']=native.infos['depth'][0]['stamp']
    if mutation=='graph':native.graph={}
    if mutation=='invalid':native.invalid='latched'
    if mutation=='oversize':native.infos['depth']=[deepcopy(native.infos['depth'][0]) for _ in range(5)]
    with pytest.raises(ValueError):buffered_native_pairs(native,graph,now_ns=1_120_000_000,now_mono=10.12)
