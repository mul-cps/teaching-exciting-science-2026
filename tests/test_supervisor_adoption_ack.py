"""Historical-only ACK; real tempFS/journal, no ROS/devices or journal rewrites."""
from copy import deepcopy
import importlib
import json
from pathlib import Path
import sys
import time
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception'))
from test_late_adoption_transaction import transaction
from so101_station_perception.supervisor_journal import SupervisorJournal
from so101_station_perception.late_adoption import bounded_read,digest,write_marker


def api():
    try:module=importlib.import_module('so101_station_perception.adoption_ack')
    except ModuleNotFoundError:module=None
    assert module is not None,'historical ACK implementation missing; committed marker alone is not completion'
    return module


def completed(tmp_path):
    journal,marker=transaction(tmp_path);marker['phase']='committed_local_success'
    candidate=marker['candidate_journal'];journal.write(candidate['policy'],consumer_epochs=candidate['consumer_epochs'],adopted=candidate['adopted'])
    write_marker(journal,marker)
    result=dict(schema='so101-late-local-adoption-status-v1',adoption_id=marker['adoption_id'],job_id=marker['owned_job']['job_id'],
        state='completed',operation_success=True,enabled=False,ready_now=False,qualified=False,execution_enabled=False,
        artifact_sha256=marker['prospective_artifact_sha256'],producer_generation=candidate['adopted']['producer_generation'],
        source_pin_sha256=marker['prospective_pin_sha256'],consumer_epochs=candidate['consumer_epochs'],transaction_phase='committed_local_success',
        historical_attestation='resident_coordinator_code_validated_no_historical_monotonic_replay')
    return journal,marker,result


def receipt(path,payload,*,acceptance=False):
    command='perception-supervisor-adopt-completed-job' if acceptance else 'perception-supervisor-status'
    service='/so101_perception_supervisor/adopt_completed_job' if acceptance else '/so101_perception_supervisor/status'
    body=dict(success=True,rpc_success=True,command=command,service=service,status=payload,message=json.dumps(payload))
    path.write_text(json.dumps(dict(exit_code=0,stdout=json.dumps(body))))
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def receipts(journal,result):
    accepted=dict(accepted=True,adoption_id=result['adoption_id'],job_id=result['job_id'],state='local_adoption_preflight',enabled=False,
        ready_now=False,operation_success=False,qualified=False,execution_enabled=False)
    a=journal.directory/'acceptance.json';b=journal.directory/'completed.json'
    return (str(a),receipt(a,accepted,acceptance=True),str(b),receipt(b,dict(qualified=False,execution_enabled=False,late_local_adoption=result)))


def test_own_successful_runtime_ack_archives_then_retires_marker_without_journal_write(tmp_path):
    journal,marker,result=completed(tmp_path);before=journal.path.read_bytes()
    try:
        reply=api().acknowledge_runtime(journal,result)
        assert reply['historical_only'] is True and reply['qualified'] is False and reply['execution_enabled'] is False
        assert journal.path.read_bytes()==before and not journal.late_marker_path.exists()
        assert not journal.late_marker_conflict and journal.late_marker is None
        assert Path(reply['archive']).read_bytes() and Path(reply['ack']).read_bytes()
    finally:journal.close()


def test_explicit_receipts_under_journal_confirm_before_restart_unknown_reconstruction(tmp_path):
    journal,marker,result=completed(tmp_path);inputs=receipts(journal,result);before=journal.path.read_bytes();journal.close()
    second=None
    try:
        # New constructor inputs must be applied BEFORE the legacy marker path.
        module=api();second=SupervisorJournal(tmp_path,'a'*64,confirmed_adoption=inputs)
        assert second.loaded['policy']['inflight'] is None and second.loaded['policy']['latched'] is None
        assert second.loaded['adopted']==marker['candidate_journal']['adopted']
        assert second.path.read_bytes()==before and not second.late_marker_path.exists()
        from so101_station_perception.supervisor_policy import SupervisorPolicy
        assert SupervisorPolicy(second.loaded['policy']).state=='disabled'
    finally:
        if second:second.close()


@pytest.mark.parametrize('mutation',['prepared','failed_result','wrong_adoption','wrong_job','wrong_gen','wrong_pin','wrong_epochs','permission'])
def test_marker_or_false_historical_completion_cannot_finalize(mutation,tmp_path):
    journal,marker,result=completed(tmp_path);before=journal.path.read_bytes()
    if mutation=='prepared':marker['phase']='prepared';write_marker(journal,marker)
    if mutation=='failed_result':result['operation_success']=False
    if mutation=='wrong_adoption':result['adoption_id']='e'*32
    if mutation=='wrong_job':result['job_id']='e'*32
    if mutation=='wrong_gen':result['producer_generation']='e'*32
    if mutation=='wrong_pin':result['source_pin_sha256']='e'*64
    if mutation=='wrong_epochs':result['consumer_epochs']={}
    if mutation=='permission':result['qualified']=True
    try:
        with pytest.raises(ValueError):api().acknowledge_runtime(journal,result)
        assert journal.path.read_bytes()==before and journal.late_marker_path.exists()
    finally:journal.close()


@pytest.mark.parametrize('mutation',['job','latch','new_attempt','handled','epoch','adopted','clock','quota_subset'])
def test_unknown_or_later_authority_never_rewound_or_acknowledged(mutation,tmp_path):
    journal,marker,result=completed(tmp_path);actual=deepcopy(marker['candidate_journal']);p=actual['policy']
    if mutation=='job':p['inflight']=marker['owned_job']
    if mutation=='latch':p['latched']='unknown transaction'
    if mutation=='new_attempt':p['attempts'].append(p['last_utc'])
    if mutation=='handled':p['handled'].append('a'*32)
    if mutation=='epoch':actual['consumer_epochs']['/so101_charuco_world']='a'*32
    if mutation=='adopted':actual['adopted']['producer_generation']='a'*32
    if mutation=='clock':p['last_mono']-=.1
    if mutation=='quota_subset':p['attempts']=[]
    journal.write(p,consumer_epochs=actual['consumer_epochs'],adopted=actual['adopted']);before=journal.path.read_bytes()
    try:
        with pytest.raises(ValueError):api().acknowledge_runtime(journal,result)
        assert journal.path.read_bytes()==before and journal.late_marker_path.exists()
    finally:journal.close()


def test_exact_aged_prefix_pruning_is_permitted_without_journal_rewrite(tmp_path):
    journal,marker,result=completed(tmp_path);actual=deepcopy(marker['candidate_journal']);p=actual['policy']
    p['last_mono']+=3601.;p['last_utc']+=3601.;p['attempts']=[]
    journal.write(p,consumer_epochs=actual['consumer_epochs'],adopted=actual['adopted']);before=journal.path.read_bytes()
    try:api().acknowledge_runtime(journal,result);assert journal.path.read_bytes()==before
    finally:journal.close()


def test_durable_ack_finishes_retirement_after_interrupted_unlink(tmp_path,monkeypatch):
    journal,marker,result=completed(tmp_path);module=api();original=module.retire
    monkeypatch.setattr(module,'retire',lambda *a:(_ for _ in ()).throw(OSError('injected before unlink')))
    with pytest.raises(OSError):module.acknowledge_runtime(journal,result)
    before=journal.path.read_bytes();journal.close();monkeypatch.setattr(module,'retire',original)
    second=SupervisorJournal(tmp_path,'a'*64)
    try:
        assert not second.late_marker_path.exists() and not second.late_marker_conflict
        assert second.path.read_bytes()==before and second.loaded['policy']['inflight'] is None
    finally:second.close()


def test_archive_conflict_preserves_marker_and_journal(tmp_path):
    journal,marker,result=completed(tmp_path);module=api();before=journal.path.read_bytes()
    key=module.sha(journal.late_marker_path.read_bytes());(tmp_path/('late-local-adoption-committed-'+key+'.json')).write_bytes(b'conflict')
    try:
        with pytest.raises(ValueError):module.acknowledge_runtime(journal,result)
        assert journal.path.read_bytes()==before and journal.late_marker_path.exists()
    finally:journal.close()


@pytest.mark.parametrize('values',[('',)*3,('x','','',''),(1,'','','')])
def test_partial_or_nonstring_confirmation_rejected(values,tmp_path):
    journal,_,_=completed(tmp_path)
    try:
        with pytest.raises(ValueError):api().acknowledge_receipts(journal,values)
    finally:journal.close()


def test_receipts_outside_journal_rejected(tmp_path):
    journal,_,result=completed(tmp_path);values=list(receipts(journal,result));values[0]=str(tmp_path.parent/'outside.json')
    try:
        with pytest.raises(ValueError):api().acknowledge_receipts(journal,values)
    finally:journal.close()


def test_retired_ack_does_not_restrict_new_activity(tmp_path):
    journal,marker,result=completed(tmp_path);api().acknowledge_runtime(journal,result)
    actual=deepcopy(marker['candidate_journal']);actual['policy']['handled'].append('9'*32)
    journal.write(actual['policy'],consumer_epochs=actual['consumer_epochs'],adopted=actual['adopted']);journal.close()
    second=SupervisorJournal(tmp_path,'a'*64)
    try:assert second.loaded['policy']['handled'][-1]=='9'*32 and not second.late_marker_conflict
    finally:second.close()


@pytest.mark.parametrize('stage',['archive','ack','retirement'])
def test_interrupted_protocol_never_rewrites_journal_and_unconfirmed_restart_is_unknown(stage,tmp_path,monkeypatch):
    journal,marker,result=completed(tmp_path);module=api();before=journal.path.read_bytes();original=module.exclusive
    def write(path,raw):
        if (stage=='archive' and '-committed-' in path.name) or (stage=='ack' and '-ack-' in path.name):raise OSError('injected durable failure')
        return original(path,raw)
    if stage=='retirement':monkeypatch.setattr(module,'retire',lambda *a:(_ for _ in ()).throw(OSError('injected unlink')))
    else:monkeypatch.setattr(module,'exclusive',write)
    with pytest.raises(OSError):module.acknowledge_runtime(journal,result)
    assert journal.path.read_bytes()==before and journal.late_marker_path.exists()
    journal.close();monkeypatch.undo();second=SupervisorJournal(tmp_path,'a'*64)
    try:
        assert second.path.read_bytes()==before
        if stage=='retirement':assert second.loaded['policy']['inflight'] is None and not second.late_marker_path.exists()
        else:assert second.loaded['policy']['latched'] and second.late_marker_path.exists()
    finally:second.close()


def test_journal_cas_change_during_archive_never_retires_marker(tmp_path,monkeypatch):
    journal,marker,result=completed(tmp_path);module=api();original=module.exclusive;changed=[]
    def write(path,raw):
        original(path,raw)
        if '-committed-' in path.name:
            actual=deepcopy(marker['candidate_journal']);actual['policy']['handled'].append('9'*32)
            journal.write(actual['policy'],consumer_epochs=actual['consumer_epochs'],adopted=actual['adopted']);changed.append(journal.path.read_bytes())
    monkeypatch.setattr(module,'exclusive',write)
    try:
        with pytest.raises(ValueError):module.acknowledge_runtime(journal,result)
        assert journal.path.read_bytes()==changed[-1] and journal.late_marker_path.exists()
    finally:journal.close()


@pytest.mark.parametrize('kind',['sha','duplicate','nonfinite','oversize','wrong_service','nonboolean_success'])
def test_invalid_actual_receipt_is_not_confirmation(kind,tmp_path):
    journal,marker,result=completed(tmp_path);values=list(receipts(journal,result));path=Path(values[0]);before=journal.path.read_bytes()
    if kind=='sha':values[1]='0'*64
    else:
        if kind=='duplicate':path.write_text('{"exit_code":0,"exit_code":0}')
        if kind=='nonfinite':path.write_text('{"exit_code":NaN}')
        if kind=='oversize':path.write_bytes(b' '*131073)
        if kind in ('wrong_service','nonboolean_success'):
            outer=json.loads(path.read_text());body=json.loads(outer['stdout'])
            body.update(service='/other') if kind=='wrong_service' else body.update(success=1)
            outer['stdout']=json.dumps(body);path.write_text(json.dumps(outer))
        values[1]=api().sha(path.read_bytes())
    try:
        with pytest.raises(ValueError):api().acknowledge_receipts(journal,values)
        assert journal.path.read_bytes()==before and journal.late_marker_path.exists()
    finally:journal.close()
