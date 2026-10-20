"""Cold historical-success ACK. Never writes journal or grants readiness."""
import hashlib,json,os,tempfile
from pathlib import Path
from .derived_lineage import require,hex_value
from .late_adoption import encoded,validate_marker,continuous_pair
from .supervisor_policy import SupervisorPolicy

PARAMETERS=('confirmed_adoption_acceptance_artifact','confirmed_adoption_acceptance_sha256',
            'confirmed_adoption_completed_artifact','confirmed_adoption_completed_sha256')
SCHEMA='so101-late-local-adoption-ack-v1'


def parse(raw):
    def pairs(items):
        result={}
        for k,v in items:require(k not in result,'duplicate ACK key');result[k]=v
        return result
    def invalid(value):raise ValueError('nonfinite ACK JSON')
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=invalid)


def raw_read(path,limit=65536):
    with path.open('rb') as stream:raw=stream.read(limit+1)
    require(len(raw)<=limit,'ACK input exceeds bounded size')
    return raw


def read(path,limit=65536):
    raw=raw_read(path,limit);return raw,parse(raw)


def sha(raw):return hashlib.sha256(raw).hexdigest()
def sync(directory):
    fd=os.open(directory,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


def exclusive(path,raw):
    require(not path.is_symlink(),'ACK archive symlink forbidden')
    if path.exists():require(raw_read(path,len(raw))==raw,'conflicting historical ACK/archive');return
    fd,name=tempfile.mkstemp(prefix='.adoption-ack-',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
        try:os.link(name,path)
        except FileExistsError:require(raw_read(path,len(raw))==raw,'concurrent conflicting ACK/archive')
        sync(path.parent);require(raw_read(path,len(raw))==raw,'ACK/archive readback differs')
    finally:os.unlink(name)


def candidate_descendant(marker,actual):
    candidate=marker['candidate_journal'];require(type(actual) is dict and set(actual)==set(candidate),'ACK journal schema differs')
    require(all(actual[k]==candidate[k] for k in candidate if k!='policy'),'ACK journal context changed')
    p=actual['policy'];q=candidate['policy'];SupervisorPolicy(p)
    require(set(p)==set(q) and all(p[k]==q[k] for k in q if k not in ('last_mono','last_utc','attempts')),'ACK journal authority changed')
    continuous_pair((q['last_mono'],q['last_utc']),(p['last_mono'],p['last_utc']))
    attempts=q['attempts'];cut=0
    while cut<len(attempts) and p['last_utc']-attempts[cut]>=3600.:cut+=1
    require(p['attempts']==attempts[cut:],'ACK quota is not exact aged prefix pruning')


def result_for(marker):
    candidate=marker['candidate_journal']
    return dict(schema='so101-late-local-adoption-status-v1',adoption_id=marker['adoption_id'],job_id=marker['owned_job']['job_id'],
        state='completed',operation_success=True,enabled=False,ready_now=False,qualified=False,execution_enabled=False,
        artifact_sha256=marker['prospective_artifact_sha256'],producer_generation=candidate['adopted']['producer_generation'],
        source_pin_sha256=marker['prospective_pin_sha256'],consumer_epochs=candidate['consumer_epochs'],transaction_phase='committed_local_success',
        historical_attestation='resident_coordinator_code_validated_no_historical_monotonic_replay')


def context(journal):
    path=journal.directory/'late-local-adoption.json'
    raw,marker=read(path);validate_marker(marker,journal)
    require(marker['phase']=='committed_local_success','historical ACK requires committed phase AND successful receipt')
    journal_raw,actual=read(journal.path);candidate_descendant(marker,actual)
    return path,raw,marker,journal_raw,actual


def retire(journal,path,marker_raw,journal_raw):
    require(raw_read(path)==marker_raw and raw_read(journal.path)==journal_raw,'ACK CAS context changed')
    path.unlink();sync(journal.directory);require(not path.exists(),'ACK active marker retirement failed')
    journal.late_marker=None;journal.late_marker_conflict=False


def acknowledge(journal,result,evidence):
    path,raw,marker,journal_raw,actual=context(journal)
    require(type(result) is dict and encoded(result)==encoded(result_for(marker)),'historical completed result differs from owned transaction')
    key=sha(raw);archive=journal.directory/('late-local-adoption-committed-'+key+'.json')
    ack=journal.directory/('late-local-adoption-ack-'+key+'.json')
    value=dict(schema=SCHEMA,marker_sha256=key,candidate_journal_sha256=marker['candidate_journal_sha256'],
        result=result,evidence=evidence,journal_sha256=sha(journal_raw),journal=actual)
    payload=encoded(value);require(len(payload)<=262144,'historical ACK exceeds256KiB')
    if ack.exists():
        require(resume_ack(journal),'existing ACK cannot retire marker')
        return dict(historical_only=True,qualified=False,execution_enabled=False,archive=str(archive),ack=str(ack))
    exclusive(archive,raw)
    require(raw_read(path,len(raw))==raw and raw_read(journal.path)==journal_raw,'ACK CAS context changed')
    exclusive(ack,payload)
    retire(journal,path,raw,journal_raw)
    return dict(historical_only=True,qualified=False,execution_enabled=False,archive=str(archive),ack=str(ack))


def acknowledge_runtime(journal,result):return acknowledge(journal,result,dict(origin='own_runtime_completed'))


def inputs(journal,values):
    require(type(values) in (tuple,list) and len(values)==4 and all(type(v) is str for v in values),'four string ACK inputs required')
    require(all(values) or not any(values),'all four historical ACK inputs or none required')
    if not any(values):return None
    paths=[]
    for name,digest in ((values[0],values[1]),(values[2],values[3])):
        path=Path(name).resolve();require(path.is_relative_to(journal.directory) and path!=journal.directory,'ACK receipt must be strictly under journal directory')
        require(hex_value(digest,64),'invalid ACK receipt SHA256');paths.append((path,digest))
    return paths


def cli(path,digest,command,service):
    raw,outer=read(path,131072);require(sha(raw)==digest,'historical receipt SHA256 differs')
    require(type(outer) is dict and type(outer.get('exit_code')) is int and outer['exit_code']==0 and type(outer.get('stdout')) is str,'invalid ROOT receipt wrapper')
    body=parse(outer['stdout']);require(type(body) is dict and body.get('success') is True and body.get('rpc_success') is True and
        body.get('command')==command and body.get('service')==service and type(body.get('message')) is str and type(body.get('status')) is dict and
        parse(body['message'])==body['status'],'invalid historical CLI completion')
    return body['status']


def acknowledge_receipts(journal,values):
    paths=inputs(journal,values)
    if paths is None or not (journal.directory/'late-local-adoption.json').exists():return
    acceptance=cli(*paths[0],'perception-supervisor-adopt-completed-job','/so101_perception_supervisor/adopt_completed_job')
    completed=cli(*paths[1],'perception-supervisor-status','/so101_perception_supervisor/status')
    require(completed.get('qualified') is False and completed.get('execution_enabled') is False,'historical status grants authority')
    result=completed.get('late_local_adoption');require(type(result) is dict,'completed historical adoption missing')
    expected=dict(accepted=True,adoption_id=result.get('adoption_id'),job_id=result.get('job_id'),state='local_adoption_preflight',
        enabled=False,ready_now=False,operation_success=False,qualified=False,execution_enabled=False)
    require(encoded(acceptance)==encoded(expected),'historical acceptance differs')
    return acknowledge(journal,result,dict(origin='explicit_historical_receipts',acceptance_sha256=paths[0][1],completed_sha256=paths[1][1]))


def resume_ack(journal):
    """Only an active marker invokes this; archived ACK never constrains later activity."""
    path,raw,marker,journal_raw,actual=context(journal);key=sha(raw)
    ack=journal.directory/('late-local-adoption-ack-'+key+'.json')
    if not ack.exists():return False
    _,value=read(ack,262144);archive=journal.directory/('late-local-adoption-committed-'+key+'.json')
    require(set(value)=={'schema','marker_sha256','candidate_journal_sha256','result','evidence','journal_sha256','journal'} and value['schema']==SCHEMA and
        value['marker_sha256']==key and value['candidate_journal_sha256']==marker['candidate_journal_sha256'] and
        encoded(value['result'])==encoded(result_for(marker)) and raw_read(archive)==raw and
        value['journal_sha256']==sha(encoded(value['journal'])),'durable ACK/archive context differs')
    candidate_descendant(marker,value['journal']);candidate_descendant(dict(marker,candidate_journal=value['journal']),actual)
    evidence=value['evidence'];require(type(evidence) is dict and (evidence==dict(origin='own_runtime_completed') or
        set(evidence)=={'origin','acceptance_sha256','completed_sha256'} and evidence['origin']=='explicit_historical_receipts' and
        all(hex_value(evidence[k],64) for k in ('acceptance_sha256','completed_sha256'))),'ACK provenance invalid')
    retire(journal,path,raw,journal_raw);return True
