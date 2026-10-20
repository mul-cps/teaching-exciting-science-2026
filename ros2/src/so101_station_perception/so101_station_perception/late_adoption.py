"""Cold, fixed-file completed-job provenance. No ROS or recovery/reset authority."""
from copy import deepcopy
import hashlib,json,os,tempfile
from .derived_lineage import require,finite,integer,hex_value
from .supervisor_policy import SupervisorPolicy

SCHEMA='so101-late-local-adoption-transaction-v1'
FIELDS={'schema','boot_id','baseline_sha256','adoption_id','owned_job','original_journal','original_journal_sha256',
    'candidate_journal','candidate_journal_sha256','prospective_artifact','prospective_artifact_sha256','prospective_pin_sha256',
    'entry_monotonic','entry_ros_ns','deadline_monotonic','phase'}


def encoded(value):return json.dumps(value,allow_nan=False,sort_keys=True,separators=(',',':')).encode()
def digest(value):return hashlib.sha256(encoded(value)).hexdigest()


def bounded_read(path):
    with path.open('rb') as stream:raw=stream.read(65537)
    require(len(raw)<=65536,'late adoption record exceeds64KiB')
    def pairs(items):
        value={}
        for key,item in items:require(key not in value,'duplicate late adoption key');value[key]=item
        return value
    return json.loads(raw,object_pairs_hook=pairs)


def validate_marker(value,journal):
    require(type(value) is dict and set(value)==FIELDS and value['schema']==SCHEMA,'unknown late adoption marker schema')
    require(value['boot_id']==journal.boot and value['baseline_sha256']==journal.baseline_sha,'late adoption marker boot/baseline changed')
    require(hex_value(value['adoption_id'],32) and value['phase'] in ('prepared','committed_local_success'),'late adoption intent/phase invalid')
    require(finite(value['entry_monotonic']) and value['entry_monotonic']>=0 and integer(value['entry_ros_ns']) and
        finite(value['deadline_monotonic']) and value['deadline_monotonic']==value['entry_monotonic']+2.,'late adoption marker clocks invalid')
    require(type(value['prospective_artifact']) is str and 0<len(value['prospective_artifact'])<=4096 and
        hex_value(value['prospective_artifact_sha256'],64) and hex_value(value['prospective_pin_sha256'],64),'late adoption artifact identity invalid')
    for kind in ('original','candidate'):
        item=value[kind+'_journal']
        require(type(item) is dict and set(item)=={'schema','boot_id','baseline_sha256','policy','consumer_epochs','adopted'} and
            item['schema']=='so101-perception-supervisor-persistence-v1' and item['boot_id']==journal.boot and
            item['baseline_sha256']==journal.baseline_sha and digest(item)==value[kind+'_journal_sha256'],'late adoption snapshot hash/context invalid')
        SupervisorPolicy(item['policy'])
        require(type(item['consumer_epochs']) is dict and set(item['consumer_epochs']).issubset({'/so101_charuco_world','/so101_colored_objects'}) and
            all(hex_value(v,32) for v in item['consumer_epochs'].values()),'late adoption snapshot epochs invalid')
        adopted=item['adopted']
        require(adopted is None or type(adopted) is dict and set(adopted)=={'job_id','artifact_sha256','producer_generation'} and
            hex_value(adopted['job_id'],32) and hex_value(adopted['artifact_sha256'],64) and hex_value(adopted['producer_generation'],32),
            'late adoption snapshot attributable artifact invalid')
    original=value['original_journal'];candidate=value['candidate_journal'];job=value['owned_job']
    require(job==original['policy']['inflight'] and type(job) is dict and job.get('state')=='accepted','late adoption original owned accepted job required')
    require(candidate['policy']['inflight'] is None and candidate['policy']['latched'] is None,'late adoption candidate cannot retain authority/latch')
    require(all(original['policy'][k]==candidate['policy'][k] for k in ('attempts','handled')) and
        candidate['policy']['last_utc']>=original['policy']['last_utc'] and candidate['policy']['last_mono']>=original['policy']['last_mono'],
        'late adoption may not rewind quota/handled/clocks')
    continuous_pair((original['policy']['last_mono'],original['policy']['last_utc']),
        (candidate['policy']['last_mono'],candidate['policy']['last_utc']))
    require(candidate['adopted']['job_id']==job['job_id'] and candidate['adopted']['artifact_sha256']==value['prospective_artifact_sha256'],
        'late adoption candidate attribution changed')
    require(len(encoded(value))<=65536,'late adoption marker exceeds64KiB')
    return value


def write_marker(journal,value):
    validate_marker(value,journal);raw=encoded(value)
    fd,name=tempfile.mkstemp(prefix='.late-adoption-',dir=journal.directory)
    try:
        with os.fdopen(fd,'wb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
        os.replace(name,journal.late_marker_path)
        fd=os.open(journal.directory,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
        require(journal.late_marker_path.read_bytes()==raw,'late adoption marker readback differs')
        journal.late_marker=deepcopy(value)
    finally:
        try:os.unlink(name)
        except FileNotFoundError:pass


def continuous_pair(previous,current):
    require(all(finite(x) and x>=0 for x in (*previous,*current)) and current[0]>=previous[0] and current[1]>=previous[1] and
        abs((current[1]-previous[1])-(current[0]-previous[0]))<=2.,'late adoption paired clock contradiction')
    return current


def original_descendant(marker,actual):
    original=marker['original_journal']
    if type(actual) is not dict or set(actual)!=set(original):return False
    if any(actual[k]!=original[k] for k in original if k!='policy'):return False
    p=actual['policy'];q=original['policy']
    if type(p) is not dict or set(p)!=set(q) or any(p[k]!=q[k] for k in q if k not in ('last_mono','last_utc','latched')):return False
    if type(p['latched']) is not str or not 0<len(p['latched'])<=512:return False
    try:continuous_pair((q['last_mono'],q['last_utc']),(p['last_mono'],p['last_utc']))
    except ValueError:return False
    return True


def known_state(marker,actual):
    return digest(actual) in (marker['original_journal_sha256'],marker['candidate_journal_sha256']) or original_descendant(marker,actual)


def compensation_snapshot(marker,actual,now,utc):
    require(known_state(marker,actual),'late adoption compensation UNKNOWN; latest accounting retained')
    original=marker['original_journal'];old=original['policy'];latest=actual['policy']
    pair=continuous_pair((old['last_mono'],old['last_utc']),(latest['last_mono'],latest['last_utc']))
    pair=continuous_pair(pair,(now,utc))  # Actual observed pair, never independent coordinate maxima.
    restored=deepcopy(original);restored['policy'].update(last_mono=pair[0],last_utc=pair[1],latched='late local adoption transaction UNKNOWN; explicit same-job confirmation required')
    return restored


def load_restart_marker(journal):
    """Runs BEFORE supervisor adopted-artifact restoration. Never rewinds newer accounting."""
    journal.late_marker_path=journal.directory/'late-local-adoption.json'
    journal.late_marker=None;journal.late_marker_conflict=False
    if not journal.late_marker_path.exists():return
    try:value=validate_marker(bounded_read(journal.late_marker_path),journal)
    except (ValueError,TypeError,KeyError,OSError):
        journal.late_marker_conflict=True
        journal.loaded=deepcopy(journal.loaded)
        journal.loaded['policy']['latched']='late local adoption marker UNKNOWN; latest accounting retained'
        return
    journal.late_marker=value
    try:
        from .adoption_ack import resume_ack
        if value['phase']=='committed_local_success' and resume_ack(journal):return
    except (ValueError,TypeError,KeyError,OSError):
        journal.late_marker_conflict=True
    actual=journal.loaded;known=not journal.late_marker_conflict and known_state(value,actual)
    if known:
        # Known candidate/compensation descendant retains its latest paired clocks.
        pair=(actual['policy']['last_mono'],actual['policy']['last_utc'])
        journal.loaded=compensation_snapshot(value,actual,*pair)
    else:journal.late_marker_conflict=True  # Preserve ALL latest durable accounting.
    journal.loaded=deepcopy(journal.loaded)
    journal.loaded['policy']['latched']='late local adoption transaction UNKNOWN; explicit same-job confirmation required'


def buffered_native_pairs(native,graph,*,now_ns,now_mono):
    """Late-local only: two actual advancing acquisitions from bounded intake."""
    require(native.invalid is None,native.invalid)
    require(native.graph is None or native.graph==graph,'native graph changed during preflight')
    options={}
    for kind in ('depth','color'):
        records=native.infos[kind];require(len(records)<=4,'native Info buffer exceeds4')
        unique={}
        for record in records:
            stamp=record['stamp'];receipt=record['receipt']
            require(integer(stamp) and finite(receipt) and stamp<=now_ns and receipt<=now_mono,'native Info future/invalid')
            require(record['gid']==graph['endpoints'][kind+'_info']['gid'],'native Info publisher generation changed')
            # Intake owns first unique receipt. Repeated stamp records cannot
            # count twice or replace that receipt with a later observation.
            if stamp not in unique:unique[stamp]=record
            else:require(unique[stamp]==record,'native duplicate receipt/context changed')
        options[kind]=[r for r in unique.values() if r['stamp']>native.start_ns and r['receipt']>=native.start_mono
            and now_mono-r['receipt']<=.250 and now_ns-r['stamp']<=500_000_000]
    matches=[(d,c) for d in options['depth'] for c in options['color'] if abs(d['stamp']-c['stamp'])<=40_000_000]
    sequences=[(a,b) for a in matches for b in matches if all(a[i]['stamp']<b[i]['stamp'] for i in (0,1))]
    if not sequences:return None
    selected=max(sequences,key=lambda s:(min(r['stamp'] for r in s[1]),min(r['stamp'] for r in s[0]),s[1][0]['stamp'],s[1][1]['stamp']))
    return deepcopy(selected)
