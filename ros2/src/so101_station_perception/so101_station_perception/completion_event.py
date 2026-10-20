"""Bounded historical source evidence. No ROS, clocks, resources or authority."""
from copy import deepcopy
import hashlib
import json
import math

SCHEMA='so101-source-admission-completion-event-v1'
READINESS_SCHEMA='so101-current-source-readiness-v1'
CONSUMERS=('/so101_charuco_world','/so101_colored_objects')
STREAMS=('depth','color','depth_info','color_info')
HEADERS=tuple(k+'_stamp_ns' for k in STREAMS)
CONTEXT=('boot_id','monotonic_clock_domain','coordinator_instance_id','job_id','accepted_reset_id',
    'producer_generation','source_pin_sha256','artifact_sha256','baseline_sha256',
    'job_started_monotonic','job_deadline_monotonic','job_start_ros_ns')
SCOPE='historical source admission completion; not current readiness or physical qualification'


def require(ok,reason):
    if not ok:raise ValueError(reason)


def finite(v):return type(v) in (int,float) and math.isfinite(v) and v>0

def integer(v):return type(v) is int and v>0

def hexd(v,n):return type(v) is str and len(v)==n and all(c in '0123456789abcdef' for c in v)


def canonical_json(value):
    try:return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
    except (TypeError,ValueError,RecursionError) as e:raise ValueError('finite bounded completion JSON required') from e


def event_digest(event):
    return hashlib.sha256(canonical_json({k:v for k,v in event.items() if k!='event_sha256'}).encode()).hexdigest()


def _flags(v):
    require(type(v) is dict and v.get('qualified') is False and v.get('execution_enabled') is False,'source evidence is not authority')


def _witness(w):
    require(type(w) is dict and set(w)=={'proof','cohort_sha256','consumed_monotonic','consumed_ros_ns',
        'rpc_request_monotonic','rpc_observed_done_monotonic','rpc_consumed_monotonic','rpc_deadline_monotonic',
        'lineage_receipt_monotonic','lineage_gid','original','source_receipts_monotonic','source_expiry_monotonic',
        'identity_request_monotonic','identity_expiry_monotonic'},'bounded exact witness fields required')
    p=w['proof'];_flags(p)
    required={'schema','qualified','execution_enabled','state','consumer_epoch','source_pin_sha256','producer_generation',
        'sequence','depth_stamp_ns','color_stamp_ns','checked_ros_ns','original_receipt_expiry_monotonic','original_header_expiry_ros_ns'}
    require(required<=set(p)<=required|{'scope'},'bounded IMAGE proof fields required')
    require(p['schema']=='so101-derived-admission-status-v1' and p['state']=='ready','actual IMAGE proof required')
    if 'scope' in p:require(type(p['scope']) is str and len(p['scope'])<=256,'bounded proof scope required')
    require(all(hexd(p[k],n) for k,n in [('consumer_epoch',32),('producer_generation',32),('source_pin_sha256',64)]),'invalid IMAGE context')
    require(all(integer(p[k]) for k in ('sequence','depth_stamp_ns','color_stamp_ns','checked_ros_ns','original_header_expiry_ros_ns')),'positive integer IMAGE clocks required')
    require(hexd(w['cohort_sha256'],64) and hexd(w['lineage_gid'],32),'invalid lineage identity')
    o=w['original'];receipts=w['source_receipts_monotonic']
    require(type(o) is dict and set(o)==set(HEADERS) and all(integer(o[k]) for k in HEADERS),'four exact original headers required')
    require(type(receipts) is dict and set(receipts)==set(STREAMS) and all(finite(v) for v in receipts.values()),'four original source receipts required')
    keys=('consumed_monotonic','rpc_request_monotonic','rpc_observed_done_monotonic','rpc_consumed_monotonic',
        'rpc_deadline_monotonic','lineage_receipt_monotonic','source_expiry_monotonic','identity_request_monotonic','identity_expiry_monotonic')
    require(all(finite(w[k]) for k in keys) and integer(w['consumed_ros_ns']) and finite(p['original_receipt_expiry_monotonic']),'finite original clocks required')
    take=w['consumed_monotonic'];ns=w['consumed_ros_ns']
    require(w['rpc_request_monotonic']<=w['rpc_observed_done_monotonic']<=w['rpc_consumed_monotonic']<=take<=w['rpc_deadline_monotonic']<=w['rpc_request_monotonic']+2.,'original RPC clock ordering invalid')
    require(max(receipts.values())<=take and w['lineage_receipt_monotonic']<=take and w['identity_request_monotonic']<=take,'future original receipt/identity')
    require(abs(w['source_expiry_monotonic']-(min(receipts.values())+.250))<=1e-9 and
        abs(w['identity_expiry_monotonic']-(w['identity_request_monotonic']+1.))<=1e-9,'original source/identity deadlines changed')
    require(take<p['original_receipt_expiry_monotonic']<=min(w['source_expiry_monotonic'],w['identity_expiry_monotonic'],w['lineage_receipt_monotonic']+.250)+1e-9,'witness expired at original consumption')
    require(max(o.values())<=ns and abs(o['depth_stamp_ns']-o['color_stamp_ns'])<=40_000_000 and
        abs(o['depth_info_stamp_ns']-o['depth_stamp_ns'])<=40_000_000 and abs(o['color_info_stamp_ns']-o['color_stamp_ns'])<=40_000_000, 'future/mismatched original Info exposure')
    require(all(p[k]==o[k] for k in ('depth_stamp_ns','color_stamp_ns')) and
        max(o['depth_stamp_ns'],o['color_stamp_ns'])<=p['checked_ros_ns']<=ns<p['original_header_expiry_ros_ns']==min(o.values())+500_000_000,'original header/checked clock invalid')
    return w


def record_witness(proof,statement,*,request_mono,observed_done_mono,take_mono,consumed_mono,consumed_ros_ns,rpc_deadline,lineage_receipt,lineage_gid):
    from .admission_proof import pin_digest
    from .derived_lineage import statement_pin
    require(proof.get('source_pin_sha256')==pin_digest(statement_pin(statement)) and
        proof.get('producer_generation')==statement['producer_generation'] and proof.get('sequence')==statement['sequence'] and
        lineage_gid==statement['outputs']['status_publisher_gid'],'witness lineage context differs')
    w=dict(proof=deepcopy(proof),cohort_sha256=hashlib.sha256(canonical_json(statement).encode()).hexdigest(),
        consumed_monotonic=consumed_mono,consumed_ros_ns=consumed_ros_ns,rpc_request_monotonic=request_mono,
        rpc_observed_done_monotonic=observed_done_mono,rpc_consumed_monotonic=take_mono,rpc_deadline_monotonic=rpc_deadline,
        lineage_receipt_monotonic=lineage_receipt,lineage_gid=lineage_gid,
        original={k:statement['original'][k] for k in HEADERS},source_receipts_monotonic=deepcopy(statement['health']['source_receipts_monotonic']),
        source_expiry_monotonic=statement['health']['source_expiry_monotonic'],identity_request_monotonic=statement['identity']['request_started_monotonic'],
        identity_expiry_monotonic=statement['identity']['expires_monotonic'])
    return _witness(w)


def _validate(event,expected_context):
    _flags(event)
    require(set(event)==set(CONTEXT)|{'schema','qualified','execution_enabled','scope','event_checked_monotonic','event_checked_ros_ns',
        'consumers','latest_valid_until_monotonic','latest_header_valid_until_ros_ns','event_sha256'},'bounded exact event fields required')
    require(event['schema']==SCHEMA and event['scope']==SCOPE,'completion event schema/scope differs')
    require(len(canonical_json(event))<=8192,'completion event exceeds8192 bytes')
    require(hexd(event['event_sha256'],64) and event_digest(event)==event['event_sha256'],'completion event mutated')
    require(type(event['boot_id']) is str and len(event['boot_id'])==36 and [i for i,c in enumerate(event['boot_id']) if c=='-']==[8,13,18,23] and hexd(event['boot_id'].replace('-',''),32) and event['monotonic_clock_domain']=='CLOCK_MONOTONIC','boot/clock domain required')
    require(all(event[k]==expected_context[k] for k in CONTEXT),'owned event context differs')
    require(all(hexd(event[k],32) for k in ('coordinator_instance_id','job_id','accepted_reset_id','producer_generation')) and
        all(hexd(event[k],64) for k in ('source_pin_sha256','artifact_sha256','baseline_sha256')),'invalid event identities')
    start=event['job_started_monotonic'];end=event['job_deadline_monotonic'];now=event['event_checked_monotonic'];ns=event['event_checked_ros_ns']
    require(all(finite(v) for v in (start,end,now)) and end==start+40. and start<=now<=end and integer(ns) and integer(event['job_start_ros_ns']),'original event job budget invalid')
    require(type(event['consumers']) is dict and set(event['consumers'])==set(CONSUMERS),'both canonical consumers required')
    ends=[];headers=[]
    for c in event['consumers'].values():
        require(type(c) is dict and set(c)=={'old_epoch','new_epoch','witnesses'} and hexd(c['old_epoch'],32) and hexd(c['new_epoch'],32) and c['old_epoch']!=c['new_epoch'],'consumer epochs invalid')
        require(type(c['witnesses']) is list and len(c['witnesses'])==2,'two original IMAGE witnesses required')
        first,last=map(_witness,c['witnesses']);a=first['proof'];b=last['proof']
        for w in (first,last):
            p=w['proof']
            require(p['consumer_epoch']==c['new_epoch'] and p['producer_generation']==event['producer_generation'] and
                p['source_pin_sha256']==event['source_pin_sha256'],'IMAGE event context changed')
            require(event['job_start_ros_ns']<min(w['original'].values()) and w['consumed_ros_ns']<=ns and
                start<=w['rpc_request_monotonic']<=w['consumed_monotonic']<=now,'future/prejob witness clocks')
        require(all(b[k]>a[k] for k in ('sequence','depth_stamp_ns','color_stamp_ns')) and
            first['consumed_monotonic']<=last['consumed_monotonic'] and first['consumed_ros_ns']<=last['consumed_ros_ns'],'two independent ordered acquisitions required')
        limit=min(b['original_receipt_expiry_monotonic'],last['source_expiry_monotonic'],last['identity_expiry_monotonic'],last['lineage_receipt_monotonic']+.250,end)
        require(now<limit and ns<b['original_header_expiry_ros_ns'],'latest IMAGE expired at event')
        ends.append(limit);headers.append(b['original_header_expiry_ros_ns'])
    require(event['latest_valid_until_monotonic']==min(ends) and event['latest_header_valid_until_ros_ns']==min(headers),'event original validity limits differ')


def prepare_completion_event(context,witnesses,*,checked_mono,checked_ros_ns):
    require(type(context) is dict and set(context)==set(CONTEXT),'exact completion context required')
    e=dict(deepcopy(context),schema=SCHEMA,qualified=False,execution_enabled=False,scope=SCOPE,
        event_checked_monotonic=checked_mono,event_checked_ros_ns=checked_ros_ns,consumers=deepcopy(witnesses))
    latest=[c['witnesses'][-1] for c in witnesses.values()]
    e['latest_valid_until_monotonic']=min(min(w['proof']['original_receipt_expiry_monotonic'],w['source_expiry_monotonic'],w['identity_expiry_monotonic'],w['lineage_receipt_monotonic']+.250,context['job_deadline_monotonic']) for w in latest)
    e['latest_header_valid_until_ros_ns']=min(w['proof']['original_header_expiry_ros_ns'] for w in latest)
    e['event_sha256']=event_digest(e);_validate(e,context);return e


def validate_completion_event(event,expected_context,*,received_mono,received_ros_ns,job_deadline):
    require(type(event) is dict,'completion event unavailable')
    _validate(event,expected_context)
    require(finite(received_mono) and integer(received_ros_ns) and event['event_checked_monotonic']<=received_mono and
        event['event_checked_ros_ns']<=received_ros_ns,'event from future/unknown clock')
    require(job_deadline==event['job_deadline_monotonic'],'owned observation budget differs')
    return dict(historical_completion_valid=True,observed_within_job_budget=received_mono<=job_deadline,
        current_readiness=False,qualified=False,execution_enabled=False)


def validate_current_readiness(value,*,take_mono,take_ros_ns):
    _flags(value)
    required={'schema','ready','checked_monotonic','checked_ros_ns','reason','original_receipt_valid_until_monotonic',
        'original_identity_valid_until_monotonic','original_local_lineage_valid_until_monotonic',
        'effective_valid_until_monotonic','original_header_valid_until_ros_ns','qualified','execution_enabled'}
    require(set(value)==required and value.get('schema')==READINESS_SCHEMA and type(value.get('ready')) is bool,'current readiness schema required')
    require(value['reason'] is None or (type(value['reason']) is str and len(value['reason'])<=512),'bounded readiness reason required')
    require(finite(take_mono) and integer(take_ros_ns) and finite(value.get('checked_monotonic')) and integer(value.get('checked_ros_ns')) and
        value['checked_monotonic']<=take_mono and value['checked_ros_ns']<=take_ros_ns,'future current readiness clocks')
    limits=[value[k] for k in ('original_receipt_valid_until_monotonic','original_identity_valid_until_monotonic','original_local_lineage_valid_until_monotonic','effective_valid_until_monotonic')]
    header=value['original_header_valid_until_ros_ns']
    require(all(v is None or finite(v) for v in limits) and (header is None or integer(header)), 'malformed original readiness deadlines')
    known=limits[:-1]
    if limits[-1] is not None:
        require(all(v is not None for v in known) and limits[-1]<=min(known),'effective original limit inconsistent')
    if not value['ready']:return False  # Past/null is negative only AFTER full known-field validation.
    require(all(v is not None for v in limits) and header is not None,'current original limits required')
    require(value['checked_monotonic']<limits[-1] and value['checked_ros_ns']<header,'server ready contradicted at original check')
    return take_mono<limits[-1] and take_ros_ns<header


def validate_status_event(response,expected_context,*,received_mono,received_ros_ns):
    """Bind the complete existing response envelope before accepting its event."""
    _flags(response)
    require(response.get('schema')=='so101-perception-recovery-v1' and response.get('state')=='completed'
        and response.get('operation_success') is True and type(response.get('ready_now')) is bool,'completed strict status envelope required')
    event=response.get('completion_event')
    result=validate_completion_event(event,expected_context,received_mono=received_mono,
        received_ros_ns=received_ros_ns,job_deadline=expected_context['job_deadline_monotonic'])
    for key in ('job_id','accepted_reset_id','producer_generation','source_pin_sha256','artifact_sha256'):
        require(response.get(key)==event[key],'outer completion identity differs from event')
    require(response.get('deadline_monotonic')==event['job_deadline_monotonic'],'outer original deadline differs')
    artifact=response.get('artifact')
    require(type(artifact) is str and 0<len(artifact)<=4096 and artifact.rsplit('/',1)[-1]==event['job_id']+'.json','bounded owned artifact path required')
    consumers=response.get('consumers')
    require(type(consumers) is dict and set(consumers)==set(CONSUMERS),'outer both IMAGE consumers required')
    fields=('schema','state','qualified','execution_enabled','consumer_epoch','source_pin_sha256','producer_generation',
        'sequence','depth_stamp_ns','color_stamp_ns','original_receipt_expiry_monotonic','original_header_expiry_ros_ns')
    for name,original in event['consumers'].items():
        c=consumers[name]
        require(type(c) is dict and set(c)=={'latest','witnesses'} and type(c['witnesses']) is list and len(c['witnesses'])==2,'outer bounded IMAGE witnesses required')
        pairs=list(zip(c['witnesses'],[r['proof'] for r in original['witnesses']]))+[(c['latest'],original['witnesses'][-1]['proof'])]
        for proof,expected in pairs:
            _flags(proof)
            require(all(integer(proof.get(k)) for k in ('sequence','depth_stamp_ns','color_stamp_ns','original_header_expiry_ros_ns')), 'outer exact integer IMAGE fields required')
            require(all(proof.get(k)==expected[k] for k in fields),'outer IMAGE proof differs from immutable evidence')
            require(integer(proof.get('checked_ros_ns')) and max(proof['depth_stamp_ns'],proof['color_stamp_ns'])<=proof['checked_ros_ns']<=event['event_checked_ros_ns'],'outer IMAGE checked clock future/inconsistent')
    readiness=response.get('current_readiness')
    result['current_readiness']=validate_current_readiness(readiness,take_mono=received_mono,take_ros_ns=received_ros_ns)
    require(response['ready_now'] is readiness['ready'],'outer readiness contradiction')
    if readiness['ready']:
        require(readiness['effective_valid_until_monotonic']<=event['latest_valid_until_monotonic'] and
            readiness['original_header_valid_until_ros_ns']<=event['latest_header_valid_until_ros_ns'],
            'current readiness renewed immutable latest IMAGE limits')
    result['reported_server_ready']=response['ready_now'];return result
