"""Metadata-only health validation: past expiry is unavailable, never renewed."""
from .derived_lineage import require,finite,integer,hex_value,statement_pin,validate_statement,native_snapshot,TOPICS
from .admission_proof import pin_digest


class HealthNotCurrent(ValueError):
    """Known missing/past health observation; does not authorize recovery."""
    def __init__(self,reason,**clocks):
        super().__init__(reason);self.clocks=clocks


def proof_context(proof,pin,epoch=None):
    require(type(proof) is dict and proof.get('schema')=='so101-derived-admission-status-v1' and
        proof.get('qualified') is False and proof.get('execution_enabled') is False and
        proof.get('state') in ('ready','awaiting','unavailable'),'invalid consumer health schema/state')
    require(proof.get('source_pin_sha256')==pin_digest(pin) and
        proof.get('producer_generation')==pin['producer_generation'],'consumer source pin/generation changed')
    actual=proof.get('consumer_epoch')
    require(hex_value(actual,32) and actual!='0'*32,'consumer epoch missing')
    require(epoch is None or actual==epoch,'unexpected external consumer epoch')


def statement_bounds(value,pin,receipt,*,now,ns):
    """Input is an unchanged statement already fully validated on ledger intake.

    Recheck immutable context, future/structural clocks BEFORE classifying old
    cached originals. Shared validation still runs at the ACTUAL decision time
    whenever current. No historical validation time or expiry is substituted.
    """
    require(statement_pin(value)==pin,'cached lineage immutable context changed')
    original=value['original'];identity=value['identity'];health=value['health']
    require(value.get('schema')=='so101-derived-rocm-rgbd-cohort-v1' and
        all(value.get(k) is False for k in ('qualified','execution_enabled','direct_camera_attestation')) and
        health['state']=='valid' and type(health['original_max_age_ns']) is int and health['original_max_age_ns']==500_000_000,
        'cached lineage health schema changed')
    require(identity['serial_number']==pin['source']['serial_number'] and identity['source_generation']==pin['source_generation'] and
        identity['device_info_request_source']==native_snapshot(value),'cached independent identity changed')
    require(set(health['source_receipts_monotonic'])==set(TOPICS),'cached source receipt keys changed')
    stamps=[original[k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')]
    require(all(integer(s) and s<=ns for s in stamps),'original health header future/invalid')
    require(original['pair_delta_ns']==abs(stamps[0]-stamps[1])<=40_000_000 and
        abs(stamps[2]-stamps[0])<=40_000_000 and abs(stamps[3]-stamps[1])<=40_000_000,'cached original pair changed')
    require(finite(receipt) and 0<=receipt<=now,'lineage receipt clock future/invalid')
    receipts=list(health['source_receipts_monotonic'].values())
    require(len(receipts)==4 and all(finite(r) and 0<=r<=now for r in receipts),'source receipt clock future/invalid')
    request,confirmed,expiry=(identity[k] for k in ('request_started_monotonic','confirmed_monotonic','expires_monotonic'))
    require(all(finite(v) for v in (request,confirmed,expiry)) and 0<=request<=confirmed<=now and
        confirmed-request<=.4 and expiry==request+1.,'identity clock future/invalid')
    require(integer(identity['confirmed_ros_ns']) and identity['confirmed_ros_ns']<=ns and
        integer(health['observation_ros_ns']) and max(stamps)<=health['observation_ros_ns']<=ns,'health observation clock future/invalid')
    require(health['source_expiry_monotonic']==min(expiry,*(r+.250 for r in receipts)),'original source expiry changed')
    end=min(receipt+.5,health['source_expiry_monotonic'],now+(min(stamps)+500_000_000-ns)/1e9)
    return end


def statement_coverage(value,pin,receipt,*,now,ns):
    end=statement_bounds(value,pin,receipt,now=now,ns=ns)
    if now>=end:
        raise HealthNotCurrent('original lineage health expired',check_monotonic=now,check_ros_ns=ns,
            lineage_receipt_monotonic=receipt,source_expiry_monotonic=value['health']['source_expiry_monotonic'],
            original_header_expiry_ros_ns=min(value['original'][k] for k in
                ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'))+500_000_000)
    validate_statement(value,now_ns=ns,now_mono=now,receipt_mono=receipt)
    return end


def proof_coverage(proof,value,pin,response_receipt,*,lineage_receipt,now,ns):
    """Validate ALL proof structure/future relationships before past expiry."""
    proof_context(proof,pin)
    require(proof['state']=='ready','consumer proof is not ready')
    statement_end=statement_bounds(value,pin,lineage_receipt,now=now,ns=ns)
    original=value['original']
    require(integer(proof.get('sequence')) and proof['sequence']==value['sequence'] and
        all(integer(proof.get(k)) and proof[k]==original[k] and proof[k]<=ns for k in ('depth_stamp_ns','color_stamp_ns')),
        'consumer original lineage mismatch/future')
    require(integer(proof.get('checked_ros_ns')) and max(proof['depth_stamp_ns'],proof['color_stamp_ns'])<=proof['checked_ros_ns']<=ns,
        'consumer checked clock future/invalid')
    expiry=proof.get('original_receipt_expiry_monotonic');header=proof.get('original_header_expiry_ros_ns')
    require(finite(expiry) and 0<expiry<=min(value['health']['source_expiry_monotonic'],value['identity']['expires_monotonic'])+1e-9,
        'consumer original receipt expiry changed/invalid')
    require(type(header) is int and header==min(original[k] for k in
        ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'))+500_000_000,
        'consumer original header expiry changed/invalid')
    require(finite(response_receipt) and 0<=response_receipt<=now,'consumer RPC receipt clock future/invalid')
    end=min(response_receipt+.250,expiry,now+(header-ns)/1e9,statement_end)
    if now>=end:
        raise HealthNotCurrent('original consumer health expired',check_monotonic=now,check_ros_ns=ns,
            response_done_monotonic=response_receipt,consumer_checked_ros_ns=proof['checked_ros_ns'],
            original_receipt_expiry_monotonic=expiry,original_header_expiry_ros_ns=header,
            depth_stamp_ns=proof['depth_stamp_ns'],color_stamp_ns=proof['color_stamp_ns'])
    return end
