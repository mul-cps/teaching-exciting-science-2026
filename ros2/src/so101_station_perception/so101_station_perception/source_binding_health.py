"""Supervisor-only source binding witnesses; NEVER selected-image admission."""
from collections import deque
from copy import deepcopy
from .derived_lineage import require,finite,integer,hex_value
from .admission_proof import pin_digest
from .supervisor_health import HealthNotCurrent,statement_bounds

SCHEMA='so101-derived-source-binding-health-v1'
HEADERS=('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')
RECEIPTS=('consumer_lineage','source_depth','source_color','source_depth_info','source_color_info',
          'consumer_depth_info','consumer_color_info')


def binding_context(proof,pin,epoch=None):
    require(type(proof) is dict and proof.get('schema')==SCHEMA and
        all(proof.get(k) is False for k in ('image_admitted','processing_progress','qualified','execution_enabled')) and
        proof.get('state') in ('ready','unavailable') and type(proof.get('binding_invalidated')) is bool,'invalid source-binding health schema/state')
    require(proof['binding_invalidated'] is False,'source-binding context invalidated')
    require(proof.get('source_pin_sha256')==pin_digest(pin) and
        proof.get('producer_generation')==pin['producer_generation'],'source-binding pin/generation changed')
    actual=proof.get('consumer_epoch')
    require(hex_value(actual,32) and actual!='0'*32,'source-binding epoch missing')
    require(epoch is None or actual==epoch,'unexpected external source-binding epoch')


def binding_coverage(proof,value,pin,response_receipt,*,lineage_receipt,now,ns):
    binding_context(proof,pin)
    require(proof['state']=='ready','source-binding health is not ready')
    statement_end=statement_bounds(value,pin,lineage_receipt,now=now,ns=ns)
    require(proof.get('camera_info_validation')=='independent_native_info' and
        proof.get('identity_validation')=='producer_attested_device_info_no_new_rpc' and
        proof.get('optical_tf_validation')=='local_exact_depth_timestamp','source-binding evidence origin changed')
    checked=proof.get('checked_monotonic');checked_ns=proof.get('checked_ros_ns')
    require(finite(checked) and finite(response_receipt) and 0<=checked<=response_receipt<=now and
        integer(checked_ns) and checked_ns<=ns,'source-binding observation clock future/invalid')
    original=value['original'];headers=proof.get('original_header_stamps_ns')
    require(type(headers) is dict and set(headers)==set(HEADERS) and all(integer(headers[k]) and
        headers[k]==original[k] and headers[k]<=checked_ns for k in HEADERS),'source-binding original headers changed/future')
    require(integer(proof.get('sequence')) and proof['sequence']==value['sequence'] and
        all(proof.get(k)==headers[k] and type(proof.get(k)) is int for k in ('depth_stamp_ns','color_stamp_ns')),
        'source-binding original sequence/pair changed')
    require(value['health']['observation_ros_ns']<=checked_ns and
        value['identity']['confirmed_monotonic']<=checked,'source-binding check predates source observation')
    deadlines=proof.get('receipt_deadlines_monotonic')
    require(type(deadlines) is dict and set(deadlines)==set(RECEIPTS) and
        all(finite(v) and 0<v and v-.250<=checked<v for v in deadlines.values()),
        'source-binding original receipt deadlines invalid/future')
    for kind in ('depth','color','depth_info','color_info'):
        require(abs(deadlines['source_'+kind]-(value['health']['source_receipts_monotonic'][kind]+.250))<=1e-9,
            'source-binding producer receipt deadline changed')
    receipt_expiry=proof.get('original_receipt_expiry_monotonic')
    identity_expiry=proof.get('original_identity_expiry_monotonic')
    effective=proof.get('effective_health_expiry_monotonic')
    header_expiry=proof.get('original_header_expiry_ros_ns')
    require(finite(receipt_expiry) and abs(receipt_expiry-min(deadlines.values()))<=1e-9,
        'source-binding receipt minimum changed')
    require(finite(identity_expiry) and identity_expiry==value['identity']['expires_monotonic'] and checked<identity_expiry,
        'source-binding original identity expiry changed/invalid')
    require(finite(effective) and abs(effective-min(receipt_expiry,identity_expiry))<=1e-9,
        'source-binding effective expiry changed')
    require(type(header_expiry) is int and header_expiry==min(headers.values())+500_000_000 and checked_ns<header_expiry,
        'source-binding original header expiry changed/invalid')
    end=min(statement_end,response_receipt+.250,effective,now+(header_expiry-ns)/1e9)
    if now>=end:
        raise HealthNotCurrent('original source-binding health expired',check_monotonic=now,check_ros_ns=ns,
            response_done_monotonic=response_receipt,consumer_checked_monotonic=checked,consumer_checked_ros_ns=checked_ns,
            original_receipt_expiry_monotonic=receipt_expiry,original_identity_expiry_monotonic=identity_expiry,
            effective_health_expiry_monotonic=effective,original_header_expiry_ros_ns=header_expiry)
    return end


class SourceBindingProgress:
    """Two advancing metadata acquisitions; image/processing progress stays false."""
    def __init__(self,pin):
        self.pin=deepcopy(pin);self.epoch=None;self.witnesses=deque(maxlen=2)
        self.current=None;self.response_receipt=None;self.lineage_receipt=None

    @property
    def complete(self):return len(self.witnesses)==2 and self.current is not None

    def observe(self,proof,statement,*,now_ns,now_mono,response_receipt,lineage_receipt):
        binding_context(proof,self.pin,self.epoch)
        binding_coverage(proof,statement,self.pin,response_receipt,lineage_receipt=lineage_receipt,now=now_mono,ns=now_ns)
        if self.witnesses:
            previous=self.witnesses[-1]
            if proof['sequence']==previous['sequence']:
                require(all(proof[k]==previous[k] for k in ('depth_stamp_ns','color_stamp_ns','receipt_deadlines_monotonic',
                    'original_header_stamps_ns','original_receipt_expiry_monotonic','original_identity_expiry_monotonic',
                    'effective_health_expiry_monotonic','original_header_expiry_ros_ns')),
                    'repeated source-binding witness cannot renew original proof')
                return  # Preserve its FIRST unique original response/lineage receipts.
            require(all(proof[k]>previous[k] for k in ('sequence','depth_stamp_ns','color_stamp_ns')),
                'source-binding acquisitions did not independently advance')
        self.witnesses.append(deepcopy(proof));self.current=deepcopy(proof);self.epoch=proof['consumer_epoch']
        self.response_receipt=response_receipt;self.lineage_receipt=lineage_receipt

    def check(self,statement,*,now_ns,now_mono):
        require(self.current is not None,'current source-binding witness unavailable')
        binding_context(self.current,self.pin,self.epoch)
        binding_coverage(self.current,statement,self.pin,self.response_receipt,
            lineage_receipt=self.lineage_receipt,now=now_mono,ns=now_ns)
        return True
