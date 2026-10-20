"""Private repair metadata admission. No readiness, image or actuator authority."""
from .derived_lineage import require,finite,integer
from .source_binding_health import binding_context,HEADERS,RECEIPTS

PROOF={'sequence','depth_stamp_ns','color_stamp_ns','checked_ros_ns','checked_monotonic',
       'original_header_stamps_ns','original_header_expiry_ros_ns','receipt_deadlines_monotonic',
       'original_receipt_expiry_monotonic','original_identity_expiry_monotonic','effective_health_expiry_monotonic'}


def binding_metadata(proof,pin,epoch,receipt,*,now,ns):
    """A current RPC may attest unavailable binding; it never attests health here."""
    binding_context(proof,pin,epoch)
    require(finite(receipt) and 0<=now-receipt<=.250,'private binding RPC receipt expired/future')
    require(proof.get('camera_info_validation')=='independent_native_info' and
        proof.get('identity_validation')=='producer_attested_device_info_no_new_rpc' and
        proof.get('optical_tf_validation')=='local_exact_depth_timestamp','private binding evidence origin changed')
    present=PROOF.intersection(proof)
    require(not present and proof['state']=='unavailable' or present==PROOF,'partial private binding numeric evidence')
    if present:
        checked=proof['checked_monotonic'];checked_ns=proof['checked_ros_ns'];headers=proof['original_header_stamps_ns'];deadlines=proof['receipt_deadlines_monotonic']
        require(finite(checked) and 0<=checked<=receipt<=now and integer(checked_ns) and checked_ns<=ns,'private binding check future/invalid')
        require(integer(proof['sequence']) and type(headers) is dict and set(headers)==set(HEADERS) and
            all(integer(v) and v<=checked_ns for v in headers.values()) and
            all(type(proof[k]) is int and proof[k]==headers[k] for k in ('depth_stamp_ns','color_stamp_ns')) and
            abs(headers['depth_stamp_ns']-headers['color_stamp_ns'])<=40_000_000 and
            abs(headers['depth_info_stamp_ns']-headers['color_info_stamp_ns'])<=40_000_000,'private binding original headers invalid/future')
        require(type(deadlines) is dict and set(deadlines)==set(RECEIPTS) and
            all(finite(v) and 0<v and v-.250<=checked<v for v in deadlines.values()),'private binding original receipt evidence invalid/future')
        r=proof['original_receipt_expiry_monotonic'];i=proof['original_identity_expiry_monotonic'];effective=proof['effective_health_expiry_monotonic']
        require(finite(r) and r==min(deadlines.values()) and finite(i) and checked<i<=checked+1. and
            finite(effective) and effective==min(r,i) and type(proof['original_header_expiry_ros_ns']) is int and
            proof['original_header_expiry_ros_ns']==min(headers.values())+500_000_000 and checked_ns<proof['original_header_expiry_ros_ns'],
            'private binding original expiry evidence contradicts acquisition')
    # Diagnostics are never admission evidence, but future/nonfinite numeric
    # observations cannot be hidden behind an unavailable state.
    diagnostics=proof.get('native_info_diagnostics')
    if diagnostics is not None:
        require(type(diagnostics) is dict and diagnostics.get('consumer_epoch')==epoch,'private binding diagnostics context changed')
        for key,value in diagnostics.get('original_header_stamps_ns',{}).items():
            require(value is None or integer(value) and value<=ns,'private binding diagnostic header future/invalid')
        streams=diagnostics.get('streams',{});require(type(streams) is dict and set(streams).issubset({'depth_info','color_info'}),'private diagnostic streams invalid')
        for record in streams.values():
            require(type(record) is dict,'private diagnostic record invalid')
            for key,value in record.items():
                if value is None:continue
                if key.endswith('_stamp_ns'):require(integer(value) and value<=ns,'private diagnostic stamp future/invalid')
                elif key.endswith('_monotonic'):require(finite(value) and 0<=value<=now,'private diagnostic receipt future/invalid')
    return False
