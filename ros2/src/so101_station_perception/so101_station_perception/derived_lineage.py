"""Explicit diagnostic source chains. Trusted local provenance, not signatures."""
from collections import deque
from copy import deepcopy
import hashlib
import json
import math
import struct

SCHEMA='so101-derived-rocm-rgbd-cohort-v1'
TOPICS=dict(depth='/camera/depth/image_rect_raw',color='/camera/color/image_raw',
            depth_info='/camera/depth/camera_info',color_info='/camera/color/camera_info')
CLOUD_TOPIC='/so101_perception/rocm/points'
LINEAGE_TOPIC='/so101_perception/rocm/lineage'
ALIGNED_TOPIC='/so101_perception/rocm/aligned_depth'
MODULES=('rocm_transport.py','rocm_client.py','rocm_worker.py','rocm_projection.py',
         'rocm_projection_backend.py','rocm_projection_kernels.py')
MAX_JSON=65536


def configure_ros_domain(environ):
    try:domain=int(environ.get('ROS_DOMAIN_ID','76'))
    except (TypeError,ValueError) as error:raise ValueError('ROS_DOMAIN_ID must be an integer in 1..232') from error
    require(1<=domain<=232,'ROS_DOMAIN_ID must be in 1..232; domain 0 is forbidden')
    environ['ROS_DOMAIN_ID']=str(domain)
    environ.setdefault('ROS_LOCALHOST_ONLY','1')


def require(condition,reason):
    if not condition:raise ValueError('derived lineage: '+reason)


def integer(value):return type(value) is int and 0<value<2**63

def finite(value):return type(value) in (int,float) and math.isfinite(value)

def hex_value(value,size):
    return type(value) is str and len(value)==size and all(c in '0123456789abcdef' for c in value)


def canonical_gid(value):
    if isinstance(value,str):
        require(hex_value(value,32) and value!='0'*32,'missing/noncanonical GID')
        return value
    try:value=bytes(value)
    except (ValueError,TypeError):raise ValueError('derived lineage: missing GID') from None
    require(len(value) in (16,24) and any(value[:16]) and not any(value[16:]),'missing/noncanonical GID')
    return value[:16].hex()


def parse_lineage(raw):
    require(type(raw) is str and 0<len(raw)<=MAX_JSON,'status exceeds bounded text')
    def pairs(items):
        result={}
        for key,value in items:
            require(key not in result,'duplicate JSON key');result[key]=value
        return result
    try:
        value=json.loads(raw,object_pairs_hook=pairs)
        require(len(json.dumps(value,allow_nan=False))<=MAX_JSON,'status exceeds bounded JSON')
    except (TypeError,ValueError,RecursionError) as error:raise ValueError('derived lineage: invalid finite JSON') from error
    require(type(value) is dict,'status object required')
    return value


def native_snapshot(value):
    source=value['source'];node=source['camera_node']
    return dict(node=node,endpoints={stream:dict(node=node,gid=source[stream+'_publisher_gid']) for stream in TOPICS})


def statement_pin(value):
    try:
        return deepcopy({key:value[key] for key in ('producer_generation','source_generation','source','calibration','implementation')} |
            {'outputs':{key:value['outputs'][key] for key in ('cloud_topic','status_topic','aligned_topic','cloud_publisher_gid','status_publisher_gid','aligned_publisher_gid','cloud_frame','aligned_frame')}})
    except (KeyError,TypeError) as error:raise ValueError('derived lineage: incomplete immutable source chain') from error


def validate_statement(value,*,now_ns,now_mono,receipt_mono):
    try:
        require(value.get('schema')==SCHEMA and all(value.get(k) is False for k in
            ('qualified','execution_enabled','direct_camera_attestation')),'unqualified diagnostic schema required')
        require(integer(now_ns) and finite(now_mono) and finite(receipt_mono) and 0<=now_mono-receipt_mono<=.5,'invalid/stale receipt clock')
        require(hex_value(value['producer_generation'],32) and integer(value['sequence']),'invalid producer generation/sequence')
        original,source,outputs,identity,health=(value[k] for k in ('original','source','outputs','identity','health'))
        stamps=[original[k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')]
        require(all(integer(t) and 0<=now_ns-t<=500_000_000 for t in stamps),'original exposures future/stale/noninteger')
        require(type(original['pair_delta_ns']) is int and original['pair_delta_ns']==abs(stamps[0]-stamps[1])<=40_000_000,'original pair mismatch')
        require(abs(stamps[2]-stamps[0])<=40_000_000 and abs(stamps[3]-stamps[1])<=40_000_000,'original CameraInfo not paired')
        require(all(type(original[k]) is str and 0<len(original[k])<=128 for k in ('depth_frame','color_frame')),'invalid optical frames')
        require(source['depth_topic']==TOPICS['depth'] and source['color_topic']==TOPICS['color'],'unsupported original topics')
        require(type(source['camera_node']) is str and source['camera_node'].startswith('/') and len(source['camera_node'])<=128,'invalid camera node')
        require(type(source['serial_number']) is str and 0<len(source['serial_number'])<=128,'actual camera serial required')
        for stream in TOPICS:canonical_gid(source[stream+'_publisher_gid'])
        snapshot=native_snapshot(value)
        generation=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()
        require(value['source_generation']==generation and identity['source_generation']==generation and
                identity['device_info_request_source']==snapshot,'source generation/request endpoint snapshot mismatch')
        require(identity['serial_number']==source['serial_number'],'DeviceInfo serial mismatch')
        request,confirmed,expiry=(identity[k] for k in ('request_started_monotonic','confirmed_monotonic','expires_monotonic'))
        require(all(finite(t) for t in (request,confirmed,expiry)) and 0<=request<=confirmed<=now_mono<expiry and
                expiry==request+1. and confirmed-request<=.4,'DeviceInfo identity lease expired/invalid')
        require(integer(identity['confirmed_ros_ns']) and 0<=now_ns-identity['confirmed_ros_ns']<=1_000_000_000,'future/stale identity ROS clock')
        require(outputs['cloud_topic']==CLOUD_TOPIC and outputs['status_topic']==LINEAGE_TOPIC and outputs['aligned_topic']==ALIGNED_TOPIC,'unsupported derived topics')
        for key in ('cloud_publisher_gid','status_publisher_gid','aligned_publisher_gid'):canonical_gid(outputs[key])
        require(len({source[k+'_publisher_gid'] for k in TOPICS}|{outputs['cloud_publisher_gid'],outputs['status_publisher_gid'],outputs['aligned_publisher_gid']})==7,'seven distinct publisher GIDs required')
        require(type(outputs['cloud_stamp_ns']) is int and type(outputs['aligned_stamp_ns']) is int and
                outputs['cloud_stamp_ns']==outputs['aligned_stamp_ns']==stamps[0] and
                outputs['cloud_frame']==original['depth_frame'] and outputs['aligned_frame']==original['color_frame'],'output original stamp/frame mismatch')
        receipts=health['source_receipts_monotonic']
        require(set(receipts)==set(TOPICS) and all(finite(t) and 0<=now_mono-t<=.250 for t in receipts.values()),'source receipts expired/future')
        expected_expiry=min(expiry,*(t+.250 for t in receipts.values()))
        require(health['state']=='valid' and health['original_max_age_ns']==500_000_000 and
                type(health['original_max_age_ns']) is int and health['source_expiry_monotonic']==expected_expiry and now_mono<expected_expiry,'source health lease invalid/expired')
        require(integer(health['observation_ros_ns']) and max(stamps)<=health['observation_ros_ns']<=now_ns,'invalid observation clock')
        calibration=value['calibration'];require(hex_value(calibration['sha256'],64),'configuration-file hash required')
        for kind,maxima in (('depth',(1280,720)),('color',(1920,1080))):
            size=calibration[kind+'_size'];k=calibration[kind+'_K'];d=calibration[kind+'_D']
            require(type(size) is list and len(size)==2 and all(type(x) is int and 1<=x<=m for x,m in zip(size,maxima)),'unsupported dimensions')
            require(type(k) is list and len(k)==4 and all(finite(x) and abs(x)<=100000 for x in k) and min(k[:2])>0,'invalid intrinsics')
            require(type(d) is list and len(d)<=8 and all(finite(x) and x==0 for x in d),'unsupported distortion')
        require(len(calibration['rotation_column_major'])==9 and len(calibration['translation_m'])==3 and
                all(finite(x) for x in calibration['rotation_column_major']+calibration['translation_m']) and
                calibration['depth_scale_m']==.001 and type(calibration['depth_scale_m']) is float,'invalid projection calibration')
        require(calibration['depth_scale_source']=='existingROS16UC1millimetercontract; notindependentphysicalmeasurement','scale provenance required')
        implementation=value['implementation'];hashes=implementation['source_sha256']
        require(type(hashes) is dict and set(hashes)==set(MODULES) and all(hex_value(v,64) for v in hashes.values()),'fixed six core/worker hashes required')
        require(all(hex_value(implementation[k],64) for k in ('worker_python_sha256','compatibility_sha256','producer_node_sha256')) and
                type(implementation['worker_python']) is str and implementation['worker_python'].startswith('/') and
                type(implementation['hardware']) is dict and bool(implementation['hardware']),'worker/runtime provenance required')
        metrics=value['metrics'];require(type(metrics['points']) is int and 0<=metrics['points']<=921600 and
            type(metrics['cloud_bytes']) is int and metrics['cloud_bytes']==metrics['points']*20 and
            all(finite(metrics[k]) and metrics[k]>=0 for k in ('backend_ms','worker_ms')),'invalid bounded cloud metrics')
        return statement_pin(value)
    except (KeyError,TypeError,OverflowError) as error:raise ValueError('derived lineage: incomplete/invalid statement') from error


def validate_graph(value,lookup,*,cloud_gid=None,lineage_gid=None,aligned_gid=None):
    """Actual graph endpoints, not node-name-only source association."""
    records={}
    for stream,topic in TOPICS.items():
        endpoints=lookup(topic);require(len(endpoints)==1,'native source graph is not unique')
        endpoint=endpoints[0];node=endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name
        gid=canonical_gid(endpoint.endpoint_gid)
        require(gid==value['source'][stream+'_publisher_gid'] and node==value['source']['camera_node'],'native publisher generation changed')
        records[stream]=dict(node=node,gid=gid)
    derived_nodes=[]
    for kind,received in (('cloud',cloud_gid),('status',lineage_gid),('aligned',aligned_gid)):
        outputs=value['outputs'];endpoints=lookup(outputs[kind+'_topic'])
        require(len(endpoints)==1,'derived source graph is not unique')
        endpoint=endpoints[0];gid=canonical_gid(endpoint.endpoint_gid)
        require(gid==outputs[kind+'_publisher_gid'] and (received is None or gid==canonical_gid(received)),'derived publisher generation changed')
        derived_nodes.append(endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name)
    require(len(set(derived_nodes))==1,'cloud/lineage producer endpoints differ')
    return dict(node=value['source']['camera_node'],endpoints=records)


def validate_cloud(value,cloud,gid):
    require(canonical_gid(gid)==value['outputs']['cloud_publisher_gid'],'received cloud generation changed')
    require(type(cloud.header.stamp.sec) is int and type(cloud.header.stamp.nanosec) is int and
            cloud.header.stamp.sec>=0 and 0<=cloud.header.stamp.nanosec<1_000_000_000,'invalid cloud original stamp fields')
    stamp=cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec
    require(stamp==value['original']['depth_stamp_ns'] and cloud.header.frame_id==value['original']['depth_frame'],'exact cloud original cohort mismatch')
    require(cloud.height==1 and cloud.width==value['metrics']['points'] and cloud.point_step==20 and
            not cloud.is_bigendian and cloud.row_step==len(cloud.data)==value['metrics']['cloud_bytes'],'packed cloud layout differs')
    require([(f.name,f.offset,f.datatype,f.count) for f in cloud.fields]==
            [('x',0,7,1),('y',4,7,1),('z',8,7,1),('rgb',16,7,1)],'packed cloud fields differ')


def validate_camera(value,depth_info,color_info,color_from_depth):
    """Compare actual native metadata/TF. Config-file SHA is a reported pin."""
    for kind,message in (('depth',depth_info),('color',color_info)):
        original=value['original'];cal=value['calibration']
        stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        require(stamp==original[kind+'_info_stamp_ns'] and message.header.frame_id==original[kind+'_frame'],'exact native CameraInfo exposure/frame mismatch')
        require([message.width,message.height]==cal[kind+'_size'],'native CameraInfo dimensions changed')
        k=[float(x) for x in message.k];d=[float(x) for x in message.d];expected=cal[kind+'_K']
        require(len(k)==9 and all(finite(x) and abs(x)<=100000 for x in k) and list(k[1:2])+list(k[3:4])+list(k[6:])==[0.,0.,0.,0.,1.],'unsupported native K layout')
        require([struct.pack('f',x) for x in (k[0],k[4],k[2],k[5])]==[struct.pack('f',x) for x in expected] and
                len(d)==len(cal[kind+'_D']) and all(finite(x) and x==0 for x in d),'native CameraInfo calibration changed')
    cal=value['calibration'];rotation=cal['rotation_column_major'];translation=cal['translation_m']
    require(all(abs(float(color_from_depth[i][j])-rotation[j*3+i])<=2e-6 for i in range(3) for j in range(3)) and
            all(abs(float(color_from_depth[i][3])-translation[i])<=2e-8 for i in range(3)),'exact optical TF differs from projection calibration')


class LineageBuffer:
    def __init__(self,pin=None):
        if pin is not None:
            try:
                require(hex_value(pin['implementation']['producer_node_sha256'],64),'producer node code hash required in pins')
                require(pin['outputs']['aligned_topic']==ALIGNED_TOPIC,'aligned topic required in pins')
                canonical_gid(pin['outputs']['aligned_publisher_gid'])
            except (KeyError,TypeError) as error:raise ValueError('derived lineage: source pins require refreshed aligned/node-hash contract') from error
        self.pin=deepcopy(pin);self.entries=deque(maxlen=8);self.invalid_reason=None
        self.last_sequence=0;self.last_depth=0;self.last_color=0

    def invalidate(self,reason):
        self.invalid_reason=str(reason);self.entries.clear()
        raise ValueError(self.invalid_reason)

    def observe_gid(self,kind,gid):
        if self.invalid_reason:raise ValueError(self.invalid_reason)
        if self.pin is not None:
            expected=(self.pin['outputs'] if kind in ('cloud','status','aligned') else self.pin['source'])[kind+'_publisher_gid']
            try:actual=canonical_gid(gid)
            except ValueError:self.invalidate('derived source GID unavailable; explicit repin/reset required')
            if actual!=expected:self.invalidate('derived source generation changed; explicit repin/reset required')

    def add(self,raw,gid,*,now_ns,now_mono):
        if self.invalid_reason:raise ValueError(self.invalid_reason)
        try:
            value=parse_lineage(raw);pin=statement_pin(value)
            if self.pin is not None and pin!=self.pin:self.invalidate('derived immutable source changed; explicit repin/reset required')
            self.observe_gid('status',gid)
            validate_statement(value,now_ns=now_ns,now_mono=now_mono,receipt_mono=now_mono)
            require(canonical_gid(gid)==value['outputs']['status_publisher_gid'],'received lineage GID differs')
            if self.pin is None:self.pin=pin
            if value['sequence']<=self.last_sequence:return False
            d,c=(value['original'][k] for k in ('depth_stamp_ns','color_stamp_ns'))
            if d<=self.last_depth or c<=self.last_color:self.invalidate('derived independent exposure replay/rollback')
            self.last_sequence,self.last_depth,self.last_color=value['sequence'],d,c
            self.entries.append((value,canonical_gid(gid),now_mono));return True
        except (ValueError,KeyError,TypeError):
            self.entries.clear();raise

    def match(self,stamp,*,now_ns,now_mono):
        if self.invalid_reason:raise ValueError(self.invalid_reason)
        matches=[item for item in self.entries if item[0]['original']['depth_stamp_ns']==stamp]
        require(bool(matches),'no exact per-cohort lineage; no old-status fallback')
        item=matches[-1];validate_statement(item[0],now_ns=now_ns,now_mono=now_mono,receipt_mono=item[2]);return item
