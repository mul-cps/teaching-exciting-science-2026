"""Native metadata-only camera pinner: no physical camera, images or GPU."""
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import uuid
import pytest
pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_perception'),str(ROOT/'tests')]


@pytest.mark.parametrize('private_executor',[False,True])
def test_actual_metadata_dds_rpc_exact_tf_and_generation_replacement(tmp_path,monkeypatch,private_executor):
    assert os.environ.get('ROS_DOMAIN_ID')=='177','isolated domain177 required'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from realsense2_camera_msgs.srv import DeviceInfo
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from so101_station_perception.camera_source import CameraSourceCapture
    from so101_station_perception.derived_lineage import TOPICS,CLOUD_TOPIC,LINEAGE_TOPIC,ALIGNED_TOPIC,native_snapshot
    from test_derived_registration import fixture
    suffix=uuid.uuid4().hex;prefix='/camera_pin_'+suffix
    canonical=set(TOPICS.values())|{CLOUD_TOPIC,LINEAGE_TOPIC,ALIGNED_TOPIC}
    original_subscribe=Node.create_subscription;original_lookup=Node.get_publishers_info_by_topic
    def subscribe(self,typ,topic,*args,**kwargs):return original_subscribe(self,typ,prefix+topic if topic in canonical else topic,*args,**kwargs)
    def lookup(self,topic,*args,**kwargs):return original_lookup(self,prefix+topic if topic in canonical else topic,*args,**kwargs)
    monkeypatch.setattr(Node,'create_subscription',subscribe);monkeypatch.setattr(Node,'get_publishers_info_by_topic',lookup)
    rclpy.init();source=Node('camera_'+suffix);producer=Node('producer_'+suffix);consumer=None;consumer2=None;thread=None;capture_executor=None
    executor=SingleThreadedExecutor();executor.add_node(source);executor.add_node(producer)
    try:
        specs={**{k:(source,Image if k in ('depth','color') else CameraInfo,t) for k,t in TOPICS.items()},
            'cloud':(producer,PointCloud2,CLOUD_TOPIC),'status':(producer,String,LINEAGE_TOPIC),'aligned':(producer,Image,ALIGNED_TOPIC)}
        pubs={kind:node.create_publisher(typ,prefix+topic,qos_profile_sensor_data) for kind,(node,typ,topic) in specs.items()}
        value=fixture();value['source']['camera_node']='/'+source.get_name()
        for kind in TOPICS:value['source'][kind+'_publisher_gid']=bytes(source.get_publishers_info_by_topic(TOPICS[kind])[0].endpoint_gid)[:16].hex()
        for kind in ('cloud','status','aligned'):value['outputs'][kind+'_publisher_gid']=bytes(source.get_publishers_info_by_topic(specs[kind][2])[0].endpoint_gid)[:16].hex()
        value['source_generation']=hashlib.sha256(json.dumps(native_snapshot(value),sort_keys=True).encode()).hexdigest()
        value['identity']['source_generation']=value['source_generation'];value['identity']['device_info_request_source']=native_snapshot(value)
        value['original']['depth_frame']='depth_'+suffix;value['original']['color_frame']='color_'+suffix
        value['outputs']['cloud_frame']=value['original']['depth_frame'];value['outputs']['aligned_frame']=value['original']['color_frame']
        broadcaster=StaticTransformBroadcaster(source);tf=TransformStamped()
        tf.header.frame_id=value['original']['depth_frame'];tf.child_frame_id=value['original']['color_frame'];tf.transform.rotation.w=1.
        tf.transform.translation.x=-value['calibration']['translation_m'][0]
        broadcaster.sendTransform(tf)
        state={'replace':False,'requests':0,'sequence':0}
        def device(request,response):
            state['requests']+=1
            if state['replace']:
                source.destroy_publisher(pubs['color_info'])
                pubs['color_info']=source.create_publisher(CameraInfo,prefix+TOPICS['color_info'],qos_profile_sensor_data)
            response.serial_number=value['source']['serial_number'];return response
        service=source.create_service(DeviceInfo,'/'+source.get_name()+'/device_info',device)
        def emit():
            mono=time.monotonic();ns=source.get_clock().now().nanoseconds-1_000_000
            state['sequence']+=1;value['sequence']=state['sequence'];value['original']['pair_delta_ns']=0
            for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][key]=ns
            value['outputs']['cloud_stamp_ns']=value['outputs']['aligned_stamp_ns']=ns
            value['identity'].update(request_started_monotonic=mono-.02,confirmed_monotonic=mono-.01,expires_monotonic=(mono-.02)+1.,confirmed_ros_ns=ns-1)
            value['health'].update(source_receipts_monotonic={kind:mono for kind in TOPICS},source_expiry_monotonic=mono+.25,observation_ros_ns=ns)
            for kind in ('depth','color'):
                info=CameraInfo();info.header.stamp.sec=ns//10**9;info.header.stamp.nanosec=ns%10**9;info.header.frame_id=value['original'][kind+'_frame']
                info.width,info.height=value['calibration'][kind+'_size'];fx,fy,cx,cy=value['calibration'][kind+'_K']
                info.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];info.d=[0.]*5;info.distortion_model="plumb_bob"
                pubs[kind+'_info'].publish(info)
            pubs['status'].publish(String(data=json.dumps(value)))
        timer=producer.create_timer(.03,emit)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        consumer=CameraSourceCapture(serial=value['source']['serial_number']);output=tmp_path/'camera.json'
        if private_executor:
            capture_executor=SingleThreadedExecutor(context=consumer.context)
            capture_executor.add_node(consumer)
        started=time.monotonic();result=consumer.capture(timeout=2.,output=output,executor=capture_executor);elapsed=time.monotonic()-started
        assert state['requests']==1 and result['schema']=='so101-camera-source-v1'
        from so101_station_perception.derived_lineage import validate_graph
        times=[];selected=result['camera_evidence']['lineage']['statement']
        for _ in range(50):
            started=time.perf_counter_ns();validate_graph(selected,consumer.get_publishers_info_by_topic);times.append(time.perf_counter_ns()-started)
        if os.environ.get('CAMERA_SOURCE_DDS_EVIDENCE'):
            Path(os.environ['CAMERA_SOURCE_DDS_EVIDENCE']).write_text(json.dumps({'scope':'actual synthetic DDS metadata-only, native DeviceInfo fake and exact static TF; no physical/GPU',
                'capture_s':elapsed,'graph_7_endpoints_mean_us':sum(times)/len(times)/1000.,'graph_7_endpoints_p95_us':sorted(times)[47]/1000.,
                'subscribed_payloads':False,'artifact':result},indent=2))

        assert json.loads(output.read_text())==result
        evidence=json.loads(output.with_name('camera.evidence.json').read_text())
        assert evidence['lineage']['statement']['sequence']>=2
        assert abs(evidence['color_from_depth'][0][3]-.015)<2e-8
        assert set(result['derived_source']['pin']['outputs'])>={'aligned_topic','aligned_publisher_gid'}
        # Only two native Info and lineage subscriptions; no image/cloud payload.
        assert all(pubs[k].get_subscription_count()==0 for k in ('depth','color','aligned','cloud'))
        assert len(consumer.core.infos['depth'])<=4 and len(consumer.core.ledger.entries)<=8
        if capture_executor:
            capture_executor.remove_node(consumer);capture_executor.shutdown(timeout_sec=1.);capture_executor=None
        consumer.destroy_node();consumer=None
        state['replace']=True;consumer2=CameraSourceCapture(serial=value['source']['serial_number'])
        rejected=tmp_path/'rejected.json'
        with pytest.raises(ValueError,match='generation|publisher|GID'):consumer2.capture(timeout=2.,output=rejected)
        assert not rejected.exists() and not rejected.with_name('rejected.evidence.json').exists()
        assert state['requests']==2 and consumer2.core.ledger.invalid_reason
    finally:
        if capture_executor:capture_executor.shutdown(timeout_sec=1.)
        if consumer:consumer.destroy_node()
        if consumer2:consumer2.destroy_node()
        executor.shutdown(timeout_sec=1.)
        if thread:thread.join(timeout=1.)
        source.destroy_node();producer.destroy_node();rclpy.shutdown()


def test_private_context_can_suppress_parent_global_node_remap():
    from rclpy.context import Context
    from so101_station_perception.camera_source import CameraSourceCapture
    context=Context();context.init(args=['--ros-args','-r','__node:=parent_recovery'],domain_id=177)
    node=None
    try:
        node=CameraSourceCapture(context=context,use_global_arguments=False)
        assert node.get_name()=='camera_capture_sources'
    finally:
        if node:node.destroy_node()
        context.shutdown()


def test_camera_only_artifact_is_not_physical_registration_source(tmp_path):
    import types
    private=types.ModuleType('so101_station_perception_tag_tracking');private.__path__=[str(ROOT/'workshop/digital_twin')]
    sys.modules.setdefault(private.__name__,private)
    from so101_station_perception.registration_monitor import RegistrationMonitor
    path=tmp_path/'camera.json';path.write_text(json.dumps({'schema':'so101-camera-source-v1','qualified':False,'derived_source':{}}))
    statuses=[]
    node=NS(cohort_path=str(path),marker_path='',allow_derived=True,joints=NS(samples=[]),clouds=deque(),infos=deque(),boards=deque(),derived_infos={},
        _publish=lambda state,**kwargs:statuses.append((state,kwargs)))
    RegistrationMonitor._load_cohort(node)
    assert node.cohort is None and 'incomplete pinned source' in statuses[-1][1]['reason']


def test_main_domain0_rejected_before_ros_startup(monkeypatch,tmp_path):
    import so101_station_perception.camera_source as module
    monkeypatch.setenv('ROS_DOMAIN_ID','00');started=[];monkeypatch.setattr(module.rclpy,'init',lambda **kwargs:started.append(True))
    with pytest.raises(ValueError,match='domain 0'):module.main(['--output',str(tmp_path/'never.json')])
    assert not started


def test_own_rpc_lease_rechecked_after_graph_and_tf_access(monkeypatch):
    import so101_station_perception.camera_source as module
    from test_camera_source_pin import prepared
    from test_derived_registration import endpoints
    _,core,value,selected,matrix,identity=prepared()
    identity.update(request_started_monotonic=99.2,confirmed_monotonic=99.3)
    clocks=iter([dict(now_ns=10_120_000_000,now_mono=100.12),dict(now_ns=10_120_000_000,now_mono=100.21)])
    monkeypatch.setattr(module.time,'monotonic',lambda:100.12)
    node=NS(core=core,identity=identity,_matrix=lambda selected:matrix,_clocks=lambda:next(clocks),get_publishers_info_by_topic=endpoints(value).__getitem__)
    with pytest.raises(ValueError,match='own DeviceInfo lease'):module.CameraSourceCapture.verify(node,selected,101.)


def test_observed_pre_rpc_generation_change_latches_but_initial_discovery_can_complete():
    from so101_station_perception.camera_source import CameraSourceCapture
    from test_camera_source_pin import prepared
    from test_derived_registration import endpoints
    _,core,value,selected,matrix,identity=prepared();graph=endpoints(value)
    node=NS(core=core,rpc_snapshot=None,graph_bound=False,get_publishers_info_by_topic=graph.__getitem__)
    graph['/camera/color/image_raw'][0].endpoint_gid=b'\xdd'*16
    with pytest.raises(ValueError):CameraSourceCapture._graph(node,value,selected['publisher_gid'])
    assert core.ledger.invalid_reason
    _,core,value,selected,matrix,identity=prepared();graph=endpoints(value);saved=graph['/camera/depth/image_rect_raw'];graph['/camera/depth/image_rect_raw']=[]
    node=NS(core=core,rpc_snapshot=None,graph_bound=False,get_publishers_info_by_topic=graph.__getitem__)
    with pytest.raises(ValueError):CameraSourceCapture._graph(node,value,selected['publisher_gid'])
    assert core.ledger.invalid_reason is None
    graph['/camera/depth/image_rect_raw']=saved
    CameraSourceCapture._graph(node,value,selected['publisher_gid']);assert node.graph_bound
    graph['/camera/color/image_raw']=[]
    with pytest.raises(ValueError):CameraSourceCapture._graph(node,value,selected['publisher_gid'])
    assert core.ledger.invalid_reason
