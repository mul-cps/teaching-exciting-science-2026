"""Actual UUID DDS metadata-only health: no image/cloud payloads, GPU or hardware."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_perception'),str(ROOT/'tests')]


def require_isolated_domain():
    assert os.environ.get('ROS_DOMAIN_ID')=='177','source health fixture requires explicit domain177'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1','source health fixture requires localhost1'


def test_actual_health_trigger_without_images_and_same_node_info_replacement(tmp_path):
    require_isolated_domain()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo,Image,PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from so101_station_perception.derived_lineage import TOPICS,ALIGNED_TOPIC,LINEAGE_TOPIC,CLOUD_TOPIC,native_snapshot,validate_statement
    from so101_station_perception.derived_aligned import DerivedAligned
    from test_derived_registration import fixture
    prefix='/source_health_'+uuid.uuid4().hex
    rclpy.init(args=[])
    source=Node('source_'+uuid.uuid4().hex);producer=Node('producer_'+uuid.uuid4().hex);consumer=Node('consumer_'+uuid.uuid4().hex)
    executor=SingleThreadedExecutor();adapter=None
    try:
        for node in (source,producer,consumer):executor.add_node(node)
        specs={**{k:(source,Image if k in ('depth','color') else CameraInfo,t) for k,t in TOPICS.items()},
               'cloud':(producer,PointCloud2,CLOUD_TOPIC),'status':(producer,String,LINEAGE_TOPIC),'aligned':(producer,Image,ALIGNED_TOPIC)}
        pubs={kind:node.create_publisher(typ,prefix+topic,qos_profile_sensor_data) for kind,(node,typ,topic) in specs.items()}
        def graph(topic):return consumer.get_publishers_info_by_topic(prefix+topic)
        def spin_until(predicate,timeout=2.):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.002)
                if predicate():return True
            return False
        assert spin_until(lambda:all(len(graph(topic))==1 for _,_,topic in specs.values()))
        value=fixture();value['source']['camera_node']='/'+source.get_name()
        for kind in TOPICS:value['source'][kind+'_publisher_gid']=bytes(graph(TOPICS[kind])[0].endpoint_gid)[:16].hex()
        for kind in ('cloud','status','aligned'):value['outputs'][kind+'_publisher_gid']=bytes(graph(specs[kind][2])[0].endpoint_gid)[:16].hex()
        snapshot=native_snapshot(value);generation=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()
        value['source_generation']=generation;value['identity'].update(source_generation=generation,device_info_request_source=snapshot)
        def fresh(sequence):
            mono=time.monotonic();ns=consumer.get_clock().now().nanoseconds-1_000_000
            value['sequence']=sequence;value['original']['pair_delta_ns']=0
            for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][key]=ns
            value['outputs']['cloud_stamp_ns']=value['outputs']['aligned_stamp_ns']=ns
            value['identity'].update(request_started_monotonic=mono-.02,confirmed_monotonic=mono-.01,expires_monotonic=(mono-.02)+1.,confirmed_ros_ns=ns-1)
            value['health'].update(source_receipts_monotonic={kind:mono for kind in TOPICS},source_expiry_monotonic=mono+.25,observation_ros_ns=ns)
            return ns
        fresh(1);mono=time.monotonic();pin=validate_statement(value,now_ns=consumer.get_clock().now().nanoseconds,now_mono=mono,receipt_mono=mono)
        path=tmp_path/'pin.json';path.write_text(json.dumps({'derived_source':{'schema':value['schema'],'qualified':False,'pin':pin}}))
        params={'depth_topic':ALIGNED_TOPIC,'color_topic':TOPICS['color'],'camera_info_topic':TOPICS['color_info'],'derived_source_cohort':str(path)}
        facade=NS(get_parameter=lambda k:NS(value=params[k]),get_clock=consumer.get_clock,
            create_subscription=lambda typ,topic,callback,*args,**kwargs:consumer.create_subscription(typ,prefix+topic,callback,*args,**kwargs),
            create_service=consumer.create_service,create_timer=consumer.create_timer,get_publishers_info_by_topic=graph)
        xyz=value['calibration']['translation_m'];tf=NS(transform=NS(translation=NS(x=xyz[0],y=xyz[1],z=xyz[2]),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        adapter=DerivedAligned(facade,NS(lookup_transform=lambda *args:tf),lambda *args:pytest.fail('image admission must not occur'),period_s=.2)
        client=source.create_client(Trigger,'/'+consumer.get_name()+'/derived_source_health')
        assert spin_until(lambda:client.service_is_ready() and all(pubs[k].get_subscription_count() for k in ('depth_info','color_info','status')))
        def publish(sequence):
            ns=fresh(sequence)
            for kind in ('depth_info','color_info'):
                camera=kind.removesuffix('_info');w,h=value['calibration'][camera+'_size'];fx,fy,cx,cy=value['calibration'][camera+'_K']
                msg=CameraInfo();msg.header.stamp.sec, msg.header.stamp.nanosec=divmod(ns,10**9)
                msg.header.frame_id=value['original'][camera+'_frame'];msg.width=w;msg.height=h
                msg.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];msg.d=[0.]*5
                pubs[kind].publish(msg)
            pubs['status'].publish(String(data=json.dumps(value)))
        publish(2)
        assert spin_until(lambda:adapter.core.ledger.entries and all(adapter.core.streams[k] for k in ('depth_info','color_info')))
        future=client.call_async(Trigger.Request());assert spin_until(future.done)
        response=future.result();response_received_mono=time.monotonic();response_received_ros_ns=consumer.get_clock().now().nanoseconds;proof=json.loads(response.message)
        assert response.success and proof['schema']=='so101-derived-source-binding-health-v1'
        assert proof['image_admitted'] is False and proof['processing_progress'] is False
        assert proof['camera_info_validation']=='independent_native_info'
        assert not adapter.core.streams['color'] and not adapter.core.streams['aligned']
        assert adapter.last_proof is None and adapter.core.emitted==0
        again=client.call_async(Trigger.Request());assert spin_until(again.done)
        repeat=json.loads(again.result().message);assert again.result().success
        assert repeat['effective_health_expiry_monotonic']==proof['effective_health_expiry_monotonic']
        # Actual graph work is measured separately from metadata/fake-graph tests.
        from so101_station_perception.derived_lineage import validate_graph
        costs=[]
        for _ in range(25):
            started=time.perf_counter_ns();validate_graph(value,graph);costs.append(time.perf_counter_ns()-started)
        if os.environ.get('SOURCE_HEALTH_DDS_EVIDENCE'):
            Path(os.environ['SOURCE_HEALTH_DDS_EVIDENCE']).write_text(json.dumps({'scope':'actual seven UUID DDS endpoints+metadata-only Trigger; no source pixels/DeviceInfo/GPU/hardware qualification',
                'graph_lookup_mean_us':sum(costs)/len(costs)/1000,'graph_lookup_p95_us':sorted(costs)[23]/1000,
                'response':proof,'matching_lineage':value,'consumer_lineage_receipt_monotonic':adapter.core.ledger.entries[-1][2],
                'response_received_monotonic':response_received_mono,'response_received_ros_ns':response_received_ros_ns,
                'image_records':0,'actual_distinct_gids':7},indent=2))
        # Same native node, new Info endpoint: persistent generation fault.
        source.destroy_publisher(pubs['color_info']);pubs['color_info']=source.create_publisher(CameraInfo,prefix+TOPICS['color_info'],qos_profile_sensor_data)
        assert spin_until(lambda:len(graph(TOPICS['color_info']))==1 and bytes(graph(TOPICS['color_info'])[0].endpoint_gid)[:16].hex()!=pin['source']['color_info_publisher_gid'])
        publish(3);assert spin_until(lambda:adapter.core.ledger.invalid_reason is not None)
        failed=client.call_async(Trigger.Request());assert spin_until(failed.done)
        assert not failed.result().success
    finally:
        if adapter:adapter.close()
        executor.shutdown(timeout_sec=1.)
        for node in (consumer,producer,source):node.destroy_node()
        rclpy.shutdown()
