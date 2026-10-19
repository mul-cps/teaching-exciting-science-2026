"""Actual seven-GID DDS, tiny synthetic pixels, isolated domain, no hardware.

The source assertion is a synthetic local producer fixture, not DeviceInfo or
physical geometry qualification. The archived producer fixture covers its RPC.
"""
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


def test_actual_seven_gids_exact_pair_and_same_node_replacement_latches(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='177', 'DDS fixture requires isolated domain177'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from std_msgs.msg import String
    from so101_station_perception.derived_lineage import TOPICS,ALIGNED_TOPIC,LINEAGE_TOPIC,CLOUD_TOPIC,native_snapshot,validate_statement
    from so101_station_perception.derived_aligned import DerivedAligned
    from test_derived_registration import fixture
    prefix='/aligned_fixture_'+uuid.uuid4().hex
    rclpy.init();source=Node('camera_'+uuid.uuid4().hex);producer=Node('producer_'+uuid.uuid4().hex);consumer=Node('consumer_'+uuid.uuid4().hex)
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
                executor.spin_once(timeout_sec=.005)
                if predicate():return True
            return False
        assert spin_until(lambda:all(len(graph(topic))==1 for _,_,topic in specs.values()))
        value=fixture();value['source']['camera_node']='/'+source.get_name()
        for kind in TOPICS:value['source'][kind+'_publisher_gid']=bytes(graph(TOPICS[kind])[0].endpoint_gid)[:16].hex()
        for kind in ('cloud','status','aligned'):value['outputs'][kind+'_publisher_gid']=bytes(graph(specs[kind][2])[0].endpoint_gid)[:16].hex()
        snapshot=native_snapshot(value);generation=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()
        value['source_generation']=generation;value['identity']['source_generation']=generation;value['identity']['device_info_request_source']=snapshot
        value['calibration'].update(color_size=[8,6],depth_size=[4,3],color_K=[8.,8.,4.,3.],depth_K=[4.,4.,2.,1.5])
        def fresh(sequence):
            mono=time.monotonic();ns=consumer.get_clock().now().nanoseconds-1_000_000
            value['sequence']=sequence;value['original']['pair_delta_ns']=0
            for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][key]=ns
            value['outputs']['cloud_stamp_ns']=value['outputs']['aligned_stamp_ns']=ns
            value['identity'].update(request_started_monotonic=mono-.02,confirmed_monotonic=mono-.01,expires_monotonic=(mono-.02)+1.,confirmed_ros_ns=ns-1)
            value['health'].update(source_receipts_monotonic={kind:mono for kind in TOPICS},source_expiry_monotonic=mono+.25,observation_ros_ns=ns)
            return ns
        fresh(1);mono=time.monotonic();pin=validate_statement(value,now_ns=consumer.get_clock().now().nanoseconds,now_mono=mono,receipt_mono=mono)
        path=tmp_path/'pins.json';path.write_text(json.dumps({'derived_source':{'schema':value['schema'],'qualified':False,'pin':pin}}))
        params={'depth_topic':ALIGNED_TOPIC,'color_topic':TOPICS['color'],'camera_info_topic':TOPICS['color_info'],'derived_source_cohort':str(path)}
        callbacks=[]
        facade=NS(get_parameter=lambda k:NS(value=params[k]),get_clock=consumer.get_clock,
            create_subscription=lambda typ,topic,callback,*args,**kwargs:consumer.create_subscription(typ,prefix+topic,callback,*args,**kwargs),
            create_service=consumer.create_service,create_timer=consumer.create_timer,get_publishers_info_by_topic=graph)
        xyz=value['calibration']['translation_m'];tf=NS(transform=NS(translation=NS(x=xyz[0],y=xyz[1],z=xyz[2]),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        adapter=DerivedAligned(facade,NS(lookup_transform=lambda *args:tf),lambda *args:callbacks.append(args))
        assert spin_until(lambda:all(pubs[k].get_subscription_count() for k in ('color','aligned','color_info','depth_info','status')))
        def publish(sequence,*,rgb_offset=0,lineage=True,future=False):
            ns=fresh(sequence)
            for kind in ('color','aligned','color_info','depth_info'):
                camera='depth' if kind=='depth_info' else 'color';w,h=value['calibration'][camera+'_size'];fx,fy,cx,cy=value['calibration'][camera+'_K']
                msg=Image() if kind in ('color','aligned') else CameraInfo()
                stamp=ns+(rgb_offset if kind=='color' else 0)+(3_600_000_000_000 if future else 0)
                msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9;msg.header.frame_id=value['original'][camera+'_frame'];msg.width=w;msg.height=h
                if isinstance(msg,Image):
                    n=3 if kind=='color' else 2;msg.encoding='rgb8' if kind=='color' else '16UC1';msg.step=w*n;msg.data=bytes(w*h*n)
                else:msg.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];msg.d=[0.]*5
                pubs[kind].publish(msg)
            if lineage:pubs['status'].publish(String(data=json.dumps(value)))
        publish(2);assert spin_until(lambda:len(callbacks)==1)
        assert adapter.check(callbacks[0][3])
        # Count real graph lookup cost separately from fake-graph metadata tests.
        from so101_station_perception.derived_lineage import validate_graph
        graph_ns=[]
        for _ in range(50):
            start=time.perf_counter_ns();validate_graph(value,graph);graph_ns.append(time.perf_counter_ns()-start)
        evidence={'scope':'50 actual DDS graph validations, seven UUID endpoints; no CV/GPU',
                  'graph_mean_us':sum(graph_ns)/len(graph_ns)/1000.,
                  'graph_p95_us':sorted(graph_ns)[int(.95*(len(graph_ns)-1))]/1000.,
                  'seven_gids':{**{k:value['source'][k+'_publisher_gid'] for k in TOPICS},
                                **{k:value['outputs'][k+'_publisher_gid'] for k in ('cloud','status','aligned')}}}
        if os.environ.get('DERIVED_ALIGNED_DDS_EVIDENCE'):
            Path(os.environ['DERIVED_ALIGNED_DDS_EVIDENCE']).write_text(json.dumps(evidence,indent=2))
        assert len({value['source'][k+'_publisher_gid'] for k in TOPICS}|{value['outputs'][k+'_publisher_gid'] for k in ('cloud','status','aligned')})==7
        publish(3,rgb_offset=1);spin_until(lambda:False,.06);assert len(callbacks)==1
        publish(4,lineage=False);spin_until(lambda:False,.06);assert len(callbacks)==1
        publish(5,future=True);spin_until(lambda:False,.06);assert len(callbacks)==1
        publish(6);assert spin_until(lambda:len(callbacks)==2)
        # Original receipt is retained: replay cannot refresh the expired token.
        old=callbacks[-1][3];spin_until(lambda:False,.27)
        with pytest.raises(ValueError):adapter.check(old)
        before=len(callbacks)
        # Same producer node; genuinely new aligned endpoint must still latch.
        producer.destroy_publisher(pubs['aligned']);pubs['aligned']=producer.create_publisher(Image,prefix+ALIGNED_TOPIC,qos_profile_sensor_data)
        assert spin_until(lambda:len(graph(ALIGNED_TOPIC))==1 and bytes(graph(ALIGNED_TOPIC)[0].endpoint_gid)[:16].hex()!=pin['outputs']['aligned_publisher_gid'])
        publish(7);assert spin_until(lambda:adapter.core.ledger.invalid_reason is not None)
        assert len(callbacks)==before
        assert adapter.pop_fault()
    finally:
        if adapter:adapter.close()
        executor.shutdown(timeout_sec=1.)
        for node in (consumer,producer,source):node.destroy_node()
        rclpy.shutdown()
