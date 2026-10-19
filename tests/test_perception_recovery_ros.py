"""Real isolated DDS; fake source/producer/consumers, REAL metadata pinner."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid

import pytest
pytest.importorskip('rclpy')
from test_derived_registration import fixture


def test_actual_dds_private_pinner_full_recovery_and_original_expiry(tmp_path,monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID')=='177','isolated domain177 required'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from realsense2_camera_msgs.srv import DeviceInfo
    from so101_station_perception.perception_recovery import PerceptionRecovery,CONSUMERS,PRODUCER_STATUS,read_artifact
    from so101_station_perception.admission_proof import pin_digest
    from so101_station_perception.derived_lineage import TOPICS,CLOUD_TOPIC,LINEAGE_TOPIC,ALIGNED_TOPIC,native_snapshot,statement_pin
    suffix=uuid.uuid4().hex;prefix='/recover_'+suffix
    canonical=set(TOPICS.values())|{CLOUD_TOPIC,LINEAGE_TOPIC,ALIGNED_TOPIC,PRODUCER_STATUS}
    original_subscribe=Node.create_subscription;original_lookup=Node.get_publishers_info_by_topic
    def subscribe(self,typ,topic,*args,**kwargs):return original_subscribe(self,typ,prefix+topic if topic in canonical else topic,*args,**kwargs)
    def lookup(self,topic,*args,**kwargs):return original_lookup(self,prefix+topic if topic in canonical else topic,*args,**kwargs)
    monkeypatch.setattr(Node,'create_subscription',subscribe);monkeypatch.setattr(Node,'get_publishers_info_by_topic',lookup)
    baseline_path=tmp_path/'baseline.json';serial='actual_serial'
    rclpy.init(args=['--ros-args','-p','enabled:=true','-p','artifact_directory:='+str(tmp_path),
                    '-p','expected_source_cohort:='+str(baseline_path),'-p','expected_serial:='+serial])
    source=Node('camera_'+suffix);producer=Node('so101_rocm_rgbd');probe=Node('recovery_probe_'+suffix)
    coordinator=None;consumers=[];executor=SingleThreadedExecutor()
    try:
        specs={**{k:(source,Image if k in ('depth','color') else CameraInfo,t) for k,t in TOPICS.items()},
            'cloud':(producer,PointCloud2,CLOUD_TOPIC),'status':(producer,String,LINEAGE_TOPIC),
            'aligned':(producer,Image,ALIGNED_TOPIC),'heartbeat':(producer,String,PRODUCER_STATUS)}
        pubs={k:n.create_publisher(typ,prefix+t,qos_profile_sensor_data) for k,(n,typ,t) in specs.items()}
        value=fixture();value['source']['camera_node']='/'+source.get_name()
        value['implementation']['hardware']={'test_only':'CPU synthetic producer, no GPU/pixels'}
        for kind in TOPICS:value['source'][kind+'_publisher_gid']=bytes(source.get_publishers_info_by_topic(TOPICS[kind])[0].endpoint_gid)[:16].hex()
        for kind in ('cloud','status','aligned'):value['outputs'][kind+'_publisher_gid']=bytes(source.get_publishers_info_by_topic(specs[kind][2])[0].endpoint_gid)[:16].hex()
        value['source_generation']=hashlib.sha256(json.dumps(native_snapshot(value),sort_keys=True).encode()).hexdigest()
        value['identity']['source_generation']=value['source_generation'];value['identity']['device_info_request_source']=native_snapshot(value)
        value['original']['depth_frame']='depth_'+suffix;value['original']['color_frame']='color_'+suffix
        value['outputs']['cloud_frame']=value['original']['depth_frame'];value['outputs']['aligned_frame']=value['original']['color_frame']
        baseline={'schema':'so101-camera-source-v1','qualified':False,'execution_enabled':False,
            'metadata_only':True,'independent_pixels_observed':False,
            'derived_source':{'schema':'so101-derived-rocm-rgbd-cohort-v1','qualified':False,'pin':statement_pin(value)},'camera_evidence':{'lineage':{'statement':deepcopy(value)}}}
        baseline_path.write_text(json.dumps(baseline))
        broadcaster=StaticTransformBroadcaster(source);tf=TransformStamped()
        tf.header.frame_id=value['original']['depth_frame'];tf.child_frame_id=value['original']['color_frame']
        tf.transform.rotation.w=1.;tf.transform.translation.x=-.015;broadcaster.sendTransform(tf)
        requests={'device':0,'reset':0};reset={'id':None}
        def device(req,response):requests['device']+=1;response.serial_number=serial;return response
        source.create_service(DeviceInfo,'/'+source.get_name()+'/device_info',device)
        def producer_reset(req,response):
            requests['reset']+=1;reset['id']=uuid.uuid4().hex;value['producer_generation']=uuid.uuid4().hex;value['sequence']=0
            response.success=True;response.message=json.dumps(dict(accepted=True,ready=False,
                accepted_reset_id=reset['id'],new_producer_generation=value['producer_generation']))
            return response
        producer.create_service(Trigger,'/so101_rocm_rgbd/reset_source',producer_reset)
        state={}
        for name in CONSUMERS:
            node=Node(name[1:]);node.declare_parameter('derived_source_cohort',str(baseline_path));consumers.append(node)
            state[name]={'epoch':uuid.uuid4().hex,'pin':baseline['derived_source']['pin'],'resets':0}
            def reset_consumer(req,response,node=node,name=name):
                artifact,_=read_artifact(node.get_parameter('derived_source_cohort').value)
                state[name].update(epoch=uuid.uuid4().hex,pin=artifact['derived_source']['pin'])
                state[name]['resets']+=1;response.success=True;response.message='reloaded; not readiness';return response
            def status(req,response,node=node,name=name):
                pin=state[name]['pin'];ready=pin==statement_pin(value) and value['sequence']>=2
                data=dict(schema='so101-derived-admission-status-v1',qualified=False,execution_enabled=False,
                    consumer_epoch=state[name]['epoch'],source_pin_sha256=pin_digest(pin),
                    producer_generation=pin['producer_generation'],state='ready' if ready else 'awaiting')
                if ready:
                    data.update(sequence=value['sequence'],depth_stamp_ns=value['original']['depth_stamp_ns'],
                        color_stamp_ns=value['original']['color_stamp_ns'],checked_ros_ns=node.get_clock().now().nanoseconds,
                        original_receipt_expiry_monotonic=value['health']['source_expiry_monotonic'],
                        original_header_expiry_ros_ns=value['original']['depth_stamp_ns']+500_000_000)
                response.success=ready;response.message=json.dumps(data);return response
            node.create_service(Trigger,name+'/reset_derived_source',reset_consumer)
            node.create_service(Trigger,name+'/derived_source_status',status)
        def emit():
            mono=time.monotonic();ns=source.get_clock().now().nanoseconds-1_000_000
            value['sequence']+=1;value['original']['pair_delta_ns']=0
            for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][k]=ns
            value['outputs']['cloud_stamp_ns']=value['outputs']['aligned_stamp_ns']=ns
            value['identity'].update(request_started_monotonic=mono-.02,confirmed_monotonic=mono-.01,
                expires_monotonic=(mono-.02)+1.,confirmed_ros_ns=ns-1)
            value['health'].update(source_receipts_monotonic={k:mono for k in TOPICS},source_expiry_monotonic=mono+.25,observation_ros_ns=ns)
            for kind in ('depth','color'):
                msg=CameraInfo();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
                msg.header.frame_id=value['original'][kind+'_frame'];msg.width,msg.height=value['calibration'][kind+'_size']
                fx,fy,cx,cy=value['calibration'][kind+'_K'];msg.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];msg.d=[0.]*5;msg.distortion_model='plumb_bob'
                pubs[kind+'_info'].publish(msg)
            pubs['status'].publish(String(data=json.dumps(value)))
            pubs['heartbeat'].publish(String(data=json.dumps(dict(schema='so101-derived-rocm-rgbd-v1',qualified=False,
                execution_enabled=False,state='ready' if value['sequence']>=2 else 'preparing',producer_generation=value['producer_generation'],reset=dict(
                accepted_reset_id=reset['id'],new_producer_generation=value['producer_generation'],
                ready=bool(reset['id']) and value['sequence']>=2,state='preparing' if value['sequence']<2 else 'ready')))))
        timer=producer.create_timer(.03,emit)
        coordinator=PerceptionRecovery()
        for node in [source,producer,probe,*consumers,coordinator]:executor.add_node(node)
        def wait(predicate,seconds=6.):
            until=time.monotonic()+seconds
            while time.monotonic()<until and not predicate():executor.spin_once(timeout_sec=.005)
            assert predicate(),coordinator.result
        starter=probe.create_client(Trigger,'/so101_perception_recovery/start_recovery')
        status_client=probe.create_client(Trigger,'/so101_perception_recovery/recovery_status')
        wait(starter.service_is_ready)
        job_started=time.monotonic()
        future=starter.call_async(Trigger.Request());wait(future.done)
        accepted=json.loads(future.result().message);assert future.result().success and accepted['ready_now'] is False
        duplicate=starter.call_async(Trigger.Request());wait(duplicate.done);assert not duplicate.result().success
        wait(lambda:coordinator.result['state'] in ('completed','failed'),seconds=10.)
        assert coordinator.result['state']=='completed',coordinator.result
        future=status_client.call_async(Trigger.Request());wait(future.done)
        completed=json.loads(future.result().message)
        assert completed['operation_success'] is True and completed['ready_now'] is True
        assert requests=={'device':1,'reset':1} and all(v['resets']==1 for v in state.values())
        artifact=json.loads(Path(completed['artifact']).read_text())
        assert artifact['camera_evidence']['independent_device_info']['serial_number']==serial
        assert artifact['derived_source']['pin']['producer_generation']==value['producer_generation']
        # No camera/image/cloud payload subscriptions were needed, even by pinner.
        assert all(pubs[k].get_subscription_count()==0 for k in ('depth','color','aligned','cloud'))
        until=time.monotonic()+.35
        while time.monotonic()<until:executor.spin_once(timeout_sec=.005)
        future=status_client.call_async(Trigger.Request());wait(future.done)
        expired=json.loads(future.result().message)
        assert expired['operation_success'] is True and expired['ready_now'] is False
        assert len(coordinator.ledger.entries)<=8
        from so101_station_perception.derived_lineage import validate_graph
        graph_times=[]
        for _ in range(50):
            tick=time.perf_counter_ns();validate_graph(value,coordinator.get_publishers_info_by_topic)
            graph_times.append((time.perf_counter_ns()-tick)/1000.)
        if os.environ.get('RECOVERY_COORDINATOR_DDS_EVIDENCE'):
            Path(os.environ['RECOVERY_COORDINATOR_DDS_EVIDENCE']).write_text(json.dumps(dict(
                scope='Synthetic DDS177; fake producer/consumers, REAL independent pinner/privateexecutor/DeviceInfo/TF/7GIDs; no physical/GPU/payload',
                requests=requests,consumer_resets={k:v['resets'] for k,v in state.items()},
                actual_graph_7_mean_us=sum(graph_times)/len(graph_times),actual_graph_7_p95_us=sorted(graph_times)[47],
                completed=completed,expired=expired,artifact=artifact),indent=2))
    finally:
        if coordinator:
            assert coordinator.close();executor.remove_node(coordinator);coordinator.destroy_node()
        executor.shutdown(timeout_sec=1.)
        for node in [*consumers,probe,producer,source]:node.destroy_node()
        rclpy.shutdown()
