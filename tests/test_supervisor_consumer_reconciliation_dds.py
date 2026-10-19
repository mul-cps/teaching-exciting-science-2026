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


def test_operator_reconciles_actual_restarted_consumer_nodes_with_real_coordinator(tmp_path,monkeypatch):
    heartbeat_gap=.45
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
    journal=tmp_path/'supervisor-journal';journal.mkdir()
    rclpy.init(args=['--ros-args','-p','enabled:=true','-p','artifact_directory:='+str(tmp_path),
                    '-p','expected_source_cohort:='+str(baseline_path),'-p','expected_serial:='+serial,
                    '-p','journal_directory:='+str(journal),'-p','recovery_artifact_directory:='+str(tmp_path)])
    source=Node('camera_'+suffix);producer=Node('so101_rocm_rgbd');probe=Node('recovery_probe_'+suffix)
    coordinator=None;supervisor=None;consumers=[];executor=SingleThreadedExecutor()
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
        availability={'fault':None,'pause_lineage':False}
        requests={'device':0,'reset':0};reset={'id':None,'started':None,'suppressed':0}
        def device(req,response):requests['device']+=1;response.serial_number=serial;return response
        source.create_service(DeviceInfo,'/'+source.get_name()+'/device_info',device)
        def producer_reset(req,response):
            availability['fault']=None;availability['pause_lineage']=False
            requests['reset']+=1;reset['started']=time.monotonic();reset['id']=uuid.uuid4().hex;value['producer_generation']=uuid.uuid4().hex;value['sequence']=0
            response.success=True;response.message=json.dumps(dict(accepted=True,ready=False,
                accepted_reset_id=reset['id'],new_producer_generation=value['producer_generation']))
            return response
        producer.create_service(Trigger,'/so101_rocm_rgbd/reset_source',producer_reset)
        state={}
        def create_consumer(name):
            node=Node(name[1:]);node.declare_parameter('derived_source_cohort',str(baseline_path))
            state[name]={'epoch':uuid.uuid4().hex,'pin':baseline['derived_source']['pin'],'resets':0,'health_calls':0,'image_calls':0}
            def reset_consumer(req,response,node=node,name=name):
                artifact,_=read_artifact(node.get_parameter('derived_source_cohort').value)
                state[name].update(epoch=uuid.uuid4().hex,pin=artifact['derived_source']['pin'])
                state[name]['resets']+=1;response.success=True;response.message='reloaded; not readiness';return response
            def status(req,response,node=node,name=name):
                state[name]['image_calls']+=1
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
            def health(req,response,node=node,name=name):
                from so101_station_perception.source_binding_health import SCHEMA,HEADERS
                state[name]['health_calls']+=1
                pin=state[name]['pin'];ready=pin==statement_pin(value) and value['sequence']>=2
                data=dict(schema=SCHEMA,state='ready' if ready else 'unavailable',binding_invalidated=False,
                    image_admitted=False,processing_progress=False,qualified=False,execution_enabled=False,
                    consumer_epoch=state[name]['epoch'],source_pin_sha256=pin_digest(pin),producer_generation=pin['producer_generation'])
                if ready:
                    source_receipts=value['health']['source_receipts_monotonic']
                    deadlines={**{'source_'+k:v+.25 for k,v in source_receipts.items()},
                        'consumer_lineage':min(source_receipts.values())+.25,
                        'consumer_depth_info':source_receipts['depth_info']+.25,'consumer_color_info':source_receipts['color_info']+.25}
                    headers={k:value['original'][k] for k in HEADERS};identity=value['identity']['expires_monotonic'];receipt=min(deadlines.values())
                    data.update(sequence=value['sequence'],depth_stamp_ns=headers['depth_stamp_ns'],color_stamp_ns=headers['color_stamp_ns'],
                        checked_ros_ns=node.get_clock().now().nanoseconds,checked_monotonic=time.monotonic(),
                        original_header_stamps_ns=headers,original_header_expiry_ros_ns=min(headers.values())+500_000_000,
                        receipt_deadlines_monotonic=deadlines,original_receipt_expiry_monotonic=receipt,
                        original_identity_expiry_monotonic=identity,effective_health_expiry_monotonic=min(receipt,identity),
                        camera_info_validation='independent_native_info',identity_validation='producer_attested_device_info_no_new_rpc',
                        optical_tf_validation='local_exact_depth_timestamp')
                response.success=ready;response.message=json.dumps(data);return response
            node.create_service(Trigger,name+'/derived_source_health',health)
            node.create_service(Trigger,name+'/reset_derived_source',reset_consumer)
            node.create_service(Trigger,name+'/derived_source_status',status)
            return node
        for name in CONSUMERS:consumers.append(create_consumer(name))
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
            if availability['fault'] is None and not availability['pause_lineage']:pubs['status'].publish(String(data=json.dumps(value)))
            # Independent DDS ordering: native/lineage keep advancing while a
            # rebuilding producer has no fresh heartbeat for >250ms.
            if reset['started'] is not None and mono-reset['started'] < heartbeat_gap:
                reset['suppressed']+=1
                return
            pubs['heartbeat'].publish(String(data=json.dumps(dict(schema='so101-derived-rocm-rgbd-v1',qualified=False,
                execution_enabled=False,fault=availability['fault'],state='unavailable' if availability['fault'] else ('ready' if value['sequence']>=2 else 'preparing'),producer_generation=value['producer_generation'],reset=dict(
                accepted_reset_id=reset['id'],new_producer_generation=value['producer_generation'],
                ready=bool(reset['id']) and value['sequence']>=2,state='preparing' if value['sequence']<2 else 'ready')))))
        timer=producer.create_timer(.03,emit)
        coordinator=PerceptionRecovery()
        from so101_station_perception.perception_supervisor import PerceptionSupervisor
        supervisor=PerceptionSupervisor()
        for node in [source,producer,probe,*consumers,coordinator,supervisor]:executor.add_node(node)
        def wait(predicate,seconds=6.):
            until=time.monotonic()+seconds
            while time.monotonic()<until and not predicate():executor.spin_once(timeout_sec=.005)
            assert predicate(),dict(coordinator=coordinator.result,supervisor=supervisor._snapshot() if supervisor is not None else None)
        requester=probe.create_client(Trigger,'/so101_perception_supervisor/reconcile_consumers')
        wait(requester.service_is_ready)
        wait(lambda:supervisor.producer is not None and len(supervisor.ledger.entries)>=2)
        original_epochs={name:state[name]['epoch'] for name in CONSUMERS}
        supervisor.epochs=dict(original_epochs);supervisor._persist()
        assert json.loads(supervisor.journal.path.read_text())['consumer_epochs']==original_epochs
        # Actual same-name consumer ROS node restarts: seven source publishers stay alive.
        for index,name in enumerate(CONSUMERS):
            executor.remove_node(consumers[index]);consumers[index].destroy_node()
            consumers[index]=create_consumer(name);executor.add_node(consumers[index])
        restarted_epochs={name:state[name]['epoch'] for name in CONSUMERS}
        assert all(restarted_epochs[n]!=original_epochs[n] for n in CONSUMERS)
        assert supervisor.policy.state=='disabled' and not supervisor.policy.enabled
        request=requester.call_async(Trigger.Request());wait(request.done)
        assert request.result().success,request.result().message
        accepted=json.loads(request.result().message)
        assert accepted['accepted'] is True and accepted['enabled'] is False and accepted['ready_now'] is False
        wait(lambda:supervisor.adopted is not None or supervisor.policy.latched,seconds=12.)
        assert supervisor.adopted is not None,dict(supervisor=supervisor._snapshot(),coordinator=coordinator.result)
        assert coordinator.result['state']=='completed' and coordinator.result['operation_success'] is True
        assert all(v['image_calls']>=2 for v in state.values())
        assert requests=={'device':2,'reset':1} and all(x['resets']==1 for x in state.values())
        assert supervisor.pin['producer_generation']==value['producer_generation']
        assert len(supervisor.policy.attempts)==1 and not supervisor.policy.handled and supervisor.policy.fault is None
        assert supervisor.policy.state=='disabled' and not supervisor.policy.enabled and supervisor.policy.stable_since is None
        persisted=json.loads(supervisor.journal.path.read_text())
        assert persisted['consumer_epochs']==supervisor.epochs
        assert all(supervisor.epochs[n]!=original_epochs[n] and supervisor.epochs[n]!=restarted_epochs[n] for n in CONSUMERS)
        assert persisted['policy']['inflight'] is None and len(persisted['policy']['attempts'])==1
        assert supervisor.reconciliation_result['state']=='completed' and supervisor.reconciliation_result['reconciliation_id']==accepted['reconciliation_id']
        old_operation=dict(coordinator.result)  # Historical completion, not current permission.
        # Restart ONLY the supervisor against SAME original baseline+journal;
        # exact owned artifact/new epochs/charged budget must restore disabled.
        expected_pin=deepcopy(supervisor.pin);expected_epochs=dict(supervisor.epochs)
        supervisor.close();executor.remove_node(supervisor);supervisor.destroy_node()
        supervisor=PerceptionSupervisor();executor.add_node(supervisor)
        assert supervisor.pin==expected_pin and supervisor.epochs==expected_epochs
        assert len(supervisor.policy.attempts)==1 and not supervisor.policy.enabled and supervisor.policy.state=='disabled'
        wait(lambda:supervisor.producer is not None and len(supervisor.ledger.entries)>=2)
        assert all(pubs[k].get_subscription_count()==0 for k in ('depth','color','aligned','cloud'))
        assert len(supervisor.raw_lineages)<=8 and all(len(q)<=4 for q in supervisor.native_infos.values())
        if os.environ.get('SUPERVISOR_DDS_EVIDENCE'):
            Path(os.environ['SUPERVISOR_DDS_EVIDENCE']).write_text(json.dumps(dict(
                scope='isolatedDDS177; actual consumer node restart + REAL coordinator/pinner/supervisor; fake source/producer; noGPU/SDK/arm/pixels',
                requests=requests,restarted_epochs=restarted_epochs,original_epochs=original_epochs,accepted_reconciliation=accepted,consumer_api_calls={k:dict(health=v['health_calls'],images=v['image_calls']) for k,v in state.items()},consumer_resets={k:v['resets'] for k,v in state.items()},
                completed=coordinator.result,supervisor=supervisor._snapshot(),expired_historical_operation=old_operation),indent=2))
    finally:
        if supervisor and os.environ.get('SUPERVISOR_DDS_EVIDENCE') and not Path(os.environ['SUPERVISOR_DDS_EVIDENCE']).exists():
            Path(os.environ['SUPERVISOR_DDS_EVIDENCE']).write_text(json.dumps(dict(
                scope='failed isolatedDDS177 diagnostic; noGPU/SDK/arm/pixels',requests=requests,
                supervisor=supervisor._snapshot(),coordinator=coordinator.result if coordinator else None),indent=2))
        if supervisor:
            supervisor.close();executor.remove_node(supervisor);supervisor.destroy_node()
        if coordinator:
            assert coordinator.close();executor.remove_node(coordinator);coordinator.destroy_node()
        executor.shutdown(timeout_sec=1.)
        for node in [*consumers,probe,producer,source]:node.destroy_node()
        rclpy.shutdown()
