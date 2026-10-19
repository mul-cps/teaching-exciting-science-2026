"""Bounded ROS-only diagnostic capture; never camera/arm SDK or cloud inputs."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import sys
import time
import uuid
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2/src/so101_station_perception')]


def parameters_names():
    names=['board_frame','depth_scale_m','max_observation_depth_m','max_object_height_m',
           'max_gripper_opening_m','observation_roi_enabled','acquisition_min_frames','max_processing_hz']
    return names+[f'{prefix}_{axis}_{edge}' for prefix in ('world_roi','pickup_roi','observation_roi')
                  for axis in 'xyz' for edge in ('min','max')]


def save_capture(output,rgb,depth,metadata):
    """Exclusive files; completion JSON last, all data read back and hashed."""
    output=Path(output);output.mkdir(exist_ok=False,parents=True)
    with (output/'rgbd.npz').open('xb') as stream:np.savez(stream,rgb=rgb,depth=depth)
    with np.load(output/'rgbd.npz',allow_pickle=False) as saved:
        assert np.array_equal(saved['rgb'],rgb) and np.array_equal(saved['depth'],depth)
    metadata=dict(metadata,qualified=False,execution_enabled=False,diagnostic_only=True)
    metadata['rgbd_sha256']=hashlib.sha256((output/'rgbd.npz').read_bytes()).hexdigest()
    raw=json.dumps(metadata,sort_keys=True,allow_nan=False).encode()
    with (output/'capture.json').open('xb') as stream:stream.write(raw)
    assert (output/'capture.json').read_bytes()==raw
    return dict(capture_sha256=hashlib.sha256(raw).hexdigest(),rgbd_sha256=metadata['rgbd_sha256'])


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True);p.add_argument('--output',required=True)
    p.add_argument('--objects-node',default='/so101_colored_objects');p.add_argument('--timeout',type=float,default=10.)
    args=p.parse_args(argv)
    if not 0<args.timeout<=10:raise ValueError('timeout must be in(0,10]')
    if Path(args.output).exists():raise FileExistsError(args.output)
    import os,rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterEvent
    from sensor_msgs.msg import CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener,TransformException
    from rosidl_runtime_py.convert import message_to_ordereddict
    from so101_station_perception.derived_aligned import DerivedAligned,received_gid
    from so101_station_perception.derived_lineage import configure_ros_domain,ALIGNED_TOPIC,TOPICS,canonical_gid,require
    from so101_station_perception.rocm_rgbd import extrinsic_matrix
    configure_ros_domain(os.environ)
    with Path(args.source).open('rb') as stream:source=stream.read(131073)
    source_sha=hashlib.sha256(source).hexdigest()
    if len(source)>131072:raise ValueError('source artifact exceeds128KiB')
    rclpy.init();node=None;executor=None;adapter=None
    try:
        node=Node('object_diagnostic_'+uuid.uuid4().hex[:10],use_global_arguments=False)
        for k,v in dict(depth_topic=ALIGNED_TOPIC,color_topic=TOPICS['color'],camera_info_topic=TOPICS['color_info'],derived_source_cohort=args.source).items():node.declare_parameter(k,v)
        executor=SingleThreadedExecutor();executor.add_node(node);buffer=Buffer();listener=TransformListener(buffer,node)
        infos=deque(maxlen=4);candidate=[None];error=['waiting for exact fresh cohort'];params=[None]
        def pair(rgb,depth,info,token):candidate[0]=token
        adapter=DerivedAligned(node,buffer,pair)
        require(adapter.core.ledger.pin==json.loads(source)['derived_source']['pin'],'source artifact changed during initialization')
        def aligned_info(message,info):
            infos.append((message,received_gid(info),time.monotonic()))
        subscription=node.create_subscription(CameraInfo,'/so101_perception/rocm/aligned_camera_info',aligned_info,qos_profile_sensor_data)
        parameter_changed=[False]
        def changed(event):
            if event.node==args.objects_node and (event.changed_parameters or event.new_parameters or event.deleted_parameters):parameter_changed[0]=True
        parameter_subscription=node.create_subscription(ParameterEvent,'/parameter_events',changed,10)
        client=node.create_client(GetParameters,args.objects_node+'/get_parameters');future=None;deadline=time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=min(.005,max(0.,deadline-time.monotonic())))
            if future is None and client.service_is_ready():future=client.call_async(GetParameters.Request(names=parameters_names()))
            if future is not None and future.done() and params[0] is None:
                values=future.result().values;require(len(values)==len(parameters_names()),'parameter snapshot incomplete')
                params[0]={k:parameter_value_to_python(v) for k,v in zip(parameters_names(),values)}
                require(all(v is not None for v in params[0].values()),'unset analysis parameter')
            token=candidate[0]
            if token is None or params[0] is None:continue
            try:
                require(not parameter_changed[0],'object parameters changed during capture')
                adapter.check(token);value=token['statement'];stamp=value['original']['depth_stamp_ns']
                info_record=next((r for r in reversed(infos) if r[0].header.stamp.sec*10**9+r[0].header.stamp.nanosec==stamp),None)
                require(info_record is not None,'matching aligned CameraInfo unavailable')
                ai,gid,receipt=info_record;require(0<=time.monotonic()-receipt<=.250,'alignedInfo receipt expired/future')
                endpoints=node.get_publishers_info_by_topic('/so101_perception/rocm/aligned_camera_info')
                require(len(endpoints)==1 and canonical_gid(endpoints[0].endpoint_gid)==canonical_gid(gid),'alignedInfo actual generation differs')
                producer=node.get_publishers_info_by_topic(ALIGNED_TOPIC)[0]
                require((producer.node_namespace,producer.node_name)==(endpoints[0].node_namespace,endpoints[0].node_name),'alignedInfo publisher is not aligned producer')
                ci=token['color_info'];require(ai.header.frame_id==ci.header.frame_id and ai.width==ci.width and ai.height==ci.height and list(ai.k)==list(ci.k) and list(ai.d)==list(ci.d),'alignedInfo/native color calibration differs')
                board=params[0]['board_frame'];rgb_stamp=value['original']['color_stamp_ns']
                tf=buffer.lookup_transform(ci.header.frame_id,board,Time(nanoseconds=rgb_stamp))
                camera_from_board=extrinsic_matrix(tf)
                rgb=np.frombuffer(token['color'].data,np.uint8).reshape(ci.height,ci.width,3).copy()
                depth=np.frombuffer(token['aligned'].data,'<u2').reshape(ci.height,ci.width).copy()
                metadata=dict(schema='object-rgbd-diagnostic-v1',original=value['original'],lineage=value,parameters=params[0],
                    source_artifact=json.loads(source),source_sha256=source_sha,source_path=str(Path(args.source).resolve()),
                    native_color_info=dict(message_to_ordereddict(ci)),native_depth_info=dict(message_to_ordereddict(token['depth_info'])),
                    aligned_info=dict(message_to_ordereddict(ai)),camera_from_board=camera_from_board.tolist(),tf_stamp_ns=rgb_stamp,
                    selected_receipts={k:r[2] for k,r in token['records'].items()},aligned_info_receipt_monotonic=receipt,
                    capture_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    component_source_sha256={name:hashlib.sha256((ROOT/'ros2/src/so101_station_objects/so101_station_objects'/name).read_bytes()).hexdigest() for name in ('objects.py','colored_object_validation.py')},
                    optional_live_reports='not subscribed; offline replay does not attest live track IDs or cached support plane')
                adapter.check(token)
                require(not parameter_changed[0],'object parameters changed during capture')
                current=node.get_publishers_info_by_topic('/so101_perception/rocm/aligned_camera_info')
                require(len(current)==1 and canonical_gid(current[0].endpoint_gid)==canonical_gid(gid),'alignedInfo generation changed during materialization')
                require(0<=time.monotonic()-receipt<=.250 and time.monotonic()<=deadline,'capture final receipt/deadline expired')
                metadata['validated_monotonic']=time.monotonic();metadata['validated_ros_ns']=node.get_clock().now().nanoseconds
                print(json.dumps(save_capture(args.output,rgb,depth,metadata)));return
            except (ValueError,IndexError,TransformException) as exc:error[0]=str(exc)
        raise TimeoutError('diagnostic capture unavailable: '+error[0])
    finally:
        if adapter is not None:adapter.close()
        if executor is not None:executor.shutdown(timeout_sec=0.)
        if node is not None:node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
