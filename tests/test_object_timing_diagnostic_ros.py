"""Synthetic CPU-only domain177 Nodes/String metadata; no hardware inputs."""
import os
import sys
import uuid
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_objects'),str(ROOT/'ros2/src/so101_station_perception')]


@pytest.fixture
def node():
    if os.environ.get('ROS_DOMAIN_ID')!='177' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('object timing fixture requires domain177 localhost1 BEFORE import/init')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from so101_station_objects.objects import ColoredObjectsNode
    rclpy.init(args=['--ros-args','-p','analysis_timing_diagnostics:=true','-r','__node:=timing_'+uuid.uuid4().hex])
    n=ColoredObjectsNode();n.require_registration_tracking=False
    executor=SingleThreadedExecutor();executor.add_node(n)
    try:yield n,executor
    finally:
        executor.remove_node(n);executor.shutdown(timeout_sec=1)
        assert n.destroy_node() is not False
        rclpy.shutdown()


def test_readonly_parameter_and_real_diagnostic_string(node):
    import json,time
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    n,executor=node
    assert n.timing_recorder is not None
    assert not n.set_parameters([Parameter('analysis_timing_diagnostics',value=False)])[0].successful
    rows=[]
    sub=n.create_subscription(String,'~/analysis_timing',lambda msg:rows.append(json.loads(msg.data)),1)
    row=n.timing_recorder.begin(None);n.timing_recorder.finish(row,outcome=0,site=1,error='expired')
    end=time.monotonic()+2
    while not rows and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    assert rows and rows[-1]['schema']=='so101-object-analysis-timing-v1'
    assert rows[-1]['rows'][0][2]==1 and rows[-1]['qualified'] is False
    n.destroy_subscription(sub)


def test_actual_guard_consumes_worker_envelope_and_keeps_report_bytes(node):
    import time,json
    n,executor=node
    n.analysis_worker._analyze=lambda job:{'objects':[],
        'support_plane':{'normal':[0.,0.,1.],'d':0.,'plane_source':'measured','support_points':20,'support_inliers':20},
        'observation_depth_range_m':[.1,1.2]}
    seen=[];original=n._publish_report
    def observe(report,stamp,**kwargs):
        before=json.dumps(report,sort_keys=True)
        original(report,stamp,**kwargs)
        assert json.dumps(report,sort_keys=True)==before
        seen.append(report)
    n._publish_report=observe
    stamp=n.get_clock().now().to_msg();row=n.timing_recorder.begin(None)
    n.analysis_worker.submit({'stamp':stamp.sec+stamp.nanosec*1e-9,'ros_stamp':stamp,'_analysis_timing':row})
    end=time.monotonic()+1
    while not seen and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    assert seen and row['_finished']
    assert all(row['clocks'][i] is not None for i in (4,5,6,7,8,9,12,13))
    assert n.timing_recorder.counts['observed']==1


def test_actual_analyze_job_counts_both_fallback_invocations_without_report_change(node,monkeypatch):
    from so101_station_objects import objects
    n,_=node;row=n.timing_recorder.begin(None);calls=[]
    def fake(*args,**kwargs):
        calls.append(kwargs)
        if len(calls)==1:raise objects.SupportPlaneUnavailable('synthetic fallback')
        return {'support_plane':{}}
    monkeypatch.setattr(objects,'analyze_rgbd',fake)
    job=dict(rgb=None,depth=None,intrinsics=None,distortion=None,scale=.001,board_from_camera=None,
             stamp=10.,support_plane_cache={'normal':[0,0,1],'d':0.,'stamp':10.},_analysis_timing=row)
    report=n._analyze_job(job)
    assert row['analyze_calls']==2 and len(calls)==2
    assert report=={'support_plane':{'plane_source':'cached_measured_support_plane','support_plane_age_s':0.}}


def test_real_analyzer_optional_metadata_full_report_parity(node):
    import json,numpy as np
    from test_ros2_objects import tabletop_scene
    n,_=node
    rgb,depth,intrinsics,distortion,board=tabletop_scene()
    job=dict(rgb=rgb,depth=depth,intrinsics=intrinsics,distortion=distortion,
             scale=.001,board_from_camera=board,stamp=10.,support_plane_cache=None)
    ordinary=n._analyze_job(job)
    row=n.timing_recorder.begin(None)
    job['_analysis_timing']=row
    observed=n._analyze_job(job)
    convert=lambda value: value.tolist() if isinstance(value,np.ndarray) else value.item() if isinstance(value,np.generic) else (_ for _ in ()).throw(TypeError())
    assert json.dumps(ordinary,sort_keys=True,default=convert,allow_nan=False)==json.dumps(observed,sort_keys=True,default=convert,allow_nan=False)
    assert row['analyze_calls']==1


def _token():
    import time
    from types import SimpleNamespace as NS
    t=time.monotonic()
    return {'statement':{'producer_generation':'a'*32,'sequence':1,
        'original':{k+'_stamp_ns':123 for k in ('depth','color','depth_info','color_info')},
        'health':{'source_receipts_monotonic':{k:t for k in ('depth','color','depth_info','color_info')}},
        'identity':dict(request_started_monotonic=t,confirmed_monotonic=t,expires_monotonic=t+1,confirmed_ros_ns=123)},
        'lineage_receipt':t,'records':{k:(NS(),'gid',t,123) for k in ('color','aligned','color_info','depth_info')}}


class RejectedSource:
    consumer_epoch='b'*32
    def __init__(self,at):self.at=at;self.calls=0
    def check(self,token):
        self.calls+=1
        if self.calls==self.at:raise ValueError('source receipts expired/future')
    def pop_fault(self):return None
    def close(self):pass


@pytest.mark.parametrize('stage',[1,2])
def test_exact_process_entry_and_presubmit_rejection_sites(node,stage):
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import TransformStamped
    from types import SimpleNamespace as NS
    n,_=node;n.timing_timer.cancel();n.derived=RejectedSource(stage)
    stamp=n.get_clock().now().to_msg()
    color=Image();color.header.stamp=stamp;color.header.frame_id='optical'
    color.height=color.width=2;color.encoding='rgb8';color.step=6;color.data=bytes([128]*12)
    depth=Image();depth.header=color.header;depth.height=depth.width=2
    depth.encoding='16UC1';depth.step=4;depth.data=bytes([32,3]*4)
    info=CameraInfo();info.header=color.header;info.height=info.width=2
    info.k=[500.,0.,1.,0.,500.,1.,0.,0.,1.];info.d=[0.]*5
    tf=TransformStamped();tf.transform.rotation.w=1.
    n.tf_buffer=NS(lookup_transform=lambda *a,**k:tf)
    n._process(color,depth,info,_token())
    assert n.derived.calls==stage
    assert len(n.timing_recorder.rows)==1
    assert n.timing_recorder.rows[0][1:3]==[0,stage]
    assert n.analysis_worker.take_result() is None


@pytest.mark.parametrize('stage',[3,4,5])
def test_exact_result_pretracker_and_prepublish_rejection_sites(node,stage):
    import time
    n,executor=node;n.timing_timer.cancel();n.derived=RejectedSource(stage-2)
    n.analysis_worker._analyze=lambda job:{'objects':[],
        'support_plane':{'normal':[0.,0.,1.],'d':0.,'plane_source':'measured','support_points':20,'support_inliers':20},
        'observation_depth_range_m':[.1,1.2]}
    stamp=n.get_clock().now().to_msg();token=_token()
    row=n.timing_recorder.begin(token,consumer_epoch=n.derived.consumer_epoch)
    n.analysis_worker.submit({'stamp':stamp.sec+stamp.nanosec*1e-9,'ros_stamp':stamp,'derived_token':token,'_analysis_timing':row})
    end=time.monotonic()+1
    while not n.timing_recorder.rows and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    assert n.timing_recorder.rows[0][1:3]==[0,stage]
    assert n.derived.calls==stage-2
    assert n.tracker.acquisition_diagnostics()['confirmed_count']==0


def test_default_off_creates_no_diagnostic_ros_entities():
    if os.environ.get('ROS_DOMAIN_ID')!='177' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('domain177 localhost1 required before init')
    import rclpy
    from rclpy.parameter import Parameter
    from so101_station_objects.objects import ColoredObjectsNode
    rclpy.init(args=['--ros-args','-r','__node:=timing_off_'+uuid.uuid4().hex])
    n=None
    try:
        n=ColoredObjectsNode()
        assert n.timing_recorder is None and n.timing_timer is None
        assert n.analysis_worker.diagnostic_losses is None
        assert not hasattr(n,'timing_pub')
        assert not n.set_parameters([Parameter('analysis_timing_diagnostics',value=True)])[0].successful
    finally:
        if n is not None:assert n.destroy_node() is not False
        rclpy.shutdown()


def test_diagnostic_publisher_no_catchup_or_postclose_publish(node,monkeypatch):
    from types import SimpleNamespace as NS
    from so101_station_objects import objects
    n,_=node;n.timing_timer.cancel()
    clock=[100.];observed=[]
    monkeypatch.setattr(objects,'time',NS(monotonic=lambda:clock[0]))  # Module binding ONLY, never global time module.
    n.timing_pub=NS(publish=lambda message:observed.append(message.data))
    n._publish_timing()
    clock[0]=100.49;n._publish_timing()
    assert len(observed)==1
    clock[0]=100.5;n._publish_timing()
    assert len(observed)==2
    n.timing_closing=True;clock[0]=101.;n._publish_timing()
    assert len(observed)==2
