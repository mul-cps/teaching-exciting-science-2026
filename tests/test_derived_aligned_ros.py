"""Native ROS API fakes. No node startup, images, SDK, GPU or hardware."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from copy import deepcopy
import pytest
pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_perception'),str(ROOT/'tests')]
from test_aligned_cohort import messages as fake_messages
from builtin_interfaces.msg import Time as Stamp

def messages(value):
    result=fake_messages(value)
    for message in result.values():
        stamp=message.header.stamp;message.header.stamp=Stamp(sec=stamp.sec,nanosec=stamp.nanosec)
    return result
from test_derived_registration import fixture,endpoints
from so101_station_perception.derived_lineage import validate_statement,TOPICS,ALIGNED_TOPIC
from so101_station_perception.derived_aligned import DerivedAligned


def setup(tmp_path,monkeypatch,period=0.):
    import so101_station_perception.derived_aligned as module
    clock=[100.12,10_120_000_000];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    value=fixture();pin=validate_statement(value,now_ns=clock[1],now_mono=clock[0],receipt_mono=clock[0])
    path=tmp_path/'pins.json';path.write_text(json.dumps({'derived_source':{'schema':value['schema'],'qualified':False,'pin':pin}}))
    params={'depth_topic':ALIGNED_TOPIC,'color_topic':TOPICS['color'],'camera_info_topic':TOPICS['color_info'],'derived_source_cohort':str(path)}
    subscriptions={};timers=[];calls=[];graph=endpoints(value)
    def subscribe(typ,topic,callback,*args,**kwargs):subscriptions[topic]=callback;return callback
    node=NS(get_parameter=lambda k:NS(value=params[k]),create_subscription=subscribe,
        create_service=lambda *args:None,create_timer=lambda period,callback:timers.append(callback) or NS(cancel=lambda:None),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock[1])),get_publishers_info_by_topic=graph.__getitem__)
    xyz=value['calibration']['translation_m'];tf=NS(transform=NS(translation=NS(x=xyz[0],y=xyz[1],z=xyz[2]),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    buffer=NS(lookup_transform=lambda *args:tf)
    adapter=DerivedAligned(node,buffer,lambda *args:calls.append(args),period_s=period)
    return adapter,value,clock,subscriptions,timers,calls,path


def send(adapter,value):
    for kind,message in messages(value).items():
        gid=value['outputs']['aligned_publisher_gid'] if kind=='aligned' else value['source'][kind+'_publisher_gid']
        adapter._intake(kind,message,bytes.fromhex(gid))
    adapter._intake('lineage',NS(data=json.dumps(value)),bytes.fromhex(value['outputs']['status_publisher_gid']))


def next_cohort(value,delta):
    value=deepcopy(value);value['sequence']+=delta
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][key]+=delta*1_000_000
    for key in ('aligned_stamp_ns','cloud_stamp_ns'):value['outputs'][key]+=delta*1_000_000
    value['health']['observation_ros_ns']+=delta*1_000_000
    return value


def test_native_callbacks_are_messageinfo_aware_and_token_rechecked(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch)
    assert len(subs)==5 and not timers
    # The subscriptions accept MessageInfo, retaining GID rather than trusting a node name.
    for kind,message in messages(v).items():
        topic=ALIGNED_TOPIC if kind=='aligned' else TOPICS[kind]
        gid=v['outputs']['aligned_publisher_gid'] if kind=='aligned' else v['source'][kind+'_publisher_gid']
        subs[topic](message,{'publisher_gid':bytes.fromhex(gid)})
    a._lineage(NS(data=json.dumps(v)),NS(publisher_gid=bytes.fromhex('6'*32)))
    assert len(calls)==1 and a.check(calls[0][3])
    clock[:]=[100.4,10_400_000_000]
    with pytest.raises(ValueError,match='expired'):a.check(calls[0][3])


def test_objects_drain_chooses_latest_before_any_image_decode(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch,.2)
    send(a,v);send(a,next_cohort(v,1));assert not calls
    timers[0]();assert calls[0][3]['statement']['sequence']==v['sequence']+1
    send(a,next_cohort(v,2));timers[0]();assert len(calls)==1
    a.close();assert not any(a.core.streams.values())


def test_reset_cannot_resurrect_old_inflight_token_and_failed_reload_stays_closed(tmp_path,monkeypatch):
    a,v,clock,subs,timers,calls,path=setup(tmp_path,monkeypatch);send(a,v);token=calls[0][3]
    response=a._reset(None,NS());assert response.success
    with pytest.raises(ValueError,match='reset'):a.check(token)
    path.write_text('{}');response=a._reset(None,NS());assert not response.success
    send(a,v);assert a.pop_fault() and len(calls)==1


def test_expired_board_admission_does_not_renew_pose_or_observation(monkeypatch):
    import numpy as np
    import so101_station_perception.charuco_world as module
    monkeypatch.setattr(module,'estimate_board_pose',lambda *args,**kwargs:(np.eye(4),6,0.,np.zeros((6,3)),np.zeros((6,2))))
    monkeypatch.setattr(module,'camera_matrices',lambda *args:(np.eye(3),np.zeros(5)))
    stamp=Stamp(sec=10,nanosec=0);message=NS(header=NS(stamp=stamp,frame_id='color'))
    old=(np.eye(4),9.9);statuses=[]
    node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),max_age=.5,optical_frame='color',
        get_parameter=lambda key:NS(value=.04),last_stamp=None,last_process_mono=0.,period=.2,
        board=None,detector=None,min_corners=6,max_error=2.,require_depth_plane=False,
        cached_optical_from_link=np.eye(4),filtered_pose=old[0],last_valid_stamp=old[1],pose_estimate=old,
        _status=lambda *args,**kwargs:statuses.append((args,kwargs)))
    def expired():raise ValueError('source lease expired after CV')
    module.CharucoWorld._pair(node,message,message,prepared_gray=np.zeros((2,2),np.uint8),admission_check=expired)
    assert node.pose_estimate is old and node.last_valid_stamp==9.9
    assert statuses[-1][0]==('occluded',)


def test_expired_tag_admission_only_clears_markers():
    import numpy as np
    from so101_station_perception.tag_observations import TagObservations
    stamp=Stamp(sec=10,nanosec=0);msg=NS(header=NS(stamp=stamp,frame_id='color'),encoding='16UC1')
    info=NS(header=msg.header,width=2,height=2,k=[1.,0.,1.,0.,1.,1.,0.,0.,1.],d=[0.]*5)
    published=[];cleared=[]
    node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),last_processing=0.,min_processing_period=.2,
        max_pair_delta=.04,camera_info=info,camera_info_max_age=.5,depth_scale=.001,
        bridge=NS(imgmsg_to_cv2=lambda *args,**kwargs:np.ones((2,2),np.uint16)),detector=NS(detect=lambda image:[]),
        diagnostic_frame='color',publisher=NS(publish=published.append),_empty=lambda *args:cleared.append(args),
        get_logger=lambda:NS(warning=lambda *args:None))
    def expired():raise ValueError('source lease expired after CV')
    TagObservations._images(node,msg,msg,prepared_gray=np.zeros((2,2),np.uint8),admission_check=expired)
    assert not published and len(cleared)==1


@pytest.mark.parametrize('notification_error',[None,'guard delivery failed'])
def test_expired_object_result_does_not_publish_or_acquire(notification_error):
    import ast
    from types import MethodType
    path=ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py'
    tree=ast.parse(path.read_text())
    callbacks={item.name:item for item in ast.walk(tree) if isinstance(item,ast.FunctionDef)
        and item.name in ('_consume_analysis','_consume_analysis_faults')}
    # Exercise the actual result guard callback and its shared fault gate. The
    # independent50ms timer must retain the same real fault method as well.
    calls=[item for item in ast.walk(tree) if isinstance(item,ast.Call) and isinstance(item.func,ast.Attribute)]
    assert any(call.func.attr=='create_guard_condition' and call.args and
        isinstance(call.args[0],ast.Attribute) and call.args[0].attr=='_consume_analysis' for call in calls)
    assert any(call.func.attr=='create_timer' and len(call.args)>=2 and
        isinstance(call.args[0],ast.Constant) and call.args[0].value==.05 and
        isinstance(call.args[1],ast.Attribute) and call.args[1].attr=='_consume_analysis_faults' for call in calls)
    namespace={}
    exec(compile(ast.fix_missing_locations(ast.Module(body=list(callbacks.values()),type_ignores=[])),str(path),'exec'),namespace)
    updates=[];publications=[];order=[]
    def expired(token):
        order.append('admission');raise ValueError('source lease expired during analysis')
    def pop_fault():order.append('fault_gate');return None
    job={'stamp':10.,'ros_stamp':Stamp(sec=10,nanosec=0),'derived_token':object()}
    def take_result():order.append('result');return job,{},None
    node=NS(derived=NS(pop_fault=pop_fault,check=expired),
        analysis_worker=NS(take_result=take_result,notification_error=notification_error),last_analysis_notification_error=None,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000,to_msg=lambda:Stamp(sec=10,nanosec=100_000_000))),
        last_analysis_result_stamp=None,require_registration_tracking=False,
        tracker=NS(update=lambda *args:updates.append(args)),_publish_unavailable=lambda *args:publications.append(args))
    node._consume_analysis_faults=MethodType(namespace['_consume_analysis_faults'],node)
    namespace['_consume_analysis'](node)
    assert not updates and node.last_analysis_result_stamp is None
    assert len(publications)==1 and publications[0][1]=='unavailable'
    if notification_error is None:
        assert order==['fault_gate','result','admission']
        assert publications[0][2]=='source lease expired during analysis'
        assert node.last_analysis_notification_error is None
    else:
        assert order==['fault_gate','result']  # Failed wakeup discards; no admission/timer publication fallback.
        assert publications[0][2]=='analysis result notification failed: '+notification_error
        assert node.last_analysis_notification_error==notification_error


def test_expired_during_object_message_materialization_publishes_no_observations():
    import ast
    from geometry_msgs.msg import PoseArray
    from vision_msgs.msg import Detection3DArray
    from visualization_msgs.msg import Marker,MarkerArray
    from std_msgs.msg import String
    path=ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py'
    fn=next(item for item in ast.walk(ast.parse(path.read_text())) if isinstance(item,ast.FunctionDef) and item.name=='_publish_report')
    namespace={'PoseArray':PoseArray,'Detection3DArray':Detection3DArray,'Marker':Marker,'MarkerArray':MarkerArray,'String':String,'json':json}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(path),'exec'),namespace)
    published=[];faults=[]
    def expired(token):raise ValueError('lease expired while materializing output')
    publisher=NS(publish=published.append)
    node=NS(board_frame='charuco_board',derived=NS(check=expired),detections_pub=publisher,markers_pub=publisher,
        grasps_pub=publisher,contacts_pub=publisher,status_pub=publisher,_publish_unavailable=lambda *args:faults.append(args))
    report={'objects':[],'stamp':10.,'support_plane':{},'observation_depth_range_m':[.1,1.2]}
    namespace['_publish_report'](node,report,Stamp(sec=10,nanosec=0),derived_token=object())
    assert not published and len(faults)==1
