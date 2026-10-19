import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS integration tests require the RoboStack environment")

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "ros2/src/so101_station_perception"))


def test_shared_frame_dispatch_decodes_once_and_preserves_stamp_and_gray_identity():
    from so101_station_perception.marker_pipeline import dispatch_shared_gray

    gray = np.zeros((480, 640), dtype=np.uint8)

    class Bridge:
        calls = 0

        def imgmsg_to_cv2(self, message, desired_encoding):
            assert desired_encoding == "mono8"
            self.calls += 1
            return gray

    class Stamp:
        sec = 123
        nanosec = 456

    class Message:
        header = type("Header", (), {"stamp": Stamp(), "frame_id": "camera_color_optical_frame"})()

    bridge = Bridge()
    color, depth, info = Message(), Message(), Message()
    calls = []
    metrics = dispatch_shared_gray(
        color, depth, info, bridge,
        lambda image, camera_info, prepared_gray: calls.append(("board", image, camera_info, prepared_gray)),
        lambda image, aligned_depth, prepared_gray: calls.append(("tags", image, aligned_depth, prepared_gray)),
    )
    assert bridge.calls == 1
    assert calls[0] == ("board", color, info, gray)
    assert calls[1] == ("tags", color, depth, gray)
    assert metrics["capture_stamp"] == pytest.approx(123.000000456)
    assert metrics["decode_ms"] >= 0 and metrics["board_ms"] >= 0 and metrics["tags_ms"] >= 0


def test_combined_frame_does_not_hit_child_standalone_rate_gate():
    import time
    from types import SimpleNamespace
    from so101_station_perception.tag_observations import TagObservations

    stamp = SimpleNamespace(sec=100, nanosec=0)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id="optical"))
    cleared = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        last_processing=time.monotonic(), min_processing_period=10.,
        max_pair_delta=.04, camera_info=None,
        _empty=lambda *args: cleared.append(args))
    # A standalone callback is throttled. A frame already scheduled by the
    # combined pipeline must reach validation despite that child timer.
    TagObservations._images(node, message, message)
    assert cleared == []
    TagObservations._images(node, message, message, prepared_gray=np.zeros((2, 2), dtype=np.uint8))
    assert len(cleared) == 1


def test_drain_rechecks_queued_age_and_never_restamps_or_reuses_exposure():
    from types import SimpleNamespace as NS
    from so101_station_perception.latest_cohort import LatestCohort
    from so101_station_perception.marker_pipeline import MarkerPipeline
    def message(ns):
        return NS(header=NS(stamp=NS(sec=ns//10**9, nanosec=ns%10**9),frame_id='optical'))
    cleared=[];statuses=[];processed=[]
    clock=[99_100_000_000]
    node=NS(scheduler=LatestCohort(5.),max_age=.5,max_pair_delta=.04,optical_frame='optical',
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock[0])),
        tags=NS(_empty=lambda stamp,frame:cleared.append(stamp)),
        _status=lambda state,**kw:statuses.append((state,kw)),
        _process_images=lambda *args:processed.append(args))
    stale=message(99_000_000_000);fresh=message(99_900_000_000)
    MarkerPipeline._images(node,stale,stale,stale);clock[0]=100_000_000_000;MarkerPipeline._drain(node)
    assert not processed and cleared==[stale.header.stamp]
    assert statuses[0][1]['stamp']==99.
    assert node.scheduler.metrics()['stale_dropped']==1
    # A future depth timestamp also rejects an otherwise current RGB cohort.
    node.scheduler=LatestCohort(5.)
    MarkerPipeline._images(node,fresh,message(100_001_000_000),fresh);MarkerPipeline._drain(node)
    assert not processed
    node.scheduler=LatestCohort(5.)
    MarkerPipeline._images(node,fresh,fresh,fresh);MarkerPipeline._drain(node)
    assert processed[0][:3]==(fresh,fresh,fresh)
    assert fresh.header.stamp.sec==99 and fresh.header.stamp.nanosec==900_000_000
    assert not node.scheduler.offer((fresh,fresh,fresh),(99_900_000_000,99_900_000_000),0.)


def test_native_executor_ingress_progresses_during_compute_and_has_one_pending(monkeypatch):
    """Actual message_filters/subscriptions/executor; no camera or hardware IO."""
    import threading
    import time
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from so101_station_perception.marker_pipeline import MarkerPipeline
    import uuid
    from rclpy.node import Node
    suffix=uuid.uuid4().hex
    prefix='/fixture_'+suffix
    original_init=Node.__init__
    def unique_init(self,node_name,*args,**kwargs):
        original_init(self,node_name+'_'+suffix,*args,**kwargs)
    monkeypatch.setattr(Node,'__init__',unique_init)
    original_publisher=Node.create_publisher
    def unique_publisher(self,kind,topic,*args,**kwargs):
        topic=topic if topic.startswith(prefix) else prefix+'/'+topic.lstrip('/')
        return original_publisher(self,kind,topic,*args,**kwargs)
    monkeypatch.setattr(Node,'create_publisher',unique_publisher)
    rclpy.init(args=['--ros-args','-p','color_topic:='+prefix+'/color',
                    '-p','depth_topic:='+prefix+'/depth','-p','camera_info_topic:='+prefix+'/info'])
    pipeline=None;publisher=None;executor=None;thread=None
    try:
        pipeline=MarkerPipeline();publisher=rclpy.create_node('scheduler_fixture_publisher')
        pubs=[publisher.create_publisher(kind,topic,qos_profile_sensor_data)
              for kind,topic in ((Image,prefix+'/color'),(Image,prefix+'/depth'),(CameraInfo,prefix+'/info'))]
        entered=threading.Event();finished=threading.Event();records=[];counts=[];active=[0,0]
        def compute(color,depth,info,wait):
            active[0]+=1;active[1]=max(active)
            records.append((color.header.stamp.sec*10**9+color.header.stamp.nanosec,time.monotonic()))
            before=pipeline.scheduler.metrics()['offered'];entered.set()
            time.sleep(.12)  # release GIL just as native OpenCV does
            counts.append((before,pipeline.scheduler.metrics()['offered']))
            active[0]-=1;finished.set()
        pipeline._process_images=compute
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(pipeline);executor.add_node(pipeline.tags)
        executor.add_node(publisher);thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        time.sleep(.15)
        sent=set();until=time.monotonic()+.9
        while time.monotonic()<until:
            stamp=publisher.get_clock().now().to_msg();sent.add(stamp.sec*10**9+stamp.nanosec)
            for pub,kind in zip(pubs,(Image,Image,CameraInfo)):
                msg=kind();msg.header.stamp=stamp;msg.header.frame_id='camera_color_optical_frame';pub.publish(msg)
            time.sleep(.02)
        assert entered.wait(.5) and finished.wait(.5)
        assert any(after>before for before,after in counts),counts
        assert active[1]==1
        assert len(records)>=3 and all(stamp in sent for stamp,_ in records)
        assert all(b[1]-a[1]>=.199 for a,b in zip(records,records[1:]))
        assert pipeline.scheduler.metrics()['overwritten']>0
    finally:
        if pipeline is not None:pipeline.close_scheduler()
        if executor is not None:
            from so101_station_perception.marker_pipeline import shutdown_executor_bounded
            assert shutdown_executor_bounded(executor)
        if thread is not None:thread.join(timeout=2.);assert not thread.is_alive()
        if pipeline is not None:pipeline.tags.destroy_node();pipeline.destroy_node()
        if publisher is not None:publisher.destroy_node()
        rclpy.shutdown()


def test_future_input_does_not_poison_current_exposure_high_water():
    from types import SimpleNamespace as NS
    from so101_station_perception.latest_cohort import LatestCohort
    from so101_station_perception.marker_pipeline import MarkerPipeline
    def message(sec):return NS(header=NS(stamp=NS(sec=sec,nanosec=0),frame_id='optical'))
    clock=[99_100_000_000]
    node=NS(scheduler=LatestCohort(5.),max_age=.5,max_pair_delta=.04,optical_frame='optical',
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)))
    bad=message(3700);good=message(100)
    MarkerPipeline._images(node,bad,bad,bad)
    MarkerPipeline._images(node,good,good,good)
    assert node.scheduler.take(1.)[0]==(good,good,good)


def test_shutdown_wait_is_bounded_and_refuses_unsafe_resource_destruction():
    from types import SimpleNamespace as NS
    from so101_station_perception import marker_pipeline as module
    assert hasattr(module,'shutdown_executor_bounded')
    calls=[]
    class Worker:
        def __init__(self,alive):self.alive=alive
        def join(self,timeout):assert 0<=timeout<=.1;calls.append('join')
        def is_alive(self):return self.alive
    executor=NS(_executor=NS(_threads=[Worker(False),Worker(False)]),
        shutdown=lambda **kw:calls.append(kw))
    assert module.shutdown_executor_bounded(executor,.1)
    assert calls[0]=={'timeout_sec':0.,'wait_for_threads':False}
    executor._executor._threads=[Worker(True)]
    assert not module.shutdown_executor_bounded(executor,.1)
    del executor._executor
    assert not module.shutdown_executor_bounded(executor,.1)


def test_shutdown_multiple_workers_share_one_deadline(monkeypatch):
    from types import SimpleNamespace as NS
    from so101_station_perception import marker_pipeline as module
    times=iter((10.,10.01,10.09));waits=[]
    monkeypatch.setattr(module.time,'monotonic',lambda:next(times))
    worker=NS(join=lambda timeout:waits.append(timeout),is_alive=lambda:False)
    executor=NS(_executor=NS(_threads=[worker,worker]),shutdown=lambda **kw:None)
    assert module.shutdown_executor_bounded(executor,.1)
    assert waits==pytest.approx([.09,.01])
