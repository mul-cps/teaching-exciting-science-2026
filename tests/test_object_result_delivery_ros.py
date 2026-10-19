"""Actual CPU executor/guard, synthetic metadata, isolated ROS domain only."""
import os,sys,time,threading,copy
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_objects'))

def _require_isolated_domain():
    if os.environ.get('ROS_DOMAIN_ID') != '177' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('object delivery ROS fixture requires explicit DDS177 localhost1')


@pytest.fixture
def node():
    rclpy=pytest.importorskip('rclpy')
    _require_isolated_domain()  # Fail before ROS init/network, never live/default domain.
    from so101_station_objects.objects import ColoredObjectsNode
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    n=ColoredObjectsNode();n.require_registration_tracking=False
    executor=SingleThreadedExecutor();executor.add_node(n)
    try:yield n,executor
    finally:
        executor.remove_node(n);executor.shutdown(timeout_sec=1)
        assert n.destroy_node() is not False
        rclpy.shutdown()


def spin_until(executor,predicate):
    deadline=time.monotonic()+1
    while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
    assert predicate()


def report():
    return {'objects':[],'support_plane':{'normal':[0.,0.,1.],'d':0.,'plane_source':'measured','support_points':20,'support_inliers':20}}


def test_actual_guard_delivers_original_stamp_without_poll_timer(node):
    n,executor=node;n.analysis_timer.cancel()
    seen=[];owner=threading.get_ident()
    n._publish_report=lambda payload,stamp,**kw:seen.append((payload,stamp,threading.get_ident()))
    n.analysis_worker._analyze=lambda job:report()
    stamp=n.get_clock().now().to_msg()
    n.analysis_worker.submit({'stamp':stamp.sec+stamp.nanosec*1e-9,'ros_stamp':stamp})
    spin_until(executor,lambda:bool(seen))
    assert seen[0][1]==stamp and seen[0][2]==owner
    assert seen[0][0]['acquisition']['confirmed_count']==0
    assert n.tracker.acquisition_min_frames==3


def test_guard_stale_result_keeps_original_header_and_is_not_published(node):
    n,executor=node;n.analysis_timer.cancel();seen=[]
    n._publish_report=lambda *a,**k:seen.append(a)
    n.analysis_worker._analyze=lambda job:report()
    stamp=n.get_clock().now().to_msg()
    notified=threading.Event()
    trigger=n.analysis_worker._on_ready
    def notify():trigger();notified.set()
    with n.analysis_worker._notification_gate:
        n.analysis_worker._on_ready=notify
    n.analysis_worker.submit({'stamp':stamp.sec+stamp.nanosec*1e-9-.6,'ros_stamp':stamp})
    assert notified.wait(.5)
    def consumed():
        with n.analysis_worker._condition:return n.analysis_worker._result is None
    spin_until(executor,consumed)
    assert n.analysis_worker._last_start is not None and not seen
    assert n.last_analysis_result_stamp is None


def test_cold_fault_timer_still_invalidates_without_results(node):
    n,executor=node;faults=[]
    class Source:
        fault='original source lease expired'
        def pop_fault(self):value=self.fault;self.fault=None;return value
        def close(self):pass
    n.derived=Source();n._publish_unavailable=lambda stamp,state,error,**kw:faults.append(error)
    n.analysis_worker.take_result=lambda:pytest.fail('healthy cold timer must not poll results')
    spin_until(executor,lambda:bool(faults))
    assert faults==['original source lease expired']


def test_notification_fault_is_observable_and_never_timer_published(node):
    n,executor=node;faults=[];outputs=[]
    def fail():raise RuntimeError('guard trigger unavailable')
    n.analysis_worker._on_ready=fail;n.analysis_worker._analyze=lambda job:report()
    n._publish_unavailable=lambda stamp,state,error,**kw:faults.append(error)
    n._publish_report=lambda *a,**kw:outputs.append(a)
    stamp=n.get_clock().now().to_msg()
    n.analysis_worker.submit({'stamp':stamp.sec+stamp.nanosec*1e-9,'ros_stamp':stamp})
    spin_until(executor,lambda:bool(faults))
    assert 'guard trigger unavailable' in faults[0] and not outputs


def test_failed_cleanup_retains_node_guard_and_context(node):
    n,executor=node
    actual=n.analysis_worker
    assert actual.close(.5)
    class Unretired:
        def close(self):return False
    n.analysis_worker=Unretired()
    try:
        assert n.destroy_node() is False
        assert n.context.ok()
        n.analysis_ready_guard.trigger()  # Still valid; no destroyed guard handle.
        assert n.get_clock().now().nanoseconds>0
    finally:n.analysis_worker=actual


def test_cleared_fault_cannot_resurrect_old_result(node):
    n,executor=node;published=[];unavailable=[]
    class Revoked:
        fault='newest cohort invalidated'
        def pop_fault(self):value=self.fault;self.fault=None;return value
        def check(self,token):raise ValueError('original source receipt expired')
        def close(self):pass
    n.derived=Revoked();n.analysis_worker._analyze=lambda job:report()
    n._publish_report=lambda *a,**kw:published.append(a)
    n._publish_unavailable=lambda stamp,state,error,**kw:unavailable.append(error)
    stamp=n.get_clock().now().to_msg()
    n.analysis_worker.submit({'stamp':stamp.sec+stamp.nanosec*1e-9,'ros_stamp':stamp,'derived_token':object()})
    spin_until(executor,lambda:any('receipt expired' in reason for reason in unavailable))
    assert not published and n.last_analysis_result_stamp is None
    assert n.tracker.acquisition_diagnostics()['confirmed_count']==0
