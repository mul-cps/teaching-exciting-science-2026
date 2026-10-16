"""Deterministic async camera identity authority and global RPC bounds."""
from types import SimpleNamespace as NS
import ast
import inspect
import textwrap
import pytest
pytest.importorskip('rclpy')
from so101_station_control.recovery_camera_identity import RecoveryCameraIdentity

class Future:
    def __init__(self): self.complete=False; self.value=None; self.cancelled=False
    def done(self): return self.complete or self.cancelled
    def result(self): return self.value
    def cancel(self): self.cancelled=True

class Client:
    def __init__(self): self.calls=[]; self.removed=[]; self.ready=True
    def service_is_ready(self): return self.ready
    def call_async(self, request):
        future=Future(); self.calls.append(future); return future
    def remove_pending_request(self, future): self.removed.append(future)

class Node:
    def __init__(self):
        self.clients=[]; self.gids=[bytes([1])*16,bytes([2])*16]; self.name='camera'; self.multiple=False
    def create_client(self,*a,**k):
        client=Client(); self.clients.append(client); return client
    def create_subscription(self,*a,**k): return NS()
    def create_timer(self,*a,**k): return NS()
    def get_publishers_info_by_topic(self,topic):
        if self.multiple: return []
        return [NS(node_namespace='/',node_name=self.name,endpoint_gid=self.gids[0 if topic=='depth' else 1])]

class Collector:
    cache_generation='1'*32
    registration={'camera_serial':'123'}
    def __init__(self): self.observations=[]; self.revoked=0
    def observe_camera_identity(self,value): self.observations.append(value)
    def invalidate_camera_identity(self): self.revoked+=1

@pytest.fixture
def rig():
    node=Node(); clock=[100.]; manager=RecoveryCameraIdentity(node,depth_topic='depth',camera_info_topic='info',clock=lambda:clock[0])
    collector=Collector(); manager.set_collector(collector)
    return manager,node,collector,clock

def complete(rig,serial='123',parameter='_123'):
    manager,node,collector,clock=rig
    node.clients[0].calls[-1].value=NS(serial_number=serial)
    node.clients[1].calls[-1].value=NS(values=[NS(type=4,string_value=parameter)])
    for client in node.clients: client.calls[-1].complete=True
    clock[0]+=.01; manager._tick()

def message(stamp=1): return NS(header=NS(stamp=NS(sec=stamp,nanosec=0)))
def info(gid): return NS(publisher_gid=gid)

def test_exact_services_and_matching_identity_admits_cached_gid(rig):
    manager,node,collector,clock=rig; manager._tick(); complete(rig)
    record=collector.observations[-1]
    assert record['request_started_monotonic']==100.
    assert record['confirmed_monotonic']==100.01
    assert record['serial_no_parameter']=='123' and len(record['generation'])==64
    assert manager.accept('depth',message(),info(node.gids[0]))==record['generation']
    assert manager.accept('depth',message(),info(node.gids[0])) is None
    assert manager.snapshot()['valid']

@pytest.mark.parametrize('fault',['serial','parameter','missing','foreign_node','delayed'])
def test_identity_rejects_mismatch_absence_and_late_response(rig,fault):
    manager,node,collector,clock=rig
    if fault=='missing': node.clients[0].ready=False
    if fault=='foreign_node': node.name='other'
    manager._tick()
    if fault in ('missing','foreign_node'):
        assert not node.clients[0].calls
    else:
        if fault=='delayed': clock[0]+=.5
        complete(rig,serial='wrong' if fault=='serial' else '123',parameter='wrong' if fault=='parameter' else '_123')
    assert not collector.observations and not manager.snapshot()['valid']

@pytest.mark.parametrize('fault',['gid','rollback','gap','lease','parameter'])
def test_stream_restart_and_parameter_changes_revoke(rig,fault):
    manager,node,collector,clock=rig; manager._tick(); complete(rig)
    assert manager.accept('depth',message(2),info(node.gids[0]))
    if fault=='gid': node.gids[0]=bytes([3])*16
    if fault=='gap': clock[0]+=.251
    if fault=='lease': clock[0]+=1.01
    if fault=='parameter': manager._parameter_event(NS(node='/camera',changed_parameters=[NS(name='serial_no')],new_parameters=[],deleted_parameters=[]))
    assert manager.accept('depth',message(1 if fault=='rollback' else 3),info(node.gids[0])) is None
    assert not manager.snapshot()['valid']

@pytest.mark.parametrize('replacement',['owner','gid'])
def test_never_done_pair_is_bounded_and_replaced_without_old_authority(rig,replacement):
    manager,node,collector,clock=rig; manager._tick()
    old=[client.calls[0] for client in node.clients]
    for _ in range(10): clock[0]+=.5; manager._tick()
    assert [len(client.calls) for client in node.clients]==[1,1]
    if replacement=='owner':
        new=Collector(); new.cache_generation='2'*32; manager.set_collector(new)
    else:
        node.gids=[bytes([3])*16,bytes([4])*16]
        clock[0]+=.5; manager._tick()
    manager._tick()
    assert all(future.cancelled for future in old)
    assert all(old[i] in node.clients[i].removed for i in range(2))
    assert [len(client.calls) for client in node.clients]==[2,2]
    # Late old replies cannot grant this new context a lease.
    for future in old: future.value=NS(serial_number='123'); future.complete=True
    assert not manager.snapshot()['valid']
    complete((manager,node,manager._collector,clock))
    assert manager.snapshot()['valid']

def test_old_collector_result_cannot_authorize_new_collector(rig):
    manager,node,collector,clock=rig; manager._tick(); other=Collector(); manager.set_collector(other)
    assert manager.accept('depth',message(),info(node.gids[0]),collector=collector) is None
    manager._tick(); complete((manager,node,other,clock))
    assert not collector.observations and len(other.observations)==1

def test_callbacks_and_status_never_query_graph_services_or_files():
    for method in (RecoveryCameraIdentity.accept,RecoveryCameraIdentity.snapshot):
        tree=ast.parse(textwrap.dedent(inspect.getsource(method)))
        names={node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)}
        assert not names & {'get_publishers_info_by_topic','call_async','read_bytes','read_text','sha256'}


def test_native_ros_services_and_dds_gid_binding():
    import os, threading, time
    import rclpy
    from rclpy.node import Node as RosNode
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter
    from realsense2_camera_msgs.srv import DeviceInfo
    from sensor_msgs.msg import Image, CameraInfo
    assert os.environ.get('ROS_DOMAIN_ID')=='75'
    rclpy.init(); camera=RosNode('camera'); consumer=RosNode('identity_consumer')
    camera.declare_parameter('serial_no','_123')
    def device(request,response): response.serial_number='123'; return response
    service=camera.create_service(DeviceInfo,'/camera/device_info',device)
    publishers=[camera.create_publisher(Image,'/identity_depth',qos_profile_sensor_data),
                camera.create_publisher(CameraInfo,'/identity_info',qos_profile_sensor_data)]
    manager=RecoveryCameraIdentity(consumer,depth_topic='/identity_depth',camera_info_topic='/identity_info')
    collector=Collector(); manager.set_collector(collector); accepted=[]
    def depth(message,message_info):
        token=manager.accept('depth',message,message_info)
        if token: accepted.append(token)
    subscription=consumer.create_subscription(Image,'/identity_depth',depth,qos_profile_sensor_data)
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(camera);executor.add_node(consumer)
    thread=threading.Thread(target=executor.spin);thread.start()
    try:
        until=time.monotonic()+3
        while not manager.snapshot()['valid'] and time.monotonic()<until: time.sleep(.005)
        assert manager.snapshot()['valid'],manager.snapshot()
        record=collector.observations[-1]
        image=Image();image.header.stamp=consumer.get_clock().now().to_msg();publishers[0].publish(image)
        until=time.monotonic()+.2
        while not accepted and time.monotonic()<until: time.sleep(.002)
        assert accepted==[record['generation']],manager.snapshot()
        # Actual ROS parameter event revokes cached authority without another image query.
        camera.set_parameters([Parameter('serial_no',value='_wrong')])
        until=time.monotonic()+.2
        while manager.snapshot()['valid'] and time.monotonic()<until: time.sleep(.002)
        assert not manager.snapshot()['valid']
    finally:
        manager.close();executor.shutdown();thread.join(2)
        consumer.destroy_node();camera.destroy_node();rclpy.shutdown()


def test_owner_replacement_during_first_rpc_keeps_pair_tracked(rig):
    manager,node,collector,clock=rig
    original=node.clients[0].call_async
    replacement=Collector();replacement.cache_generation='2'*32
    def interrupted(request):
        future=original(request)
        manager.set_collector(replacement)
        return future
    node.clients[0].call_async=interrupted
    manager._tick()
    assert len(manager._pending['futures'])==2
    old=list(manager._pending['futures'])
    node.clients[0].call_async=original
    manager._tick()
    assert all(future.cancelled for future in old)
    assert manager._pending['collector'] is replacement
    assert len(manager._pending['futures'])==2
    assert [len(client.removed) for client in node.clients]==[1,1]


def test_failed_pending_removal_cannot_expand_rpc_bound(rig):
    manager,node,collector,clock=rig;manager._tick()
    def fail(future):raise RuntimeError('client unavailable')
    node.clients[1].remove_pending_request=fail
    other=Collector();other.cache_generation='2'*32;manager.set_collector(other)
    for _ in range(4):clock[0]+=.5;manager._tick()
    assert [len(client.calls) for client in node.clients]==[1,1]
    assert manager._pending is not None and not manager.snapshot()['valid']


def test_multiple_publishers_never_issue_identity_queries(rig):
    manager,node,collector,clock=rig
    original=node.get_publishers_info_by_topic
    node.get_publishers_info_by_topic=lambda topic: original(topic)*2
    manager._tick()
    assert not node.clients[0].calls and not manager.snapshot()['valid']


def test_missing_message_gid_fails_closed(rig):
    manager,node,collector,clock=rig;manager._tick();complete(rig)
    assert manager.accept('depth',message(),{'source_timestamp':123}) is None
    assert manager.snapshot()['reason']=='ROS message metadata publisher GID unavailable'


def test_partial_rpc_construction_keeps_existing_future_bounded(rig):
    manager,node,collector,clock=rig
    def fail(request):raise RuntimeError('second service call failed')
    node.clients[1].call_async=fail
    manager._tick()
    assert len(manager._pending['futures'])==1
    for _ in range(5):clock[0]+=.5;manager._tick()
    assert len(node.clients[0].calls)==1
    other=Collector();other.cache_generation='2'*32;manager.set_collector(other)
    manager._tick()
    assert node.clients[0].calls[0].cancelled
    assert len(node.clients[0].calls)==2


def test_partial_identity_construction_releases_created_ros_entities():
    node=Node();removed=[]
    original=node.create_client
    def create(*args,**kwargs):
        if node.clients:raise ValueError('second client construction failed')
        return original(*args,**kwargs)
    node.create_client=create;node.destroy_client=removed.append
    with pytest.raises(ValueError,match='second client'):
        RecoveryCameraIdentity(node,depth_topic='depth',camera_info_topic='info')
    assert removed==node.clients and len(removed)==1


def test_native_delayed_old_owner_response_cannot_grant_new_lease():
    import os, threading, time
    import rclpy
    from rclpy.node import Node as RosNode
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from realsense2_camera_msgs.srv import DeviceInfo
    from sensor_msgs.msg import Image, CameraInfo
    assert os.environ.get('ROS_DOMAIN_ID')=='75'
    rclpy.init();camera=RosNode('camera');consumer=RosNode('identity_delayed_consumer')
    camera.declare_parameter('serial_no','_123')
    entered=threading.Event();release=threading.Event();requests=[]
    def device(request,response):
        requests.append(time.monotonic())
        if len(requests)==1:
            entered.set();release.wait(2);response.serial_number='wrong-old-response'
        else:response.serial_number='123'
        return response
    service=camera.create_service(DeviceInfo,'/camera/device_info',device)
    publishers=[camera.create_publisher(Image,'/identity_delayed_depth',qos_profile_sensor_data),
                camera.create_publisher(CameraInfo,'/identity_delayed_info',qos_profile_sensor_data)]
    manager=RecoveryCameraIdentity(consumer,depth_topic='/identity_delayed_depth',camera_info_topic='/identity_delayed_info')
    previous=Collector();manager.set_collector(previous)
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(camera);executor.add_node(consumer)
    thread=threading.Thread(target=executor.spin);thread.start()
    try:
        assert entered.wait(2)
        until=time.monotonic()+1
        while (manager._pending is None or not manager._pending['expired']) and time.monotonic()<until:time.sleep(.005)
        assert manager._pending is not None and manager._pending['expired']
        assert not manager.snapshot()['valid'] and len(manager._pending['futures'])==2
        old=list(manager._pending['futures'])
        replacement=Collector();replacement.cache_generation='2'*32
        replaced=time.monotonic();manager.set_collector(replacement)
        until=time.monotonic()+.2
        while not all(future.cancelled() or future.done() for future in old) and time.monotonic()<until:time.sleep(.002)
        assert old[0].cancelled()
        assert all(future.cancelled() or future.done() for future in old)
        release.set()
        until=time.monotonic()+1
        while not replacement.observations and time.monotonic()<until:time.sleep(.005)
        assert not previous.observations
        assert replacement.observations and manager.snapshot()['valid'],manager.snapshot()
        assert replacement.observations[-1]['request_started_monotonic']>=replaced
        assert len(manager._device._pending_requests)<=1 and len(manager._parameters._pending_requests)<=1
    finally:
        release.set();manager.close();executor.shutdown();thread.join(2)
        consumer.destroy_node();camera.destroy_node();rclpy.shutdown()
