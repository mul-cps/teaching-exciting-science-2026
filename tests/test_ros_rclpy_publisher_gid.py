"""Actual RMW per-message GIDs, no device topics or hardware (domain75 only)."""
import json
import os
import time
import uuid

import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from std_msgs.msg import String


def _assert_alias_loaded():
    paths=sorted(set(line.split()[-1] for line in open('/proc/self/maps')
                     if 'librmw_cyclonedds_cpp' in line))
    assert len(paths)==1 and paths[0].endswith('/librmw_cyclonedds_cpp_so101_gid.so'), paths
    assert '/ros2/.cache/rmw-cyclonedds-gid-2.2.4/install/' in paths[0]
    print(json.dumps(dict(alias_receiver_library=paths[0])))


@pytest.mark.parametrize('raw',[False,True])
def test_actual_message_gid_matches_endpoint_and_changes_with_publisher(raw):
    if os.environ.get('ROS_DOMAIN_ID')!='75':pytest.skip('requires synthetic ROS domain75')
    rclpy.init()
    _assert_alias_loaded()
    node=Node('rclpy_gid_overlay_'+uuid.uuid4().hex)
    topic='/isolated_rclpy_gid/t_'+uuid.uuid4().hex
    received=[]
    def callback(message,info):received.append(info)
    subscription=None
    gids=[]
    try:
        subscription=node.create_subscription(String,topic,callback,10,raw=raw)
        for generation in range(2):
            publisher=node.create_publisher(String,topic,10)
            deadline=time.monotonic()+3
            accepted=None
            while time.monotonic()<deadline:
                publisher.publish(String(data='synthetic-'+str(generation)))
                rclpy.spin_once(node,timeout_sec=.020)
                endpoints=node.get_publishers_info_by_topic(topic)
                if len(endpoints)!=1:continue
                expected=bytes(endpoints[0].endpoint_gid)
                for info in received:
                    actual=info.get('publisher_gid') if isinstance(info,dict) else getattr(info,'publisher_gid',None)
                    assert actual is not None, 'native take_message binding omitted publisher_gid'
                    actual=bytes(actual)
                    if actual==expected:accepted=actual;break
                if accepted is not None:break
            assert accepted is not None and any(accepted)
            gids.append(accepted)
            print(json.dumps(dict(raw=raw,generation=generation,message_gid=accepted.hex(),
                                  graph_gid=expected.hex(),matched=True)))
            node.destroy_publisher(publisher)
            received.clear()
            until=time.monotonic()+1
            while node.get_publishers_info_by_topic(topic) and time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.010)
        assert gids[0]!=gids[1]
    finally:
        if subscription is not None:node.destroy_subscription(subscription)
        node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('raw',[False,True])
def test_actual_gid_from_separate_publisher_process(raw):
    if os.environ.get('ROS_DOMAIN_ID')!='75':pytest.skip('requires synthetic ROS domain75')
    import subprocess
    import sys
    rclpy.init();_assert_alias_loaded();node=Node('gid_remote_sub_'+uuid.uuid4().hex)
    topic='/isolated_rclpy_gid/t_'+uuid.uuid4().hex
    received=[]
    def callback(message,info):received.append(info)
    subscription=node.create_subscription(String,topic,callback,10,raw=raw)
    observed=[];child=None
    publisher_script="""
import sys,rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init();node=Node(sys.argv[2]);publisher=node.create_publisher(String,sys.argv[1],10)
node.create_timer(.005,lambda:publisher.publish(String(data='separate-process')))
try:rclpy.spin(node)
finally:node.destroy_node();rclpy.shutdown()
"""
    try:
        for generation in range(2):
            name='gid_remote_pub_'+uuid.uuid4().hex
            child_env=dict(os.environ)
            # Publisher uses installed, unmodified Cyclone, like the real C++
            # camera. Only the receiving process needs the native lookup fix.
            child_env.pop('LD_LIBRARY_PATH',None)
            child_env['AMENT_PREFIX_PATH']=sys.prefix
            child_env['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
            child=subprocess.Popen([sys.executable,'-c',publisher_script,topic,name],env=child_env,
                stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
            deadline=time.monotonic()+5;accepted=None
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.020)
                endpoints=[entry for entry in node.get_publishers_info_by_topic(topic) if entry.node_name==name]
                if len(endpoints)!=1:continue
                expected=bytes(endpoints[0].endpoint_gid)
                for info in received:
                    actual=info.get('publisher_gid') if isinstance(info,dict) else getattr(info,'publisher_gid',None)
                    assert actual is not None
                    if bytes(actual)==expected:accepted=bytes(actual);break
                if accepted is not None:break
            assert accepted is not None, 'remote actual message GID did not match its graph endpoint'
            observed.append(accepted)
            print(json.dumps(dict(raw=raw,separate_process=True,generation=generation,
                message_gid=accepted.hex(),graph_gid=expected.hex(),matched=True)))
            child.terminate();child.wait(timeout=3);child=None;received.clear()
            until=time.monotonic()+1
            while node.get_publishers_info_by_topic(topic) and time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.010)
        assert observed[0]!=observed[1]
    finally:
        if child is not None:child.terminate();child.wait(timeout=3)
        node.destroy_subscription(subscription);node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('raw',[False,True])
def test_alias_publisher_to_stock_receiver_interoperability(raw):
    """Stock receiver retains its known metadata bug; verify actual CDR payload."""
    if os.environ.get('ROS_DOMAIN_ID')!='75':pytest.skip('requires synthetic ROS domain75')
    import subprocess
    import sys
    rclpy.init();_assert_alias_loaded()
    node=Node('alias_pub_'+uuid.uuid4().hex)
    topic='/isolated_rclpy_gid/t_'+uuid.uuid4().hex
    script='''
import json,sys,time,rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
rclpy.init();node=Node('stock_sub_'+sys.argv[3]);seen=[]
def callback(message):
    if sys.argv[2]=='1':message=deserialize_message(message,String)
    seen.append(message.data)
sub=node.create_subscription(String,sys.argv[1],callback,10,raw=sys.argv[2]=='1')
end=time.monotonic()+4
while time.monotonic()<end and len(set(seen))<2:rclpy.spin_once(node,timeout_sec=.02)
paths=sorted(set(line.split()[-1] for line in open('/proc/self/maps') if 'librmw_cyclonedds_cpp' in line))
print(json.dumps(dict(seen=sorted(set(seen)),libraries=paths)),flush=True)
node.destroy_node();rclpy.shutdown()
'''
    child_env=dict(os.environ,RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',AMENT_PREFIX_PATH=sys.base_prefix)
    child_env.pop('LD_LIBRARY_PATH',None)
    child=subprocess.Popen([sys.executable,'-c',script,topic,str(int(raw)),uuid.uuid4().hex],
        env=child_env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        for generation in range(2):
            pub=node.create_publisher(String,topic,10)
            end=time.monotonic()+.6
            while time.monotonic()<end:
                pub.publish(String(data=str(generation)));rclpy.spin_once(node,timeout_sec=.005)
            node.destroy_publisher(pub)
        out,err=child.communicate(timeout=5)
        assert child.returncode==0,err
        record=json.loads(out.strip().splitlines()[-1]);print(json.dumps(record))
        assert record['seen']==['0','1']
        assert record['libraries']==[sys.base_prefix+'/lib/librmw_cyclonedds_cpp.so']
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()
