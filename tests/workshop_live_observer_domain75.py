"""Isolated domain75 integration fixture. Publishes fake observations, no devices/services."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def publisher():
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('workshop_domain75_fake_source')
    pubs = [node.create_publisher(JointState, topic, 1) for topic in ('/leader/joint_states','/follower/joint_states')]
    status_pub = node.create_publisher(String, '/follower/teleop_status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    started=time.monotonic()
    try:
        while time.monotonic()-started < 5:
            elapsed=time.monotonic()-started
            stale=1.8 < elapsed < 3.4
            msg=JointState()
            msg.name=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
            msg.position=[.1,.2,.3,.4,.5]
            stamp=node.get_clock().now().to_msg()
            if stale: stamp.sec-=10
            msg.header.stamp=stamp
            for pub in pubs: pub.publish(msg)
            status=String();status.data=json.dumps({'schema':1,'stamp_s':time.monotonic()-(10 if stale else 0),'state':'active','teleop_mode':'relative','reason':'isolated_fake_source'})
            status_pub.publish(status)
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.destroy_node();rclpy.try_shutdown()


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '75':
        raise SystemExit('This fake publisher is restricted to isolated domain75')
    if '--publisher' in sys.argv:
        publisher();return
    spec=importlib.util.spec_from_file_location('observer',ROOT/'tools/live_observer_ros.py')
    observer=importlib.util.module_from_spec(spec);spec.loader.exec_module(observer)
    child=subprocess.Popen([sys.executable,__file__,'--publisher'])
    try:
        report=observer.collect(5)
        for label in ('leader', 'follower', 'teleop'):
            states=[s[label]['state'] for s in report['samples']]
            assert 'fresh' in states and 'stale' in states, (label, states)
            stale_index=states.index('stale')
            assert 'fresh' in states[stale_index+1:], (label, states)
        for sample in report['samples']:
            if sample['leader']['state']=='stale': assert 'degrees' not in sample['leader']
        print(json.dumps({'domain':75,'fake_ros':True,'source_fresh_stale_reconnected':True,'report':report}))
    finally:
        try: child.wait(timeout=1)
        except subprocess.TimeoutExpired:
            child.terminate()
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: child.kill();child.wait()


if __name__=='__main__':main()
