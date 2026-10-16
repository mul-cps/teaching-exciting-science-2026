"""Hardware-free publisher/take cost diagnostic on synthetic ROS domain75."""
import argparse
import json
import os
from pathlib import Path
import statistics
import time
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--messages',type=int,default=2000)
    parser.add_argument('--label',default='unspecified')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='75':raise ValueError('benchmark requires synthetic domain75')
    if not 100<=args.messages<=10000:raise ValueError('bounded100..10000samples required')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init();node=Node('gid_cost_'+uuid.uuid4().hex);topic='/gid_cost/t_'+uuid.uuid4().hex
    received=[]
    def callback(message,info):received.append(info)
    subscriber=node.create_subscription(String,topic,callback,10);publisher=node.create_publisher(String,topic,10)
    samples=[]
    try:
        for index in range(args.messages+100):
            previous=len(received);started=time.perf_counter()
            publisher.publish(String(data='bounded-native-gid-cost'))
            deadline=time.monotonic()+1
            while len(received)==previous and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.010)
            if len(received)==previous:raise RuntimeError('synthetic sample timed out')
            if index>=100:samples.append(time.perf_counter()-started)
        expected=bytes(node.get_publishers_info_by_topic(topic)[0].endpoint_gid)
        matching=sum(bytes(info.get('publisher_gid',b''))==expected for info in received)
        libraries=sorted(set(line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                             if 'librmw_cyclonedds_cpp' in line))
        ordered=sorted(samples)
        print(json.dumps(dict(label=args.label,hardware_access=False,physical_qualification=False,
            samples=len(samples),mean_us=statistics.mean(samples)*1e6,median_us=statistics.median(samples)*1e6,
            p95_us=ordered[int(.95*len(ordered))]*1e6,p99_us=ordered[int(.99*len(ordered))]*1e6,
            max_us=max(samples)*1e6,message_gids_matching_graph=matching,total_received=len(received),
            rclpy_path=rclpy.__file__,loaded_rmw_libraries=libraries,
            scope='same-process publish+executor+take+Pythoncallback; includes DDS scheduling, not isolated native call')))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
