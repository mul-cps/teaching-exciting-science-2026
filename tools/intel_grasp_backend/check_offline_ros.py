#!/usr/bin/env python3
"""Exercise the actual recorded-PCD ROS service in a nonzero isolated domain."""
import argparse
import json
import os
from pathlib import Path
import time

try:
    from .ros_offline_backend import OfflineGraspBackend
except ImportError:
    from ros_offline_backend import OfflineGraspBackend
import rclpy
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GraspPlanning
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from so101_station_scene.intel_grasp_adapter import checked_grasps


def check(build_manifest, pcd, provenance, output):
    domain = int(os.environ.get('ROS_DOMAIN_ID', '177'))
    if not 1 <= domain <= 232 or domain == 76:
        raise ValueError('use an isolated nonzero test domain, not live station domain 76')
    os.environ.setdefault('ROS_DOMAIN_ID', '177')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    executor = SingleThreadedExecutor()
    server = client_node = None
    try:
        server = OfflineGraspBackend(build_manifest, pcd, provenance, output/'attempts')
        client_node = Node('recorded_gpd_verification_client')
        executor.add_node(server); executor.add_node(client_node)
        client = client_node.create_client(GraspPlanning, '/offline/intel_grasp/plan')
        poses, statuses = [], []
        client_node.create_subscription(PoseArray, '/offline/intel_grasp/poses', poses.append, 1)
        client_node.create_subscription(String, '/offline/intel_grasp/status', lambda m: statuses.append(json.loads(m.data)), 10)
        deadline = time.monotonic()+12.
        def spin_until(predicate):
            while not predicate():
                if time.monotonic() >= deadline:
                    raise TimeoutError('actual offline ROS diagnostic did not finish')
                executor.spin_once(timeout_sec=.01)
        spin_until(lambda: client.service_is_ready() and server.poses.get_subscription_count() and server.status.get_subscription_count())
        request = GraspPlanning.Request(group_name='offline_reference')
        request.target.id = server.dataset_id
        future = client.call_async(request)
        spin_until(future.done)
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError('actual offline engine service failed: '+response.error_code.message)
        spin_until(lambda: poses and statuses)
        original_ns = server.source['acquisition_stamp_ns']
        for grasp in response.grasps:
            stamp = grasp.grasp_pose.header.stamp
            assert stamp.sec*1_000_000_000+stamp.nanosec == original_ns
        assert poses[0].header == response.grasps[0].grasp_pose.header
        now = client_node.get_clock().now().nanoseconds*1e-9
        rejected = False
        try:
            checked_grasps(response.grasps, frame=server.source['frame_id'], now=now,
                           max_age=.5, earliest_stamp=original_ns*1e-9, max_candidates=8)
        except ValueError:
            rejected = True
        if not rejected:
            raise RuntimeError('recorded diagnostic unexpectedly passed live freshness gate')
        report = {'schema': 'actual-recorded-gpd-ros-proof-v1', 'domain': domain,
                  'service': '/offline/intel_grasp/plan', 'grasps': len(response.grasps),
                  'recorded_source': True, 'qualified': False, 'execution_enabled': False,
                  'original_stamp_preserved': True, 'live_adapter_rejected_recorded_data': rejected,
                  'source_dataset_id': server.dataset_id, 'status': statuses[-1],
                  'source_age_at_response_s': now-original_ns*1e-9,
                  'response_message': response.error_code.message}
        output.mkdir(parents=True, exist_ok=True)
        (output/'ROS-PROOF.json').write_text(json.dumps(report, indent=2)+'\n')
        return report
    finally:
        for node in (server, client_node):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown(); rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-manifest', type=Path, required=True)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--provenance', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(check(args.build_manifest, args.pcd, args.provenance, args.output), indent=2))
