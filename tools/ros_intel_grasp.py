#!/usr/bin/env python3
"""Request optional Intel grasp proposals from a source-stamped ROS request JSON."""
import argparse
import json
import os
from pathlib import Path
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('request_json', type=Path)
    args = parser.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.msg import MoveItErrorCodes
    from moveit_msgs.srv import GraspPlanning
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    request = GraspPlanning.Request()
    # Never synthesize a fresh stamp for stale sensor data.
    set_message_fields(request, json.loads(args.request_json.read_text()))
    domain = int(os.environ.get('ROS_DOMAIN_ID', '76'))
    if not 1 <= domain <= 232:
        raise ValueError('Intel grasp quick action requires ROS_DOMAIN_ID from 1 through 232')
    os.environ.setdefault('ROS_DOMAIN_ID', '76')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = None
    try:
        node = Node('intel_grasp_quick_action')
        client = node.create_client(GraspPlanning, '/station/intel_grasp/plan')
        if not client.wait_for_service(timeout_sec=1.):
            raise RuntimeError('optional adapter unavailable; launch intel_grasp.launch.py enabled:=true')
        future = client.call_async(request)
        deadline = time.monotonic() + 3.
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('adapter response timed out; do not automatically retry')
            rclpy.spin_once(node, timeout_sec=.02)
        response = future.result()
        print(json.dumps({'qualified': False, 'execution_enabled': False,
                          'pose_semantics': 'upstream_eef_parent',
                          'eef_parent_link_verified': False,
                          'target_association_verified': False,
                          'target_id_semantics': 'request provenance only; backend does not establish track association',
                          'response': message_to_ordereddict(response)}))
        return 0 if response.error_code.val == MoveItErrorCodes.SUCCESS else 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
