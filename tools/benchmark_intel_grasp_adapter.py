#!/usr/bin/env python3
"""Bounded full-message bridge CPU benchmark; synthetic messages only."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import statistics
import time


def main():
    path = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_scene/so101_station_scene/intel_grasp_adapter.py'
    spec = importlib.util.spec_from_file_location('intel_grasp_bench', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from geometry_msgs.msg import PoseArray
    from moveit_msgs.msg import Grasp
    from moveit_msgs.srv import GraspPlanning
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectoryPoint
    results = []
    for mode in ('typical', 'maximum'):
        value = Grasp()
        value.grasp_pose.header.frame_id = 'base_link'
        value.grasp_pose.header.stamp.sec = 100
        value.grasp_pose.pose.orientation.w = 1.
        value.pre_grasp_approach.direction.header.frame_id = 'base_link'
        value.pre_grasp_approach.direction.vector.z = -1.
        value.pre_grasp_approach.desired_distance = .05
        if mode == 'maximum':
            value.id = '\U0001f642'*128
            value.allowed_touch_objects = ['\U0001f642'*128]*8
            for posture in (value.pre_grasp_posture, value.grasp_posture):
                posture.header.frame_id = '\U0001f642'*128
                posture.joint_names = ['\U0001f642'*128]*8
                posture.points = [JointTrajectoryPoint(positions=[.1]*8, velocities=[.1]*8,
                                                       accelerations=[.1]*8, effort=[.1]*8)
                                  for _ in range(4)]
        batch_size = 32 if mode == 'maximum' else 8
        batch = [value]*batch_size
        samples = []
        payload = {'state': 'proposals', 'detail': ('\U0001f642'*512 if mode == 'maximum' else ''), 'qualified': False,
                   'execution_enabled': False, 'ik_validated': False,
                   'collision_checked': False, 'tool_mapping_qualified': False,
                   'backend': 'intel/ros2_grasp_library',
                   'inspected_upstream_revision': module.UPSTREAM_REVISION,
                   'backend_runtime_verified': False,
                   'target_association_verified': False,
                   'target_id_semantics': 'request provenance only; backend does not establish track association',
                   'eef_parent_link_verified': False, 'request_target_id': ('\U0001f642' if mode == 'maximum' else 't')*128,
                   'source_stamp': 100., 'frame_id': 'base_link', 'count': batch_size,
                   'pose_semantics': 'upstream_eef_parent', 'request_stamp': 100.}
        for _ in range(1000):
            start = time.perf_counter_ns()
            checked = module.checked_grasps(batch, frame='base_link', now=100.1, max_age=.5,
                                            earliest_stamp=100., max_candidates=batch_size)
            poses = PoseArray(header=deepcopy(checked[0].grasp_pose.header),
                              poses=[deepcopy(g.grasp_pose.pose) for g in checked])
            status = String(data=json.dumps(payload, sort_keys=True))
            response = GraspPlanning.Response(grasps=checked)
            sizes = [len(serialize_message(message)) for message in (response, poses, status)]
            samples.append((time.perf_counter_ns()-start)*1e-3)
        ordered = sorted(samples[100:])
        results.append({'mode': mode, 'batch_size': batch_size, 'samples': len(ordered),
                        'mean_us': statistics.mean(ordered),
                        'p95_us': ordered[int(.95*len(ordered))],
                        'serialized_bytes_response_poses_status': sizes})
    print(json.dumps({'cases': results, 'default_launch_node_count': 0,
                      'scope': 'whole bounded payload validation/materialization/JSON/serialization; excludes DDS/backend'}, indent=2))


if __name__ == '__main__':
    main()
