#!/usr/bin/env python3
"""Explicitly recorded, unqualified ROS GraspPlanning diagnostics for offline GPD."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import uuid

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2/src/so101_station_scene')]
try:
    from .run_reference import checked_inputs, digest, run_once
except ImportError:
    from run_reference import checked_inputs, digest, run_once

import rclpy
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import Grasp, MoveItErrorCodes
from moveit_msgs.srv import GraspPlanning
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Header, String
from so101_station_scene.scene_geometry import matrix_quaternion_xyzw


def response_from_recorded(report):
    source = report['source_provenance']
    stamp = source['acquisition_stamp_ns']
    if (report.get('recorded_source') is not True or report.get('live_timestamp_refresh') is not False
            or not isinstance(stamp, int) or stamp <= 0 or not source['frame_id']
            or not 0 < len(report['grasps']) <= 5):
        raise ValueError('bounded original recorded source and grasps required')
    response = GraspPlanning.Response()
    for index, item in enumerate(report['grasps']):
        if item['origin'] not in ('upstream_neural_classified', 'upstream_geometric_fallback'):
            raise ValueError('actual classifier/fallback origin required')
        arrays = [np.asarray(item[name], dtype=float) for name in ('bottom', 'binormal', 'axis', 'approach')]
        if any(a.shape != (3,) or not np.isfinite(a).all() for a in arrays) or not np.isfinite(item['score']):
            raise ValueError('finite actual engine pose/score required')
        bottom, closing, lateral, approach = arrays
        rotation = np.column_stack((closing, lateral, approach))
        if not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5, rtol=0) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5):
            raise ValueError('actual upstream grasp basis is not a proper rotation')
        grasp = Grasp()
        grasp.id = 'recorded-'+source['pcd_sha256'][:12]+'-g'+str(index)
        header = Header(frame_id=source['frame_id'])
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        grasp.grasp_pose.header = header
        position, orientation = grasp.grasp_pose.pose.position, grasp.grasp_pose.pose.orientation
        position.x, position.y, position.z = map(float, bottom)
        orientation.x, orientation.y, orientation.z, orientation.w = map(float, matrix_quaternion_xyzw(rotation))
        grasp.grasp_quality = float(item['score'])  # Raw logit difference, not probability.
        for translation, sign in ((grasp.pre_grasp_approach, 1.), (grasp.post_grasp_retreat, -1.)):
            translation.direction.header = deepcopy(header)
            translation.direction.vector.x, translation.direction.vector.y, translation.direction.vector.z = map(float, approach*sign)
            translation.min_distance, translation.desired_distance = .06, .1  # Upstream reference defaults.
        # Unknown jaw mapping/posture and actual robot EEF parent are never invented.
        response.grasps.append(grasp)
    response.error_code.val = MoveItErrorCodes.SUCCESS
    response.error_code.message = 'Recorded diagnostic only; unqualified; no target association or tool mapping'
    response.error_code.source = 'offline_intel_gpd'
    return response


class OfflineGraspBackend(Node):
    def __init__(self, build_manifest, pcd, provenance, output_root, *, timeout_s=10.):
        super().__init__('offline_intel_gpd_backend')
        if not .1 <= timeout_s <= 30.:
            raise ValueError('offline timeout must be .1..30 s')
        _, self.source = checked_inputs(build_manifest, pcd, provenance)
        self.paths = tuple(map(Path, (build_manifest, pcd, provenance)))
        self.frozen_metadata = (digest(build_manifest), digest(provenance))
        self.output_root, self.timeout = Path(output_root), timeout_s
        self.dataset_id = 'offline-pcd:'+self.source['pcd_sha256']
        self.pending = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='offline-gpd')
        group = ReentrantCallbackGroup()
        self.status = self.create_publisher(String, '/offline/intel_grasp/status', 10)
        self.poses = self.create_publisher(PoseArray, '/offline/intel_grasp/poses', 1)
        self.service = self.create_service(GraspPlanning, '/offline/intel_grasp/plan',
                                           self._request, callback_group=group)
        self.timer = self.create_timer(.02, self._poll, callback_group=group)

    def _report(self, state, **extra):
        payload = {'state': state, 'recorded_source': True, 'qualified': False,
                   'execution_enabled': False, 'target_association_verified': False,
                   'tool_mapping_qualified': False, 'eef_parent_link_verified': False,
                   'live_timestamp_refresh': False, 'ik_validated': False, 'collision_checked': False,
                   'pose_semantics': 'GPD hand base; x binormal, y hand axis, z approach; zero EEF offset',
                   'source_dataset_id': self.dataset_id, 'frame_id': self.source['frame_id'],
                   'source_acquisition_stamp_ns': self.source['acquisition_stamp_ns'], **extra}
        self.status.publish(String(data=json.dumps(payload, sort_keys=True, allow_nan=False)))

    def _failure(self, state, detail=''):
        self._report(state, detail=str(detail)[:512])
        response = GraspPlanning.Response()
        response.error_code.val = MoveItErrorCodes.FAILURE
        response.error_code.message = state+': '+str(detail)[:256]
        response.error_code.source = 'offline_intel_gpd'
        return response

    async def _request(self, request, response):
        if request.group_name != 'offline_reference' or request.target.id != self.dataset_id:
            return self._failure('invalid_dataset_request', 'group offline_reference and exact offline-pcd SHA required')
        target = request.target
        pose = target.pose
        pose_values = (pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        requested_ns = target.header.stamp.sec*1_000_000_000+target.header.stamp.nanosec
        if (request.candidate_grasps or request.movable_obstacles or request.support_surfaces
                or any((target.primitives, target.primitive_poses, target.meshes, target.mesh_poses,
                        target.planes, target.plane_poses, target.subframe_names, target.subframe_poses))
                or pose_values != (0., 0., 0., 0., 0., 0., 1.) or target.type.key or target.type.db
                or target.operation != target.ADD
                or target.header.frame_id not in ('', self.source['frame_id'])
                or requested_ns not in (0, self.source['acquisition_stamp_ns'])):
            return self._failure('invalid_dataset_request', 'recorded dataset ID only; caller cannot override source geometry/time')
        if self.pending is not None:
            return self._failure('busy')
        if (digest(self.paths[0]), digest(self.paths[2])) != self.frozen_metadata:
            return self._failure('changed_source_metadata')
        completion = Future(executor=self.executor)
        output = self.output_root/uuid.uuid4().hex
        job = self.worker.submit(run_once, *self.paths, output, timeout_s=self.timeout)
        self.pending = (job, completion, output)
        return await completion

    def _poll(self):
        if self.pending is None or not self.pending[0].done():
            return
        job, completion, output = self.pending
        self.pending = None
        try:
            report = job.result()
            if report['source_provenance'] != self.source:
                raise ValueError('recorded source changed during engine run')
            response = response_from_recorded(report)
            self.poses.publish(PoseArray(header=deepcopy(response.grasps[0].grasp_pose.header),
                                         poses=[deepcopy(g.grasp_pose.pose) for g in response.grasps]))
            self._report('recorded_proposals', result=str(output/'recorded-result.json'),
                         classification=report['classification'], count=len(response.grasps),
                         proposal_origins=[g['origin'] for g in report['grasps']],
                         score_semantics='positive raw logit minus negative raw logit; not probability')
        except Exception as error:
            response = self._failure('offline_engine_failure', error)
        completion.set_result(response)

    def destroy_node(self):
        # The bounded subprocess helper owns only this offline CPU child.
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-manifest', type=Path, required=True)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--provenance', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    args, ros_args = parser.parse_known_args(argv)
    domain = int(os.environ.get('ROS_DOMAIN_ID', '76'))
    if not 1 <= domain <= 232:
        raise ValueError('offline backend requires ROS_DOMAIN_ID from 1 through 232')
    os.environ.setdefault('ROS_DOMAIN_ID', '76')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = OfflineGraspBackend(args.build_manifest, args.pcd, args.provenance, args.output_root)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
