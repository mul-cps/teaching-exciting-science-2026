#!/usr/bin/env python3
"""Explicit offline GPU private-worker/ABI/wire test; no ROS node or devices.

Frozen recorded headers are retained. Diagnostic repeats are not acquisitions.
ROS serialization is optional and does not initialize DDS or publish messages.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from workshop.perception.rocm_client import ProjectionWorker
from benchmark_rocm_projection import load_oracle, reference_texture, summarize


def run(oracle, library, python, *, samples=10, ros=False):
    record, calibration, depth, rgb, expected_aligned, xyz, uv, hashes = load_oracle(oracle)
    packed, valid = reference_texture(xyz, uv, rgb)
    expected = np.zeros((*xyz.shape[:2], 5), '<u4')
    expected[..., :3] = xyz.view(np.uint32); expected[..., 4] = packed
    expected_cloud = expected[valid].tobytes(); del expected
    capture = record.get('source_provenance', {}).get('capture', {})
    headers = {key: capture.get(key) for key in ('depth_header', 'color_header')}
    metadata = dict(recorded_original_headers=headers, original_oracle_sha256=hashes['oracle.json'],
        diagnostic_repeats_are_new_acquisitions=False, qualified=False)
    if ros:
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
        from std_msgs.msg import String
    started = time.monotonic()
    worker = ProjectionWorker(record, python, library, enabled=True)
    report = dict(schema='so101-rocm-offline-private-worker-proof-v1', qualified=False,
        diagnostic_repeats_are_new_acquisitions=False, ros_node_started=False,
        physical_camera_used=False, gpu_executed=True, original_headers=headers, input_sha256=hashes,
        samples=[], all_pass=True)
    try:
        while not worker.ready:
            worker.poll(); time.sleep(.001)
        report['cold_startup_ms'] = (time.monotonic()-started)*1000
        report['hardware'] = worker.hardware; report['implementation'] = worker.provenance
        for sample in range(samples):
            start = time.perf_counter()
            worker.offer(depth, rgb, metadata)
            result = None
            while result is None:
                result = worker.poll()
                if result is None: time.sleep(.001)
            owned_ms = (time.perf_counter()-start)*1000
            serialize_start = time.perf_counter(); wire_bytes = None
            if ros:
                image = Image(); image.width, image.height = calibration.color_size
                original = headers['depth_header']
                if original:
                    image.header.stamp.sec = original['stamp']['sec']; image.header.stamp.nanosec = original['stamp']['nanosec']
                image.header.frame_id = 'camera_color_optical_frame'; image.encoding = '16UC1'; image.step = image.width*2
                image.data = result['aligned']
                info = CameraInfo(); info.header = image.header; info.width = image.width; info.height = image.height
                fx, fy, cx, cy = map(float, calibration.color_k)
                info.distortion_model = 'plumb_bob'; info.d = [0.]*5
                info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
                info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
                cloud = PointCloud2(); cloud.header.stamp = image.header.stamp; cloud.header.frame_id = 'camera_depth_optical_frame'
                cloud.height = 1; cloud.width = len(result['cloud'])//20; cloud.point_step = 20
                cloud.row_step = len(result['cloud']); cloud.is_dense = True; cloud.data = result['cloud']
                cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                                for name, offset in (('x', 0), ('y', 4), ('z', 8), ('rgb', 16))]
                # Explicitly offline benchmark status, not a fake DeviceInfo or
                # derived-camera lineage.8KB payload bounds representative cost.
                status = String(data=json.dumps(dict(metadata, benchmark_status_padding='x'*8192), allow_nan=False))
                wire_bytes = [len(serialize_message(message)) for message in (image, info, cloud, status)]
            total_ms = (time.perf_counter()-start)*1000
            serialization_ms = (time.perf_counter()-serialize_start)*1000
            # Parity comparisons are outside timing, and examine complete output.
            alignment_difference = int(np.count_nonzero(np.frombuffer(result['aligned'], '<u2').reshape(expected_aligned.shape) != expected_aligned))
            cloud_matches = result['cloud'].tobytes() == expected_cloud
            same_metadata = result['metadata'] == metadata
            passed = alignment_difference == 0 and cloud_matches and same_metadata
            report['all_pass'] &= passed
            report['samples'].append(dict(sample=sample, complete_ms=total_ms, owned_output_ms=owned_ms,
                ros_materialize_serialize_ms=serialization_ms if ros else None,
                backend_ms=result['backend_ms'], worker_ms=result['worker_ms'],
                aligned_different=alignment_difference, complete_cloud_bytes_match=cloud_matches,
                original_metadata_unchanged=same_metadata, ros_wire_bytes=wire_bytes, pass_gate=passed))
            if not passed: break
        report['whole_timing'] = summarize([value['complete_ms'] for value in report['samples']])
    finally:
        worker.close()
        report['private_process_reaped'] = worker.process.poll() is not None
    report['scope'] = 'input slot ownership + actual private3.13 GPU worker+native wirepack + one owned ROS array copy + optional actual message serialization; includes1ms parent poll sleeps; excludes physical capture/ROS subscriptions/DDS publication/registration or control qualification'
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--oracle', type=Path, required=True)
    parser.add_argument('--compatibility-library', type=Path, required=True)
    parser.add_argument('--worker-python', default='/home/bjoern/miniforge3/bin/python')
    parser.add_argument('--samples', type=int, default=10); parser.add_argument('--ros', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.gpu is not True or not 1 <= args.samples <= 20:
        parser.error('explicit--gpu and bounded1..20samples required; no CPU fallback')
    try: report = run(args.oracle, args.compatibility_library, args.worker_python, samples=args.samples, ros=args.ros)
    except Exception as error:
        report = dict(all_pass=False, gpu_requested=True, qualified=False,
            error=(type(error).__name__+': '+str(error))[:4096])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: report.get(key) for key in ('all_pass', 'error', 'whole_timing', 'cold_startup_ms')}))
    return 0 if report['all_pass'] else 3


if __name__ == '__main__': raise SystemExit(main())
