#!/usr/bin/env python3
"""CPU-only whole fixed-slot/packing/serialization cost; never executes GPU code."""
import argparse
import json
from pathlib import Path
import socket
import statistics
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from workshop.perception.rocm_transport import SharedSlots, SlotLayout, encode_packet, decode_packet, pack_cloud, WirePacker
from benchmark_rocm_projection import load_oracle, reference_texture


def summarize(values):
    return dict(samples=len(values), mean_ms=statistics.mean(values),
                p95_ms=float(np.percentile(values, 95)), maximum_ms=max(values))


def benchmark(oracle, *, samples, ros=False, wire_library=None, ros_owned_arrays=False):
    if not 3 <= samples <= 20: raise ValueError('bounded3..20 samples required')
    record, c, depth, rgb, aligned, xyz, uv, hashes = load_oracle(oracle)
    packed, valid = reference_texture(xyz, uv, rgb)
    wire_packer = WirePacker(wire_library) if wire_library else None
    shared = SharedSlots.create(SlotLayout(c.depth_size, c.color_size))
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    parent.settimeout(.1); child.settimeout(.1)
    if ros:
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
        from std_msgs.msg import String
    entries = []
    try:
        for name in ('frozen_native', 'maximum_valid_points'):
            points = xyz if name == 'frozen_native' else np.full(xyz.shape, np.float32(.5), np.float32)
            keep = valid if name == 'frozen_native' else np.ones(valid.shape, bool)
            totals = []; packing = []; serialization = []; wire_bytes = None
            for repeat in range(samples+1):
                start = time.perf_counter()
                shared.write_input(0, depth, rgb)
                job = dict(kind='job', slot=0, sequence=repeat+1,
                    metadata=dict(depth_stamp_ns=1792431136546146973, color_stamp_ns=1792431136546170898,
                                  source_generation='a'*64, producer_generation='b'*32))
                parent.send(encode_packet(job)); received = decode_packet(child.recv(65537))
                packing_start = time.perf_counter()
                if wire_packer:
                    cloud_size = wire_packer.pack_into(points, packed, keep, shared.cloud_output_view(0))
                else:
                    cloud = pack_cloud(points, packed, keep); cloud_size = len(cloud)
                packing_ms = (time.perf_counter()-packing_start)*1000
                if wire_packer: shared.write_aligned(0, aligned)
                else: shared.write_output(0, aligned, cloud)
                child.send(encode_packet(dict(kind='result', slot=0, sequence=received['sequence'],
                    metadata=received['metadata'], cloud_bytes=cloud_size, qualified=False)))
                response = decode_packet(parent.recv(65537))
                aligned_bytes, cloud_bytes = (shared.owned_ros_arrays if ros_owned_arrays else shared.owned_output)(0, response['cloud_bytes'])
                serialization_start = time.perf_counter()
                if ros:
                    image = Image(); image.width, image.height = c.color_size
                    image.header.frame_id = 'camera_color_optical_frame'; image.header.stamp.sec = 1792431136
                    image.header.stamp.nanosec = 546146973; image.encoding = '16UC1'; image.step = image.width*2; image.data = aligned_bytes
                    info = CameraInfo(); info.header = image.header; info.width = image.width; info.height = image.height
                    info.distortion_model = 'plumb_bob'; info.d = [0.]*5
                    fx, fy, cx, cy = map(float, c.color_k)
                    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]; info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                    info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
                    message = PointCloud2(); message.header.frame_id = 'camera_depth_optical_frame'
                    message.header.stamp = image.header.stamp; message.height = 1; message.width = len(cloud_bytes)//20
                    message.point_step = 20; message.row_step = len(cloud_bytes); message.is_dense = True; message.data = cloud_bytes
                    message.fields = [PointField(name=field, offset=offset, datatype=PointField.FLOAT32, count=1)
                                      for field, offset in (('x', 0), ('y', 4), ('z', 8), ('rgb', 16))]
                    lineage = String(data=json.dumps(dict(response, lineage_padding='x'*8192), allow_nan=False))
                    wire_bytes = [len(serialize_message(item)) for item in (image, info, message, lineage)]
                elapsed = (time.perf_counter()-start)*1000
                if repeat == 0:
                    # Independent field/bit-order oracle, outside timed samples.
                    expected = np.zeros((*points.shape[:2], 5), '<u4')
                    expected[..., :3] = points.view(np.uint32); expected[..., 4] = packed
                    actual_cloud = cloud_bytes.tobytes() if ros_owned_arrays else cloud_bytes
                    if actual_cloud != expected[keep.astype(bool)].tobytes():
                        raise AssertionError('complete wire bytes differ from independent CPU bit oracle')
                if repeat:
                    totals.append(elapsed); packing.append(packing_ms)
                    serialization.append((time.perf_counter()-serialization_start)*1000)
            entries.append(dict(case=name, points=int(np.count_nonzero(keep)), cloud_bytes=cloud_size, complete_wire_bytes_match=True,
                total_cpu_boundary=summarize(totals), packing=summarize(packing),
                ros_materialize_serialize=summarize(serialization) if ros else None, ros_wire_bytes=wire_bytes))
    finally: parent.close(); child.close(); shared.close()
    return dict(schema='rocm-projection-cpu-boundary-benchmark-v1', qualified=False, gpu_executed=False,
        diagnostic_repeats_are_new_acquisitions=False, input_sha256=hashes, cases=entries,
        wire_packer='native_direct_mmap' if wire_packer else 'numpy_legacy_reference',
        ownership='direct_owned_ROS_array' if ros_owned_arrays else 'intermediate_owned_bytes',
        scope='input ownership copies + socket JSON roundtrip + vector point packing + mmap output write/owned read + optional actual ROS message materialization/serialization; excludes GPU/SDK/child scheduling/DDS',
        python_executable=sys.executable)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--oracle', type=Path, required=True); parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=10); parser.add_argument('--ros', action='store_true')
    parser.add_argument('--wire-pack-library', type=Path)
    parser.add_argument('--owned-ros-arrays', action='store_true')
    args = parser.parse_args(); result = benchmark(args.oracle, samples=args.samples, ros=args.ros, wire_library=args.wire_pack_library, ros_owned_arrays=args.owned_ros_arrays)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
