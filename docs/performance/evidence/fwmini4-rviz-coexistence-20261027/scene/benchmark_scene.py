#!/usr/bin/env python3
"""CPU helper/serialization cost; no node, executor, DDS transport, camera or GPU."""
from pathlib import Path
import argparse
import gc
import hashlib
import json
import statistics
import time
import tracemalloc
from rclpy.serialization import serialize_message
from visual_scene import SceneMessages

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=100000)
    parser.add_argument('--serialization-iterations', type=int, default=1000)
    args = parser.parse_args()
    assert args.iterations >= 10000 and args.serialization_iterations >= 100
    scene = SceneMessages()
    identities = (id(scene.cloud.data), id(scene.joints.position), tuple(id(m) for m in scene.markers.markers))
    cloud_hash = hashlib.sha256(scene.cloud.data).hexdigest()
    for i in range(1000):
        scene.update_joint(i*33333333, i)
        if i % 3 == 0:
            scene.update_world(i*33333333)
    gc.collect()
    gc.disable()
    joint_times, world_times = [], []
    for _ in range(7):
        start = time.perf_counter_ns()
        for i in range(args.iterations):
            scene.update_joint(i*33333333, i)
        joint_times.append((time.perf_counter_ns()-start)/args.iterations)
        start = time.perf_counter_ns()
        for i in range(args.iterations):
            scene.update_world(i*100000000)
        world_times.append((time.perf_counter_ns()-start)/args.iterations)
    serialization = {}
    for name, message, hz in (('joints', scene.joints, 30), ('cloud', scene.cloud, 10), ('markers', scene.markers, 10)):
        serialized = serialize_message(message)
        start = time.perf_counter_ns()
        for _ in range(args.serialization_iterations):
            serialized = serialize_message(message)
        elapsed = time.perf_counter_ns()-start
        serialization[name] = {'samples': args.serialization_iterations, 'bytes': len(serialized), 'mean_ns': elapsed/args.serialization_iterations, 'target_hz': hz}
    tracemalloc.start()
    for i in range(args.iterations):
        scene.update_joint(i*33333333, i)
        if i % 3 == 0:
            scene.update_world(i*33333333)
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    gc.enable()
    assert identities == (id(scene.cloud.data), id(scene.joints.position), tuple(id(m) for m in scene.markers.markers))
    assert cloud_hash == hashlib.sha256(scene.cloud.data).hexdigest()
    report = {'scope': 'native ROS message update + native rclpy serialization; excludes Node, DDS publication/transport, robot_state_publisher, actual resident RGB, RViz, driver and GPU',
              'helper_warmup_iterations': 1000, 'iterations_per_trial': args.iterations, 'trials': 7,
              'joint_update_samples_ns': joint_times, 'world_update_samples_ns': world_times,
              'joint_update_median_ns': statistics.median(joint_times), 'world_update_median_ns': statistics.median(world_times),
              'serialization': serialization, 'steady_helper_python_tracemalloc_current_bytes': current, 'steady_helper_python_tracemalloc_peak_bytes': peak,
              'point_count': 20000, 'cloud_payload_bytes': len(scene.cloud.data), 'cloud_payload_sha256': cloud_hash,
              'hot_message_payload_identity_stable': True,
              'ros_serialization_allocates_by_design': True,
              'estimated_helper_and_serialization_cpu_ms_per_second':
                  (30*statistics.median(joint_times)+10*statistics.median(world_times)+sum(v['target_hz']*v['mean_ns'] for v in serialization.values()))/1000000,
              'visual_scene_sha256': hashlib.sha256((ROOT/'visual_scene.py').read_bytes()).hexdigest(), 'gpu_or_usb': False}
    output = json.dumps(report, indent=2, sort_keys=True)+'\n'
    (ROOT/'cpu-microbenchmark.json').write_text(output)
    print(output, end='')


if __name__ == '__main__':
    main()
