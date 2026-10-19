#!/usr/bin/env python3
"""Compare CPU inference on fixed actual GPD images, with identical batch-eight policy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import time

import cv2
import numpy as np
try:
    from .convert_legacy_model import PINS
except ImportError:
    from convert_legacy_model import PINS


def benchmark(build_manifest, native_result, audit_path, *, samples=5):
    if not 1 <= samples <= 20:
        raise ValueError('one to twenty CPU samples required')
    build = json.loads(Path(build_manifest).read_text())
    native = json.loads(Path(native_result).read_text())
    audit_path = Path(audit_path)
    audit = json.loads(audit_path.read_text())
    shape = audit['shape']
    if len(shape) != 4 or shape[1:] != [60, 60, 15] or not 1 <= shape[0] <= 512:
        raise ValueError('bounded original actual grasp image audit required')
    data_bytes = audit_path.with_suffix('.inputs.u8').read_bytes()
    if len(data_bytes) != int(np.prod(shape)):
        raise ValueError('actual image byte count mismatch')
    data = np.frombuffer(data_bytes, np.uint8).reshape(shape).transpose(0, 3, 1, 2).astype(np.float32)
    source = Path(build['source_paths']['gpd'])
    for name, expected in PINS.items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest() != expected:
            raise ValueError('original model source pin mismatch')
    cv2.setNumThreads(1)
    net = cv2.dnn.readNetFromCaffe(str(Path(build['model']).with_suffix('.prototxt')),
        str(source/'caffe/15channels/single_view_15_channels.caffemodel'))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    scratch = np.zeros((8, 15, 60, 60), np.float32)
    timings = []
    for repeat in range(samples+1):
        scores = []
        start = time.perf_counter_ns()
        for offset in range(0, len(data), 8):
            count = min(8, len(data)-offset)
            scratch[:count] = data[offset:offset+count]
            scratch[count:] = 0
            net.setInput(scratch)
            logits = net.forward()
            scores.extend((logits[:count, 1]-logits[:count, 0]).tolist())
        elapsed = (time.perf_counter_ns()-start)/1e6
        if repeat:
            timings.append(elapsed)
    np.testing.assert_allclose(native['scores'], scores, rtol=1e-5, atol=2e-3)
    np.testing.assert_array_equal(np.asarray(native['scores']) >= 500., np.asarray(scores) >= 500.)
    def summarize(values):
        ordered = sorted(values)
        return {'samples': len(values), 'mean_ms': statistics.mean(values),
                'median_ms': statistics.median(values),
                'p95_ms': ordered[min(len(ordered)-1, int(.95*len(ordered)))],
                'minimum_ms': min(values), 'maximum_ms': max(values)}
    return {'schema': 'fixed-actual-gpd-image-cpu-benchmark-v1', 'images': len(data),
            'actual_image_sha256': hashlib.sha256(data_bytes).hexdigest(),
            'native_result_sha256': hashlib.sha256(Path(native_result).read_bytes()).hexdigest(),
            'build_manifest_sha256': hashlib.sha256(Path(build_manifest).read_bytes()).hexdigest(),
            'batch_size': 8, 'cpp_opencv_version': native['opencv'], 'reference_opencv_version': cv2.__version__,
            'cpp_onnx_cpu': summarize(native['samples_ms']), 'original_caffe_cpu': summarize(timings),
            'threshold_decisions_match': True, 'maximum_abs_score_error': float(np.max(np.abs(np.asarray(native['scores'])-scores))),
            'scope': 'fixed actual-image classifier inference only; excludes grasp generation, model/input IO, DDS and audit IO',
            'loadavg': os.getloadavg(), 'qualified': False, 'amd_gpu_acceleration_claimed': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-manifest', type=Path, required=True)
    parser.add_argument('--native-result', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = benchmark(args.build_manifest, args.native_result, args.audit)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
