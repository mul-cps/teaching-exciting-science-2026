"""Saved RGB detector timing: python -m tools.benchmark_tag_tracking --rgb image.png.

Measures detector construction separately from its first and repeated calls.
This is image processing throughput, not physical calibration or identity proof.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from workshop.digital_twin.tag_tracking import TagTracker


def benchmark_detector(rgb, samples=30, detector_scale=2):
    if not isinstance(samples, int) or samples < 1:
        raise ValueError('samples must be a positive integer')
    start = time.perf_counter()
    tracker = TagTracker(detector_scale=detector_scale)
    construction_ms = (time.perf_counter() - start) * 1000
    start = time.perf_counter()
    first = tracker.detect(rgb)
    first_ms = (time.perf_counter() - start) * 1000
    elapsed, counts = [], []
    for _ in range(samples):
        start = time.perf_counter()
        found = tracker.detect(rgb)
        elapsed.append((time.perf_counter() - start) * 1000)
        counts.append(len(found))
    return {'samples': samples, 'source_shape': list(rgb.shape),
            'detector_scale': detector_scale, 'opencv_version': cv2.__version__,
            'opencv_threads': cv2.getNumThreads(), 'construction_ms': construction_ms,
            'first_detection_ms': first_ms, 'median_ms': float(np.median(elapsed)),
            'p95_ms': float(np.percentile(elapsed, 95)), 'max_ms': float(np.max(elapsed)),
            'first_tag_count': len(first), 'tag_count_range': [min(counts), max(counts)],
            'physical_calibration_qualified': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rgb', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--detector-scale', type=float, default=2)
    parser.add_argument('--opencv-threads', type=int, default=4)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.opencv_threads < 1:
        parser.error('--opencv-threads must be positive')
    cv2.setNumThreads(args.opencv_threads)
    # Read once so atomic replacement of a recorder's latest.png cannot change
    # the provenance hash after the benchmark has processed another frame.
    image_bytes = args.rgb.read_bytes()
    bgr = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        parser.error('Could not read saved RGB image')
    report = benchmark_detector(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                                args.samples, args.detector_scale)
    report['image_path'] = str(args.rgb.resolve())
    report['image_sha256'] = hashlib.sha256(image_bytes).hexdigest()
    encoded = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    print(encoded, end='')


if __name__ == '__main__':
    main()
