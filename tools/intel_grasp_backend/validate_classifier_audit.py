#!/usr/bin/env python3
"""Compare actual upstream grasp-image inference to the pinned original Caffe CPU model."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
try:
    from .convert_legacy_model import PINS
except ImportError:
    from convert_legacy_model import PINS


def validate(source, audit, adapted_prototxt):
    source, audit = Path(source), Path(audit)
    for name, digest in PINS.items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest() != digest:
            raise ValueError('original model pin mismatch: '+name)
    metadata_bytes = audit.read_bytes()
    metadata = json.loads(metadata_bytes)
    shape = metadata['shape']
    if (len(shape) != 4 or shape[1:] != [60, 60, 15]
            or not isinstance(shape[0], int) or not 1 <= shape[0] <= 512
            or metadata['layout'] != 'NHWC' or metadata['dtype'] != 'uint8'
            or metadata['input_scale'] != 1.):
        raise ValueError('unexpected actual-image audit layout/bounds/preprocessing')
    image_path = audit.with_suffix('.inputs.u8')
    image_bytes = image_path.read_bytes()
    if len(image_bytes) != int(np.prod(shape)):
        raise ValueError('actual-image audit byte count mismatch')
    images = np.frombuffer(image_bytes, dtype=np.uint8).reshape(shape)
    # The pinned inference MemoryData has no transform_param. AddMatVector's
    # default scale is 1 and mean is 0; raw 15 channels retain their original order.
    inputs = images.transpose(0, 3, 1, 2).astype(np.float32)
    net = cv2.dnn.readNetFromCaffe(str(adapted_prototxt),
        str(source/'caffe/15channels/single_view_15_channels.caffemodel'))
    cv2.setNumThreads(1)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    start = time.perf_counter()
    net.setInput(inputs)
    original_logits = net.forward()
    reference_ms = (time.perf_counter()-start)*1000
    actual_logits = np.asarray(metadata['logits'])
    actual_scores = np.asarray(metadata['scores'])
    if actual_logits.shape != (shape[0], 2) or actual_scores.shape != (shape[0],):
        raise ValueError('actual-image audit logit/score count mismatch')
    np.testing.assert_allclose(actual_logits, original_logits, rtol=1e-5, atol=1e-3)
    reference_scores = original_logits[:, 1]-original_logits[:, 0]
    np.testing.assert_allclose(actual_scores, reference_scores, rtol=1e-5, atol=2e-3)
    np.testing.assert_array_equal(actual_scores >= 500., reference_scores >= 500.)
    return {'schema': 'actual-gpd-image-caffe-onnx-cpu-parity-v1', 'images': shape[0],
            'actual_image_bytes_sha256': hashlib.sha256(image_bytes).hexdigest(),
            'audit_metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest(),
            'source_model_sha256': PINS, 'reference_opencv': cv2.__version__,
            'reference_inference_ms': reference_ms, 'all_scores_at_threshold_match': True,
            'maximum_abs_logit_error': float(np.max(np.abs(actual_logits-original_logits))),
            'maximum_abs_score_error': float(np.max(np.abs(actual_scores-reference_scores))),
            'original_threshold': 500., 'images_at_original_threshold': int((reference_scores >= 500).sum()),
            'score_minimum': float(reference_scores.min()), 'score_maximum': float(reference_scores.max()),
            'input_scale': 1., 'channel_order': 'original GPD HWC15 to NCHW; no RGB conversion',
            'score_formula': 'positive raw logit[1] minus negative raw logit[0]',
            'qualified': False, 'execution_enabled': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('gpd_source', type=Path)
    parser.add_argument('audit_json', type=Path)
    parser.add_argument('adapted_prototxt', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.gpd_source, args.audit_json, args.adapted_prototxt)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
