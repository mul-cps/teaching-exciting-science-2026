#!/usr/bin/env python3
"""Convert the pinned original GPD IR-v2 graph to ONNX; no training or normalization."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

PINS = {
    'models/openvino/fp32/single_view_15_channels.xml': 'dcc1618dbc1246fd7ff32004c4978704c9e7c0b4103a1e9a9713122fb1464725',
    'models/openvino/fp32/single_view_15_channels.bin': '0bd9e3efbe702893aa2480d108c8508674bf1144df141527c028a2496b561497',
    'caffe/15channels/lenet_15_channels.prototxt': '07b6fa70dbdc80d181e4cac37fcdf3e32dc2803dbaa88344c0f8e43d7222145e',
    'caffe/15channels/single_view_15_channels.caffemodel': 'e26d3f93dae7a57be1333bb48b0fe80cb1686762b576ff7cf1ac9a6b0c4f065a',
}


def convert(source, output, *, validate=False):
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    source, output = Path(source), Path(output)
    inputs = {}
    for name, digest in PINS.items():
        data = (source/name).read_bytes()
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError('unrecognized model source: '+name)
        inputs[name] = data
    xml, binary, prototxt, caffe_weights = PINS
    layers = list(ET.fromstring(inputs[xml]).find('layers'))
    if [(l.attrib['name'], l.attrib['type']) for l in layers] != [
            ('data', 'Input'), ('conv1', 'Convolution'), ('pool1', 'Pooling'),
            ('conv2', 'Convolution'), ('pool2', 'Pooling'), ('ip1', 'FullyConnected'),
            ('relu1', 'ReLU'), ('ip2', 'FullyConnected')]:
        raise ValueError('unexpected original model graph')
    nodes, tensors, blobs = [], [], {}
    previous = 'data'
    for layer in layers[1:]:
        name, kind = layer.attrib['name'], layer.attrib['type']
        if kind in ('Convolution', 'FullyConnected'):
            shape = {'conv1': (20, 15, 5, 5), 'conv2': (50, 20, 5, 5),
                     'ip1': (500, 7200), 'ip2': (2, 500)}[name]
            for key, dims in (('weights', shape), ('biases', (shape[0],))):
                part = layer.find('blobs').find(key).attrib
                array = np.frombuffer(inputs[binary], dtype='<f4', count=int(part['size'])//4,
                                      offset=int(part['offset'])).reshape(dims).copy()
                blobs[name, key] = array
                tensors.append(numpy_helper.from_array(array, name+'_'+key))
            if kind == 'Convolution':
                node = helper.make_node('Conv', [previous, name+'_weights', name+'_biases'],
                                        [name], name=name, kernel_shape=[5, 5])
            else:
                if name == 'ip1':
                    nodes.append(helper.make_node('Flatten', [previous], ['flatten'], axis=1))
                    previous = 'flatten'
                node = helper.make_node('Gemm', [previous, name+'_weights', name+'_biases'],
                                        [name], name=name, transB=1)
        elif kind == 'Pooling':
            node = helper.make_node('MaxPool', [previous], [name], name=name,
                                    kernel_shape=[2, 2], strides=[2, 2], ceil_mode=1)
        else:
            node = helper.make_node('Relu', [previous], [name], name=name)
        nodes.append(node)
        previous = name
    graph = helper.make_graph(nodes, 'original_gpd_15_channel_cpu',
        [helper.make_tensor_value_info('data', TensorProto.FLOAT, ['N', 15, 60, 60])],
        [helper.make_tensor_value_info('ip2', TensorProto.FLOAT, ['N', 2])], tensors)
    model = helper.make_model(graph, producer_name='station-offline-gpd-compat',
                               opset_imports=[helper.make_opsetid('', 13)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, output)
    original = inputs[prototxt].decode()
    second_layer = original.index('layer {', original.index('layer {')+1)
    caffe_input = output.with_suffix('.prototxt')
    caffe_input.write_text('name: "LeNet"\ninput: "data"\ninput_shape { dim: 1 dim: 15 dim: 60 dim: 60 }\n'+original[second_layer:])
    report = {'source_sha256': PINS, 'onnx_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
              'opset': 13, 'normalization': 'none; preserves original inference input',
              'weights_modified': False, 'qualified': False, 'execution_enabled': False}
    if validate:
        import cv2
        cv2.setNumThreads(1)
        caffe = cv2.dnn.readNetFromCaffe(str(caffe_input), str(source/caffe_weights))
        converted = cv2.dnn.readNetFromONNX(str(output))
        for network in (caffe, converted):
            network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        for name in ('conv1', 'conv2', 'ip1', 'ip2'):
            values = caffe.getLayer(caffe.getLayerId(name)).blobs
            for key, value in zip(('weights', 'biases'), values):
                np.testing.assert_array_equal(blobs[name, key].ravel(), value.ravel())
        results = []
        rng = np.random.default_rng(20261019)
        for case, data in [('zeros', np.zeros((1, 15, 60, 60), np.float32)),
                           ('single', rng.integers(0, 256, (1, 15, 60, 60)).astype(np.float32)),
                           ('batch8', rng.integers(0, 256, (8, 15, 60, 60)).astype(np.float32))]:
            caffe.setInput(data); converted.setInput(data)
            before, after = caffe.forward(), converted.forward()
            np.testing.assert_allclose(after, before, rtol=1e-5, atol=1e-3)
            results.append({'case': case, 'max_abs_logit_error': float(np.max(np.abs(before-after))),
                            'logits_first': after[0].tolist()})
        report['cpu_validation'] = {'opencv': cv2.__version__, 'weight_arrays_bit_exact': True,
                                    'cases': results, 'rtol': 1e-5, 'atol': 1e-3}
    output.with_suffix('.conversion.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('gpd_source', type=Path)
    parser.add_argument('output_onnx', type=Path)
    parser.add_argument('--validate', action='store_true')
    args = parser.parse_args()
    print(json.dumps(convert(args.gpd_source, args.output_onnx, validate=args.validate), indent=2))
