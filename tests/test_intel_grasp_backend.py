"""Portable unit contracts; actual upstream-engine proofs are recorded separately."""
import ast
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tools.intel_grasp_backend import build_reference, capture_to_pcd, run_reference, runtime_env


def source_capture(tmp_path, *, points=None):
    source = tmp_path/'capture'; source.mkdir()
    values = np.array([[.1, .2, .4], [.3, .4, .5], [0, 0, 1.3]], np.float32) if points is None else points
    np.savez(source/'cloud-stride2.npz', points_optical_m=values,
             rgb=np.tile(np.array([[12, 34, 56]], np.uint8), (len(values), 1)))
    (source/'capture.json').write_text(json.dumps({'stamp_ns': 123456789123,
        'color_frame': 'camera_color_optical_frame', 'model_sha256': 'a'*64,
        'pointcloud_provenance': {'frozen': True}}))
    return source


def test_pcd_export_preserves_original_acquisition_and_rgb(tmp_path):
    source = source_capture(tmp_path)
    output = tmp_path/'recorded.pcd'
    report = capture_to_pcd.export(source, output)
    assert report['acquisition_stamp_ns'] == 123456789123
    assert report['live_timestamp_refresh'] is False
    assert report['original_points'] == 3 and report['exported_points'] == 2
    header, body = output.read_bytes().split(b'DATA binary\n', 1)
    assert b'acquisition_ns=123456789123' in header
    values = np.frombuffer(body, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgba', '<u4')])
    assert len(values) == 2 and values['rgba'][0] == (255<<24)|(12<<16)|(34<<8)|56
    assert report['pcd_sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    with pytest.raises(FileExistsError):
        capture_to_pcd.export(source, output)


def test_nonfinite_cloud_fails_before_pcd_creation(tmp_path):
    source = source_capture(tmp_path, points=np.array([[0, 0, np.nan]], np.float32))
    output = tmp_path/'bad.pcd'
    with pytest.raises(ValueError, match='finite'):
        capture_to_pcd.export(source, output)
    assert not output.exists()


def test_converter_environment_does_not_mix_python_abis(monkeypatch):
    for name in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'CONDA_PREFIX', 'AMENT_PREFIX_PATH'):
        monkeypatch.setenv(name, '/synthetic/foreign-abi')
    monkeypatch.setenv('ROS_DOMAIN_ID', '177')
    environment = runtime_env.converter_environment()
    assert not {'PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'CONDA_PREFIX', 'AMENT_PREFIX_PATH'} & environment.keys()
    assert environment['ROS_DOMAIN_ID'] == '177'


def test_build_refuses_unrelated_cache_and_excessive_parallelism(tmp_path):
    cache = tmp_path/'unrelated'; cache.mkdir(); (cache/'important').write_text('preserve')
    with pytest.raises(ValueError, match='nonempty'):
        build_reference.build(cache, '/synthetic/python')
    assert (cache/'important').read_text() == 'preserve'
    with pytest.raises(ValueError, match='two jobs'):
        build_reference.build(tmp_path/'new', '/synthetic/python', jobs=3)
    assert not (tmp_path/'new').exists()


def test_gpg_patch_is_exact_idempotent_and_refuses_other_edits(tmp_path, monkeypatch):
    source = tmp_path/'gpg'; path = source/'include/gpg/cloud_camera.h'; path.parent.mkdir(parents=True)
    original = '\n'.join(f'bool operator ()(const Eigen::Vector{n}i& a, const Eigen::Vector{n}i& b) {{}}' for n in (3, 4))
    path.write_text(original)
    monkeypatch.setattr(build_reference.subprocess, 'check_output', lambda *a, **k: original.encode())
    first = build_reference.patch_comparators(source)
    assert path.read_text().count(' const {}') == 2
    assert build_reference.patch_comparators(source) == first
    path.write_text('unrelated source edit')
    with pytest.raises(ValueError, match='unrelated'):
        build_reference.patch_comparators(source)


def inputs(tmp_path):
    pcd = tmp_path/'recorded.pcd'; capture_to_pcd.export(source_capture(tmp_path), pcd)
    engine = tmp_path/'fixture-engine'; engine.write_bytes(b'synthetic non-executable fixture')
    model = tmp_path/'fixture-model'; model.write_bytes(b'synthetic model')
    build = {'schema': 'station-offline-gpd-build-v1', 'model': str(model),
        'model_sha256': run_reference.digest(model), 'engine': str(engine), 'sources': {},
        'binaries_sha256': {str(engine): run_reference.digest(engine)}}
    source = {'schema': 'station-frozen-native-pcd-v1', 'live_timestamp_refresh': False,
        'acquisition_stamp_ns': 123456789123, 'frame_id': 'camera_color_optical_frame',
        'pcd_sha256': run_reference.digest(pcd)}
    manifest = tmp_path/'build.json'; manifest.write_text(json.dumps(build))
    provenance = tmp_path/'source.json'; provenance.write_text(json.dumps(source))
    return manifest, pcd, provenance


def test_modified_pcd_or_binary_rejected(tmp_path):
    manifest, pcd, provenance = inputs(tmp_path)
    build, source = run_reference.checked_inputs(manifest, pcd, provenance)
    assert source['acquisition_stamp_ns'] == 123456789123
    Path(build['engine']).write_bytes(b'changed binary')
    with pytest.raises(ValueError, match='binary hash'):
        run_reference.checked_inputs(manifest, pcd, provenance)


def test_run_binds_per_grasp_source_and_refuses_output_reuse(tmp_path, monkeypatch):
    manifest, pcd, provenance = inputs(tmp_path)
    def fake_run(command, **kwargs):
        # Unit fixture only; actual engine/weights are qualified by separate recorded proof.
        Path(command[3]).write_text(json.dumps({'schema': 'station-offline-upstream-gpd-v1',
            'qualified': False, 'execution_enabled': False, 'classification': {'scored_images': 0},
            'grasps': [{'origin': 'upstream_geometric_fallback', 'qualified': False}]}))
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(run_reference.subprocess, 'run', fake_run)
    output = tmp_path/'attempt'
    result = run_reference.run_once(manifest, pcd, provenance, output)
    assert result['recorded_source'] is True and result['live_timestamp_refresh'] is False
    assert (output/'source-build-manifest.json').read_bytes() == manifest.read_bytes()
    assert (output/'source-pcd-provenance.json').read_bytes() == provenance.read_bytes()
    grasp = result['grasps'][0]
    assert grasp['source_acquisition_stamp_ns'] == 123456789123
    assert grasp['target_association_verified'] is False
    assert grasp['tool_mapping_qualified'] is False and grasp['execution_enabled'] is False
    with pytest.raises(FileExistsError):
        run_reference.run_once(manifest, pcd, provenance, output)


def test_no_live_sensor_motion_or_shell_hooks_in_optional_python():
    directory = Path(build_reference.__file__).parent
    forbidden = {'torch', 'pyrealsense2', 'serial', 'control_msgs.action', 'rclpy.action'}
    for path in directory.glob('*.py'):
        tree = ast.parse(path.read_text())
        imports = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        imports |= {alias.name for n in ast.walk(tree) if isinstance(n, ast.Import) for alias in n.names}
        assert not forbidden & imports, path
        assert not any(isinstance(n, ast.keyword) and n.arg == 'shell' and
                       isinstance(n.value, ast.Constant) and n.value.value is True for n in ast.walk(tree))
    classifier = (directory/'opencv_classifier.cpp').read_text()
    assert 'DNN_BACKEND_OPENCV' in classifier and 'DNN_TARGET_CPU' in classifier
    assert 'setUseOpenCL(false)' in classifier and 'batch_size_ = 8' in classifier
    assert 'images.size() > 512' in classifier
    assert 'p.create_image_batches_ = false' in (directory/'offline_gpd.cpp').read_text()


@pytest.mark.parametrize('mutation', ['huge_points', 'huge_width', 'compressed', 'ascii',
                                      'wrong_size', 'duplicate', 'truncated', 'nonfinite', 'header_cap'])
def test_pcd_guard_rejects_before_subprocess(tmp_path, monkeypatch, mutation):
    manifest, pcd, provenance = inputs(tmp_path)
    data = pcd.read_bytes()
    if mutation == 'huge_points': data = data.replace(b'POINTS 2\n', b'POINTS 999999999999\n')
    elif mutation == 'huge_width': data = data.replace(b'WIDTH 2\n', b'WIDTH 999999999999\n')
    elif mutation == 'compressed': data = data.replace(b'DATA binary\n', b'DATA binary_compressed\n')
    elif mutation == 'ascii': data = data.replace(b'DATA binary\n', b'DATA ascii\n')
    elif mutation == 'wrong_size': data = data.replace(b'SIZE 4 4 4 4', b'SIZE 100000 4 4 4')
    elif mutation == 'duplicate': data = data.replace(b'POINTS 2\n', b'POINTS 2\nPOINTS 2\n')
    elif mutation == 'truncated': data = data[:-1]
    elif mutation == 'header_cap': data = b'#'+b'x'*4096+b'\n'+data
    elif mutation == 'nonfinite':
        import struct
        offset = data.index(b'DATA binary\n')+len(b'DATA binary\n')
        data = data[:offset]+struct.pack('<f', float('nan'))+data[offset+4:]
    pcd.write_bytes(data)
    source = json.loads(provenance.read_text()); source['pcd_sha256'] = run_reference.digest(pcd)
    provenance.write_text(json.dumps(source))
    calls = []
    monkeypatch.setattr(run_reference.subprocess, 'run', lambda *a, **k: calls.append(a))
    with pytest.raises(ValueError, match='PCD'):
        run_reference.run_once(manifest, pcd, provenance, tmp_path/'must-not-exist')
    assert calls == [] and not (tmp_path/'must-not-exist').exists()


def test_cpp_decoder_bound_before_cloudcamera_and_matches_python(tmp_path):
    import shutil
    import subprocess
    compiler = shutil.which('c++')
    if not compiler:
        pytest.skip('standalone C++17 compiler unavailable')
    directory = Path(build_reference.__file__).parent
    source = tmp_path/'probe.cpp'
    source.write_text('#include "pcd_guard.hpp"\n#include <iostream>\nint main(int n,char**v){try{auto p=read_station_pcd(v[1]);std::cout<<p.size();return 0;}catch(...){return 4;}}\n')
    binary = tmp_path/'probe'
    subprocess.run([compiler, '-std=c++17', '-O2', '-I'+str(directory), str(source), '-o', str(binary)], check=True)
    _, pcd, _ = inputs(tmp_path)
    assert subprocess.check_output([str(binary), str(pcd)]) == b'2'
    data = pcd.read_bytes()
    for bad in (data.replace(b'POINTS 2\n', b'POINTS 99999999999\n'), data[:-1],
                data.replace(b'DATA binary\n', b'DATA binary_compressed\n')):
        pcd.write_bytes(bad)
        assert subprocess.run([str(binary), str(pcd)]).returncode == 4
    engine = (directory/'offline_gpd.cpp').read_text()
    assert engine.index('read_station_pcd(argv[1])') < engine.index('PointCloudRGB::Ptr decoded') < engine.index('CloudCamera cloud(')
    assert 'CloudCamera cloud(argv[1]' not in engine


def test_pcd_finite_guard_is_vectorized_and_accepts_maximum(tmp_path):
    import inspect
    from tools.intel_grasp_backend.pcd_guard import checked_pcd
    tree = ast.parse(inspect.getsource(checked_pcd))
    assert 'np.ndarray' in inspect.getsource(checked_pcd)
    assert 'np.isfinite(xyz).all()' in inspect.getsource(checked_pcd)
    assert 'iter_unpack' not in inspect.getsource(checked_pcd)
    # Header loops are allowed; point rows must remain a bounded NumPy view.
    for node in ast.walk(tree):
        if isinstance(node, ast.For):
            assert not isinstance(node.target, ast.Tuple)
    path = tmp_path/'maximum.pcd'
    header = ('VERSION .7\nFIELDS x y z rgba\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n'
              'WIDTH 1000000\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1000000\nDATA binary\n')
    path.write_bytes(header.encode()+bytes(16_000_000))
    assert checked_pcd(path)['points'] == 1_000_000
