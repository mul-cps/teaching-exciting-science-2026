"""CPU-only parser, fixture factory and actual-module benchmark loop tests."""
from pathlib import Path
import hashlib
import json
import os
import struct
import subprocess

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'
SOURCE = PACKAGE/'test/benchmark_geometry.cpp'
NAMES = ['geometry_coordinates', 'geometry_support_scan',
         'geometry_support_offsets', 'geometry_support_scatter',
         'geometry_valid_count']


def fixture_inputs(tmp_path, precision='--software-binary64'):
    """Independent zero-depth 2x2 fixture; known valid/support counts both zero."""
    fixture = tmp_path/'fixture.bin'
    data = struct.pack('<8sIIQdd', b'GEOQv1\0\0', 2, 2, 17, .001, 1.2)
    data += struct.pack('<12d', 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)
    data += struct.pack('<6d', -10, 10, -10, 10, -10, 10)
    data += struct.pack('<8d', 0, 0, .1, 0, 0, .1, .1, .1)
    data += struct.pack('<4I4f', 0, 0, 0, 0, 0, 0, 0, 0)
    fixture.write_bytes(data)
    shaders = tmp_path/'shaders'
    shaders.mkdir()
    capability = 11 if precision == '--software-binary64' else 10
    minimal = struct.pack('<7I', 0x07230203, 0x10000, 0, 1, 0, (2 << 16) | 17, capability)
    for name in NAMES:
        (shaders/(name+'.spv')).write_bytes(minimal)
    return fixture, shaders


def invoke(binary, fixture, shaders, output, *options, mode='--offline',
           precision='--software-binary64', success=True, environment=None,
           expected_valid=0):
    command = [str(binary), mode, precision, str(fixture), str(shaders), str(output),
               '--expected-valid', str(expected_valid), '--expected-support', '0', *map(str, options)]
    result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=20)
    if success:
        assert result.returncode == 0, result.stderr
        report = json.loads(result.stdout)
        assert json.loads(output.read_text()) == report
        return report
    assert result.returncode != 0
    return result


@pytest.fixture(scope='module')
def offline_binary(tmp_path_factory):
    binary = tmp_path_factory.mktemp('benchmark-offline')/'benchmark'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                    '-DGEOMETRY_BENCHMARK_CPU_ONLY', str(SOURCE), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    symbols = subprocess.check_output(['nm', '-u', str(binary)], text=True)
    assert 'vkCreate' not in symbols and 'Context' not in symbols
    return binary


def build_synthetic_benchmark(tmp_path, stub_transform=None):
    """Real benchmark loop + real production modules, existing fake Vulkan ABI."""
    prefix = (PACKAGE/'test/preparation_host_benchmark.cpp').read_text().split(
        'template<class F>double measure', 1)[0]
    prefix = prefix.replace('#include "so101_station_vulkan/alignment.hpp"\n', '')
    start = prefix.index('void* operator new(')
    end = prefix.index('namespace so101_station_vulkan {', start)
    prefix = prefix[:start] + prefix[end:]
    prefix = prefix.replace('Context::Context(){',
                            'Context::Context(){enabled_features.features.shaderInt64=VK_TRUE;'
                            'properties.vendorID=0x1002;std::strcpy(properties.deviceName,"SYNTHETIC CPU DRIVER");')
    prefix = prefix.replace('namespace {\nuint64_t heap_allocations',
                            'namespace {\nunsigned pending_once[4096]{};VkFence geometry_fence=VK_NULL_HANDLE;\nuint64_t heap_allocations')
    prefix = prefix.replace('return complete[uintptr_t(f)]?VK_SUCCESS:VK_NOT_READY;',
                            'if(std::getenv("BENCH_FAKE_STALL")&&f==geometry_fence)return VK_NOT_READY;'
                            'if(pending_once[uintptr_t(f)]){--pending_once[uintptr_t(f)];return VK_NOT_READY;}'
                            'return complete[uintptr_t(f)]?VK_SUCCESS:VK_NOT_READY;')
    prefix = prefix.replace('complete[uintptr_t(f)]=!hold;return VK_SUCCESS;',
                            'complete[uintptr_t(f)]=!hold;'
                            'if(submits>1){geometry_fence=f;pending_once[uintptr_t(f)]=std::getenv("BENCH_FAKE_NO_POLL")?0:1;}return VK_SUCCESS;')
    fixture = tmp_path/'synthetic.cpp'
    fixture.write_text(prefix + r'''
#include <time.h>
namespace so101_station_vulkan {
std::shared_ptr<Context> Context::shared(){return std::make_shared<Context>();}
}
extern "C" void vkGetPhysicalDeviceFeatures(VkPhysicalDevice,VkPhysicalDeviceFeatures* out){
 *out={};out->shaderFloat64=std::getenv("BENCH_FAKE_NO_F64")?VK_FALSE:VK_TRUE;
 out->shaderInt64=std::getenv("BENCH_FAKE_NO_INT64")?VK_FALSE:VK_TRUE;
}
int test_clock_gettime(clockid_t id,timespec* t){
 int rc=::clock_gettime(id,t);if(std::getenv("BENCH_FAKE_STALL")&&id==CLOCK_MONOTONIC){static long ticks=0;ticks+=11;t->tv_sec+=ticks;}return rc;
}
#define clock_gettime test_clock_gettime
#define main actual_benchmark_main
#define GEOMETRY_BENCHMARK_SYNTHETIC_DRIVER
#define GEOMETRY_BENCHMARK_PROVENANCE "CPU SYNTHETIC ONLY; actual benchmark source and real geometry.cpp/frame_pool.cpp; no Vulkan loader"
#include "''' + str(SOURCE) + r'''"
#undef main
int main(int argc,char** argv){int rc=actual_benchmark_main(argc,argv);if(live_command_pools)return 99;return rc;}
''')
    if stub_transform is not None:
        fixture.write_text(stub_transform(fixture.read_text()))
    binary = tmp_path/'synthetic'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                    '-ffp-contract=off', '-fno-fast-math', '-frounding-math',
                    '-I', str(PACKAGE/'include'), str(fixture),
                    str(PACKAGE/'src/geometry.cpp'), str(PACKAGE/'src/frame_pool.cpp'),
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    symbols = subprocess.check_output(['nm', '-u', str(binary)], text=True)
    assert 'vkCreateInstance' not in symbols and 'vkCreateDevice' not in symbols
    return binary


def test_offline_no_device_and_fresh_file(offline_binary, tmp_path):
    fixture, shaders = fixture_inputs(tmp_path)
    out = tmp_path/'report.json'
    report = invoke(offline_binary, fixture, shaders, out)
    assert report['fixture_sha256'] == hashlib.sha256(fixture.read_bytes()).hexdigest()
    assert report['offline_validation_only'] and not report['gpu_executed']
    assert not report['samples'] and report['device'] == 'not_queried'
    for field in ('qualified', 'unique_delivery', 'frame_age_qualified',
                  'capture_qualified', 'full_geometry_qualified',
                  'throughput_qualified', 'performance_acceptance', 'full_station_qualified'):
        assert report[field] is False
    assert report['requested_poll_sleep_us'] == 50
    old = out.read_bytes()
    invoke(offline_binary, fixture, shaders, out, success=False)
    assert out.read_bytes() == old
    invoke(offline_binary, fixture, shaders, tmp_path/'no-gpu.json',
           mode='--run', success=False)


def test_configuration_and_fixture_corruption(offline_binary, tmp_path):
    fixture, shaders = fixture_inputs(tmp_path)
    original = fixture.read_bytes()
    for key, value in [('--iterations', 0), ('--iterations', 513),
                       ('--trials', 0), ('--trials', 9), ('--warmup', 33),
                       ('--poll-us', 0), ('--poll-us', 10001),
                       ('--cpu-budget-ms', 0), ('--cpu-budget-ms', 10001)]:
        invoke(offline_binary, fixture, shaders, tmp_path/'invalid.json',
               key, value, success=False)
    variants = [b'bad', original[:-1], original+b'extra']
    for offset, value in [(8, struct.pack('<I', 641)), (16, struct.pack('<Q', 0)),
                          (24, struct.pack('<d', float('nan'))),
                          (40, struct.pack('<d', float('inf'))),
                          (184, struct.pack('<d', float('nan')))]:
        changed = bytearray(original); changed[offset:offset+len(value)] = value
        variants.append(changed)
    for data in variants:
        fixture.write_bytes(data)
        invoke(offline_binary, fixture, shaders, tmp_path/'bad-fixture.json', success=False)


def test_actual_module_cpu_synthetic_loop(tmp_path):
    binary = build_synthetic_benchmark(tmp_path)
    fixture, shaders = fixture_inputs(tmp_path)
    environment = dict(os.environ, GEOMETRY_BENCHMARK_EXTERNAL_WATCHDOG='1')
    report = invoke(binary, fixture, shaders, tmp_path/'synthetic.json',
                    '--iterations', 4, '--trials', 2, '--warmup', 2,
                    mode='--run', environment=environment)
    assert report['synthetic_driver'] and not report['gpu_executed']
    assert len(report['samples']) == 8 and len(report['trial_samples']) == 2
    assert all(s['valid'] == s['support'] == 0 for s in report['samples'])
    assert all(s['submit_to_finish_ns'] > 0 and s['process_cpu_ns'] > 0 for s in report['samples'])
    assert all(s['sleep_calls'] >= 1 and s['observed_sleep_wall_ns'] >= 50000 for s in report['samples'])
    assert all(t['cpp_new_calls'] == t['station_buffer_allocations'] == 0 for t in report['trial_samples'])
    invoke(binary, fixture, shaders, tmp_path/'wrong-count.json', '--warmup', 0,
           mode='--run', environment=environment, expected_valid=1, success=False)
    stalled = dict(environment, BENCH_FAKE_STALL='1')
    result = invoke(binary, fixture, shaders, tmp_path/'stalled.json', mode='--run',
                    environment=stalled, success=False)
    assert result.returncode == 2 and 'pending benchmark GPU work' in result.stderr
    native = tmp_path/'native'
    native.mkdir()
    native_fixture, native_shaders = fixture_inputs(native, '--native-float64')
    native_report = invoke(binary, native_fixture, native_shaders, native/'native.json',
                           '--iterations', 2, '--trials', 1, '--warmup', 1,
                           mode='--run', precision='--native-float64', environment=environment)
    assert native_report['precision'] == 'native_float64'
    invoke(binary, native_fixture, native_shaders, native/'no-f64.json',
           mode='--run', precision='--native-float64',
           environment=dict(environment, BENCH_FAKE_NO_F64='1'), success=False)
    no_feature = dict(environment, BENCH_FAKE_NO_INT64='1')
    invoke(binary, fixture, shaders, tmp_path/'no-feature.json', mode='--run',
           environment=no_feature, success=False)


def test_independent_two_fixture_factory(tmp_path):
    from tools.performance import prepare_geometry_benchmark as factory
    first, second = tmp_path/'first', tmp_path/'second'
    a, b = factory.prepare(first), factory.prepare(second)
    assert list(a['cases']) == ['all_valid', 'full_xyz_rotation']
    for name, case in a['cases'].items():
        assert (case['width'], case['height']) == (640, 480)
        assert (case['expected_valid'], case['expected_support']) == (307200, 4800)
        assert (first/(name+'.bin')).read_bytes() == (second/(name+'.bin')).read_bytes()
        assert case['expected_sha256'] == b['cases'][name]['expected_sha256']
    rotation = np.load(first/'full_xyz_rotation.npz')['transform'][:3, :3]
    assert np.all(rotation != 0) and np.all(np.abs(rotation) != 1)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), rtol=0, atol=1e-15)
    assert abs(np.linalg.det(rotation)-1) < 1e-15
    with pytest.raises(FileExistsError):
        factory.prepare(first)
