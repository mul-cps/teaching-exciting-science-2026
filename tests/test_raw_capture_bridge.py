"""CPU-only contracts for the typed raw-camera ownership bridge."""
from pathlib import Path
import re
import os
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / 'ros2/src/so101_station_vulkan'


def body(source, signature):
    start = source.index('{', source.index(signature))
    level = 1
    for end in range(start + 1, len(source)):
        level += (source[end] == '{') - (source[end] == '}')
        if not level:
            return source[start + 1:end]
    raise AssertionError('unbalanced body')


def test_typed_lifetime_and_shutdown_order():
    header = (P / 'include/so101_station_vulkan/raw_capture_bridge.hpp').read_text()
    assert header.index('std::unique_ptr<Alignment> depth_') < header.index('LatestFrameQueue<RawFrame> queue_') < header.index('std::unique_ptr<Pipeline> core_')
    assert 'FrameLease core;' in header and 'AlignmentLease depth;' in header
    assert 'shared_ptr<void>' not in header
    source = (P / 'src/raw_capture_bridge.cpp').read_text()
    assert 'core_.reset();' in body(source, 'RawCaptureBridge::~RawCaptureBridge(')
    submit = body(source, 'RawCaptureBridge::dispatch_pending(')
    assert submit.index('validate(f)') < submit.index('depth_->submit(f.depth)') < submit.index('core_->submit_device_depth(f.core,f.depth.aligned())')
    assert 'failed_=true;throw;' in submit
    collect = body(source, 'RawCaptureBridge::collect(')
    assert collect.index('core_->ready') < collect.index('depth_->ready') < collect.index('core_->finish') < collect.index('queue_.take_completed')
    assert 'failed_=true;throw;' in collect
    validate = body(source, 'RawCaptureBridge::validate(')
    for field in ('width', 'height', 'frame_number', 'depth_frame_number', 'timestamp_ns', 'calibration_generation', 'hardware_timestamp_ms', 'depth_hardware_timestamp_ms', 'timestamp_domain', 'depth_timestamp_domain', 'publication_session_token', 'serial'):
        assert f'm.{field}' in validate
    assert 'core_->owns(f.core)' in validate and 'depth_->owns(f.depth)' in validate


def test_bounded_bridge_hot_path():
    source = (P / 'src/raw_capture_bridge.cpp').read_text()
    for method in ('acquire', 'validate', 'offer', 'dispatch_pending', 'collect'):
        hot = body(source, f'RawCaptureBridge::{method}(')
        assert not re.search(r'\b(?:new|malloc|calloc|realloc|memcpy|memmove)\b|make_shared|make_unique|std::vector|\.create\(|vk(?:Wait|Map|Allocate|Create)|vk(?:Queue|Device)WaitIdle|std::hash|\bwhile\s*\(|\bfor\s*\(', hot)
    assert 'Context::shared' not in source
    assert 'vkGetFenceStatus' not in body(source, 'RawCaptureBridge::dispatch_pending(')
    assert 'm.serial.clear()' in source  # preserve reserved serial capacity
    alignment = (P / 'src/alignment.cpp').read_text()
    acquire = body(alignment, 'Alignment::acquire(')
    assert 'm.serial.clear()' in acquire and 's.metadata={}' not in acquire


def test_actual_leases_with_synthetic_driver(tmp_path):
    """Opt in SDK include paths; this executable never links the Vulkan loader."""
    vulkan = os.environ.get('RAW_BRIDGE_VULKAN_INCLUDE')
    sdk = os.environ.get('RAW_BRIDGE_SDK_PREFIX')
    if not vulkan or not sdk:
        pytest.skip('set RAW_BRIDGE_VULKAN_INCLUDE and RAW_BRIDGE_SDK_PREFIX for CPU host build')
    compiler = os.environ.get('CXX') or shutil.which('c++')
    assert compiler
    shaders = tmp_path / 'shaders'
    shaders.mkdir()
    for entry in ('splat', 'finalize'):
        (shaders / f'{entry}.spv').write_bytes(b'\x03\x02\x23\x07')
    binary = tmp_path / 'raw_capture_bridge_host_test'
    sources = ['test/raw_capture_bridge_host_test.cpp', 'src/raw_capture_bridge.cpp', 'src/alignment.cpp', 'src/alignment_reference.cpp', 'src/frame_pool.cpp']
    subprocess.run([compiler, '-O2', '-std=c++17', '-ffp-contract=off', '-I', str(P / 'include'), '-I', vulkan, '-I', f'{sdk}/include', *(str(P / path) for path in sources), '-L', f'{sdk}/lib', f'-Wl,-rpath,{sdk}/lib', '-lrealsense2', '-o', str(binary)], check=True, capture_output=True, text=True)
    output = subprocess.check_output([str(binary), str(shaders)], text=True)
    assert 'hot_heap_allocations=0; hot_buffer_allocations=0; hot_waits=0' in output
    assert 'PASS' in output
    print(output)
