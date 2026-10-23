"""CPU-only real Pipeline submission with a synthetic Vulkan driver, never a GPU."""
import re
import shutil
import subprocess
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def test_actual_device_depth_submission_host_contract(tmp_path):
    compiler = shutil.which('g++')
    assert compiler, 'g++ is required for the CPU-only driver test'
    shader_dir = tmp_path / 'shaders'
    shader_dir.mkdir()
    for name in ('convert', 'candidates', 'connect', 'compress', 'reduce', 'gather', 'morphology', 'tiled_components'):
        (shader_dir / (name + '.spv')).write_bytes(bytes(4))
    binary = tmp_path / 'device_depth_host_test'
    subprocess.run([compiler, '-std=c++17', '-O2', '-pthread', '-I' + str(PACKAGE / 'include'),
                    str(PACKAGE / 'test/device_depth_host_test.cpp'),
                    *[str(PACKAGE / ('src/' + name + '.cpp')) for name in ('gpu_pipeline', 'cpu_pipeline', 'frame_pool')],
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    result = subprocess.run([str(binary), str(shader_dir)], check=True, capture_output=True, text=True)
    assert 'hot_allocations=0 waits=0' in result.stdout


def test_borrowed_device_depth_never_reads_maps_allocates_or_waits():
    source = (PACKAGE / 'src/gpu_pipeline.cpp').read_text()
    start = source.index('void Pipeline::submit(')
    end = source.index('void Pipeline::finish(', start)
    hot = source[start:end]
    assert not re.search(r'vk(?:DeviceWaitIdle|QueueWaitIdle|WaitForFences|GetFenceStatus|Allocate\w*|Create\w*|MapMemory)\s*\(', hot)
    assert not re.search(r'\b(?:new|malloc|calloc|realloc|memcpy)\b|\.(?:resize|reserve|push_back|emplace_back)\s*\(', hot)
    assert 'packed_z16->mapped' not in hot
    assert 'submit_device_depth' in hot and 'submit_impl' in hot
