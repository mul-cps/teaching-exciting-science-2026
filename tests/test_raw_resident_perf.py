"""Independent raw-resident hot-path review; CPU fake driver only."""
from pathlib import Path
import json
import os
import re
import shutil
import subprocess
import pytest

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def body(source, signature):
    begin = source.index('{', source.index(signature))
    depth = 1
    for end in range(begin + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[begin + 1:end]
    raise AssertionError('unbalanced body')


@pytest.mark.parametrize('profile,color_n,depth_n', [
    ('vga', 640*480, 640*480),
    ('720p', 1280*720, 1280*720),
    ('1080p', 1920*1080, 1280*720),
])
def test_actual_combined_modules_with_synthetic_vulkan(tmp_path, profile, color_n, depth_n):
    sdk = os.environ.get('RAW_BRIDGE_SDK_PREFIX')
    vulkan = os.environ.get('RAW_BRIDGE_VULKAN_INCLUDE')
    if not sdk or not vulkan:
        pytest.skip('set RAW_BRIDGE_SDK_PREFIX and RAW_BRIDGE_VULKAN_INCLUDE for CPU native test')
    compiler = os.environ.get('CXX') or shutil.which('c++')
    assert compiler
    shaders = tmp_path / 'shaders'
    shaders.mkdir()
    for entry in ('convert', 'candidates', 'connect', 'compress', 'reduce', 'gather', 'morphology', 'tiled_components', 'splat', 'finalize'):
        (shaders / (entry + '.spv')).write_bytes(b'\x03\x02\x23\x07')
    binary = tmp_path / 'raw_resident_perf'
    camera = (P / 'src/camera_component.cpp').read_text()
    (tmp_path / 'camera_metadata_shared.h').write_text('{' + body(camera, 'void original_metadata(') + '}')
    sources = ['test/raw_resident_perf_benchmark.cpp', *['src/' + name + '.cpp' for name in ('gpu_pipeline', 'cpu_pipeline', 'frame_pool', 'alignment', 'alignment_reference', 'raw_capture_bridge', 'raw_camera_admission')]]
    subprocess.run([compiler, '-O2', '-std=c++17', '-pthread', '-ffp-contract=off', '-I', str(tmp_path), '-I', str(P / 'include'), '-I', vulkan, '-I', sdk + '/include', *[str(P / source) for source in sources], '-L', sdk + '/lib', '-Wl,-rpath,' + sdk + '/lib', '-lrealsense2', '-o', str(binary)], check=True, capture_output=True, text=True)
    # Omit the optional argument for VGA to retain the original CLI contract.
    arguments = [str(binary), str(shaders), str(shaders)] + ([] if profile == 'vga' else [profile])
    report = json.loads(subprocess.check_output(arguments, text=True))
    if destination := os.environ.get('RAW_RESIDENT_PERF_ARTIFACTS'):
        destination = Path(destination) / profile
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(binary, destination / binary.name)
        shutil.copy2(tmp_path / 'camera_metadata_shared.h', destination / 'camera_metadata_shared.h')
        shutil.copytree(shaders, destination / 'synthetic-shaders', dirs_exist_ok=True)
        (destination / 'microbenchmark.json').write_text(json.dumps(report, indent=2) + '\n')
    assert report['hot_heap_allocations'] == report['hot_buffer_allocations'] == report['hot_wait_calls'] == 0
    assert report['profile'] == profile
    assert report['color_pixels'] == color_n and report['depth_pixels'] == depth_n
    assert report['iterations'] == 100000 and report['warmup_iterations'] >= 1000
    assert report['host_ns_per_cycle'] > 0
    assert report['rgb_upload_bytes'] == 3*color_n and report['raw_upload_bytes'] == 2*depth_n
    assert report['aligned_d2d_bytes'] == 2*color_n
    if profile == 'vga':
        assert report['rgb_upload_bytes'] == 921600 and report['raw_upload_bytes'] == 614400
        assert report['aligned_d2d_bytes'] == 614400
    assert report['alignment_readback_bytes'] == 4 and report['core_readback_bytes'] == 163856
    assert report['ordered_submissions'] == 2 and report['distinct_command_pools']
    assert report['source_retained_through_consumer_fence']
    assert report['actual_source_core_faults_rejected'] and report['actual_failure_shutdown_consumer_first']
    print(json.dumps(report, sort_keys=True))


def test_combined_normal_path_has_no_new_wait_allocation_mapping_or_dense_download():
    bridge = (P / 'src/raw_capture_bridge.cpp').read_text()
    core = (P / 'src/gpu_pipeline.cpp').read_text()
    for hot in [body(bridge, 'RawCaptureBridge::' + name + '(') for name in ('acquire', 'validate', 'offer', 'dispatch_pending', 'collect')] + [body(core, 'Pipeline::submit_impl(')]:
        assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b|make_shared|make_unique|std::vector|\.(?:resize|reserve|create)\(|vk(?:Wait|Map|Allocate|Create)|vk(?:Queue|Device)WaitIdle|std::hash', hot)
    dispatch = body(bridge, 'RawCaptureBridge::dispatch_pending(')
    assert dispatch.index('depth_->submit') < dispatch.index('core_->submit_device_depth')
    assert not re.search(r'ready\(|vkGetFenceStatus', dispatch)
    collect = body(bridge, 'RawCaptureBridge::collect(')
    assert collect.index('core_->ready') < collect.index('depth_->ready') < collect.index('core_->finish') < collect.index('queue_.take_completed')
    assert 'failed_=true;throw;' in collect


def test_camera_normal_raw_path_uses_batched_copies_and_reserved_status():
    camera = (P / 'src/camera_component.cpp').read_text()
    for name in ('tick', 'capture_pending', 'copy_raw_planes', 'copy_rows', 'inspect_original', 'verify_original', 'original_metadata', 'report_status'):
        hot = body(camera, name + '(')
        assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b|make_shared|make_unique|std::vector|std::to_string|\.(?:resize|reserve)\(|vk(?:Wait|Map|Allocate|Create)|diagnostic_|camera_generation\(|probe_loaded_sdk_alignment\(|vendor_alignment_\.process', hot), name
    copied = body(camera, 'copy_rows(')
    assert copied.count('source.get_data()') == copied.count('source.get_stride_in_bytes()') == 1
    assert 'if(stride==int(row_bytes))' in copied
    assert 'size_t(row_bytes)*unsigned(height)' in copied
    assert 'y<unsigned(height)' in copied
    report = body(camera, 'report_status(')
    assert 'char text[4096]' in report and 'std::snprintf' in report
    assert 'status_message_.data.assign' in report and 'static_cast<const std_msgs::msg::String&>' in report
    assert 'status_message_.data.reserve(4096)' in camera
    assert 'm.serial=serial_' in body(camera, 'original_metadata(')
    make_bridge = body(camera, 'make_bridge(')
    assert 'std::array<RawFrame,kSlots> reserved' in make_bridge
    assert 'metadata.serial.reserve(serial_.size())' in make_bridge
    assert 'f.depth.metadata().serial.reserve(serial_.size())' in make_bridge
    capture = body(camera, 'capture_pending(')
    assert capture.index('verify_original') < capture.index('bridge_->offer')
    assert 'if(bridge_->gpu_alignment())copy_raw_planes' in capture
    assert 'else copy_vendor_planes' in capture
    failed = body(camera, 'latch_failure(')
    assert failed.index('timer_->cancel()') < failed.index('stop_owned_camera()')
    assert 'publication_gate().invalidate(this' in failed
    assert 'now()' not in failed


def test_cold_selfcheck_is_distinct_bounded_and_retains_original_raw_view():
    camera = (P / 'src/camera_component.cpp').read_text()
    cold = body(camera, 'cold_live_selfcheck(')
    assert cold.index('raw_reference=frame.depth.upload()') < cold.index('cold->dispatch_pending()')
    assert 'alignment_reference(source_.alignment,raw_reference,' in cold
    assert 'cpu.metadata=metadata;cpu.calibration=core_calibration()' in cold
    assert 'pass<3' in cold and 'std::chrono::steady_clock::now()+10s' in cold
    assert 'gate.require_complete()' in cold
    admission = (P / 'src/raw_camera_admission.cpp').read_text()
    verify = body(admission, 'ColdCameraSelfcheck::verify(')
    assert 'passes_>=3' in verify and 'failed_=true;throw;' in verify
    assert 'std::vector' not in verify
    assert '>1e-7f' in verify and 'std::isfinite' in verify
    assert 'aligned[i]!=vendor||reference[i]!=vendor' in verify
    assert 'gpu.cpu_components[i]==cpu.cpu_components[i]' in verify


def test_publication_generation_is_locked_through_publish_and_failure():
    header = (P / 'include/so101_station_vulkan/raw_camera_admission.hpp').read_text()
    publish = body(header, 'bool publish(')
    invalidate = body(header, 'void invalidate(const void* owner,Publish&&')
    assert publish.index('std::lock_guard<std::mutex>') < publish.index('generation_!=generation') < publish.index('publish_current()')
    assert '!owner_||!generation' in publish
    assert invalidate.index('std::lock_guard<std::mutex>') < invalidate.index('if(!owner||owner_!=owner)return') < invalidate.index('generation_=0') < invalidate.index('publish_unknown()')
    admission = (P / 'src/raw_camera_admission.cpp').read_text()
    release = body(admission, 'ResidentPublicationGate::release(')
    assert 'if(owner_==owner)' in release and 'generation_=0;owner_=nullptr' in release
    camera = (P / 'src/camera_component.cpp').read_text()
    assert camera.index('ResidentCameraOwner ownership_') < camera.index('rs2::context cameras_')
    assert 'publication_gate().publish(slot.metadata.calibration_generation' in camera
    cold = body(camera, 'cold_live_selfcheck(')
    assert 'catch(...){publication_gate().invalidate(this);stop_owned_camera();throw;}' in cold
    verify = body(admission, 'ColdCameraSelfcheck::verify(')
    assert 'native_nonzero<24||aligned_nonzero<24' in verify
    assert 'gpu.component_count>kMaxComponents||cpu.component_count>kMaxComponents' in verify
