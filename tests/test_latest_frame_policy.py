"""Source wiring for the separately CPU-tested bounded frame queue."""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2/src/so101_station_vulkan/src/camera_component.cpp'

def body(text,name):
    start=text.index('{',text.index(name));depth=1;i=start+1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[start+1:i-1]

def test_newest_native_arrival_is_selected_only_when_worker_idle():
    text=SOURCE.read_text();capture=body(text,'void capture_pending(');worker=body(text,'void worker_loop(')
    assert 'LatestNativeScheduler<rs2::frameset,FrameMetadata> scheduler_' in text
    assert capture.index('verify_original(native)') < capture.index('source_admission_.observe(last_input_)') < capture.index('scheduler_.accept(')
    assert 'bridge_->' not in capture and 'copy_raw_planes(' not in capture and 'copy_vendor_planes(' not in capture
    assert worker.index('bridge_->collect(') < worker.index('scheduler_.take(') < worker.index('bridge_->dispatch_pending();')
    assert worker.index('frames_->publish(') < worker.index('scheduler_.complete(')
    assert 'std::thread([this]{capture_loop();})' in text and 'std::thread([this]{worker_loop();})' in text
    tick=body(text,'void tick()');assert 'poll_for_frames' not in tick and 'bridge_->' not in tick

def test_original_source_metadata_and_truthful_age_remain_visible():
    text=SOURCE.read_text();capture=body(text,'void capture_pending(')
    metadata=body(text,'void original_metadata(')
    assert 'm.timestamp_ns=receipt;' in metadata
    assert 'now()' not in metadata and 'now()' not in capture
    assert capture.index('original_metadata(last_input_,native,received_ros_ns)') < capture.index('scheduler_.accept(')
    assert 'publication_session_token=publication_session_token_' in metadata
    for field in ('frame_number','depth_frame_number','hardware_timestamp_ms',
                  'depth_hardware_timestamp_ms','timestamp_domain','depth_timestamp_domain'):
        original='depth' if field.startswith('depth_') else 'color'
        assignment=metadata.split('m.'+field+'=',1)[1].split(';',1)[0]
        assert original+'.get_' in assignment
    assert 'auto color=native.get_color_frame();auto depth=native.get_depth_frame();' in metadata
    helper=(ROOT/'ros2/src/so101_station_vulkan/include/so101_station_vulkan/latest_frame_queue.hpp').read_text()
    assert 'format_unqualified_measurement(measurement_.data' in text
    assert 'static_cast<const std_msgs::msg::String &>(measurement_)' in text
    for field in ('source_received_ros_ns','received_age_ms','hardware_timestamp_ms',
                  'depth_hardware_timestamp_ms','duplicate_or_regressed'):
        assert field in text+helper
    constructor=body(text,'explicit CameraComponent(')
    assert constructor.index('probe_loaded_sdk_alignment(') < constructor.index('raw_admission_failure(') < constructor.index('camera_.start(')
    assert constructor.index('require_raw_or_cpu_fallback(mode,fallback_reason_)') < constructor.index('camera_.start(')
    assert constructor.index('cold_live_selfcheck(') < constructor.index('publication_gate().admit(') < constructor.index('create_wall_timer(')
    raw=body(text,'void copy_raw_planes(')
    assert 'vendor_alignment_' not in raw and '.process(' not in raw
    assert 'geometry_admission=unqualified; execution_eligible=false' in text
    assert 'geometry_admission=unqualified; execution_eligible=false' in helper
