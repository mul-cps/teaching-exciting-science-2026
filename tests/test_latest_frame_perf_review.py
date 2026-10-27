"""Independent source/bounds checks; native test exercises actual CPU leases."""
from pathlib import Path
import re

PACKAGE=Path(__file__).resolve().parents[1]/'ros2/src/so101_station_vulkan'


def body(text,symbol):
    start=text.index('{',text.index(symbol));end=start+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start+1:end-1]


def test_formatter_has_a_static_whole_output_bound_with_no_per_frame_reserve():
    text=(PACKAGE/'include/so101_station_vulkan/latest_frame_queue.hpp').read_text()
    fmt=body(text,'void format_unqualified_measurement(')
    number=body(text,'void measurement_append_number(')
    capacity=int(re.search(r'kMeasurementTextCapacity=(\d+)',text).group(1))
    buffer=int(re.search(r'char buffer\[(\d+)\]',number).group(1))
    # All numeric writes fit the fixed scratch range. Count BOTH optional-age
    # branches in this conservative bound; even that sum must fit one reserve.
    literals=re.findall(r'text\+="([^"\\]*)"',fmt)
    fixed=sum(len(s) for s in literals)
    fields=fmt.count('measurement_append_number(')
    assert fields==10 and buffer==64 and fixed+fields*buffer < capacity==1024
    assert 'text.size()+length>text.capacity()' in number
    assert 'text.capacity()<kMeasurementTextCapacity' in fmt
    assert 'text.clear()' in fmt
    for code in (fmt,number):
        assert not re.search(r'\.reserve\(|std::to_string\(|ostringstream|\b(?:malloc|calloc|realloc|new|sleep|wait|open)\b',code)
    camera=(PACKAGE/'src/camera_component.cpp').read_text()
    assert camera.count('measurement_.data.reserve(kMeasurementTextCapacity)')==1
    assert 'measurements_->publish(static_cast<const std_msgs::msg::String &>(measurement_))' in camera


def test_accepted_source_identity_is_original_and_receipt_precedes_alignment():
    camera=(PACKAGE/'src/camera_component.cpp').read_text()
    capture=body(camera,'void capture_pending(')
    tick=body(camera,'void tick()')
    metadata=body(camera,'void original_metadata(')
    for field,source,method in (
        ('frame_number','color','get_frame_number'),
        ('depth_frame_number','depth','get_frame_number'),
        ('hardware_timestamp_ms','color','get_timestamp'),
        ('depth_hardware_timestamp_ms','depth','get_timestamp'),
        ('timestamp_domain','color','get_frame_timestamp_domain'),
        ('depth_timestamp_domain','depth','get_frame_timestamp_domain'),
    ):
        assert re.search(r'm\.'+field+r'\s*=\s*(?:int\()?'+source+r'\.'+method+r'\(\)',metadata)
    assert 'auto color=native.get_color_frame();auto depth=native.get_depth_frame();' in metadata
    assert 'm.timestamp_ns=receipt;' in metadata
    assert 'now()' not in metadata
    assert capture.index('original_metadata(last_input_,native,received_ros_ns)') < capture.index('source_admission_.observe(') < capture.index('scheduler_.accept(')
    assert 'now()' not in capture and 'bridge_->' not in capture
    polling=body(camera,'void capture_loop(');worker=body(camera,'void worker_loop(')
    assert polling.index('poll_for_frames(')<polling.index('now().nanoseconds()')<polling.index('capture_pending(')
    assert 'copy_raw_planes' not in polling and 'frames_->publish' not in polling
    assert worker.index('copy_raw_planes(')<worker.index('bridge_->dispatch_pending(')
    assert worker.index('copy_vendor_planes(')<worker.index('bridge_->dispatch_pending(')
    assert worker.index('frames_->publish(')<worker.index('scheduler_.complete(')
    constructor=body(camera,'explicit CameraComponent(')
    assert constructor.index('probe_loaded_sdk_alignment(')<constructor.index('raw_admission_failure(')<constructor.index('camera_.start(')
    assert constructor.index('require_raw_or_cpu_fallback(mode,fallback_reason_)')<constructor.index('camera_.start(')
    assert constructor.index('cold_live_selfcheck(')<constructor.index('publication_gate().admit(')<constructor.index('create_wall_timer(')
    objects=body(camera,'class ObjectsComponent')
    assert objects.index('publication_gate().publish(')<objects.index('measurements_->publish(')
    # Both node callbacks retain their default mutually exclusive groups.
    assert 'Reentrant' not in camera and 'create_callback_group' not in camera
