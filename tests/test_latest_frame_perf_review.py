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
    for field,source,method in (
        ('frame_number','original_color','get_frame_number'),
        ('depth_frame_number','original_depth','get_frame_number'),
        ('hardware_timestamp_ms','original_color','get_timestamp'),
        ('depth_hardware_timestamp_ms','original_depth','get_timestamp'),
        ('timestamp_domain','original_color','get_frame_timestamp_domain'),
        ('depth_timestamp_domain','original_depth','get_frame_timestamp_domain'),
    ):
        assert re.search(r'slot\.metadata\.'+field+r'\s*=\s*(?:int\()?'+source+r'\.'+method+r'\(\)',capture)
    assert 'slot.metadata.timestamp_ns = received_ros_ns;' in capture
    assert 'now()' not in capture
    assert capture.index('accept_source(')<capture.index('discard_pending(')<capture.index('compute_->acquire(')<capture.index('alignment_.process(')
    assert tick.index('compute_->finish(')<tick.index('poll_for_frames(')<tick.index('capture_pending(')<tick.index('dispatch_pending(')
    assert 'return;' not in tick
    assert camera.index('strict Vulkan camera refused')<camera.index('camera_.start(')
    # Both node callbacks retain their default mutually exclusive groups.
    assert 'Reentrant' not in camera and 'create_callback_group' not in camera
