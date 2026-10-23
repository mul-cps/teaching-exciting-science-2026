"""Source wiring for the separately CPU-tested bounded frame queue."""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2/src/so101_station_vulkan/src/camera_component.cpp'

def body(text,name):
    start=text.index('{',text.index(name));depth=1;i=start+1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[start+1:i-1]

def test_newest_arrival_is_considered_before_pending_dispatch():
    text=SOURCE.read_text();tick=body(text,'void tick()')
    assert 'LatestFrameQueue<FrameLease> queue_' in text
    assert tick.index('poll_for_frames') < tick.index('dispatch_pending();')
    assert 'capture_pending(native, received_ros_ns)' in tick
    assert tick.index('now().nanoseconds()') < tick.index('capture_pending(')
    assert 'return;' not in tick[:tick.index('dispatch_pending();')]

def test_original_source_metadata_and_truthful_age_remain_visible():
    text=SOURCE.read_text();capture=body(text,'void capture_pending(')
    assert 'slot.metadata.timestamp_ns = received_ros_ns;' in capture
    assert 'slot.metadata.timestamp_ns = uint64_t(now()' not in capture
    for field in ('frame_number','depth_frame_number','hardware_timestamp_ms',
                  'depth_hardware_timestamp_ms','timestamp_domain','depth_timestamp_domain'):
        original='original_depth' if field.startswith('depth_') else 'original_color'
        line=next(line for line in capture.splitlines() if 'slot.metadata.'+field+' =' in line)
        assert original+'.get_' in line
    helper=(ROOT/'ros2/src/so101_station_vulkan/include/so101_station_vulkan/latest_frame_queue.hpp').read_text()
    assert 'format_unqualified_measurement(measurement_.data' in text
    assert 'static_cast<const std_msgs::msg::String &>(measurement_)' in text
    for field in ('source_received_ros_ns','received_age_ms','hardware_timestamp_ms',
                  'depth_hardware_timestamp_ms','duplicate_or_regressed'):
        assert field in text+helper
    assert 'strict Vulkan camera refused' in text
    assert 'geometry_admission=unqualified; execution_eligible=false' in helper
