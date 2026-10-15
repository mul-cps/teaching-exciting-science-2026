import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS integration tests require the RoboStack environment")

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "ros2/src/so101_station_perception"))


def test_shared_frame_dispatch_decodes_once_and_preserves_stamp_and_gray_identity():
    from so101_station_perception.marker_pipeline import dispatch_shared_gray

    gray = np.zeros((480, 640), dtype=np.uint8)

    class Bridge:
        calls = 0

        def imgmsg_to_cv2(self, message, desired_encoding):
            assert desired_encoding == "mono8"
            self.calls += 1
            return gray

    class Stamp:
        sec = 123
        nanosec = 456

    class Message:
        header = type("Header", (), {"stamp": Stamp(), "frame_id": "camera_color_optical_frame"})()

    bridge = Bridge()
    color, depth, info = Message(), Message(), Message()
    calls = []
    metrics = dispatch_shared_gray(
        color, depth, info, bridge,
        lambda image, camera_info, prepared_gray: calls.append(("board", image, camera_info, prepared_gray)),
        lambda image, aligned_depth, prepared_gray: calls.append(("tags", image, aligned_depth, prepared_gray)),
    )
    assert bridge.calls == 1
    assert calls[0] == ("board", color, info, gray)
    assert calls[1] == ("tags", color, depth, gray)
    assert metrics["capture_stamp"] == pytest.approx(123.000000456)
    assert metrics["decode_ms"] >= 0 and metrics["board_ms"] >= 0 and metrics["tags_ms"] >= 0
