import sys
from pathlib import Path

import pytest

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "ros2/src/so101_station_objects"))

geometry_msgs = pytest.importorskip("geometry_msgs.msg")
visualization_msgs = pytest.importorskip("visualization_msgs.msg")
vision_msgs = pytest.importorskip("vision_msgs.msg")

from so101_station_objects.objects import elevated_pose_copy  # noqa: E402


def test_raised_label_pose_does_not_mutate_detection_or_box_marker():
    detection = vision_msgs.Detection3D()
    detection.bbox.center.position.z = 0.002
    detection.bbox.size.z = 0.004
    hypothesis = vision_msgs.ObjectHypothesisWithPose()
    hypothesis.pose.pose = detection.bbox.center
    detection.results.append(hypothesis)
    box = visualization_msgs.Marker()
    box.pose = detection.bbox.center

    label = visualization_msgs.Marker()
    label.pose = elevated_pose_copy(detection.bbox.center,
                                    detection.bbox.size.z / 2 + 0.015)

    assert detection.bbox.center.position.z == pytest.approx(0.002)
    assert detection.results[0].pose.pose.position.z == pytest.approx(0.002)
    assert box.pose.position.z == pytest.approx(0.002)
    assert label.pose.position.z == pytest.approx(0.019)
    assert detection.bbox.center.position.z + detection.bbox.size.z / 2 == pytest.approx(0.004)
    assert label.pose.position.z > detection.bbox.center.position.z + detection.bbox.size.z / 2
