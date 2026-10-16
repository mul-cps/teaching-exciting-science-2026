"""Actual ROS producer and station-venv owner recovery then measured home."""
import pytest
pytest.importorskip('rclpy')
from test_ros_recovery_producer_native import test_native_ros_depth_reaches_actual_station_owner as run_fixture


def test_actual_owner_prepare_recovers_homes_and_confirms_readiness(tmp_path,monkeypatch):
    run_fixture(tmp_path,monkeypatch,'arm_prepare_required')
