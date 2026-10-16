"""Actual selected-placement executor path against synthetic ROS endpoints."""
import json
import os
import subprocess
import sys
import uuid
import numpy as np
import pytest
pytest.importorskip('rclpy')
from std_msgs.msg import String
from test_ros_grasp_cli_native import (_write_calibrations, _start_fixture, _cleanup,
    _communicate_cli, ROOT, TRACK_ID, FRAME_ID, TIP_LINK, TEST_ARM_ACTION, PLACE_POSE)


@pytest.mark.parametrize('area_width', [.30, .02])
def test_selected_area_executes_or_rejects_before_open(monkeypatch, tmp_path, area_width):
    if os.environ.get('ROS_DOMAIN_ID') != '75':
        pytest.skip('isolated native ROS domain 75 required')
    files, description = _write_calibrations(tmp_path)
    camera = json.loads(files['camera-arm.json'].read_text())
    camera['camera_frame'] = 'synthetic_camera'
    files['camera-arm.json'].write_text(json.dumps(camera))
    execution = json.loads(files['execution.json'].read_text())
    execution.update(placement_goal_from_tip=np.eye(4).tolist(), placement_footprint_m=[.08,.08],
                     placement_tool_footprint_m=[.08,.08], placement_clearance_m=.005,
                     placement_registration_uncertainty_m=.002)
    files['execution.json'].write_text(json.dumps(execution))
    fixture, bridge, executor, stop, spin_thread = _start_fixture(monkeypatch, tmp_path)
    fixture.set_robot_description(description)
    selected = str(uuid.uuid4())
    publisher = fixture.create_publisher(String, '/station/placement_goal_observation', 10)
    pose = np.eye(4)
    pose[:3, 3] = PLACE_POSE[:3]
    corners = np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]])*area_width/2+pose[:3,3]
    def publish():
        stamp = fixture.get_clock().now().nanoseconds
        message = String()
        message.data = json.dumps(dict(valid=True, state='tracked', selection_id=selected,
            frame_id='synthetic_camera', stamp_ns=stamp, camera_from_goal_stamp_ns=stamp,
            camera_from_goal=pose.tolist(), goal_corners_camera=corners.tolist(), fit_rms_m=.001,
            goal_geometry={'width_m': area_width, 'height_m': area_width},
            plane_metrics={'plane_rms_m': .001}))
        publisher.publish(message)
    fixture.create_timer(.04, publish)
    output = tmp_path/'selected-result.json'
    command = [sys.executable, str(ROOT/'tools/ros_plan_grasp.py'), '--execute', '--track-id', TRACK_ID,
        '--frame', FRAME_ID, '--tip-link', TIP_LINK, '--place-pose', *map(str, PLACE_POSE),
        '--execution-config', str(files['execution.json']), '--placement-selection', selected,
        '--trajectory-action', TEST_ARM_ACTION, '--timeout', '45', '--planning-time', '.5', '--output', str(output)]
    process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        stdout, _ = _communicate_cli(process, fixture, 18)
        if area_width == .30:
            assert process.returncode == 0, stdout
            assert json.loads(output.read_text())['state'] == 'pick_place_complete'
            assert fixture.plans[3]['position'] == pytest.approx(PLACE_POSE[:3])
        else:
            assert process.returncode != 0
            assert 'footprint' in stdout
            assert not fixture.gripper_commands
            assert fixture.arm_goal_count == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(3)
        _cleanup(fixture, bridge, executor, stop, spin_thread)
