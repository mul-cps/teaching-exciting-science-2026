from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_upstream_manifest_pins_stack_and_recursive_driver():
    manifest = yaml.safe_load((ROOT / "ros2/upstream.repos").read_text())
    repo = manifest["repositories"]["so101-ros-physical-ai"]
    assert repo["type"] == "git"
    assert repo["version"] == "58318c905a2c61289fa907de85cb8473322fbe68"
    assert repo["url"] == "https://github.com/legalaspro/so101-ros-physical-ai.git"
    assert "4c0fdbfe16c84c686f8ace09526c52d98d0110ca" in (ROOT / "ros2/upstream.repos").read_text()


def test_rig01_export_is_joint_identity_only_and_provenanced():
    mapping = yaml.safe_load((ROOT / "ros2/config/rig01_joint_map.yaml").read_text())
    assert mapping["calibration_sha256"] == "bc688147848f386899f8f34eb09cdb285fcdb0fc8de576b78a50a8efaf7f424c"
    assert mapping["read_only"] is True
    assert mapping["qualified"] is False
    assert mapping["zero_source"] == "midpoint_of_calibration_range_min_and_range_max"
    assert mapping["radians_per_raw_count"] == 2 * 3.141592653589793 / 4095
    assert {joint: values["servo_id"] for joint, values in mapping["joints"].items()} == {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }
    assert {joint: values["software_zero_raw"] for joint, values in mapping["joints"].items() if "software_zero_raw" in values} == {
        "shoulder_pan": 2107.5,
        "shoulder_lift": 2151.5,
        "elbow_flex": 1951.5,
        "wrist_flex": 1995.5,
        "wrist_roll": 2047.5,
    }
    assert mapping["joints"]["gripper"]["mapping"] == "normalized_fraction_only"
    assert mapping["joints"]["gripper"]["mapping_qualified"] is False
    assert not any(key in mapping for key in ("homing_offset", "range_min", "range_max", "eeprom"))


def test_mock_moveit_overlay_never_defaults_to_real_hardware():
    launch = (ROOT / "ros2/src/so101_station_bringup/launch/mock_moveit.launch.py").read_text()
    assert '"hardware_type": "mock"' in launch
    assert "follower_moveit_demo.launch.py" in launch
    assert "serial" not in launch.lower()


def test_wrapper_exposes_mock_rviz_and_plan_only_by_default():
    wrapper = (ROOT / "ros2/scripts/so101.sh").read_text()
    assert "  rviz)" in wrapper
    assert "hardware_type:=mock" in wrapper or "mock_moveit.launch.py" in wrapper
    assert '"$RUNTIME" run --' in wrapper
    assert "--hardware" not in wrapper


def test_real_hardware_stays_blocked_after_patch_until_station_map_review():
    wrapper = (ROOT / "ros2/scripts/so101.sh").read_text()
    assert '  real)' in wrapper
    assert "real startup blocked" in wrapper
    assert "unqualified Rig01 mapping" in wrapper
    assert "read_only mode is not a motion authorization" in wrapper


def test_moveit_limits_use_station_bound_and_jerk_smoothing_after_totg():
    limits = yaml.safe_load((ROOT / "ros2/config/rig01_moveit_limits.yaml").read_text())
    for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
        item = limits["joint_limits"][joint]
        assert item["min_position"] == -1.57079632679
        assert item["max_position"] == 1.57079632679
        assert item["has_jerk_limits"] is True
        assert item["max_jerk"] > 0
    assert "rig01_moveit_limits.yaml" in (ROOT / "ros2/scripts/so101.sh").read_text()
    assert "AddTimeOptimalParameterization" in (ROOT / "ros2/scripts/so101.sh").read_text()
    assert "AddRuckigTrajectorySmoothing" in (ROOT / "ros2/scripts/so101.sh").read_text()


def test_plan_smoke_persists_trajectory_without_execution():
    smoke = (ROOT / "ros2/src/so101_station_bringup/scripts/plan_only_smoke.py").read_text()
    launch = (ROOT / "ros2/src/so101_station_bringup/launch/plan_only_smoke.launch.py").read_text()
    assert "message_to_yaml" in smoke
    assert "plan-only-smoke.yaml" in smoke
    assert "robot.execute" not in smoke
    assert ".plan()" in smoke
    assert 'ik_state.set_from_ik(group, target_pose, tip, 0.5)' in smoke
    assert "arm.set_goal_state(robot_state=ik_state)" in smoke
    assert "len(points) < 2" in smoke
    assert "planning_scene.is_state_valid(start_state, group)" in smoke
    assert "START STATE VALID" in smoke
    assert "MoveItConfigsBuilder" in launch
    assert "parameters=[moveit_config.to_dict()]" in launch
    assert "plan_only_smoke.launch.py" in (ROOT / "ros2/scripts/so101.sh").read_text()


def test_servo_launch_uses_jazzy_api_and_mock_controller_topic():
    servo = yaml.safe_load((ROOT / "ros2/config/servo.yaml").read_text())["moveit_servo"]
    assert servo["move_group_name"] == "manipulator"
    assert servo["command_out_topic"] == "/follower/arm_trajectory_controller/joint_trajectory"
    assert servo["joint_topic"] == "/follower/joint_states"
    assert servo["publish_period"] < servo["max_expected_latency"] / 3
    launch = (ROOT / "ros2/src/so101_station_bringup/launch/servo_mock.launch.py").read_text()
    assert "robot_description_kinematics" in launch
    assert '"hardware_type": "mock"' in launch
    assert "servo_mock.launch.py" in (ROOT / "ros2/scripts/so101.sh").read_text()


def test_bringup_package_matches_ament_cmake_build_files():
    package = (ROOT / "ros2/src/so101_station_bringup/package.xml").read_text()
    cmake = (ROOT / "ros2/src/so101_station_bringup/CMakeLists.txt").read_text()
    assert "<buildtool_depend>ament_cmake</buildtool_depend>" in package
    assert "<build_type>ament_cmake</build_type>" in package
    assert "ament_package()" in cmake


def test_live_bringup_declares_perception_runtime_dependency_and_installs_charuco_plane():
    package = (ROOT / "ros2/src/so101_station_bringup/package.xml").read_text()
    perception_cmake = (ROOT / "ros2/src/so101_station_perception/CMakeLists.txt").read_text()
    launch = (ROOT / "ros2/src/so101_station_bringup/launch/station_live.launch.py").read_text()
    assert "<exec_depend>so101_station_perception</exec_depend>" in package
    assert "ament_python_install_package(${PROJECT_NAME})" in perception_cmake
    assert 'include(\'so101_station_perception\', \'perception.launch.py\'' in launch
    assert "include('so101_station_control', 'moveit_resident.launch.py'" in launch


def test_live_station_keeps_measured_home_gate_and_single_camera_owner():
    launch = (ROOT / "ros2/src/so101_station_bringup/launch/station_live.launch.py").read_text()
    perception = (ROOT / "ros2/src/so101_station_perception/launch/perception.launch.py").read_text()
    control = (ROOT / "ros2/src/so101_station_control/so101_station_control/trajectory_server.py").read_text()
    assert launch.count("'perception.launch.py'") == 1
    assert launch.count("'leader_teleop.launch.py'") == 1
    assert '"camera_name": "camera"' in perception
    assert "_home" in control and "_saved_home_pose" in control
    assert "home_error" in control


def test_live_launch_forwards_qualified_gripper_calibration_and_starts_scene_updater():
    resident = (ROOT / "ros2/src/so101_station_control/launch/moveit_resident.launch.py").read_text()
    live = (ROOT / "ros2/src/so101_station_bringup/launch/station_live.launch.py").read_text()
    scene = (ROOT / "ros2/src/so101_station_scene/launch/station_scene.launch.py").read_text()

    assert 'DeclareLaunchArgument("gripper_jaw_calibration", default_value=""' in resident
    assert '"gripper_jaw_calibration": LaunchConfiguration("gripper_jaw_calibration")' in resident
    assert 'DeclareLaunchArgument("gripper_action_name"' in resident
    assert "'gripper_jaw_calibration': ''" in live
    assert "'gripper_action_name', 'gripper_jaw_calibration'" in live
    assert "'so101_station_scene', 'station_scene.launch.py'" in live
    assert "'use_planning_scene': 'true'" in live
    assert "executable=\"scene_updater\"" in scene
