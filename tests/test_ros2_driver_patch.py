import os
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / "ros2/patches/feetech-read-only.patch"
EXPECTED_DRIVER_REV = "4c0fdbfe16c84c686f8ace09526c52d98d0110ca"
EXPECTED_UPSTREAM_REV = "58318c905a2c61289fa907de85cb8473322fbe68"


def driver_source() -> Path:
    candidates = [
        Path(os.environ.get("SO101_FEETECH_SOURCE", "/nonexistent")),
        ROOT / "ros2/.cache/upstream/so101-ros-physical-ai/feetech_ros2_driver",
        Path.home() / ".cache/so101-ros-upstream/feetech_ros2_driver",
    ]
    for candidate in candidates:
        if (candidate / "src/feetech_ros2_driver.cpp").is_file():
            return candidate
    pytest.skip("pinned Feetech source is not fetched; run so101.sh fetch first")


def cxx_function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for pos in range(opening, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : pos]
    raise AssertionError(f"unterminated function: {signature}")


def braced_block(source: str, marker: str) -> str:
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 0
    for pos in range(opening, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : pos]
    raise AssertionError(f"unterminated block: {marker}")


@pytest.fixture(scope="module")
def patched_source_root(tmp_path_factory: pytest.TempPathFactory) -> Path:
    source_dir = driver_source()
    source_root = source_dir.parent
    driver_revision = subprocess.run(
        ["git", "-C", str(source_dir), "rev-parse", "HEAD"],
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()
    upstream_revision = subprocess.run(
        ["git", "-C", str(source_root), "rev-parse", "HEAD"],
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()
    assert driver_revision == EXPECTED_DRIVER_REV
    assert upstream_revision == EXPECTED_UPSTREAM_REV

    copy = tmp_path_factory.mktemp("patched-so101-upstream") / "source"
    shutil.copytree(
        source_root,
        copy,
        ignore=shutil.ignore_patterns(".git", "build", "install", "log"),
    )

    # The shared upstream cache may already have this patch applied by the
    # workspace wrapper. Recreate every touched path from its pinned commit
    # before testing application so this fixture is deterministic.
    root_paths = (
        "so101_description/urdf/so101_arm.urdf.xacro",
        "so101_description/urdf/ros2_control/so101_ros2_control.xacro",
    )
    driver_paths = (
        "include/feetech_ros2_driver/feetech_ros2_driver.hpp",
        "src/feetech_ros2_driver.cpp",
    )
    for relative in root_paths:
        content = subprocess.run(
            ["git", "-C", str(source_root), "show", f"HEAD:{relative}"],
            check=True,
            text=True,
            capture_output=True,
        ).stdout
        target = copy / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    for relative in driver_paths:
        content = subprocess.run(
            ["git", "-C", str(source_dir), "show", f"HEAD:{relative}"],
            check=True,
            text=True,
            capture_output=True,
        ).stdout
        target = copy / "feetech_ros2_driver" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)

    subprocess.run(
        ["patch", "--batch", "-p1", "-i", str(PATCH)],
        cwd=copy,
        check=True,
        text=True,
        capture_output=True,
    )
    return copy


@pytest.fixture(scope="module")
def patched_driver(patched_source_root: Path) -> str:
    return (patched_source_root / "feetech_ros2_driver/src/feetech_ros2_driver.cpp").read_text()


def test_read_only_configure_guards_every_motor_write(patched_driver: str):
    configure = cxx_function_body(
        patched_driver,
        "CallbackReturn FeetechHardwareInterface::configure_joints_(",
    )
    guarded_writes = braced_block(configure, "if (!read_only_)")
    write_calls = (
        "communication_protocol_->disable_torque(",
        "communication_protocol_->write(",
        "communication_protocol_->lock_eprom(",
        "communication_protocol_->set_torque(",
    )

    for call in write_calls:
        assert call in guarded_writes
        assert configure.count(call) == guarded_writes.count(call)


def test_read_only_runtime_methods_cannot_write_or_enable_torque(patched_driver: str):
    write = cxx_function_body(
        patched_driver,
        "hardware_interface::return_type FeetechHardwareInterface::write(",
    )
    deactivate = cxx_function_body(
        patched_driver,
        "CallbackReturn FeetechHardwareInterface::on_deactivate(",
    )

    assert "if (read_only_)" in write
    assert write.index("if (read_only_)") < write.index("sync_write_position(")
    assert "if (read_only_)" in deactivate
    assert deactivate.index("if (read_only_)") < deactivate.index("sync_write(")

    export_commands = cxx_function_body(
        patched_driver,
        "std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces(",
    )
    assert "if (read_only_)" in export_commands
    assert export_commands.index("if (read_only_)") < export_commands.index("command_interfaces.emplace_back")


def test_read_only_still_reads_measured_joint_positions(patched_driver: str):
    read = cxx_function_body(
        patched_driver,
        "hardware_interface::return_type FeetechHardwareInterface::read(",
    )

    assert "communication_protocol_->sync_read(" in read
    assert "state_hw_positions_[index]" in read
    assert "state_software_zeros_[index]" in read


def test_mode_defaults_read_only_and_only_explicit_false_restores_writes(patched_driver: str):
    initialize = patched_driver[
        patched_driver.index("read_only_ = true;") : patched_driver.index(
            "CallbackReturn FeetechHardwareInterface::init_transport_()"
        )
    ]

    assert "read_only_ = true" in patched_driver
    assert 'read_only_it->second == "false"' in initialize
    assert 'read_only_it->second == "true"' in initialize
    assert "invalid read_only value" in initialize


def test_state_mapping_file_is_separate_and_drives_logical_zero_scale(patched_driver: str):
    mapping = cxx_function_body(
        patched_driver,
        "CallbackReturn FeetechHardwareInterface::load_state_mapping_()",
    )
    configure = cxx_function_body(
        patched_driver,
        "CallbackReturn FeetechHardwareInterface::configure_joints_(",
    )
    read = cxx_function_body(
        patched_driver,
        "hardware_interface::return_type FeetechHardwareInterface::read(",
    )

    assert 'hardware_parameters.find("state_mapping_file")' in mapping
    assert 'entry["servo_id"]' in mapping
    assert 'entry["software_zero_raw"]' in mapping
    assert 'root["radians_per_raw_count"]' in mapping
    assert 'hardware_parameters.find("joint_config_file")' not in mapping
    assert "state_software_zeros_[i]" in configure
    assert "state_radians_per_count_[i]" in configure
    assert "state_software_zeros_.assign(info_.joints.size(), feetech_driver::kStsMidpoint)" in patched_driver
    assert "state_radians_per_count_.assign(info_.joints.size(), 2.0 * std::numbers::pi / feetech_driver::kStsResolution)" in patched_driver
    assert "present_position - state_software_zeros_[index]" in read
    assert "state_radians_per_count_[index]" in read


def test_state_only_xacro_defaults_safe_and_passes_mapping_file(patched_source_root: Path):
    robot = (patched_source_root / "so101_description/urdf/so101_arm.urdf.xacro").read_text()
    control = (patched_source_root / "so101_description/urdf/ros2_control/so101_ros2_control.xacro").read_text()

    assert '<xacro:arg name="read_only" default="true"/>' in robot
    assert '<xacro:arg name="state_mapping_file" default=""/>' in robot
    assert 'read_only="$(arg read_only)"' in robot
    assert 'state_mapping_file="$(arg state_mapping_file)"' in robot
    assert '<param name="read_only">${read_only}</param>' in control
    assert '<param name="state_mapping_file">${state_mapping_file}</param>' in control


def test_unqualified_gripper_mapping_is_not_used_as_angular_calibration(patched_driver: str):
    mapping = cxx_function_body(
        patched_driver,
        "CallbackReturn FeetechHardwareInterface::load_state_mapping_()",
    )
    read = cxx_function_body(
        patched_driver,
        "hardware_interface::return_type FeetechHardwareInterface::read(",
    )

    assert "normalized_fraction_only" in mapping
    assert "unqualified angular state" in mapping
    assert 'entry["raw_min"]' not in mapping
    assert 'entry["raw_max"]' not in mapping
    assert "native STS midpoint conversion" in mapping
    assert "state_software_zeros_[index]" in read
