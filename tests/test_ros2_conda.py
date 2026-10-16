import os
import subprocess
from pathlib import Path

import yaml
import pytest


ROOT = Path(__file__).resolve().parents[1]
CONDA_DIR = ROOT / "ros2/conda"
ENVIRONMENT = CONDA_DIR / "environment.yml"
WRAPPER = CONDA_DIR / "so101-conda.sh"


def run_wrapper(*args: str, env: dict[str, str] | None = None, cwd=None):
    test_env = os.environ.copy()
    if env is None or "ROS_DOMAIN_ID" not in env:
        test_env.pop("ROS_DOMAIN_ID", None)
    test_env.update(env or {})
    return subprocess.run(
        [str(WRAPPER), *args],
        cwd=ROOT if cwd is None else cwd,
        env=test_env,
        text=True,
        capture_output=True,
        check=False,
    )


def test_environment_declares_isolated_pinned_ros_jazzy_control_stack():
    data = yaml.safe_load(ENVIRONMENT.read_text(encoding="utf-8"))
    packages = {entry.split("=")[0]: entry for entry in data["dependencies"] if isinstance(entry, str)}

    assert data["name"] == "so101-jazzy"
    assert data["channels"] == ["conda-forge", "robostack-jazzy"]
    assert packages["python"] == "python=3.12"
    for package in (
        "ros-jazzy-desktop",
        "ros-jazzy-moveit",
        "ros-jazzy-moveit-py",
        "ros-jazzy-pick-ik",
        "ros-jazzy-moveit-servo",
        "ros-jazzy-ros2-control",
        "ros-jazzy-ros2-controllers",
        "ros-jazzy-realsense2-camera",
        "ros-jazzy-tf2-tools",
        "colcon-common-extensions",
        "cxx-compiler",
        "cmake",
        "ninja",
        "scipy",
    ):
        assert package in packages
    assert packages["scipy"] == "scipy=1.16.2"


@pytest.mark.parametrize("discovery,expected", [("", "LOCALHOST"), ("SUBNET", "SUBNET")])
def test_conda_run_clears_inherited_ros_python_and_loader_environment(tmp_path: Path, discovery, expected):
    fake_conda = tmp_path / "conda"
    capture = tmp_path / "capture"
    fake_conda.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "mkdir -p \"$(dirname \"$TEST_CONDA_ARGS\")\"\n"
        "printf '%s\\n' \"$@\" > \"$TEST_CONDA_ARGS\"\n"
        "env | sort > \"$TEST_CONDA_ENV\"\n"
    )
    fake_conda.chmod(0o755)
    environment_prefix = tmp_path / "environment"
    (environment_prefix / "lib").mkdir(parents=True)
    (environment_prefix / "lib/librmw_cyclonedds_cpp.so").touch()
    env = {
        "SO101_CONDA_EXE": str(fake_conda),
        "SO101_CONDA_PREFIX": str(environment_prefix),
        "TEST_CONDA_ARGS": str(capture / "args"),
        "TEST_CONDA_ENV": str(capture / "environment"),
        "AMENT_PREFIX_PATH": "/opt/ros/iron",
        "COLCON_PREFIX_PATH": "/opt/ros/iron",
        "CMAKE_PREFIX_PATH": "/opt/ros/iron",
        "PYTHONPATH": "/opt/ros/iron/lib/python3/site-packages",
        "PYTHONHOME": "/opt/ros/iron",
        "ROS_DISTRO": "iron",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": discovery,
        "LD_LIBRARY_PATH": "/opt/ros/iron/lib",
        "DISPLAY": ":77",
        "XAUTHORITY": "/tmp/test.xauth",
    }

    result = run_wrapper("run", "ros2", "pkg", "list", env=env)

    assert result.returncode == 0, result.stderr
    args = (capture / "args").read_text().splitlines()
    assert args[:4] == ["run", "--no-capture-output", "--prefix", str(tmp_path / "environment")]
    assert args[-3:] == ["ros2", "pkg", "list"]
    clean_environment = dict(
        line.split("=", 1)
        for line in (capture / "environment").read_text().splitlines()
        if "=" in line
    )
    for name in (
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "PYTHONPATH",
        "PYTHONHOME",
        "ROS_DISTRO",
        "LD_LIBRARY_PATH",
    ):
        assert name not in clean_environment
    assert clean_environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] == expected
    assert clean_environment["ROS_DOMAIN_ID"] == "76"
    assert clean_environment["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert clean_environment["DISPLAY"] == ":77"
    assert clean_environment["XAUTHORITY"] == "/tmp/test.xauth"


def test_conda_run_respects_explicit_rmw_override(tmp_path: Path):
    fake_conda = tmp_path / "conda"
    capture = tmp_path / "capture"
    fake_conda.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "mkdir -p \"$(dirname \"$TEST_CONDA_ARGS\")\"\n"
        "env | sort > \"$TEST_CONDA_ENV\"\n"
    )
    fake_conda.chmod(0o755)
    environment_prefix = tmp_path / "environment"
    (environment_prefix / "lib").mkdir(parents=True)
    (environment_prefix / "lib/librmw_cyclonedds_cpp.so").touch()

    result = run_wrapper(
        "run", "ros2", "topic", "list",
        env={
            "SO101_CONDA_EXE": str(fake_conda),
            "SO101_CONDA_PREFIX": str(environment_prefix),
            "TEST_CONDA_ARGS": str(capture / "args"),
            "TEST_CONDA_ENV": str(capture / "environment"),
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        },
    )

    assert result.returncode == 0, result.stderr
    clean_environment = dict(
        line.split("=", 1)
        for line in (capture / "environment").read_text().splitlines()
        if "=" in line
    )
    assert clean_environment["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"


def test_conda_run_respects_explicit_nonzero_domain_id(tmp_path: Path):
    fake_conda = tmp_path / "conda"
    capture = tmp_path / "capture"
    fake_conda.write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        "mkdir -p \"$(dirname \"$TEST_CONDA_ENV\")\"\n"
        "env | sort > \"$TEST_CONDA_ENV\"\n"
    )
    fake_conda.chmod(0o755)
    result = run_wrapper(
        "run", "ros2", "topic", "list",
        env={
            "SO101_CONDA_EXE": str(fake_conda),
            "SO101_CONDA_PREFIX": str(tmp_path / "environment"),
            "TEST_CONDA_ENV": str(capture / "environment"),
            "ROS_DOMAIN_ID": "75",
        },
    )
    assert result.returncode == 0, result.stderr
    environment = dict(
        line.split("=", 1)
        for line in (capture / "environment").read_text().splitlines()
        if "=" in line
    )
    assert environment["ROS_DOMAIN_ID"] == "75"


def test_conda_wrapper_rejects_domain_zero_before_invoking_conda(tmp_path: Path):
    fake_conda = tmp_path / "conda"
    marker = tmp_path / "called"
    fake_conda.write_text(f"#!/usr/bin/env bash\ntouch '{marker}'\n")
    fake_conda.chmod(0o755)
    result = run_wrapper(
        "run", "ros2", "topic", "list",
        env={"SO101_CONDA_EXE": str(fake_conda), "ROS_DOMAIN_ID": "0"},
    )
    assert result.returncode != 0
    assert "ROS_DOMAIN_ID=0 is disabled" in result.stderr
    assert not marker.exists()


def test_create_uses_repository_environment_file_and_dedicated_prefix(tmp_path: Path):
    fake_conda = tmp_path / "conda"
    capture = tmp_path / "create-args"
    fake_conda.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "printf '%s\\n' \"$@\" > \"$TEST_CONDA_ARGS\"\n"
    )
    fake_conda.chmod(0o755)

    result = run_wrapper(
        "create",
        env={
            "SO101_CONDA_EXE": str(fake_conda),
            "SO101_CONDA_PREFIX": str(tmp_path / "envs/so101-jazzy"),
            "TEST_CONDA_ARGS": str(capture),
        },
    )

    assert result.returncode == 0, result.stderr
    args = capture.read_text().splitlines()
    assert args == [
        "env",
        "create",
        "--yes",
        "--solver",
        "libmamba",
        "--prefix",
        str(tmp_path / "envs/so101-jazzy"),
        "--file",
        str(ENVIRONMENT),
    ]


def test_wrapper_refuses_real_device_startup_without_invoking_conda(tmp_path: Path):
    fake_conda = tmp_path / "conda"
    called = tmp_path / "called"
    fake_conda.write_text(f"#!/usr/bin/env bash\ntouch '{called}'\n")
    fake_conda.chmod(0o755)

    result = run_wrapper("real", env={"SO101_CONDA_EXE": str(fake_conda)})

    assert result.returncode != 0
    assert "real hardware remains gated" in result.stderr
    assert not called.exists()


def test_help_lists_conda_environment_commands_without_creating_environment():
    result = run_wrapper("--help")

    assert result.returncode == 0
    assert "create|fetch|build|clean-build|station-live|rviz|servo|plan-smoke|inspect|packages|run|shell|real" in result.stdout


def test_station_live_uses_clean_conda_ros_environment_and_default_cyclone(tmp_path: Path):
    fake_conda = tmp_path / "conda"
    capture = tmp_path / "capture"
    fake_conda.write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        "mkdir -p \"$(dirname \"$TEST_CONDA_ARGS\")\"\n"
        "printf '%s\\n' \"$@\" > \"$TEST_CONDA_ARGS\"\n"
        "env | sort > \"$TEST_CONDA_ENV\"\n"
    )
    fake_conda.chmod(0o755)
    environment_prefix = tmp_path / "envs/ros"
    (environment_prefix / "lib").mkdir(parents=True)
    (environment_prefix / "lib/librmw_cyclonedds_cpp.so").touch()
    env = {
        "SO101_CONDA_EXE": str(fake_conda),
        "SO101_CONDA_PREFIX": str(environment_prefix),
        "TEST_CONDA_ARGS": str(capture / "args"),
        "TEST_CONDA_ENV": str(capture / "environment"),
        "ROS_DISTRO": "iron",
        "PYTHONPATH": "/opt/ros/iron/lib/python3/site-packages",
    }
    result = run_wrapper("station-live", "rviz:=false", env=env, cwd=tmp_path)
    assert result.returncode == 0, result.stderr
    args = (capture / "args").read_text().splitlines()
    assert "station_live.launch.py" in args
    assert f"station_root:={ROOT}" in args
    assert args[-1] == "rviz:=false"
    clean_environment = dict(
        line.split("=", 1)
        for line in (capture / "environment").read_text().splitlines()
        if "=" in line
    )
    assert clean_environment["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert "PYTHONPATH" not in clean_environment


def test_clean_build_discards_only_ros_overlay_prefixes_before_rebuild():
    source = WRAPPER.read_text(encoding="utf-8")
    assert 'clean-build)' in source
    assert 'rm -rf -- "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"' in source
    assert 'exec "$0" build' in source
