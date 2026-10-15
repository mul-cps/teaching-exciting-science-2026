import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "ros2/scripts/runtime.sh"


def run_runtime(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(RUNTIME), *args],
        cwd=ROOT,
        text=True,
        capture_output=True,
        env={**os.environ, "ROS2_IMAGE": "localhost/so101-ros2:test"},
        check=False,
    )


def test_help_documents_safe_default_and_explicit_hardware_gate():
    result = run_runtime("--help")

    assert result.returncode == 0
    assert "plan|shell|run|build" in result.stdout
    assert "Hardware is disabled by default" in result.stdout
    assert "--hardware --device PATH --station-lock PATH" in result.stdout


def test_plan_mounts_workspace_and_x11_without_usb_or_devices():
    result = run_runtime("plan")

    assert result.returncode == 0, result.stderr
    assert "--network host" in result.stdout
    assert f"{ROOT}:/ws:Z" in result.stdout
    assert "/tmp/.X11-unix:/tmp/.X11-unix:ro" in result.stdout
    assert "--device" not in result.stdout
    assert "/dev/bus/usb" not in result.stdout


def test_gpu_is_opt_in():
    default = run_runtime("plan")
    gpu = run_runtime("plan", "--gpu")

    assert "--device /dev/dri" not in default.stdout
    assert "--device /dev/dri" in gpu.stdout
    assert "--group-add keep-groups" in gpu.stdout


def test_hardware_requires_explicit_device_and_station_lock():
    result = run_runtime("plan", "--hardware")

    assert result.returncode != 0
    assert "requires --device and --station-lock" in result.stderr


def test_stateful_build_directories_live_in_user_cache():
    result = run_runtime("plan")

    assert result.returncode == 0, result.stderr
    assert ".cache/so101-ros2/build:/ws/build:Z" in result.stdout
    assert ".cache/so101-ros2/install:/ws/install:Z" in result.stdout
    assert ".cache/so101-ros2/log:/ws/log:Z" in result.stdout


def test_run_forwards_command_without_allocating_a_tty():
    result = run_runtime("plan", "--", "ros2", "launch", "so101_bringup", "view.launch.py")

    assert result.returncode == 0, result.stderr
    assert "--interactive" not in result.stdout
    assert "--tty" not in result.stdout
    assert "ros2 launch so101_bringup view.launch.py" in result.stdout


def test_run_requires_a_command():
    result = run_runtime("run")

    assert result.returncode != 0
    assert "run requires a command after --" in result.stderr
