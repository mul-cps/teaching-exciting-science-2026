import os
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "ros2/scripts/runtime.sh"
CONTAINERFILE = ROOT / "ros2/container/Containerfile"
DOMAIN_ENTRYPOINT = ROOT / "ros2/scripts/ros-domain-entrypoint.sh"


def run_runtime(*args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(RUNTIME), *args],
        cwd=ROOT,
        text=True,
        capture_output=True,
        env={**os.environ, "ROS2_IMAGE": "localhost/so101-ros2:test", **(env or {})},
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
    assert "--env ROS_DOMAIN_ID=76" in result.stdout
    assert "--env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in result.stdout
    assert "--env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in result.stdout


def test_runtime_respects_explicit_nonzero_domain_and_rejects_zero():
    selected = run_runtime("plan", env={
        "ROS_DOMAIN_ID": "75",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
    })
    assert selected.returncode == 0, selected.stderr
    assert "--env ROS_DOMAIN_ID=75" in selected.stdout
    assert "--env RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in selected.stdout
    assert "--env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in selected.stdout

    disabled = run_runtime("plan", env={"ROS_DOMAIN_ID": "0"})
    assert disabled.returncode != 0
    assert "ROS_DOMAIN_ID=0 is disabled" in disabled.stderr


def test_container_image_defaults_to_domain_76_and_entrypoint_rejects_zero():
    containerfile = CONTAINERFILE.read_text()
    assert "ROS_DOMAIN_ID=76" in containerfile
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in containerfile
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in containerfile
    assert 'ENTRYPOINT ["/usr/local/bin/ros-domain-entrypoint"]' in containerfile

    disabled = subprocess.run(
        [str(DOMAIN_ENTRYPOINT)],
        env={**os.environ, "ROS_DOMAIN_ID": "0"},
        text=True,
        capture_output=True,
        check=False,
    )
    assert disabled.returncode == 2
    assert "ROS_DOMAIN_ID=0 is disabled" in disabled.stderr


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


def test_hardware_rejects_noncanonical_station_lock(tmp_path: Path):
    state_home = tmp_path / "state"
    foreign_lock = tmp_path / "foreign.lock"
    foreign_lock.touch()

    result = run_runtime(
        "plan",
        "--hardware",
        "--device",
        "/dev/null",
        "--station-lock",
        str(foreign_lock),
        env={"XDG_STATE_HOME": str(state_home)},
    )

    assert result.returncode != 0
    assert "must use canonical station lock" in result.stderr


def test_hardware_holds_canonical_lock_for_full_podman_lifecycle(tmp_path: Path):
    state_home = tmp_path / "state"
    canonical_lock = state_home / "exciting-science/station.lock"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    marker = tmp_path / "podman-started"
    release = tmp_path / "release-podman"
    fake_podman = fake_bin / "podman"
    fake_podman.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "if flock -n \"$TEST_STATION_LOCK\" -c true; then\n"
        "  echo 'lock was not held inside podman' >&2; exit 81\n"
        "fi\n"
        "touch \"$TEST_PODMAN_MARKER\"\n"
        "while [[ ! -e \"$TEST_PODMAN_RELEASE\" ]]; do sleep 0.02; done\n"
    )
    fake_podman.chmod(0o755)
    env = {
        **os.environ,
        "ROS2_IMAGE": "localhost/so101-ros2:test",
        "XDG_STATE_HOME": str(state_home),
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "TEST_STATION_LOCK": str(canonical_lock),
        "TEST_PODMAN_MARKER": str(marker),
        "TEST_PODMAN_RELEASE": str(release),
    }
    process = subprocess.Popen(
        [
            str(RUNTIME), "run", "--hardware", "--device", "/dev/null",
            "--station-lock", str(canonical_lock), "--", "true",
        ],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 5
        while not marker.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert marker.exists(), "fake Podman did not start while runtime was running"
        assert process.poll() is None, "runtime exited before fake Podman lifecycle ended"
        contender = subprocess.run(
            ["flock", "-n", str(canonical_lock), "-c", "true"],
            check=False,
        )
        assert contender.returncode != 0, "station lock was released while Podman was running"
    finally:
        release.touch()
        stdout, stderr = process.communicate(timeout=5)
    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"


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
