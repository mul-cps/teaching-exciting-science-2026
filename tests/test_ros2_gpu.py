"""GPU container routing checks; tests use a stub runtime and never start ROS."""

import os
from pathlib import Path
import subprocess
import sys


REPOSITORY = Path(__file__).resolve().parents[1]
SO101 = REPOSITORY / "ros2/scripts/so101.sh"
RUNTIME = REPOSITORY / "ros2/scripts/runtime.sh"
GLX_PROBE = REPOSITORY / "ros2/scripts/glx-renderer-probe.py"
CONTROL_SOURCE = REPOSITORY / "ros2/src/so101_station_control"
GPU_SOURCE = REPOSITORY / "ros2/src/so101_station_gpu"
sys.path.insert(0, str(GPU_SOURCE))
from so101_station_gpu import gpu_renderer  # noqa: E402


def _invoke_so101(tmp_path, action, *, with_dri):
    dri = tmp_path / "dri"
    if with_dri:
        dri.mkdir()
    capture = tmp_path / "args.txt"
    stub = tmp_path / "runtime-stub.sh"
    stub.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$SO101_CAPTURE_ARGS"\n')
    stub.chmod(0o755)
    env = os.environ.copy()
    env.update({"SO101_RUNTIME_SH": str(stub), "SO101_DRI_DIR": str(dri),
                "SO101_CAPTURE_ARGS": str(capture)})
    subprocess.run(["bash", str(SO101), action], env=env, check=True, capture_output=True, text=True)
    return capture.read_text().splitlines()


def test_gui_actions_request_gpu_only_when_host_dri_exists(tmp_path):
    for action in ("rviz", "servo"):
        action_dir = tmp_path / action
        action_dir.mkdir()
        args = _invoke_so101(action_dir, action, with_dri=True)
        assert args[:3] == ["run", "--gpu", "--"]
        assert args[-3:] == ["launch", "so101_station_bringup",
                             "mock_moveit.launch.py" if action == "rviz"
                             else "servo_mock.launch.py"]


def test_gui_action_falls_back_when_dri_is_absent(tmp_path):
    args = _invoke_so101(tmp_path, "rviz", with_dri=False)
    assert args[:2] == ["run", "--"]
    assert "--gpu" not in args


def test_non_gui_action_does_not_request_gpu_even_when_available(tmp_path):
    args = _invoke_so101(tmp_path, "inspect", with_dri=True)
    assert args[:2] == ["run", "--"]
    assert "--gpu" not in args


def test_runtime_gpu_mode_forwards_dri_and_host_groups_without_software_override(tmp_path):
    env = os.environ.copy()
    env["HOME"] = str(tmp_path)
    result = subprocess.run(["bash", str(RUNTIME), "plan", "--gpu"], env=env,
                            check=True, capture_output=True, text=True)
    assert "--device /dev/dri" in result.stdout
    assert "--group-add keep-groups" in result.stdout
    runtime_source = RUNTIME.read_text()
    assert "LIBGL_ALWAYS_SOFTWARE" not in runtime_source


def test_glx_probe_reports_renderer_without_enabling_software_fallback():
    source = GLX_PROBE.read_text()
    probe_source = (GPU_SOURCE / "so101_station_gpu/gpu_renderer.py").read_text()
    assert "GL_RENDERER" in probe_source
    assert "LIBGL_ALWAYS_SOFTWARE" in source
    assert 'os.environ["LIBGL_ALWAYS_SOFTWARE"] = "true"' not in probe_source
    assert 'os.environ["LIBGL_ALWAYS_SOFTWARE"] = "false"' in probe_source
    assert "TemporaryDirectory" in probe_source


def test_rviz_launch_uses_gpu_guard_and_clears_inherited_preload():
    launch = (CONTROL_SOURCE / "launch/moveit_resident.launch.py").read_text()
    assert 'executable="rviz_gpu_guard"' in launch
    assert 'rviz_environment = {"LD_PRELOAD": rviz_preload_value()}' in launch
    assert gpu_renderer.rviz_preload_value(
        {"LD_PRELOAD": "/unrelated/global.so"}, exists=lambda _: False
    ) == ""


def test_rviz_launches_use_guard_in_station_and_perception_entrypoints():
    moveit_launch = (CONTROL_SOURCE / "launch/moveit_resident.launch.py").read_text()
    perception_launch = (REPOSITORY / "ros2/src/so101_station_perception/launch/perception.launch.py").read_text()
    assert 'executable="rviz_gpu_guard"' in moveit_launch
    assert 'executable="rviz_gpu_guard"' in perception_launch
    assert 'executable="rviz2"' not in moveit_launch + perception_launch


def test_mock_and_servo_launches_replace_upstream_rviz_with_guard():
    bringup = REPOSITORY / "ros2/src/so101_station_bringup/launch"
    for filename in ("mock_moveit.launch.py", "servo_mock.launch.py"):
        launch = (bringup / filename).read_text()
        assert '"use_rviz": "false"' in launch
        assert "guarded_moveit_rviz.launch.py" in launch
    guarded = (bringup / "guarded_moveit_rviz.launch.py").read_text()
    assert 'package="so101_station_gpu"' in guarded
    assert 'executable="rviz_gpu_guard"' in guarded


def test_rviz_preload_override_must_name_existing_libraries():
    assert gpu_renderer.rviz_preload_value(
        {"SO101_RVIZ_LD_PRELOAD": "/a.so:/b.so"}, exists=lambda _: True
    ) == "/a.so:/b.so"
    try:
        gpu_renderer.rviz_preload_value(
            {"SO101_RVIZ_LD_PRELOAD": "/a.so:/missing.so"},
            exists=lambda value: value != "/missing.so",
        )
    except ValueError as exc:
        assert "existing library paths" in str(exc)
    else:
        raise AssertionError("invalid custom preload must fail")


def test_rviz_preflight_rejects_known_software_renderers():
    for renderer in ("llvmpipe (LLVM 21.1.0)", "softpipe", "Software Rasterizer"):
        try:
            gpu_renderer.require_hardware_renderer({"GL_RENDERER": renderer})
        except RuntimeError as exc:
            assert "requires GPU rendering" in str(exc)
        else:
            raise AssertionError(f"software renderer accepted: {renderer}")
    for renderer in ("(null)", "unknown"):
        try:
            gpu_renderer.require_hardware_renderer({"GL_RENDERER": renderer})
        except RuntimeError as exc:
            assert "GL_RENDERER" in str(exc)
        else:
            raise AssertionError(f"unidentified renderer accepted: {renderer}")


def test_rviz_preflight_accepts_hardware_renderer_and_requires_identity():
    gpu_renderer.require_hardware_renderer({"GL_RENDERER": "AMD Radeon 8060S"})
    try:
        gpu_renderer.require_hardware_renderer({"GL_RENDERER": ""})
    except RuntimeError as exc:
        assert "GL_RENDERER" in str(exc)
    else:
        raise AssertionError("missing renderer identity must fail")


def test_guard_exec_preserves_rviz_arguments(monkeypatch):
    info = {"GL_VENDOR": "AMD", "GL_RENDERER": "AMD Radeon 8060S", "GL_VERSION": "4.6"}
    monkeypatch.setattr(gpu_renderer, "rviz_preload_value", lambda: "/usr/lib64/libdrm.so.2")
    def probe_with_expected_environment():
        assert os.environ["LD_PRELOAD"] == "/usr/lib64/libdrm.so.2"
        assert os.environ["LIBGL_ALWAYS_SOFTWARE"] == "false"
        assert os.environ["GALLIUM_DRIVER"] == "radeonsi"
        return info
    monkeypatch.setattr(gpu_renderer, "probe_renderer", probe_with_expected_environment)
    monkeypatch.setattr(gpu_renderer.shutil, "which", lambda name: "/opt/ros/bin/rviz2")
    invoked = {}

    def fake_exec(path, argv, env):
        invoked.update(path=path, argv=argv, env=env)

    monkeypatch.setattr(gpu_renderer.os, "execvpe", fake_exec)
    gpu_renderer.main(["-d", "station.rviz", "--ros-args", "-r", "x:=y"])
    assert invoked["path"] == "/opt/ros/bin/rviz2"
    assert invoked["argv"] == ["/opt/ros/bin/rviz2", "-d", "station.rviz",
                               "--ros-args", "-r", "x:=y"]
    assert invoked["env"]["LD_PRELOAD"] == "/usr/lib64/libdrm.so.2"
    assert invoked["env"]["LIBGL_ALWAYS_SOFTWARE"] == "false"
    assert invoked["env"]["GALLIUM_DRIVER"] == "radeonsi"
