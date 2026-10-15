"""GPU container routing checks; tests use a stub runtime and never start ROS."""

import os
from pathlib import Path
import subprocess


REPOSITORY = Path(__file__).resolve().parents[1]
SO101 = REPOSITORY / "ros2/scripts/so101.sh"
RUNTIME = REPOSITORY / "ros2/scripts/runtime.sh"
GLX_PROBE = REPOSITORY / "ros2/scripts/glx-renderer-probe.py"


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
    assert "GL_RENDERER" in source
    assert "LIBGL_ALWAYS_SOFTWARE" in source
    assert 'os.environ["LIBGL_ALWAYS_SOFTWARE"] =' not in source
    assert "TemporaryDirectory" in source
