"""Guard cold-only source stabilization and its actual allocation-free helper cost."""
from pathlib import Path
import json
import os
import re
import shutil
import subprocess

PACKAGE = Path(__file__).resolve().parents[1] / "ros2/src/so101_station_vulkan"


def body(source, signature):
    start = source.index("{", source.index(signature))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if not depth:
            return source[start + 1:end]
    raise AssertionError("unbalanced C++ function")


def code_only(source):
    return re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"', "", source,
                  flags=re.DOTALL)


def test_real_startup_gate_sequences_do_not_allocate(tmp_path):
    """A per-observation new/string/vector regression must fail the measured budget."""
    compiler = os.environ.get("CXX") or shutil.which("c++")
    assert compiler, "a C++ compiler is required for the CPU-only gate benchmark"
    binary = tmp_path / "camera_startup_perf"
    subprocess.run([
        compiler, "-O2", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-I", str(PACKAGE / "include"),
        str(PACKAGE / "test/camera_startup_perf_benchmark.cpp"), "-o", str(binary),
    ], check=True, capture_output=True, text=True)
    report = json.loads(subprocess.check_output([str(binary)], text=True))
    assert report["schema"] == "camera-startup-perf-v1"
    expected_observations = {"valid_advancing": 61, "counter_resets": 70}
    assert {sequence["name"] for sequence in report["sequences"]} == set(expected_observations)
    assert len(report["sequences"]) == 2
    for sequence in report["sequences"]:
        assert sequence["median_ns_per_observation"] > 0
        assert len(sequence["samples"]) == report["trials"] == 7
        for sample in sequence["samples"]:
            assert sample["heap_allocations"] == 0
            assert sample["ready"] == report["cycles_per_trial"] == 20000
            assert sample["observations"] == 20000 * expected_observations[sequence["name"]]
    if destination := os.environ.get("CAMERA_STARTUP_PERF_ARTIFACTS"):
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(binary, destination / binary.name)
        (destination / "microbenchmark.json").write_text(json.dumps(report, indent=2) + "\n")


def test_stabilization_waits_cannot_enter_normal_capture_processing():
    """A startup wait, CPU alignment, allocation or GPU sync in raw callbacks is forbidden."""
    camera = (PACKAGE / "src/camera_component.cpp").read_text()
    forbidden = re.compile(
        r"CameraStartupGate|wait_startup_stable|try_wait_for_frames|wait_original|"
        r"\b\w*(?:[Ww]ait|[Ss]leep)\w*\s*\(|vendor_alignment_\.process|"
        r"\b(?:new|malloc|calloc|realloc)\b|make_shared|make_unique|"
        r"std::(?:vector|to_string)|\.(?:resize|reserve)\s*\(|"
        r"vk(?:Wait|Map|Allocate|Create|QueueWaitIdle|DeviceWaitIdle)"
    )
    for signature in (
        "void tick()", "void capture_pending(", "void verify_original(",
        "CameraSourceObservation inspect_original(", "void copy_raw_planes(",
        "void copy_rows(", "void original_metadata(", "void report_status(",
    ):
        assert not forbidden.search(code_only(body(camera, signature))), signature
    # No startup observation is allowed on any steady path, even without waits.
    for signature in ("void tick()", "void capture_pending(", "void verify_original("):
        assert ".observe(" not in code_only(body(camera, signature))


def test_gate_has_only_scalar_state_and_no_external_runtime_calls():
    """The helper must remain independent of SDK/ROS/GPU and heap-backed state."""
    header = (PACKAGE / "include/so101_station_vulkan/camera_startup_gate.hpp").read_text()
    code = code_only(header)
    assert not re.search(
        r"\b(?:new|malloc|calloc|realloc|free)\b|make_shared|make_unique|"
        r"std::(?:string|vector|map|deque|list)|rs2::|rclcpp::|\bVk\w*|"
        r"\b\w*(?:[Ww]ait|[Ss]leep)\w*\s*\(|(?:cuda|hip)\w*\s*\(", code
    )
    assert set(re.findall(r"#include\s*[<\"]([^>\"]+)", header)) <= {
        "cmath", "cstdint", "stdexcept"
    }


def test_source_stabilization_only_runs_before_publication_admission():
    """Rechecking startup from a timer or after admission would delay steady frames."""
    camera = (PACKAGE / "src/camera_component.cpp").read_text()
    constructor = body(camera, "explicit CameraComponent(")
    normal_begin = constructor.index("publication_gate().admit(")
    assert "wait_startup_stable(" not in constructor[normal_begin:]
    stable = body(camera, "void wait_startup_stable()")
    assert "publication_gate().admit(" not in stable
    assert "frames_->publish(" not in stable and "bridge_->" not in stable
    assert "gate.check_deadline(" in stable and "gate.observe(" in stable
    assert "stop_owned_camera();throw;" in stable
