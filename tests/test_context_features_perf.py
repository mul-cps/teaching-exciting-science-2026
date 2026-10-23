"""CPU synthetic-driver cache/feature guards; never load Vulkan or a GPU."""
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
    raise AssertionError("unbalanced C++ body")


def test_actual_context_cache_has_no_hot_queries_allocations_creates_or_waits(tmp_path):
    compiler = os.environ.get("CXX") or shutil.which("c++")
    assert compiler, "C++ compiler required for CPU-only Context test"
    binary = tmp_path / "context_features_perf"
    command = [compiler, "-O2", "-std=c++17", "-pthread", "-ffunction-sections",
               "-fdata-sections", "-I", str(PACKAGE / "include"),
               str(PACKAGE / "test/context_features_perf_benchmark.cpp"),
               str(PACKAGE / "src/vulkan_context.cpp"), "-Wl,--gc-sections", "-o", str(binary)]
    if include := os.environ.get("CONTEXT_FEATURES_VULKAN_INCLUDE"):
        command[command.index("-I"):] = ["-I", include] + command[command.index("-I"):]
    subprocess.run(command, check=True, capture_output=True, text=True)
    report = json.loads(subprocess.check_output([str(binary)], text=True))
    assert report["trials"] == 7
    expected = {"physical_api12_fp64": (True, False, 0, 1),
                "physical_api12_no_fp64": (False, False, 0, 1),
                "missing_video_extension_fp64": (True, False, 1, 1),
                "video_fp64": (True, True, 1, 1),
                "video_create_fallback_fp64": (True, False, 1, 2)}
    assert {row["name"] for row in report["scenarios"]} == set(expected)
    for row in report["scenarios"]:
        assert (row["fp64_supported_and_enabled"], row["video_enabled"],
                row["cold_features2_queries"], row["cold_device_creates"]) == expected[row["name"]]
        assert row["cold_feature_queries"] == 1
        assert row["hot_acquisitions_per_trial"] == 200000
        assert all(row[key] == 0 for key in ("hot_heap_allocations", "hot_vulkan_calls",
                                            "hot_feature_queries", "hot_device_creates", "hot_waits"))
        assert row["median_hot_ns"] > 0 and row["median_cold_context_ns"] > 0
        assert len(row["hot_samples_ns"]) == len(row["cold_samples_ns"]) == 7
    if destination := os.environ.get("CONTEXT_FEATURES_PERF_ARTIFACTS"):
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(binary, destination / binary.name)
        (destination / "microbenchmark.json").write_text(json.dumps(report, indent=2) + "\n")
        (destination / "compile-command.json").write_text(json.dumps(command, indent=2) + "\n")


def test_feature_negotiation_cannot_enter_cache_hit_or_per_frame_paths():
    """Moving a physical-feature query into retained-owner/frame work must fail."""
    context = (PACKAGE / "src/vulkan_context.cpp").read_text()
    shared = body(context, "Context::shared()")
    assert not re.search(r"\bvk\w+\s*\(|\b(?:new|malloc|calloc|realloc)\b|sleep|wait_for", shared)
    # std::make_shared is allowed only when the weak cache expired, never before it.
    miss = body(shared, "if(!c)")
    assert "std::make_shared<Context>()" in miss
    before_miss = shared[:shared.index("if(!c)")]
    assert "make_shared" not in before_miss and "make_unique" not in shared
    core = (PACKAGE / "src/gpu_pipeline.cpp").read_text()
    for signature in ("Pipeline::submit_impl(", "Pipeline::finish("):
        hot = body(core, signature)
        assert not re.search(r"vkGetPhysicalDevice\w*|vkEnumerate\w*|vkCreateDevice|Context::shared\s*\(", hot)
    cold = body(context, "Context::Context()")
    assert cold.count("vkGetPhysicalDeviceFeatures(physical,&supported_compute)") == 1
    assert cold.index("vkGetPhysicalDeviceFeatures(physical,&supported_compute)") < cold.index("if(instance_version>=VK_API_VERSION_1_3")
