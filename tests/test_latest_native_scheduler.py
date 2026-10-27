"""CPU-only latest-native/admission/lifecycle contracts; no SDK or GPU."""
from pathlib import Path
import json
import shutil
import subprocess

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def test_actual_latest_native_scheduler_contracts_and_hot_cost(tmp_path):
    binary = tmp_path / 'latest-native'
    built = subprocess.run([shutil.which('c++'), '-std=c++17', '-O2', '-pthread', '-Wall', '-Wextra', '-Werror',
                            '-I', str(P / 'include'), str(P / 'test/latest_native_scheduler_test.cpp'), '-o', str(binary)], capture_output=True, text=True)
    assert built.returncode == 0, built.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    assert report['offered_per_overload_case'] == 300
    assert report['overload_cases'] == 2
    assert report['max_pending'] == report['max_active'] == 1
    assert report['hot_heap_allocations'] == 0
    assert report['host_ns_per_cycle'] > 0
    assert report['blocked_publication_intake_progress'] and report['contention_discard_nonblocking']
