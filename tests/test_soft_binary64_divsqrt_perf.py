"""CPU-only independent div/sqrt cost and pure-integer resource regression gate."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import platform
import re
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2/src/so101_station_vulkan'
SPEC = importlib.util.spec_from_file_location('existing_soft_resource_guard', ROOT / 'tests/test_soft_binary64_perf.py')
GUARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GUARD)


def resource_guard(source):
    code = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    GUARD.assert_bounded_primitive_loops(code)
    assert not re.search(r'\b(?:float|double|__int128|uint128_t|new|malloc|calloc|realloc|static|extern|thread_local)\b', code)
    calls = set(re.findall(r'\b(\w+)\s*\(', code))
    assert all(name.startswith('sf64_') or name in {'if', 'for', 'return', 'int', 'uint64_t', 'uint32_t', 'bool'} for name in calls)


def test_integer_resource_contract():
    resource_guard((PACKAGE / 'shaders/soft_binary64.slang').read_text())


@pytest.mark.parametrize('insertion', [
    'double hidden=0;', '__int128 hidden=0;', 'malloc(8);',
    'vkDeviceWaitIdle(device);', 'printf("hidden");', 'while(true){}',
    'InterlockedAdd(counter,1);', 'static uint64_t hidden=0;',
])
def test_resource_guard_rejects_mutants(insertion):
    source = (PACKAGE / 'shaders/soft_binary64.slang').read_text()
    start = source.index('{', source.index('sf64_div_bits('))
    with pytest.raises(AssertionError):
        resource_guard(source[:start+1] + insertion + source[start+1:])


def test_actual_divsqrt_cpu_cost(tmp_path):
    source = PACKAGE / 'test/soft_binary64_divsqrt_perf_benchmark.cpp'
    binary = tmp_path / 'divsqrt_perf'
    command = [os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-ffp-contract=off', '-fno-fast-math', '-frounding-math', str(source), '-o', str(binary)]
    subprocess.run(command, check=True, capture_output=True, text=True, timeout=60)
    report = json.loads(subprocess.check_output([str(binary), '20261027'], text=True, timeout=60))
    assert len(report['scenarios']) == 6
    assert {(r['population'], r['operation']) for r in report['scenarios']} == {(p, o) for p in ['ordinary', 'subnormal', 'special'] for o in ['div', 'sqrt']}
    for row in report['scenarios']:
        assert row['hot_heap_allocations'] == 0 and row['input_bytes'] == 16384
        for key in ['software_samples_ns', 'native_samples_ns', 'software_cpu_samples_ns', 'native_cpu_samples_ns']:
            assert len(row[key]) == 7 and all(x > 0 for x in row[key])
    if destination := os.environ.get('SOFT_BINARY64_DIVSQRT_PERF_ARTIFACTS'):
        folder = Path(destination)
        folder.mkdir(parents=True, exist_ok=True)
        report.update(compile_command=command, compiler=subprocess.check_output([command[0], '--version'], text=True).splitlines()[0], platform=platform.platform(), cpu=Path('/proc/cpuinfo').read_text().split('model name',1)[1].splitlines()[0], source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in [source, PACKAGE / 'shaders/soft_binary64.slang', Path(__file__)]})
        (folder / 'benchmark.json').write_text(json.dumps(report, indent=2)+'\n')
