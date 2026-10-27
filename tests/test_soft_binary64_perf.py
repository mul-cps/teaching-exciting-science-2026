"""Bounded CPU performance/static guards; never link Vulkan or dispatch GPU work."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start:end + 1]
    raise AssertionError('unbalanced source')


def assert_bounded_primitive_loops(code):
    """Only the two audited integer recurrences may contain loop keywords."""
    allowed_positions = []
    for signature, header in (
        ('sf64_div_bits(', 'for (int bit = 61; bit >= 0; --bit)'),
        ('sf64_sqrt_bits(', 'for (int pair = 62; pair >= 0; --pair)'),
    ):
        function_body = body(code, signature)
        assert function_body.count(header) == 1, signature
        start = code.index('{', code.index(signature))
        allowed_positions.append(start + function_body.index(header))
    actual_positions = [match.start() for match in re.finditer(r'\b(?:for|while|do)\b', code)]
    assert actual_positions == sorted(allowed_positions), 'unexpected primitive loop'


@pytest.mark.parametrize('old,new', [
    ('bit >= 0', 'bit >= bound'),
    ('--bit', '++bit'),
    ('pair >= 0', 'pair >= -1'),
    ('for (int bit', 'for (;;) {} for (int bit'),
    ('for (int bit', 'while (true) {} for (int bit'),
    ('for (int bit', 'do {} while (false); for (int bit'),
])
def test_bounded_primitive_loops_reject_mutations(old, new):
    primitive = (PACKAGE / 'shaders/soft_binary64.slang').read_text()
    code = re.sub(r'/\*.*?\*/|//[^\n]*', '', primitive, flags=re.S)
    assert old in code
    with pytest.raises(AssertionError):
        assert_bounded_primitive_loops(code.replace(old, new, 1))


def test_bounded_primitive_loops_reject_relocation():
    code = (
        'uint64_t sf64_div_bits() { for (int pair = 62; pair >= 0; --pair) {} }'
        'uint64_t sf64_sqrt_bits() { for (int bit = 61; bit >= 0; --bit) {} }'
    )
    with pytest.raises(AssertionError):
        assert_bounded_primitive_loops(code)


def test_actual_shared_integer_coordinate_benchmark(tmp_path):
    shader = (PACKAGE / 'shaders/geometry_soft.slang').read_text()
    transform = 'uint64_t transform(' + shader.split('uint64_t transform(', 1)[1].split('[numthreads', 1)[0]
    coordinate = shader.split(' uint64_t z=sf64_from_f32_bits(zbits);', 1)[1].split(' arena.Store<uint64_t>', 1)[0]
    header = tmp_path / 'soft_binary64_geometry_expr.hpp'
    header.write_text('struct ShaderExpression {\nstruct {uint count=1;} p;\nuint64_t calibration[14];\n' + transform.replace('){', ') const{') + '\nstd::array<uint64_t,3> coordinate(uint zbits) const {uint i=0; uint64_t z=sf64_from_f32_bits(zbits);\n' + coordinate + '\nreturn {bx,by,bz};}};\n')
    binary = tmp_path / 'soft_binary64_perf'
    command = [os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-ffp-contract=off', '-fno-fast-math', '-frounding-math', '-I', str(tmp_path), str(PACKAGE / 'test/soft_binary64_perf_benchmark.cpp'), '-o', str(binary)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    report = json.loads(subprocess.check_output([str(binary), '20261023'], text=True))
    assert report['mul_per_coordinate'] == 11 and report['add_per_coordinate'] == 9
    assert len(report['scenarios']) == 3
    for row in report['scenarios']:
        assert row['hot_heap_allocations'] == 0
        assert row['software_median_ns'] > 0 and row['native_ordered_median_ns'] > 0
        assert len(row['software_samples_ns']) == len(row['native_samples_ns']) == 7
    if destination := os.environ.get('SOFT_BINARY64_PERF_ARTIFACTS'):
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        report['compile_command'] = command
        report['source_sha256'] = {str(p.relative_to(PACKAGE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (PACKAGE / 'shaders/soft_binary64.slang', PACKAGE / 'shaders/geometry_soft.slang', PACKAGE / 'test/soft_binary64_perf_benchmark.cpp')}
        (destination / 'benchmark.json').write_text(json.dumps(report, indent=2) + '\n')


def test_primitive_and_resident_hot_bodies_have_no_new_host_work():
    primitive = (PACKAGE / 'shaders/soft_binary64.slang').read_text()
    code = re.sub(r'/\*.*?\*/|//[^\n]*', '', primitive, flags=re.S)
    assert not re.search(r'\b(?:double|float|__int128|uint128_t|malloc|calloc|realloc|new|printf|cout|static|thread_local|extern)\b', code)
    assert_bounded_primitive_loops(code)
    # Every primitive call remains within its pure integer module or a scalar cast.
    calls = set(re.findall(r'\b(\w+)\s*\(', code))
    assert all(x.startswith('sf64_') or x in {'for', 'if', 'return', 'int', 'uint64_t', 'uint32_t', 'bool'} for x in calls)
    for file, signatures in {
        'src/geometry.cpp': ['GeometryPreparation::submit(', 'GeometryPreparation::finish('],
        'src/vulkan_context.cpp': ['Context::shared()'],
    }.items():
        current = (PACKAGE / file).read_text()
        baseline = subprocess.check_output(['git', 'show', f'c53d8d544bd6a7927d5cdd8029c354057c3b165c:ros2/src/so101_station_vulkan/{file}'], cwd=PACKAGE, text=True)
        for signature in signatures:
            assert body(current, signature) == body(baseline, signature)
