"""Offline corpus tests: compile/run native oracle; never construct Vulkan objects."""
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tools/performance/qualify_soft_binary64.cpp'


@pytest.fixture(scope='module')
def executable(tmp_path_factory):
    path = tmp_path_factory.mktemp('soft-binary64-probe') / 'probe'
    command = [os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
               '-ffp-contract=off', '-fno-fast-math', '-frounding-math',
               '-DSOFT_BINARY64_CPU_ONLY', str(SOURCE), '-o', str(path)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    # This build cannot create a Vulkan instance even if the mode dispatch regresses.
    symbols = subprocess.check_output(['nm', '-u', str(path)], text=True)
    assert 'vkCreate' not in symbols and 'Context' not in symbols
    return path


def invoke(executable, *args, success=True):
    result = subprocess.run([str(executable), *map(str, args)], text=True,
                            capture_output=True, timeout=30)
    if success:
        assert result.returncode == 0, result.stderr
        return json.loads(result.stdout)
    assert result.returncode != 0
    return json.loads(result.stderr)


def rows(path):
    data = path.read_bytes()
    magic, count, seed = struct.unpack_from('<8sQQ', data)
    assert magic == b'SF64P001'
    assert len(data) == 24 + count * 56
    return seed, list(struct.iter_unpack('<7Q', data[24:]))


def test_determinism_sha_header_and_oracle(executable, tmp_path):
    a, b, c = [tmp_path / name for name in ('a.bin', 'b.bin', 'c.bin')]
    report = invoke(executable, '--generate', a)
    invoke(executable, '--generate', b, '--count', 65536, '--seed', 17)
    invoke(executable, '--generate', c, '--seed', 18)
    assert a.read_bytes() == b.read_bytes() != c.read_bytes()
    assert report['corpus_sha256'] == hashlib.sha256(a.read_bytes()).hexdigest()
    assert report['generator'] == 'native-volatile-ieee754-rne-v1'
    assert report['case_count'] == 65536 and report['seed'] == 17
    assert report['gpu_executed'] is False
    validated = invoke(executable, '--validate', a)
    assert validated['corpus_sha256'] == report['corpus_sha256']
    seed, cases = rows(a)
    assert seed == 17 and len(cases) == 65536
    by_pair = {(r[0], r[1]): r for r in cases}
    assert by_pair[0, 0][3:5] == (0, 0)
    assert by_pair[0x8000000000000000, 0x8000000000000000][3:5] == (0x8000000000000000, 0)
    assert by_pair[0x7ff0000000000000, 0][4] == 0xfff8000000000000
    assert by_pair[1, 0x3fe0000000000000][4] == 0
    assert by_pair[3, 0x3fe0000000000000][4] == 2
    assert by_pair[0x7fefffffffffffff, 0x4000000000000000][4] == 0x7ff0000000000000
    assert by_pair[0, 0][6] == 14  # equal and both finite
    assert any(r[2] == 1 and r[5] == 0x36a0000000000000 for r in cases)
    assert any(r[2] == 0x80000000 and r[5] == 0x8000000000000000 for r in cases)
    for r in cases:
        for expected in r[3:6]:
            if expected & 0x7fffffffffffffff > 0x7ff0000000000000:
                assert expected == 0xfff8000000000000


def test_no_overwrite_and_invalid_generation(executable, tmp_path):
    output = tmp_path / 'immutable.bin'
    output.write_bytes(b'keep-existing')
    invoke(executable, '--generate', output, success=False)
    assert output.read_bytes() == b'keep-existing'
    for args in [('--count', 0), ('--count', 4000001), ('--seed', 0),
                 ('--seed', -1), ('--unknown', 3)]:
        destination = tmp_path / 'never-created.bin'
        invoke(executable, '--generate', destination, *args, success=False)
        assert not destination.exists()


def test_parser_rejects_corruption(executable, tmp_path):
    original = tmp_path / 'source.bin'
    invoke(executable, '--generate', original, '--count', 16384)
    data = original.read_bytes()
    mutations = [b'', b'BADMAGIC' + data[8:], data[:-1], data + b'x',
                 data[:8] + struct.pack('<Q', 4000001) + data[16:]]
    for offset in (24 + 2 * 8, 24 + 3 * 8, 24 + 6 * 8):
        changed = bytearray(data)
        changed[offset + (7 if offset == 24 + 2 * 8 else 0)] ^= 0x80
        mutations.append(changed)
    for i, mutated in enumerate(mutations):
        path = tmp_path / f'corrupt-{i}.bin'
        path.write_bytes(mutated)
        invoke(executable, '--validate', path, success=False)


def test_upstream_rows_are_validated_before_append(executable, tmp_path):
    upstream = tmp_path / 'upstream'
    upstream.mkdir()
    (upstream / 'f64_add.txt').write_text('3FF0000000000000 3FF0000000000000 4000000000000000 00\n')
    (upstream / 'f64_mul.txt').write_text('0000000000000000 7FF0000000000000 7FF8000000000000 10\n')
    (upstream / 'f32_to_f64.txt').write_text('00000001 36A0000000000000 00\n')
    path = tmp_path / 'with-upstream.bin'
    report = invoke(executable, '--generate', path, '--count', 16384,
                    '--testfloat-dir', upstream)
    assert report['testfloat_rows'] == 3 and report['case_count'] == 16387
    invoke(executable, '--validate', path)
    assert rows(path)[1][-1][5] == 0x36a0000000000000
    (upstream / 'f64_add.txt').write_text('3FF0000000000000 3FF0000000000000 3FF0000000000000 00\n')
    rejected = tmp_path / 'rejected.bin'
    error = invoke(executable, '--generate', rejected, '--count', 16384,
                   '--testfloat-dir', upstream, success=False)
    assert 'disagrees with TestFloat' in error['error']
    assert not rejected.exists()


def test_probe_is_diagnostic_and_calls_all_six_apis():
    shader = (ROOT / 'tools/performance/soft_binary64_probe.slang').read_text()
    for name in ('add_bits', 'mul_bits', 'from_f32_bits', 'less', 'less_equal', 'is_finite'):
        assert f'sf64_{name}(' in shader
    source = SOURCE.read_text()
    assert '#include "soft_binary64.slang"' not in source
    assert 'vkDeviceWaitIdle' not in source and 'vkQueueWaitIdle' not in source
    assert 'constexpr uint32_t kBatch=16384' in source
    assert 'SOFT_BINARY64_EXTERNAL_WATCHDOG' in source
    assert 'if(in_flight)std::_Exit(2)' in source


def test_generator_restores_callers_fenv(tmp_path):
    # Include the actual CLI in-process to inspect its caller state after --generate.
    harness = tmp_path / 'environment.cpp'
    harness.write_text('#define SOFT_BINARY64_CPU_ONLY\n#define main probe_main\n'
                       '#include "' + str(SOURCE) + '"\n#undef main\n'
                       'int main(int argc,char** argv) {\n'
                       ' std::fesetround(FE_DOWNWARD);std::feraiseexcept(FE_INEXACT);\n'
                       '#if defined(__SSE__)\n'
                       ' _mm_setcsr(_mm_getcsr() | (1u<<15) | (1u<<6));auto csr=_mm_getcsr();\n'
                       '#endif\n'
                       ' int result=probe_main(argc,argv);\n'
                       ' if(result || std::fegetround()!=FE_DOWNWARD || !(std::fetestexcept(FE_INEXACT)&FE_INEXACT)) return 3;\n'
                       '#if defined(__SSE__)\n'
                       ' if(_mm_getcsr()!=csr)return 4;\n'
                       '#endif\n return 0; }\n')
    binary = tmp_path / 'environment'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                    '-fno-fast-math', '-frounding-math', str(harness),
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    invoke(binary, '--generate', tmp_path / 'restored.bin', '--count', 16384)


def test_offline_spirv_capability_admission(executable, tmp_path):
    # The structural gate is separate from spirv-val; it rejects forbidden/missing
    # capabilities before device creation. Complete shader validation is offline.
    def module(*capabilities):
        return struct.pack('<5I', 0x07230203, 0x10000, 0, 1, 0) + b''.join(
            struct.pack('<2I', (2 << 16) | 17, cap) for cap in capabilities)
    path = tmp_path / 'capabilities.spv'
    good = module(1, 11)
    path.write_bytes(good)
    report = invoke(executable, '--validate-spirv', path)
    assert report['int64'] is True and report['float64'] is False
    assert report['gpu_executed'] is False
    assert report['spirv_sha256'] == hashlib.sha256(good).hexdigest()
    for malformed in (module(1), module(1, 10, 11), good[:-1],
                      good[:20] + struct.pack('<I', 0), b'\0' * 20):
        path.write_bytes(malformed)
        invoke(executable, '--validate-spirv', path, success=False)
