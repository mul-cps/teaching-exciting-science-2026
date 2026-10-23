"""CPU-only arithmetic qualification; no ROS, Vulkan context or GPU required."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import struct

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2/src/so101_station_vulkan"
MODULE = PACKAGE / "shaders/soft_binary64.slang"
HOST = PACKAGE / "test/soft_binary64_host_test.cpp"
FLAGS = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-ffp-contract=off", "-fno-fast-math", "-frounding-math"]


def compile_host(source, output):
    subprocess.run([os.environ.get("CXX", "g++"), *FLAGS, str(source), "-o", str(output)], check=True, capture_output=True, text=True)


def test_integer_source_contract():
    source = MODULE.read_text()
    source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
    assert not re.search(r"\b(double|float|float16_t|float64_t|__int128|uint128_t|int128_t)\b", source)
    assert not re.search(r"\b(import|RWStructuredBuffer|groupshared|static)\b", source)
    # Enforce absence of file-scope variables or mutable state. Only functions,
    # a product value struct and preprocessor directives are allowed at depth 0.
    depth = 0
    for line in source.splitlines():
        stripped = line.strip()
        if depth == 0 and stripped and not stripped.startswith("#"):
            assert re.match(r"(?:uint64_t|int|bool|Sf64Product)\s+\w+\s*\(|struct\s+Sf64Product\s*\{", stripped), stripped
        depth += line.count("{") - line.count("}")
    assert depth == 0


def test_hardware_oracle_and_correction_mutants(tmp_path):
    executable = tmp_path / "host"
    compile_host(HOST, executable)
    run = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
    assert "random_pairs=1000000 random_conversions=200000" in run.stdout
    # Both defects are plausible silent numeric corruptions. They must compile
    # successfully and fail behavioral assertions against independent results.
    mutations = {
        "tie": ("if (roundBits == 0x200) sig &= ~uint64_t(1);", "if (false) sig &= ~uint64_t(1);"),
        "pack_or": ("(uint64_t(sign) << 63) + (uint64_t(exp) << 52) + sig", "(uint64_t(sign) << 63) | (uint64_t(exp) << 52) | sig"),
    }
    for name, (old, new) in mutations.items():
        source = MODULE.read_text()
        assert source.count(old) == 1
        mutant = tmp_path / f"{name}.slang"
        mutant.write_text(source.replace(old, new))
        harness = tmp_path / f"{name}.cpp"
        harness.write_text(HOST.read_text().replace('"../shaders/soft_binary64.slang"', f'"{mutant}"'))
        binary = tmp_path / name
        compile_host(harness, binary)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        assert result.returncode == 1, (name, result.stdout, result.stderr)
        assert "got=" in result.stderr


def test_slang_spirv_integer_capability(tmp_path):
    import pytest
    compiler = shutil.which("slangc")
    candidate = Path.home() / ".local/opt/slang-2026.19/bin/slangc"
    if compiler is None and candidate.is_file():
        compiler = str(candidate)
    if compiler is None:
        pytest.skip("Slang compiler unavailable; CPU conformance remains covered")
    entry = tmp_path / "probe.slang"
    entry.write_text(f'#include "{MODULE}"\n' + """
RWStructuredBuffer<uint64_t> data;
[shader("compute")][numthreads(1,1,1)]
void main(uint3 tid : SV_DispatchThreadID) {
    uint64_t a=data[tid.x*2], b=data[tid.x*2+1];
    data[tid.x*2]=sf64_add_bits(a,b)^sf64_mul_bits(a,b)^sf64_from_f32_bits(uint32_t(a));
    data[tid.x*2+1]=uint64_t(sf64_less(a,b))|uint64_t(sf64_less_equal(a,b))<<1|uint64_t(sf64_is_finite(a))<<2;
}
""")
    assembly = tmp_path / "probe.spvasm"
    subprocess.run([compiler, str(entry), "-entry", "main", "-stage", "compute", "-target", "spirv-asm", "-profile", "spirv_1_5", "-o", str(assembly)], check=True, capture_output=True, text=True)
    text = assembly.read_text()
    binary = tmp_path / "probe.spv"
    subprocess.run([compiler, str(entry), "-entry", "main", "-stage", "compute", "-target", "spirv", "-profile", "spirv_1_5", "-o", str(binary)], check=True, capture_output=True, text=True)
    raw = binary.read_bytes()
    words = struct.unpack("<" + "I" * (len(raw) // 4), raw)
    assert words[0] == 0x07230203
    capabilities = set()
    cursor = 5
    while cursor < len(words):
        length, opcode = words[cursor] >> 16, words[cursor] & 65535
        assert length > 0
        if opcode == 17:  # OpCapability
            capabilities.add(words[cursor + 1])
        if opcode == 22:  # OpTypeFloat
            assert words[cursor + 2] != 64
        cursor += length
    assert cursor == len(words)
    assert 11 in capabilities  # Int64
    assert 10 not in capabilities  # Float64
    assert "OpCapability Int64" in text
    assert "Float64" not in text
    assert not re.search(r"OpTypeFloat\s+64\b", text)


def test_upstream_testfloat_vectors(tmp_path):
    import pytest
    generator = os.environ.get("SF64_TESTFLOAT_GEN")
    if not generator:
        pytest.skip("Set SF64_TESTFLOAT_GEN to unmodified upstream testfloat_gen for additional oracle vectors")
    executable = tmp_path / "host"
    compile_host(HOST, executable)
    for operation, count in (("f64_add", 46464), ("f64_mul", 46464), ("f32_to_f64", 600)):
        vectors = tmp_path / (operation + ".txt")
        with vectors.open("w") as stream:
            subprocess.run([generator, "-level", "1", "-rnear_even", "-tininessafter", operation], stdout=stream, check=True)
        result = subprocess.run([str(executable), operation, str(vectors)], check=True, capture_output=True, text=True)
        assert f"vectors={count}" in result.stdout
