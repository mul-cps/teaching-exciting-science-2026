"""CPU-only admission, ABI and frozen-kernel guards for the opt-in prototype."""
from pathlib import Path
import hashlib

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2/src/so101_station_vulkan'


def test_software_precision_is_explicit_and_actual_int64_enabled():
    header = (PACKAGE / 'include/so101_station_vulkan/geometry.hpp').read_text()
    source = (PACKAGE / 'src/geometry.cpp').read_text()
    assert 'enum class GeometryPrecision { NativeFloat64, SoftwareBinary64 };' in header
    assert 'GeometryPrecision::NativeFloat64' in source
    assert 'case GeometryPrecision::SoftwareBinary64:' in source
    assert 'enabled_features.features.shaderInt64' in source
    assert 'invalid geometry precision' in source


def test_software_uses_bit_abi_ordered_comparisons_and_frozen_sampling():
    native = (PACKAGE / 'shaders/geometry.slang').read_text()
    assert hashlib.sha256(native.encode()).hexdigest() == '2c8e3f34a07e3efedca035a62fb5ccb97331b28d8c302d788de569625a6694b1'
    soft = (PACKAGE / 'shaders/geometry_soft.slang').read_text()
    assert 'StructuredBuffer<uint64_t> calibration' in soft
    assert '#include "soft_binary64.slang"' in soft
    assert 'double' not in soft and 'sf64_mul_bits' in soft and 'sf64_add_bits' in soft
    assert 'sf64_from_f32_bits(zbits)' in soft
    assert 'sf64_from_f32_bits(0x3dcccccdu)' in soft  # exact .10f
    assert 'sf64_less_equal' in soft
    # Scan, offsets and reduction must stay byte-identical to the original.
    a = '// Inclusive Hillis-Steele scan'
    b = '[numthreads(256,1,1)]\nvoid geometry_support_scatter'
    assert soft.split(a)[1].split(b)[0] == native.split(a)[1].split(b)[0]
    a = '// Exact integer reduction'
    assert soft.split(a)[1] == native.split(a)[1]
    for line in native.splitlines():
        if '[unroll]for(int dy=' in line or 'if(sx>=0&&sy>=0' in line:
            assert line in soft
    assert 'arena.Store<uint64_t>(selectedOffset()+24*rank,arena.Load<uint64_t>(24*i))' in soft


def test_cache_tracks_helper_parent_and_separate_opt_in_build():
    cmake = (PACKAGE / 'cmake/software_geometry.cmake').read_text()
    for token in ('soft_binary64.slang', 'geometry_soft.slang', 'string(SHA256',
                  'CMAKE_CONFIGURE_DEPENDS', 'PRECOMPILED_SOFTWARE_GEOMETRY_SHADER_DIR',
                  'provenance.txt', 'source.sha256', 'software-geometry-shaders',
                  'SOFTWARE_GEOMETRY_PARENT_HASH', 'SOFTWARE_GEOMETRY_HELPER_HASH'):
        assert token in cmake
    assert 'DEPENDS shaders/geometry_soft.slang shaders/soft_binary64.slang' in cmake
    assert '"${SOFTWARE_GEOMETRY_IDENTITY}" "${SOFTWARE_GEOMETRY_COMPILER_PATH}"' in cmake
    assert 'CMAKE_CONFIGURE_DEPENDS "${SOFTWARE_GEOMETRY_COMPILER_PATH}"' in cmake
    qualifier = (PACKAGE / 'test/qualify_geometry.cpp').read_text()
    assert '--software-binary64' in qualifier
    assert 'software_binary64' in qualifier and 'physical_camera' in qualifier


def test_actual_shader_transform_matches_ordered_native_binary64_on_cpu(tmp_path):
    """Compile the shader's real arithmetic and flag expressions as C++.

    This proves operation ordering on the CPU; it does not execute SPIR-V.
    """
    import os
    import subprocess
    shader = (PACKAGE / 'shaders/geometry_soft.slang').read_text()
    transform = shader.split('uint64_t transform(', 1)[1].split('[numthreads', 1)[0]
    coordinates = shader.split(' uint64_t z=sf64_from_f32_bits(zbits);', 1)[1].split(' arena.Store<uint64_t>', 1)[0]
    valid = shader.split(' bool valid=', 1)[1].split(';', 1)[0]
    inside = shader.split(' bool inside=', 1)[1].split(';', 1)[0]
    harness = tmp_path / 'coordinates.cpp'
    harness.write_text('''#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>
#include <random>
#include <iostream>
using uint=uint32_t;
''' + f'#include "{PACKAGE / "shaders/soft_binary64.slang"}"\n' + '''
struct {uint count=1;} p;
uint64_t calibration[21];
uint64_t bits(double x){uint64_t r;std::memcpy(&r,&x,8);return r;}
''' + 'uint64_t transform(' + transform + '''
int main(){
 std::mt19937_64 rng(20261023);std::uniform_real_distribution<double> random(-2.,2.);
 for(uint trial=0;trial<20000;trial++){
  double values[21];for(uint k=0;k<21;k++)values[k]=random(rng);
  for(uint k=14;k<20;k+=2){values[k]=-.5;values[k+1]=.5;}
  values[20]=double(float(1.2));for(uint k=0;k<21;k++)calibration[k]=bits(values[k]);
  float depth=float(random(rng));
  if(trial<8){const float edge[]={.10f,std::nextafter(.10f,0.f),1.2f,std::nextafter(1.2f,2.f),0.f,-0.f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()};depth=edge[trial];}
  uint zbits;std::memcpy(&zbits,&depth,4);uint i=0;
  uint64_t z=sf64_from_f32_bits(zbits);
''' + coordinates + '''
  double native[3];double nx=values[0]*double(depth),ny=values[1]*double(depth);
  for(uint row=0;row<3;row++){
   uint b=2+4*row;double v=values[b]*nx;
   v=v+values[b+1]*ny;v=v+values[b+2]*double(depth);native[row]=v+values[b+3];
  }
  uint64_t actual[]={bx,by,bz};
  for(uint k=0;k<3;k++)if(!std::isnan(native[k])&&bits(native[k])!=actual[k])return 1;
  uint base=14;
  bool valid=''' + valid + ''';
  bool inside=''' + inside + ''';
  bool expected_valid=std::isfinite(depth)&&double(depth)>=double(.10f)&&double(depth)<=values[20];
  bool expected_inside=native[0]>=values[14]&&native[0]<=values[15]&&native[1]>=values[16]&&native[1]<=values[17]&&native[2]>=values[18]&&native[2]<=values[19];
  if(valid!=expected_valid||inside!=expected_inside)return 2;
 }
 std::cout<<"20000 shader-coordinate and ordered-flag CPU cases passed\\n";
}
''')
    binary = tmp_path / 'coordinates'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-ffp-contract=off', '-fno-fast-math', str(harness), '-o', str(binary)], check=True, capture_output=True, text=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert '20000 shader-coordinate' in result.stdout


def test_compiler_identity_mode_switch_and_offline_cache_are_behavioral(tmp_path):
    """Real compilation counters catch stale SPVs and premature provenance claims."""
    import json
    import shutil
    import subprocess
    import sys
    import pytest
    compiler = Path.home() / '.local/opt/slang-2026.19/bin/slangc'
    if not compiler.is_file():
        pytest.skip('Slang unavailable: build identity test needs real offline compilation')
    fixture = tmp_path / 'source'
    for directory in ('shaders', 'cmake', 'third_party/softfloat'):
        (fixture / directory).mkdir(parents=True)
    for name in ('geometry_soft.slang', 'soft_binary64.slang', 'hash_shader.cmake'):
        shutil.copy2(PACKAGE / 'shaders' / name, fixture / 'shaders' / name)
    for name in ('NOTICE', 'provenance.json'):
        shutil.copy2(PACKAGE / 'third_party/softfloat' / name, fixture / 'third_party/softfloat' / name)
    shutil.copy2(PACKAGE / 'cmake/software_geometry.cmake', fixture / 'cmake/software_geometry.cmake')
    (fixture / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(identity_test NONE)\nset(GEOMETRY_ENTRIES geometry_coordinates)\ninclude(cmake/software_geometry.cmake)\n')
    counter = tmp_path / 'invocations.json'
    counter.write_text('[]')
    wrapper = tmp_path / 'compiler'

    def write_compiler(version):
        wrapper.write_text(f'#!{sys.executable}\n' + f'''import json, pathlib, subprocess, sys
if sys.argv[1:] == ['-version']:
 print({version!r}); sys.exit(0)
p = pathlib.Path({str(counter)!r})
records = json.loads(p.read_text()); records.append({version!r}); p.write_text(json.dumps(records))
sys.exit(subprocess.call([{str(compiler)!r}, *sys.argv[1:]]))
''')
        wrapper.chmod(0o755)

    def command(*args, ok=True):
        # The workspace wrapper provides the canonical clean build environment.
        cmd = [str(ROOT / 'ros2/conda/so101-conda.sh'), 'run', 'cmake', *map(str, args)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if ok:
            assert result.returncode == 0, result.stdout + result.stderr
        else:
            assert result.returncode != 0
        return result.stdout + result.stderr

    build = tmp_path / 'build'
    output = build / 'software-geometry-shaders'
    def configure(*args, ok=True):
        return command('-S', fixture, '-B', build, f'-DSLANGC_EXECUTABLE={wrapper}', *args, ok=ok)
    def compile():
        command('--build', build, '--target', 'software_geometry_spirv')
    write_compiler('wrapper-v1')
    configure()
    compile()
    assert json.loads(counter.read_text()) == ['wrapper-v1']
    provenance = output / 'provenance.txt'
    previous = provenance.read_text()
    previous_mtime = provenance.stat().st_mtime_ns
    spv_mtime = (output / 'geometry_coordinates.spv').stat().st_mtime_ns
    configure()
    compile()
    assert json.loads(counter.read_text()) == ['wrapper-v1']
    assert provenance.stat().st_mtime_ns == previous_mtime
    assert (output / 'geometry_coordinates.spv').stat().st_mtime_ns == spv_mtime

    # Same path, changed binary/version: configure must not mislabel old SPVs.
    write_compiler('wrapper-v2')
    configure()
    assert provenance.read_text() == previous
    compile()
    assert json.loads(counter.read_text()) == ['wrapper-v1', 'wrapper-v2']
    assert 'version=wrapper-v2' in provenance.read_text()
    assert f'compiler_sha256={hashlib.sha256(wrapper.read_bytes()).hexdigest()}' in provenance.read_text()
    assert provenance.stat().st_mtime_ns >= (output / 'geometry_coordinates.spv.sha256').stat().st_mtime_ns

    # Compiler file dependency must also trigger automatic CMake regeneration.
    write_compiler('wrapper-v3')
    compile()
    assert json.loads(counter.read_text()) == ['wrapper-v1', 'wrapper-v2', 'wrapper-v3']
    assert 'version=wrapper-v3' in provenance.read_text()

    cache = tmp_path / 'offline'
    shutil.copytree(output, cache)
    configure(f'-DPRECOMPILED_SOFTWARE_GEOMETRY_SHADER_DIR={cache}')
    compile()
    assert provenance.read_bytes() == (cache / 'provenance.txt').read_bytes()
    assert len(json.loads(counter.read_text())) == 3
    configure('-DPRECOMPILED_SOFTWARE_GEOMETRY_SHADER_DIR=')
    compile()
    assert json.loads(counter.read_text()) == ['wrapper-v1', 'wrapper-v2', 'wrapper-v3', 'wrapper-v3']

    configure(f'-DPRECOMPILED_SOFTWARE_GEOMETRY_SHADER_DIR={cache}')
    helper = fixture / 'shaders/soft_binary64.slang'
    helper.write_text(helper.read_text() + '\n// invalidate helper only\n')
    assert 'transitive source digest mismatch' in configure(ok=False)
    # Installation must carry the source-derived binary attribution materials.
    helper.write_bytes((PACKAGE / 'shaders/soft_binary64.slang').read_bytes())
    configure()
    install = tmp_path / 'install'
    command('--install', build, '--prefix', install)
    for name in ('NOTICE', 'provenance.json'):
        assert (install / 'share/identity_test/software-geometry-shaders/third_party/softfloat' / name).read_bytes() == (PACKAGE / 'third_party/softfloat' / name).read_bytes()
