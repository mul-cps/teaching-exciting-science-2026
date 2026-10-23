"""Exercise actual CMake offline gates without ROS, a driver, or GPU dispatch."""
import hashlib
from pathlib import Path
import shutil
import subprocess

import pytest

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def digest(data):
    return hashlib.sha256(data).hexdigest()


@pytest.mark.parametrize('kind,parent,entries', [
    ('core', 'resident.slang', ('convert', 'candidates', 'connect', 'compress', 'reduce', 'gather', 'morphology', 'tiled_components')),
    ('alignment', 'alignment.slang', ('splat', 'finalize')),
])
def test_offline_cache_rejects_helper_only_mutation_and_parent_only_identity(tmp_path, kind, parent, entries):
    cmake = shutil.which('cmake')
    assert cmake
    source = tmp_path / 'source'
    shaders = source / 'shaders'
    shaders.mkdir(parents=True)
    for name in (parent, 'exact_divide.slang'):
        shutil.copyfile(P / 'shaders' / name, shaders / name)
    if kind == 'core':
        text = (P / 'CMakeLists.txt').read_text()
        fragment = text[text.index('set(SLANGC_EXECUTABLE'):text.index('add_library(resident_rgbd')]
        variable = 'PRECOMPILED_SHADER_DIR'
    else:
        text = (P / 'cmake/alignment.cmake').read_text()
        fragment = text[:text.index('add_library(resident_alignment')]
        variable = 'PRECOMPILED_ALIGNMENT_SHADER_DIR'
    (source / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(cache_guard NONE)\n' + fragment)
    cache = tmp_path / 'cache'
    cache.mkdir()
    # Opaque bytes are sufficient: source/digest validation, never shader execution.
    for entry in entries:
        data = entry.encode()
        (cache / (entry + '.spv')).write_bytes(data)
        (cache / (entry + '.spv.sha256')).write_text(f'{digest(data)}  {entry}.spv\n')
    parent_hash = digest((shaders / parent).read_bytes())
    helper_hash = digest((shaders / 'exact_divide.slang').read_bytes())
    transitive = digest(f'{parent}={parent_hash}\nexact_divide.slang={helper_hash}\n'.encode())
    def identity(value):
        if kind == 'core':
            (cache / 'provenance.txt').write_text(f'source_sha256={value}\n')
        else:
            (cache / 'source.sha256').write_text(value + '\n')
    def configure(label):
        return subprocess.run([cmake, '-S', str(source), '-B', str(tmp_path / label),
                               f'-D{variable}={cache}'], capture_output=True, text=True)
    identity(transitive)
    result = configure('matching')
    assert result.returncode == 0, result.stdout + result.stderr
    identity(parent_hash)
    assert configure('obsolete_parent_only').returncode != 0
    identity(transitive)
    with (shaders / 'exact_divide.slang').open('a') as stream:
        stream.write('// helper-only provenance mutation\n')
    result = configure('helper_mutated')
    assert result.returncode != 0
    assert 'source' in result.stderr and ('hash' in result.stderr or 'digest' in result.stderr)


def test_shared_helper_is_explicit_in_all_build_and_install_dependencies():
    for path, parent in ((P / 'CMakeLists.txt', 'resident.slang'),
                         (P / 'cmake/alignment.cmake', 'alignment.slang')):
        text = path.read_text()
        assert f'DEPENDS shaders/{parent} shaders/exact_divide.slang' in text
        assert f'install(FILES shaders/{parent} shaders/exact_divide.slang DESTINATION' in text
        configure = next(line for line in text.splitlines() if 'CMAKE_CONFIGURE_DEPENDS' in line)
        assert 'shaders/exact_divide.slang' in configure
    probe = (P / 'cmake/alignment_divide_probe.cmake').read_text()
    assert 'foreach(SOURCE shaders/alignment.slang shaders/exact_divide.slang test/alignment_divide_probe.slang)' in probe
    assert 'DEPENDS test/alignment_divide_probe.slang shaders/alignment.slang shaders/exact_divide.slang' in probe
