"""Compile the exact upstream patch predicate without loading ROS or the SDK."""
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'ros2/patches/realsense-ros-4.58.4-pointcloud-texture-bounds.patch'


def predicate():
    added = '\n'.join(line[1:] for line in PATCH.read_text().splitlines() if line.startswith('+') and not line.startswith('+++'))
    return added[added.index('static inline bool texture_sample_offset'):added.index('        if (!texture_frame)')]


def test_actual_cpp_predicate_boundaries(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler missing')
    program = '#include <cmath>\n#include <cstddef>\n#include <cassert>\n#include <limits>\n' + predicate() + r'''
int main() {
    size_t o=0;
    const int w=1920,h=1080,s=w*3;
    const size_t n=size_t(s)*h;
    assert(texture_sample_offset(0,0,w,h,s,3,n,o) && o==0);
    const float last=std::nextafter(1.f,0.f);
    assert(texture_sample_offset(last,0,w,h,s,3,n,o) && o==size_t(w-1)*3);
    assert(texture_sample_offset(0,last,w,h,s,3,n,o) && o==size_t(h-1)*s);
    assert(texture_sample_offset(last,last,w,h,s,3,n,o) && o==n-3);
    // Dense deterministic grid: every admitted sample selects the identical
    // original float-product index. No new double-product interior semantics.
    for(int q=0;q<100000;++q) {
        const float u=float(q)/100000.f;
        const float v=float((q*7919)%100000)/100000.f;
        const float x=u*w,y=v*h;
        const bool original_safe=double(x)<w && double(y)<h;
        const bool accepted=texture_sample_offset(u,v,w,h,s,3,n,o);
        assert(accepted==original_safe);
        if(accepted) assert(o==size_t(static_cast<int>(y))*s+size_t(static_cast<int>(x))*3);
    }
    // Check each nextafter edge against the actual original float product;
    // reject if rounding reaches the bound, before converting to integer.
    for(int dim: {1,3,640,1080,1920,16777219}) {
        const float product=last*dim;
        const bool safe=double(product)<dim;
        const bool accepted=texture_sample_offset(last,0,dim,1,dim,1,size_t(dim),o);
        assert(accepted==safe);
        if(accepted) assert(o==size_t(static_cast<int>(product)));
    }
    for(float bad: {1.f,-0.01f,std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}) {
        assert(!texture_sample_offset(bad,.5f,w,h,s,3,n,o));
        assert(!texture_sample_offset(.5f,bad,w,h,s,3,n,o));
    }
    assert(!texture_sample_offset(.5f,.5f,w,h,s,3,n-1,o));
    assert(!texture_sample_offset(.5f,.5f,w,h,s-1,3,n,o));
    assert(!texture_sample_offset(.5f,.5f,w,h,0,3,n,o));
    assert(!texture_sample_offset(.5f,.5f,w,h,s,5,n,o));
    assert(!texture_sample_offset(.5f,.5f,0,h,s,3,n,o));
    assert(texture_sample_offset(last,last,w,h,s+8,3,size_t(s+8)*h,o));
    assert(o==size_t(h-1)*(s+8)+size_t(w-1)*3);
}
'''
    program = program.replace('#include <cassert>', '#include <cassert>\n#include <initializer_list>')
    source = tmp_path / 'predicate.cpp'
    source.write_text(program)
    binary = tmp_path / 'predicate'
    subprocess.run([compiler, '-std=c++17', '-O2', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_patch_copy_uses_validated_offset_and_stride():
    patch = PATCH.read_text()
    assert '+            const bool valid_color_pixel = texture_sample_offset' in patch
    assert '+        const int texture_stride = texture_frame.get_stride_in_bytes();' in patch
    assert '-                    int offset = (pixy * texture_width + pixx) * num_colors;' in patch
    assert '+                    reverse_memcpy' not in patch  # existing copy remains after predicate


def test_builder_source_inventory_rejects_extra_file(tmp_path, monkeypatch):
    import io
    import sys
    import tarfile
    build = (ROOT / 'ros2/conda/build-realsense-cpu-texture-bounds.sh').read_text()
    script = build.split("<<'PYCPUCHECK'\n", 1)[1].split('\nPYCPUCHECK', 1)[0]
    archive = tmp_path / 'source.tar.gz'
    with tarfile.open(archive, 'w:gz') as target:
        member = tarfile.TarInfo('wrapper/source.cpp')
        member.size = 4
        target.addfile(member, io.BytesIO(b'good'))
    source = tmp_path / 'source'
    (source / 'wrapper').mkdir(parents=True)
    (source / 'wrapper/source.cpp').write_bytes(b'good')
    monkeypatch.setattr(sys, 'argv', ['source-check', str(archive), str(source)])
    exec(compile(script, 'source-check', 'exec'), {})
    (source / 'wrapper/injected.cpp').write_text('unexpected CMake glob input')
    with pytest.raises(AssertionError, match='Source inventory differs'):
        exec(compile(script, 'source-check', 'exec'), {})


def test_builder_provenance_binds_builder_and_final_source():
    build = (ROOT / 'ros2/conda/build-realsense-cpu-texture-bounds.sh').read_text()
    assert "'builder_sha256':hashlib.sha256(pathlib.Path(sys.argv[5]).read_bytes()).hexdigest()" in build
    assert "'source_final_sha256':source_digest" in build
    assert "'source_file_sha256':source_files" in build


def test_cpu_builder_has_no_glsl_source_dependency():
    build = (ROOT / 'ros2/conda/build-realsense-cpu-texture-bounds.sh').read_text()
    assert 'glsl-align.patch' not in build
    assert 'librealsense-$SDK_SHA' not in build
    assert 'BUILD_ACCELERATE_GPU_WITH_GLSL=OFF' in build
    assert 'build-realsense-cpu-texture-bounds.sh' in build
