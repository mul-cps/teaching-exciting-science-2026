"""CPU contracts for selected camera grids; never opens USB or creates Vulkan."""
from pathlib import Path
import os
import shutil
import subprocess
import pytest

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def test_actual_profile_identity_and_cold_coverage(tmp_path):
    sdk = os.environ.get('RAW_BRIDGE_SDK_PREFIX')
    if not sdk:
        pytest.skip('set RAW_BRIDGE_SDK_PREFIX for the CPU SDK software-frame tests')
    binary = tmp_path / 'camera-profile-contracts'
    command = [os.environ.get('CXX') or shutil.which('c++'), '-O2', '-std=c++17', '-pthread', '-ffp-contract=off',
               '-I', str(P / 'include'), '-I', sdk + '/include',
               *[str(P / name) for name in ('test/raw_camera_admission_test.cpp', 'src/raw_camera_admission.cpp',
                    'src/alignment_reference.cpp', 'src/vulkan_context.cpp', 'src/frame_pool.cpp')],
               '-L', sdk + '/lib', '-Wl,-rpath,' + sdk + '/lib', '-lrealsense2', '-lvulkan', '-o', str(binary)]
    built = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'high_resolution_profiles cold_native_depth_coverage' in result.stdout


def cpp_body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start:end + 1]
    raise AssertionError('unbalanced C++ function')


def test_actual_copy_rows_requires_exact_destination_plane_size_and_checks_padding(tmp_path):
    camera = (P / 'src/camera_component.cpp').read_text()
    unit = tmp_path / 'rows.cpp'
    unit.write_text(r'''
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <limits>
#include <cassert>
namespace rs2 {
struct video_frame {
 const uint8_t* bytes; int width=4,height=3,stride=16,bpp=3; size_t size=44;
 const void* get_data()const{return bytes;} int get_width()const{return width;}
 int get_height()const{return height;} int get_stride_in_bytes()const{return stride;}
 int get_bytes_per_pixel()const{return bpp;} size_t get_data_size()const{return size;}
};
}
void copy_rows(uint8_t* destination,size_t capacity,const rs2::video_frame& source,unsigned row_bytes)
''' + cpp_body(camera, 'void copy_rows(') + r'''
template<class F>void rejects(F f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}assert(bad);}
int main(){
 std::array<uint8_t,48> bytes;for(unsigned i=0;i<bytes.size();i++)bytes[i]=uint8_t(i);
 std::array<uint8_t,36> out{};rs2::video_frame source{bytes.data()};
 copy_rows(out.data(),out.size(),source,12);
 for(unsigned y=0;y<3;y++)for(unsigned x=0;x<12;x++)assert(out[12*y+x]==bytes[16*y+x]);
 source.stride=12;source.size=36;copy_rows(out.data(),out.size(),source,12);
 assert(std::memcmp(out.data(),bytes.data(),36)==0);
 // A short SDK-aligned-like source must never leave the last destination row stale.
 auto short_aligned=source;short_aligned.height=2;
 rejects([&]{copy_rows(out.data(),out.size(),short_aligned,12);});
 rejects([&]{copy_rows(out.data(),37,source,12);});
 rejects([&]{copy_rows(out.data(),35,source,12);});
 for(int height:{0,-1,std::numeric_limits<int>::max()}){auto bad=source;bad.height=height;rejects([&]{copy_rows(out.data(),out.size(),bad,12);});}
 for(int stride:{0,-1,11}){auto bad=source;bad.stride=stride;rejects([&]{copy_rows(out.data(),out.size(),bad,12);});}
 source.stride=16;source.size=43;rejects([&]{copy_rows(out.data(),out.size(),source,12);});
 source.size=44;source.width=5;rejects([&]{copy_rows(out.data(),out.size(),source,12);});
 source.width=4;source.bpp=2;rejects([&]{copy_rows(out.data(),out.size(),source,12);});
 source.bpp=3;source.bytes=nullptr;rejects([&]{copy_rows(out.data(),out.size(),source,12);});
 source.bytes=bytes.data();rejects([&]{copy_rows(nullptr,out.size(),source,12);});
 rejects([&]{copy_rows(out.data(),out.size(),source,0);});
}
''')
    binary = tmp_path / 'rows'
    subprocess.run([shutil.which('c++'), '-std=c++17', '-O2', str(unit), '-o', str(binary)], check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)


def test_selected_profile_is_explicit_and_all_plane_extents_are_reported():
    camera = (P / 'src/camera_component.cpp').read_text()
    launch = (P / 'launch/resident_rgbd.launch.py').read_text()
    assert 'declare_parameter<std::string>("profile","realtime")' in camera
    assert "DeclareLaunchArgument('profile', default_value='realtime', choices=['realtime', 'medium', 'high_quality', 'vga', '720p', '1080p'])" in launch
    assert "'profile': ParameterValue(LaunchConfiguration('profile'), value_type=str)" in launch
    assert 'kWidth' not in camera and 'kHeight' not in camera and 'kPixels' not in camera
    for field in ('profile=%s', 'color_width=%u', 'color_height=%u', 'depth_width=%u', 'depth_height=%u', 'fps=%u', 'vulkan_allocation_count=%llu'):
        assert field in cpp_body(camera, 'void report_status(')
    assert 'ci.width=k.width;ci.height=k.height;' in camera
    assert camera.count('validate_selected_profiles(') >= 5
    assert 'get_bytes_per_pixel()' in cpp_body(camera, 'CameraSourceObservation inspect_original(')
