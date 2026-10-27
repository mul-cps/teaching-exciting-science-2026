"""CPU-only cost of the actual checked camera plane-copy helper.

Synthetic SDK getters supply preallocated native planes; this is local host
memory cost, not fwmini4, USB, SDK getter, driver or GPU performance.
"""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess

P = Path(__file__).resolve().parents[1]/'ros2/src/so101_station_vulkan'


def actual_body(source):
    start = source.index('{', source.index('void copy_rows('))
    depth = 1
    for end in range(start+1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start:end+1]
    raise AssertionError('unbalanced copy_rows')


def test_actual_copy_rows_preallocated_tight_and_padded_plane_pair_cost(tmp_path):
    camera = (P/'src/camera_component.cpp').read_text()
    helper = actual_body(camera)
    # There are no blocking operations or allocating helpers in this extracted
    # function. The separate raw-resident resource suite owns broader guards.
    assert not re.search(r'\b(?:new|malloc|calloc|realloc|wait|sleep|usleep)\b|\bvk(?:Wait|Allocate|Create|Map)\w*\b|std::(?:vector|this_thread)|\.(?:resize|reserve)\(', helper)
    calls = set(re.findall(r'\b([A-Za-z_]\w*(?:::\w+)*)\s*\(', helper))
    assert calls <= {'get_data','get_stride_in_bytes','get_height','get_width',
                     'get_bytes_per_pixel','get_data_size','size_t','unsigned','int',
                     'if','for','std::runtime_error','std::memcpy'}
    code = r'''
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <vector>
static bool hot=false;static size_t heap_allocations=0;
void* operator new(size_t n){if(hot)++heap_allocations;if(void* p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
namespace rs2 {
struct video_frame {
 const uint8_t* bytes;int width,height,stride,bpp;size_t size;
 const void* get_data()const{return bytes;}int get_width()const{return width;}
 int get_height()const{return height;}int get_stride_in_bytes()const{return stride;}
 int get_bytes_per_pixel()const{return bpp;}size_t get_data_size()const{return size;}
};
}
// Keep the actual production body unchanged; prevent the repeated memcpy calls
// being inlined/coalesced into one final observable copy by the host compiler.
__attribute__((noinline)) void copy_rows(uint8_t* destination,size_t capacity,const rs2::video_frame& source,unsigned row_bytes)
ACTUAL_BODY
void require(bool value){if(!value)throw std::runtime_error("actual plane-copy benchmark contract failed");}
constexpr size_t guard=64;constexpr uint8_t sentinel=0xa5,padding=0xed;
struct Plane{
 unsigned width,height,bpp,row,stride;size_t bytes;
 std::vector<uint8_t> source,destination;
 Plane(unsigned w,unsigned h,unsigned pixel,unsigned extra):width(w),height(h),bpp(pixel),row(w*pixel),stride(row+extra),bytes(size_t(row)*h),source(size_t(stride)*h+2*guard,padding),destination(bytes+2*guard,sentinel){
  for(unsigned y=0;y<height;y++)for(unsigned x=0;x<row;x++)source[guard+size_t(y)*stride+x]=uint8_t((x+13*y+7*pixel)%251);
 }
 rs2::video_frame frame()const{return {source.data()+guard,int(width),int(height),int(stride),int(bpp),size_t(stride)*(height-1)+row};}
 uint8_t* output(){return destination.data()+guard;}
 void verify()const{
  // Check every copied byte, every source padding byte and both complete
  // source/destination guards outside timing; no per-frame verification cost.
  for(unsigned y=0;y<height;y++){
   for(unsigned x=0;x<row;x++){
    auto expected=uint8_t((x+13*y+7*bpp)%251);
    require(source[guard+size_t(y)*stride+x]==expected);
    require(destination[guard+size_t(y)*row+x]==expected);
   }
   for(unsigned x=row;x<stride;x++)require(source[guard+size_t(y)*stride+x]==padding);
  }
  for(size_t x=0;x<guard;x++){
   require(destination[x]==sentinel&&destination[guard+bytes+x]==sentinel);
   require(source[x]==padding&&source[guard+size_t(stride)*height+x]==padding);
  }
 }
};
int main()try{
 std::puts("{\"scope\":\"actual copy_rows body, synthetic SDK frame getters; local CPU only\",\"GPU_executed\":false,\"samples\":[");
 unsigned scenario=0;
 for(unsigned width:{1280u,1920u})for(bool padded:{false,true}){
  const unsigned height=width==1280?720:1080;
  Plane rgb(width,height,3,padded?64:0),depth(1280,720,2,padded?32:0);
  const auto color_frame=rgb.frame(),depth_frame=depth.frame();
  const size_t bytes=rgb.bytes+depth.bytes;require(bytes==(width==1280?4608000:8064000));
  const auto copy=[&](){copy_rows(rgb.output(),rgb.bytes,color_frame,rgb.row);copy_rows(depth.output(),depth.bytes,depth_frame,depth.row);};
  constexpr unsigned warmup=10,iterations=100;
  for(unsigned i=0;i<warmup;i++)copy();rgb.verify();depth.verify();
  heap_allocations=0;hot=true;
  const auto start=std::chrono::steady_clock::now();
  for(unsigned i=0;i<iterations;i++)copy();
  const auto end=std::chrono::steady_clock::now();hot=false;
  require(heap_allocations==0);rgb.verify();depth.verify();
  const double ns=std::chrono::duration<double,std::nano>(end-start).count()/iterations;
  std::printf("%s{\"color_width\":%u,\"color_height\":%u,\"depth_width\":1280,\"depth_height\":720,\"padded_stride\":%s,\"color_stride\":%u,\"depth_stride\":%u,\"bytes_per_plane_pair\":%zu,\"iterations\":%u,\"warmup\":%u,\"nominal_timed_copy_bytes\":%zu,\"ns_per_plane_pair\":%.3f,\"hot_heap_allocations\":%zu,\"hot_waits\":0,\"all_content_padding_and_guards_checked\":true}",scenario++?",\n":"",width,height,padded?"true":"false",rgb.stride,depth.stride,bytes,iterations,warmup,bytes*iterations,ns,heap_allocations);
 }
 std::puts("]}");return 0;
}catch(const std::exception& e){std::fprintf(stderr,"%s\n",e.what());return 1;}
'''.replace('ACTUAL_BODY', helper)
    unit, binary = tmp_path/'actual-plane-copy.cpp', tmp_path/'actual-plane-copy'
    unit.write_text(code)
    compiler = os.environ.get('CXX') or shutil.which('c++')
    assert compiler
    command = [compiler,'-O2','-std=c++17','-ffp-contract=off',str(unit),'-o',str(binary)]
    compiled = subprocess.run(command,capture_output=True,text=True,timeout=60)
    (tmp_path/'compile.log').write_text(compiled.stdout+compiled.stderr)
    assert compiled.returncode == 0,compiled.stderr
    result = subprocess.run([str(binary)],capture_output=True,text=True,timeout=30)
    assert result.returncode == 0,result.stdout+result.stderr
    payload = json.loads(result.stdout)
    assert payload['GPU_executed'] is False
    assert len(payload['samples']) == 4
    for sample in payload['samples']:
        assert sample['bytes_per_plane_pair'] == (4608000 if sample['color_width']==1280 else 8064000)
        assert sample['iterations'] == 100 and sample['warmup'] == 10
        assert sample['nominal_timed_copy_bytes'] == 100*sample['bytes_per_plane_pair']
        assert sample['ns_per_plane_pair'] > 0  # no noisy absolute timing limit
        assert sample['hot_heap_allocations'] == sample['hot_waits'] == 0
        assert sample['all_content_padding_and_guards_checked'] is True
    assert [row['padded_stride'] for row in payload['samples']] == [False,True,False,True]
    payload['camera_source_sha256'] = hashlib.sha256(camera.encode()).hexdigest()
    payload['actual_helper_sha256'] = hashlib.sha256(helper.encode()).hexdigest()
    payload['command'] = command
    (tmp_path/'plane-copy-cost.json').write_text(json.dumps(payload,indent=2)+'\n')
    if destination := os.environ.get('RESIDENT_PLANE_COPY_PERF_ARTIFACTS'):
        destination = Path(destination)
        destination.mkdir(parents=True,exist_ok=True)
        for path in (unit,binary,tmp_path/'compile.log',tmp_path/'plane-copy-cost.json'):
            shutil.copy2(path,destination/path.name)
    print(json.dumps(payload))
