"""Actual Alignment host recording for independent native-depth/color grids.

Synthetic Vulkan entry points record command arguments; no loader/GPU is used.
"""
from pathlib import Path
import os
import shutil
import subprocess
import pytest

P = Path(__file__).resolve().parents[1]/'ros2/src/so101_station_vulkan'


def test_independent_extents_record_native_upload_color_outputs_and_preallocation_limits(tmp_path):
    sdk = os.environ.get('RAW_BRIDGE_SDK_PREFIX')
    vulkan = os.environ.get('RAW_BRIDGE_VULKAN_INCLUDE')
    if not sdk or not vulkan:
        pytest.skip('set RAW_BRIDGE_SDK_PREFIX and RAW_BRIDGE_VULKAN_INCLUDE for CPU host build')
    # Reuse the existing specific fake ABI rather than implementing a second
    # driver. Compile only actual Alignment host/reference modules below.
    driver = (P/'test/raw_capture_bridge_host_test.cpp').read_text().split('namespace so101_station_vulkan {\nPipeline::Pipeline', 1)[0]
    driver = driver.replace('uint64_t heap_allocations=0,', 'size_t allocated_bytes=0;uint32_t dispatches[2]{};unsigned dispatch_count=0,copy_count=0,fill_count=0;VkBufferCopy copies[4]{};struct Fill{VkDeviceSize offset,size;uint32_t value;};Fill fills[2]{};uint32_t pushed[23]{};\nuint64_t heap_allocations=0,')
    driver = driver.replace('++buffer_allocations;owner=', '++buffer_allocations;allocated_bytes+=n;owner=')
    driver = driver.replace('const VkBufferCopy* copy){if(', 'const VkBufferCopy* copy){require(copy_count<4);copies[copy_count++]=*copy;if(')
    driver = driver.replace('void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize,VkDeviceSize,uint32_t){}', 'void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize offset,VkDeviceSize bytes,uint32_t value){require(fill_count<2);fills[fill_count++]={offset,bytes,value};}')
    driver = driver.replace('void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t,const void*){}', 'void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t bytes,const void* data){require(bytes==sizeof(pushed));std::memcpy(pushed,data,bytes);}')
    driver = driver.replace('void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){}', 'void vkCmdDispatch(VkCommandBuffer,uint32_t x,uint32_t y,uint32_t z){require(dispatch_count<2&&y==1&&z==1);dispatches[dispatch_count++]=x;}')
    code = driver + r'''
struct Profile{unsigned dw,dh,cw,ch;size_t depth_n,color_n,normal_bytes;};
int main(int argc,char**argv)try{
 require(argc==2);
 const Profile profiles[]={
  {640,480,640,480,307200,307200,28876824},
  {640,480,1280,720,307200,921600,69427224},
  {640,480,1920,1080,307200,2073600,145459224},
  {1280,720,640,480,921600,307200,46080024},
  {1280,720,1280,720,921600,921600,86630424},
  {1280,720,1920,1080,921600,2073600,162662424}};
 for(auto q:profiles)for(bool diagnostics:{false,true}){
  AlignmentCalibration k;k.convention=AlignmentConvention::Scalar;k.generation=7;
  k.depth={int(q.dw),int(q.dh),320,240,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};
  k.color={int(q.cw),int(q.ch),320,240,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};
  k.depth_to_color={{1,0,0,0,1,0,0,0,1},{0,0,0}};
  auto context=std::make_shared<Context>();
  // All storage/dispatch/push failures must precede even the first buffer.
  for(unsigned limit=0;limit<3;limit++){
   auto bad=std::make_shared<Context>();
   if(limit==0)bad->properties.limits.maxStorageBufferRange=uint32_t(std::max(q.depth_n,q.color_n)*16-1);
   if(limit==1)bad->properties.limits.maxComputeWorkGroupCount[0]=uint32_t(std::max((q.depth_n+255)/256,(q.color_n/2+255)/256)-1);
   if(limit==2)bad->properties.limits.maxPushConstantsSize=91;
   auto before=buffer_allocations;bool rejected=false;try{Alignment invalid(bad,k,argv[1],diagnostics);}catch(const std::runtime_error&){rejected=true;}
   require(rejected&&buffer_allocations==before);
  }
  allocated_bytes=0;Alignment alignment(context,k,argv[1],diagnostics);
  require(allocated_bytes==q.normal_bytes+(diagnostics?54*q.color_n:0));
  auto lease=alignment.acquire();require(bool(lease));
  require(lease.aligned().size==2*q.color_n&&lease.xyz().size==16*q.color_n);
  require(lease.metadata().width==q.cw&&lease.metadata().height==q.ch);
  auto before_heap=heap_allocations,before_buffers=buffer_allocations,before_waits=wait_calls;
  dispatch_count=copy_count=fill_count=0;alignment.submit(lease);require(alignment.ready(lease));
  require(heap_allocations==before_heap&&buffer_allocations==before_buffers&&wait_calls==before_waits);
  require(fill_count==2&&fills[0].offset==0&&fills[0].size==4*q.color_n&&fills[0].value==UINT32_MAX&&fills[1].offset==4*q.color_n&&fills[1].size==4&&fills[1].value==0);
  require(dispatch_count==2&&dispatches[0]==(q.depth_n+255)/256&&dispatches[1]==(q.color_n/2+255)/256);
  require(pushed[19]==q.dw&&pushed[20]==q.dh&&pushed[21]==q.cw&&pushed[22]==q.ch);
  require(copy_count==(diagnostics?4:2)&&copies[0].size==2*q.depth_n);
  require(copies[1].srcOffset==4*q.color_n&&copies[1].size==4);
  if(diagnostics){
   require(copies[2].size==2*q.color_n&&copies[2].dstOffset==0);
   require(copies[3].size==16*q.color_n&&copies[3].dstOffset==2*q.color_n);
   require(reinterpret_cast<uintptr_t>(lease.diagnostic_xyz())-reinterpret_cast<uintptr_t>(lease.diagnostic_depth())==2*q.color_n);
  }
  auto retained=lease;lease={};auto b=alignment.acquire(),c=alignment.acquire();
  require(bool(b)&&bool(c)&&!alignment.acquire());
 }
 // Actual host hot-path cost: fake commands record dispatch/copy arguments;
 // only the existing four-byte status destination is touched by the fake driver.
 // No frame-sized CPU copy or synthetic kernel execution occurs here.
 for(unsigned profile=0;profile<2;profile++){
  const auto q=profiles[profile==0?4:5];AlignmentCalibration k;
  k.convention=AlignmentConvention::Scalar;k.generation=7;
  k.depth={int(q.dw),int(q.dh),320,240,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};
  k.color={int(q.cw),int(q.ch),320,240,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};
  k.depth_to_color={{1,0,0,0,1,0,0,0,1},{0,0,0}};
  auto context=std::make_shared<Context>();Alignment alignment(context,k,argv[1],false);
  const auto cycle=[&](){
   dispatch_count=copy_count=fill_count=0;auto lease=alignment.acquire();require(bool(lease));
   alignment.submit(lease);require(alignment.ready(lease));
   require(fill_count==2&&fills[0].offset==0&&fills[0].size==4*q.color_n&&fills[0].value==UINT32_MAX&&fills[1].offset==4*q.color_n&&fills[1].size==4&&fills[1].value==0);
   require(dispatch_count==2&&copy_count==2&&copies[0].size==2*q.depth_n&&copies[1].size==4);
  };
  constexpr unsigned warmup=1000,iterations=100000;
  for(unsigned i=0;i<warmup;i++)cycle();
  const auto before_heap=heap_allocations,before_buffers=buffer_allocations,before_waits=wait_calls;
  const auto start=std::chrono::steady_clock::now();
  for(unsigned i=0;i<iterations;i++)cycle();
  const auto end=std::chrono::steady_clock::now();
  const auto hot_heap=heap_allocations-before_heap,hot_buffers=buffer_allocations-before_buffers,hot_waits=wait_calls-before_waits;
  require(hot_heap==0&&hot_buffers==0&&hot_waits==0);
  const double ns=std::chrono::duration<double,std::nano>(end-start).count()/iterations;
  std::cout<<"PERF {\"profile\":\""<<(profile==0?"720p":"1080p")<<"\",\"iterations\":"<<iterations<<",\"warmup\":"<<warmup<<",\"ns_per_cycle\":"<<ns<<",\"hot_heap\":"<<hot_heap<<",\"hot_buffers\":"<<hot_buffers<<",\"hot_waits\":"<<hot_waits<<",\"GPU_executed\":false}\n";
 }
 std::cout<<"PASS profiles=6 diagnostics_modes=2 preallocation_rejections=36 hot_heap=0 hot_buffers=0 hot_waits=0 GPU=false\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
'''
    unit, binary = tmp_path/'alignment-extents.cpp', tmp_path/'alignment-extents'
    unit.write_text(code)
    shaders = tmp_path/'synthetic-shaders'
    shaders.mkdir()
    for name in ('splat','finalize'):
        (shaders/(name+'.spv')).write_bytes(b'\x03\x02\x23\x07')
    command = [os.environ.get('CXX') or shutil.which('c++'), '-O2', '-std=c++17', '-ffp-contract=off',
               '-I',str(P/'include'),'-I',vulkan,'-I',sdk+'/include',str(unit),
               str(P/'src/alignment.cpp'),str(P/'src/alignment_reference.cpp'),
               '-L',sdk+'/lib','-Wl,-rpath,'+sdk+'/lib','-lrealsense2','-o',str(binary)]
    compile_result = subprocess.run(command,capture_output=True,text=True,timeout=60)
    assert compile_result.returncode == 0,compile_result.stderr
    symbols = subprocess.check_output(['nm','-u',str(binary)],text=True)
    assert 'vkCreate' not in symbols and 'vkQueue' not in symbols
    result = subprocess.run([str(binary),str(shaders)],capture_output=True,text=True,timeout=30)
    assert result.returncode == 0,result.stdout+result.stderr
    assert 'profiles=6 diagnostics_modes=2' in result.stdout and 'GPU=false' in result.stdout
    import json
    timings = [json.loads(line.removeprefix('PERF ')) for line in result.stdout.splitlines() if line.startswith('PERF ')]
    assert [row['profile'] for row in timings] == ['720p','1080p']
    for row in timings:
        assert row['iterations'] == 100000 and row['warmup'] == 1000 and row['ns_per_cycle'] > 0
        assert row['hot_heap'] == row['hot_buffers'] == row['hot_waits'] == 0
        assert row['GPU_executed'] is False
    (tmp_path/'alignment-host-cost.json').write_text(json.dumps(timings,indent=2)+'\n')
    (tmp_path/'compile-command.json').write_text(json.dumps(command,indent=2)+'\n')
    print(json.dumps(dict(scope='actual Alignment host orchestration with synthetic Vulkan, excludes driver/GPU cost',timings=timings)))
