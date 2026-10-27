// Actual module host paths with an explicitly synthetic Vulkan driver. No loader,
// instance, physical device, queue or GPU work is used. Driver timings are excluded.
#include "so101_station_vulkan/pipeline.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>
using namespace so101_station_vulkan;
namespace {
uint64_t heap_allocations=0,buffer_allocations=0,wait_calls=0,submits=0,next_handle=1;
VkResult submit_result=VK_SUCCESS,fence_result=VK_SUCCESS;
unsigned live_command_pools=0;
uint32_t storage_limit=256*1024*1024,dispatch_x=65535,dispatch_y=65535,push_limit=128;
struct Event {unsigned kind=0,pipeline=0,x=0,y=0,z=0;VkBuffer src{},dst{};VkBufferCopy copy{};VkPipelineStageFlags source_stage=0,dest_stage=0;VkAccessFlags source_access=0,dest_access=0;};
Event events[1024];unsigned event_count=0,created_pipelines=0,current_pipeline=0;bool trace=false;VkPipeline pipeline_handles[1024]{};
Event& event(unsigned kind){if(event_count>=1024)std::abort();auto& e=events[event_count++];e={};e.kind=kind;return e;}
bool hold=false;bool complete[4096]{};
template<class T>T handle(){return (T)(uintptr_t(next_handle++));}
void require(bool ok){if(!ok)throw std::runtime_error("preparation host contract failed");}
}
void* operator new(size_t n){if(auto p=std::malloc(n?n:1)){++heap_allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
namespace so101_station_vulkan {
Context::Context(){properties.limits.maxPushConstantsSize=push_limit;properties.limits.maxComputeWorkGroupCount[0]=dispatch_x;properties.limits.maxComputeWorkGroupCount[1]=dispatch_y;properties.limits.maxComputeWorkGroupCount[2]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeWorkGroupSize[1]=16;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=storage_limit;enabled_features.features.shaderFloat64=VK_TRUE;}
Context::~Context()=default;
std::shared_ptr<Context> Context::shared(){return std::make_shared<Context>();}
Buffer::~Buffer(){reset();}
void Buffer::reset()noexcept{std::free(mapped);mapped=nullptr;buffer=VK_NULL_HANDLE;size=0;}
void Buffer::create(Context& c,size_t n,VkBufferUsageFlags,VkMemoryPropertyFlags flags){++buffer_allocations;owner=&c;size=n;buffer=handle<VkBuffer>();if(flags&VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT){mapped=std::calloc(n,1);if(!mapped)throw std::bad_alloc();}}
}
extern "C" {
#define CREATE_ONE(name,Info,Type) VkResult name(VkDevice,const Info*,const VkAllocationCallbacks*,Type* out){*out=handle<Type>();return VK_SUCCESS;}
VkResult vkCreateCommandPool(VkDevice,const VkCommandPoolCreateInfo*,const VkAllocationCallbacks*,VkCommandPool* out){*out=handle<VkCommandPool>();++live_command_pools;return VK_SUCCESS;}
CREATE_ONE(vkCreateDescriptorSetLayout,VkDescriptorSetLayoutCreateInfo,VkDescriptorSetLayout)
CREATE_ONE(vkCreatePipelineLayout,VkPipelineLayoutCreateInfo,VkPipelineLayout)
CREATE_ONE(vkCreateShaderModule,VkShaderModuleCreateInfo,VkShaderModule)
CREATE_ONE(vkCreateDescriptorPool,VkDescriptorPoolCreateInfo,VkDescriptorPool)
CREATE_ONE(vkCreateQueryPool,VkQueryPoolCreateInfo,VkQueryPool)
#define DESTROY(name,Type) void name(VkDevice,Type,const VkAllocationCallbacks*){}
void vkDestroyCommandPool(VkDevice,VkCommandPool,const VkAllocationCallbacks*){--live_command_pools;} DESTROY(vkDestroyDescriptorSetLayout,VkDescriptorSetLayout)
DESTROY(vkDestroyPipelineLayout,VkPipelineLayout) DESTROY(vkDestroyShaderModule,VkShaderModule)
DESTROY(vkDestroyDescriptorPool,VkDescriptorPool) DESTROY(vkDestroyPipeline,VkPipeline)
DESTROY(vkDestroyFence,VkFence) DESTROY(vkDestroyQueryPool,VkQueryPool)
VkResult vkCreateFence(VkDevice,const VkFenceCreateInfo* info,const VkAllocationCallbacks*,VkFence* out){if(fence_result!=VK_SUCCESS)return fence_result;*out=handle<VkFence>();complete[uintptr_t(*out)]=(info->flags&VK_FENCE_CREATE_SIGNALED_BIT)!=0;return VK_SUCCESS;}
VkResult vkGetFenceStatus(VkDevice,VkFence f){return complete[uintptr_t(f)]?VK_SUCCESS:VK_NOT_READY;}
VkResult vkResetFences(VkDevice,uint32_t count,const VkFence* f){if(trace)event(4);for(unsigned i=0;i<count;i++)complete[uintptr_t(f[i])]=false;return VK_SUCCESS;}
VkResult vkWaitForFences(VkDevice,uint32_t,const VkFence*,VkBool32,uint64_t){++wait_calls;return VK_SUCCESS;}
VkResult vkAllocateCommandBuffers(VkDevice,const VkCommandBufferAllocateInfo* info,VkCommandBuffer* out){for(unsigned i=0;i<info->commandBufferCount;i++)out[i]=handle<VkCommandBuffer>();return VK_SUCCESS;}
void vkFreeCommandBuffers(VkDevice,VkCommandPool,uint32_t,const VkCommandBuffer*){}
VkResult vkAllocateDescriptorSets(VkDevice,const VkDescriptorSetAllocateInfo* info,VkDescriptorSet* out){for(unsigned i=0;i<info->descriptorSetCount;i++)out[i]=handle<VkDescriptorSet>();return VK_SUCCESS;}
VkResult vkCreateComputePipelines(VkDevice,VkPipelineCache,uint32_t n,const VkComputePipelineCreateInfo*,const VkAllocationCallbacks*,VkPipeline* out){for(unsigned i=0;i<n;i++){out[i]=handle<VkPipeline>();pipeline_handles[created_pipelines++]=out[i];}return VK_SUCCESS;}
void vkUpdateDescriptorSets(VkDevice,uint32_t,const VkWriteDescriptorSet*,uint32_t,const VkCopyDescriptorSet*){}
VkResult vkResetCommandBuffer(VkCommandBuffer,VkCommandBufferResetFlags){if(trace)event(6);return VK_SUCCESS;}
VkResult vkBeginCommandBuffer(VkCommandBuffer,const VkCommandBufferBeginInfo*){return VK_SUCCESS;}
VkResult vkEndCommandBuffer(VkCommandBuffer){return VK_SUCCESS;}
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer src,VkBuffer dst,uint32_t n,const VkBufferCopy* copy){if(trace){require(n==1);auto& e=event(1);e.src=src;e.dst=dst;e.copy=*copy;}}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize,VkDeviceSize,uint32_t){}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags src,VkPipelineStageFlags dst,VkDependencyFlags,uint32_t n,const VkMemoryBarrier* b,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){if(trace){require(n==1);auto& e=event(2);e.source_stage=src;e.dest_stage=dst;e.source_access=b->srcAccessMask;e.dest_access=b->dstAccessMask;}}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t,const void*){}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline pipeline){for(unsigned i=0;i<created_pipelines;i++)if(pipeline==pipeline_handles[i])current_pipeline=i%8;}
void vkCmdDispatch(VkCommandBuffer,uint32_t x,uint32_t y,uint32_t z){if(trace){auto& e=event(3);e.pipeline=current_pipeline;e.x=x;e.y=y;e.z=z;}}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo*,VkFence f){if(trace)event(5);++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}

extern "C" {
void vkCmdResetQueryPool(VkCommandBuffer,VkQueryPool,uint32_t,uint32_t){}
void vkCmdWriteTimestamp(VkCommandBuffer,VkPipelineStageFlagBits,VkQueryPool,uint32_t){}
VkResult vkGetQueryPoolResults(VkDevice,VkQueryPool,uint32_t,uint32_t,size_t,void*,VkDeviceSize,VkQueryResultFlags){return VK_SUCCESS;}
}

template<class F>void rejected(F fn){event_count=0;trace=true;bool failed=false;try{fn();}catch(const std::exception&){failed=true;}trace=false;require(failed&&event_count==0);}
template<class F>void rejected_before_buffers(F fn,const char* reason){
 auto before=buffer_allocations;bool failed=false;
 try{fn();}catch(const std::exception& e){failed=std::string(e.what()).find(reason)!=std::string::npos;if(!failed)std::cerr<<"expected rejection: "<<reason<<" actual: "<<e.what()<<"\n";}
 require(failed&&buffer_allocations==before);
}
void resource_contracts(const char* shaders){
 // Real constructors and resource accounting: rejection cannot allocate even one Vulkan buffer.
 for(auto wh:{std::pair<unsigned,unsigned>{0,480},{640,0},{1921,1080},{UINT32_MAX,UINT32_MAX}}){
  rejected_before_buffers([&]{Pipeline p(wh.first,wh.second,"vulkan",shaders);},"dimensions");
  rejected_before_buffers([&]{FramePool p(wh.first,wh.second,nullptr,false);},"dimensions");
 }
 storage_limit=128*1024*1024;
 rejected_before_buffers([&]{Pipeline p(1920,1080,"vulkan",shaders);},"scratch");
 {auto before=buffer_allocations;Pipeline p(1920,1080,"auto",shaders);require(p.backend()=="cpu"&&p.fallback_reason().find("scratch")!=std::string::npos&&buffer_allocations==before);}
 storage_limit=256*1024*1024;
 dispatch_y=65535;
 rejected_before_buffers([&]{Pipeline p(1,1048561,"vulkan",shaders);},"dispatch");
 dispatch_x=8099;
 rejected_before_buffers([&]{Pipeline p(1920,1080,"vulkan",shaders);},"dispatch");
 dispatch_x=65535;
 rejected_before_buffers([&]{Pipeline p(1048561,1,"vulkan",shaders);},"dispatch");
 dispatch_y=67;
 rejected_before_buffers([&]{Pipeline p(1920,1080,"vulkan",shaders);},"dispatch");
 dispatch_y=65535;push_limit=47;
 rejected_before_buffers([&]{Pipeline p(640,480,"vulkan",shaders);},"push");
 push_limit=128;
 // Exercise every bound storage binding independently, including diagnostic scratch.
 storage_limit=1000;
 rejected_before_buffers([&]{Pipeline p(640,480,"vulkan",shaders);},"input");
 storage_limit=2047;
 rejected_before_buffers([&]{Pipeline p(1,1,"vulkan",shaders);},"compact");
 storage_limit=150000000;
 rejected_before_buffers([&]{Pipeline p(1920,1080,"vulkan",shaders,true);},"scratch");
 storage_limit=256*1024*1024;
 for(auto wh:{std::pair<unsigned,unsigned>{1280,720},{1920,1080}}){
  const size_t n=size_t(wh.first)*wh.second,rgb=3*n,depth=2*n;
  storage_limit=wh.first==1280?128*1024*1024:static_cast<uint32_t>(72*n); // 1080p exact range boundary.
  Pipeline p(wh.first,wh.second,"vulkan",shaders);require(p.backend()=="vulkan");auto f=p.acquire();auto& s=f.slot();
  require(s.cpu_masks.size()==n&&s.cpu_xyz.size()==4*n&&s.input.size==((5*n+3)&~size_t(3))&&s.scratch.size==72*n&&s.compact.size==compact_bytes());
  Buffer source;source.create(*p.context(),depth,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  auto allocs=heap_allocations,buffers=buffer_allocations,waits=wait_calls;
  event_count=0;trace=true;p.submit_device_depth(f,source);trace=false;
  require(heap_allocations==allocs&&buffer_allocations==buffers&&wait_calls==waits);
  unsigned copies=0,dispatches=0;
  for(unsigned i=0;i<event_count;i++){const auto& e=events[i];if(e.kind==1){
   if(copies==0)require(e.src==s.upload.buffer&&e.dst==s.input.buffer&&e.copy.dstOffset==0&&e.copy.size==rgb);
   if(copies==1)require(e.src==source.buffer&&e.dst==s.input.buffer&&e.copy.dstOffset==rgb&&e.copy.size==depth);
   if(copies==2)require(e.src==s.compact.buffer&&e.dst==s.readback.buffer&&e.copy.size==compact_bytes());++copies;
  }if(e.kind==3){++dispatches;require(e.z==1);if(e.pipeline==7)require(e.x==(wh.first+15)/16&&e.y==(wh.second+15)/16);else require(e.x==(n+255)/256&&e.y==1);}}
  require(copies==3&&dispatches==71);
 }
 storage_limit=256*1024*1024;
 std::cout<<"high-resolution resource/admission and derived copy contracts passed\n";
}
int main(int argc,char** argv){
 require(argc==2);resource_contracts(argv[1]);Pipeline pipeline(640,480,"vulkan",argv[1]);auto f=pipeline.acquire();auto& s=f.slot();
 Buffer source;source.create(*pipeline.context(),640*480*2,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
 rejected([&]{pipeline.submit_device_depth({},source);});
 auto foreign_pool=std::make_shared<FramePool>(640,480,pipeline.context(),false);auto foreign=foreign_pool->acquire();
 rejected([&]{pipeline.submit_device_depth(foreign,source);});foreign={};foreign_pool.reset();
 Context other_device;auto owner=source.owner;source.owner=&other_device;rejected([&]{pipeline.submit_device_depth(f,source);});source.owner=nullptr;rejected([&]{pipeline.submit_device_depth(f,source);});source.owner=owner;
 auto buffer=source.buffer;source.buffer=VK_NULL_HANDLE;rejected([&]{pipeline.submit_device_depth(f,source);});source.buffer=buffer;
 --source.size;rejected([&]{pipeline.submit_device_depth(f,source);});source.size+=2;rejected([&]{pipeline.submit_device_depth(f,source);});--source.size;
 for(auto alias:{s.input.buffer,s.upload.buffer}){source.buffer=alias;rejected([&]{pipeline.submit_device_depth(f,source);});}source.buffer=buffer;
 s.metadata.width=639;rejected([&]{pipeline.submit_device_depth(f,source);});s.metadata.width=640;
 s.metadata.height=479;rejected([&]{pipeline.submit_device_depth(f,source);});s.metadata.height=480;
 s.calibration.fx=0;rejected([&]{pipeline.submit_device_depth(f,source);});s.calibration.fx=600;
 s.metadata.calibration_generation=1;rejected([&]{pipeline.submit_device_depth(f,source);});s.metadata.calibration_generation=0;
 auto input_size=s.input.size;s.input.size=3*640*480;rejected([&]{pipeline.submit_device_depth(f,source);});s.input.size=input_size;
 auto upload_size=s.upload.size;s.upload.size=3*640*480-1;rejected([&]{pipeline.submit_device_depth(f,source);});s.upload.size=upload_size;
 auto memory=handle<VkDeviceMemory>();s.input.memory=memory;source.memory=memory;rejected([&]{pipeline.submit_device_depth(f,source);});s.input.memory=VK_NULL_HANDLE;source.memory=VK_NULL_HANDLE;
 Pipeline cpu(640,480,"cpu",argv[1]);auto cpu_frame=cpu.acquire();rejected([&]{cpu.submit_device_depth(cpu_frame,source);});
 // Producer queue submission precedes the consumer: no wait/poll/readback.
 event_count=0;trace=true;VkSubmitInfo producer{VK_STRUCTURE_TYPE_SUBMIT_INFO};vkQueueSubmit(pipeline.context()->queue,1,&producer,VK_NULL_HANDLE);
 auto allocs=heap_allocations,buffers=buffer_allocations,waits=wait_calls;
 pipeline.submit_device_depth(f,source);trace=false;
 require(heap_allocations==allocs&&buffer_allocations==buffers&&wait_calls==waits);
 require(events[0].kind==5&&events[event_count-1].kind==5);
 Event resident_events[1024];auto resident_count=event_count;std::copy(events,events+event_count,resident_events);
 unsigned copies=0,dispatches=0,barriers=0;
 for(unsigned i=0;i<event_count;i++){auto& e=events[i];dispatches+=e.kind==3;barriers+=e.kind==2;if(e.kind!=1)continue;
  if(copies==0)require(e.src==s.upload.buffer&&e.dst==s.input.buffer&&e.copy.srcOffset==0&&e.copy.dstOffset==0&&e.copy.size==921600);
  if(copies==1)require(e.src==source.buffer&&e.dst==s.input.buffer&&e.copy.srcOffset==0&&e.copy.dstOffset==921600&&e.copy.size==614400);
  if(copies==2)require(e.src==s.compact.buffer&&e.dst==s.readback.buffer&&e.copy.srcOffset==0&&e.copy.dstOffset==0&&e.copy.size==s.compact.size);
  ++copies;
 }
 require(copies==3&&dispatches==71&&barriers==74);
 rejected([&]{pipeline.submit_device_depth(f,source);});
 auto legacy=pipeline.acquire();event_count=0;trace=true;pipeline.submit(legacy);trace=false;
 unsigned li=0,ri=0; // After uploads all dispatches, dependencies and readback must match.
 while(li<event_count&&events[li].kind!=2)++li;while(ri<resident_count&&resident_events[ri].kind!=2)++ri;
 require(event_count-li==resident_count-ri);
 for(;li<event_count;li++,ri++){auto& a=events[li];auto& b=resident_events[ri];require(a.kind==b.kind&&a.pipeline==b.pipeline&&a.source_stage==b.source_stage&&a.dest_stage==b.dest_stage&&a.source_access==b.source_access&&a.dest_access==b.dest_access);if(a.kind==1)require(a.copy.size==b.copy.size&&a.copy.srcOffset==0&&a.copy.dstOffset==0);}
 bool upload=false;for(unsigned i=0;i<event_count;i++)if(events[i].kind==1&&!upload){auto& e=events[i];require(e.src==legacy.slot().upload.buffer&&e.dst==legacy.slot().input.buffer&&e.copy.size==1536000&&e.copy.srcOffset==0&&e.copy.dstOffset==0);upload=true;}require(upload);
 Pipeline unaligned(17,19,"vulkan",argv[1]);auto odd=unaligned.acquire();Buffer odd_source;odd_source.create(*unaligned.context(),17*19*2,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
 rejected([&]{unaligned.submit_device_depth(odd,odd_source);});
 std::cout<<"resident host-driver contracts passed: RGB=921600 resident_D2D=614400 legacy_upload=1536000 dispatches=71 barriers=74 hot_allocations=0 waits=0\n";
}
