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
struct Event {unsigned kind=0,pipeline=0;VkBuffer src{},dst{};VkBufferCopy copy{};VkPipelineStageFlags source_stage=0,dest_stage=0;VkAccessFlags source_access=0,dest_access=0;};
Event events[1024];unsigned event_count=0,created_pipelines=0,current_pipeline=0;bool trace=false;VkPipeline pipeline_handles[8]{};
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
Context::Context(){properties.limits.maxPushConstantsSize=128;properties.limits.maxComputeWorkGroupCount[0]=65535;properties.limits.maxComputeWorkGroupCount[1]=65535;properties.limits.maxComputeWorkGroupCount[2]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeWorkGroupSize[1]=16;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=128*1024*1024;enabled_features.features.shaderFloat64=VK_TRUE;}
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
VkResult vkResetFences(VkDevice,uint32_t count,const VkFence* f){for(unsigned i=0;i<count;i++)complete[uintptr_t(f[i])]=false;return VK_SUCCESS;}
VkResult vkWaitForFences(VkDevice,uint32_t,const VkFence*,VkBool32,uint64_t){++wait_calls;return VK_SUCCESS;}
VkResult vkAllocateCommandBuffers(VkDevice,const VkCommandBufferAllocateInfo* info,VkCommandBuffer* out){for(unsigned i=0;i<info->commandBufferCount;i++)out[i]=handle<VkCommandBuffer>();return VK_SUCCESS;}
void vkFreeCommandBuffers(VkDevice,VkCommandPool,uint32_t,const VkCommandBuffer*){}
VkResult vkAllocateDescriptorSets(VkDevice,const VkDescriptorSetAllocateInfo* info,VkDescriptorSet* out){for(unsigned i=0;i<info->descriptorSetCount;i++)out[i]=handle<VkDescriptorSet>();return VK_SUCCESS;}
VkResult vkCreateComputePipelines(VkDevice,VkPipelineCache,uint32_t n,const VkComputePipelineCreateInfo*,const VkAllocationCallbacks*,VkPipeline* out){for(unsigned i=0;i<n;i++){out[i]=handle<VkPipeline>();pipeline_handles[created_pipelines++]=out[i];}return VK_SUCCESS;}
void vkUpdateDescriptorSets(VkDevice,uint32_t,const VkWriteDescriptorSet*,uint32_t,const VkCopyDescriptorSet*){}
VkResult vkResetCommandBuffer(VkCommandBuffer,VkCommandBufferResetFlags){return VK_SUCCESS;}
VkResult vkBeginCommandBuffer(VkCommandBuffer,const VkCommandBufferBeginInfo*){return VK_SUCCESS;}
VkResult vkEndCommandBuffer(VkCommandBuffer){return VK_SUCCESS;}
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer src,VkBuffer dst,uint32_t n,const VkBufferCopy* copy){if(trace){require(n==1);auto& e=event(1);e.src=src;e.dst=dst;e.copy=*copy;}}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize,VkDeviceSize,uint32_t){}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags src,VkPipelineStageFlags dst,VkDependencyFlags,uint32_t n,const VkMemoryBarrier* b,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){if(trace){require(n==1);auto& e=event(2);e.source_stage=src;e.dest_stage=dst;e.source_access=b->srcAccessMask;e.dest_access=b->dstAccessMask;}}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t,const void*){}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline pipeline){for(unsigned i=0;i<8;i++)if(pipeline==pipeline_handles[i])current_pipeline=i;}
void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){if(trace)event(3).pipeline=current_pipeline;}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo*,VkFence f){++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}

extern "C" {
void vkCmdResetQueryPool(VkCommandBuffer,VkQueryPool,uint32_t,uint32_t){}
void vkCmdWriteTimestamp(VkCommandBuffer,VkPipelineStageFlagBits,VkQueryPool,uint32_t){}
VkResult vkGetQueryPoolResults(VkDevice,VkQueryPool,uint32_t,uint32_t,size_t,void*,VkDeviceSize,VkQueryResultFlags){return VK_SUCCESS;}
}
template<class F>double measure(F fn){constexpr unsigned n=100000;auto start=std::chrono::steady_clock::now();for(unsigned i=0;i<n;i++)fn();return std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/n;}
int main(int argc,char** argv){
 require(argc==2);Pipeline pipeline(640,480,"vulkan",argv[1],true);auto frame=pipeline.acquire();
 trace=true;pipeline.submit(frame);trace=false;require(pipeline.ready(frame));pipeline.finish(frame);
 unsigned parent_copies=0,dispatches=0,barriers=0,order[71]{};
 for(unsigned i=0;i<event_count;i++){
  const auto& e=events[i];barriers+=e.kind==2;
  if(e.kind==3){require(dispatches<71);order[dispatches++]=e.pipeline;}
  if(e.kind!=1)continue;
  if(e.src==e.dst)++parent_copies;
  else if(e.copy.size==frame.slot().scratch.size){
   // Keep the conservative final diagnostic visibility contract.
   unsigned previous=i;while(previous&&events[previous].kind!=2)--previous;const auto& barrier=events[previous];
   require((barrier.source_stage&(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT))==(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT));
   require((barrier.source_access&(VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT))==(VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT));
  }
 }
 require(parent_copies==0&&dispatches==71&&barriers==74);
 require(order[0]==0&&order[1]==1&&order[2]==7);
 for(unsigned round=0;round<32;round++)require(order[3+2*round]==3&&order[4+2*round]==2);
 require(order[67]==3&&order[68]==4&&order[69]==6&&order[70]==5);
 // Every immutable compress / atomic connect dispatch is followed by the
 // existing compute-write -> compute-read/write dependency, with no transfer.
 unsigned seen=0;
 for(unsigned i=0;i<event_count;i++)if(events[i].kind==3){
  if(seen>=3&&seen<=67){require(i+1<event_count);const auto& b=events[i+1];
   require(b.kind==2&&b.source_stage==VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT&&b.dest_stage==VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
   require(b.source_access==VK_ACCESS_SHADER_WRITE_BIT);
   require(b.dest_access==(VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT));}
  ++seen;
 }
 frame={};
 auto allocations_before=heap_allocations,buffers_before=buffer_allocations,waits_before=wait_calls;
 double ns=measure([&]{auto f=pipeline.acquire();pipeline.submit(f);require(pipeline.ready(f));pipeline.finish(f);});
 auto allocations=heap_allocations-allocations_before,buffers=buffer_allocations-buffers_before,waits=wait_calls-waits_before;
 require(!allocations&&!buffers&&!waits);
 std::cout<<"{\"scope\":\"actual atomic-fusion Pipeline submit/finish, synthetic Vulkan only; excludes driver and GPU\",\"iterations\":100000,\"host_cycle_ns\":"<<ns<<",\"parent_copies\":"<<parent_copies<<",\"parent_copy_bytes_per_frame\":"<<0<<",\"dispatches\":"<<dispatches<<",\"barriers\":"<<barriers<<",\"hot_heap_allocations\":"<<allocations<<",\"hot_buffer_allocations\":"<<buffers<<",\"hot_wait_calls\":"<<waits<<"}\n";
}
