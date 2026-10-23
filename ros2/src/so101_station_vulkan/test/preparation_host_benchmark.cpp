// Actual module host paths with an explicitly synthetic Vulkan driver. No loader,
// instance, physical device, queue or GPU work is used. Driver timings are excluded.
#include "so101_station_vulkan/alignment.hpp"
#include "so101_station_vulkan/geometry.hpp"
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
bool hold=false;bool complete[4096]{};
template<class T>T handle(){return (T)(uintptr_t(next_handle++));}
void require(bool ok){if(!ok)throw std::runtime_error("preparation host contract failed");}
}
void* operator new(size_t n){if(auto p=std::malloc(n?n:1)){++heap_allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
namespace so101_station_vulkan {
Context::Context(){properties.limits.maxPushConstantsSize=128;properties.limits.maxComputeWorkGroupCount[0]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=128*1024*1024;enabled_features.features.shaderFloat64=VK_TRUE;}
Context::~Context()=default;
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
VkResult vkCreateComputePipelines(VkDevice,VkPipelineCache,uint32_t n,const VkComputePipelineCreateInfo*,const VkAllocationCallbacks*,VkPipeline* out){for(unsigned i=0;i<n;i++)out[i]=handle<VkPipeline>();return VK_SUCCESS;}
void vkUpdateDescriptorSets(VkDevice,uint32_t,const VkWriteDescriptorSet*,uint32_t,const VkCopyDescriptorSet*){}
VkResult vkResetCommandBuffer(VkCommandBuffer,VkCommandBufferResetFlags){return VK_SUCCESS;}
VkResult vkBeginCommandBuffer(VkCommandBuffer,const VkCommandBufferBeginInfo*){return VK_SUCCESS;}
VkResult vkEndCommandBuffer(VkCommandBuffer){return VK_SUCCESS;}
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer,VkBuffer,uint32_t,const VkBufferCopy*){}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize,VkDeviceSize,uint32_t){}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags,VkPipelineStageFlags,VkDependencyFlags,uint32_t,const VkMemoryBarrier*,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t,const void*){}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline){}
void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo*,VkFence f){++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}
template<class F>double measure(F fn){constexpr unsigned n=100000;auto start=std::chrono::steady_clock::now();for(unsigned i=0;i<n;i++)fn();return std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/n;}
int main(int argc,char** argv){
 require(argc==3);auto context=std::make_shared<Context>();
 AlignmentCalibration ac;ac.generation=1;ac.convention=AlignmentConvention::Scalar;ac.depth.width=ac.color.width=640;ac.depth.height=ac.color.height=480;ac.depth.fx=ac.depth.fy=ac.color.fx=ac.color.fy=600;ac.depth.ppx=ac.color.ppx=320;ac.depth.ppy=ac.color.ppy=240;ac.depth_to_color.rotation[0]=ac.depth_to_color.rotation[4]=ac.depth_to_color.rotation[8]=1;
 Alignment alignment(context,ac,argv[1]);
 GeometryCalibration gc;gc.width=640;gc.height=480;gc.generation=1;gc.depth_scale_m=double(.001f);gc.camera_rays.resize(640*480);gc.board_from_camera[0]=gc.board_from_camera[5]=gc.board_from_camera[10]=1;
 GeometryPreparation geometry(context,gc,argv[2]);auto pool=std::make_shared<FramePool>(640,480,context,false);auto source=pool->acquire();source.slot().metadata.calibration_generation=1;source.slot().calibration.generation=1;source.slot().submitted=true;
 complete[uintptr_t(source.slot().fence)]=true;
 // Partially initialized modules destroy their already-created private pools.
 auto pools_before_failure=live_command_pools;fence_result=VK_ERROR_OUT_OF_HOST_MEMORY;
 bool init_rejected=false;try{GeometryPreparation partial(context,gc,argv[2]);}catch(const std::runtime_error&){init_rejected=true;}require(init_rejected&&live_command_pools==pools_before_failure);
 init_rejected=false;try{Alignment partial(context,ac,argv[1]);}catch(const std::runtime_error&){init_rejected=true;}require(init_rejected&&live_command_pools==pools_before_failure);fence_result=VK_SUCCESS;
 // Strict actual-enabled-feature admission, and failure clears retained source.
 context->enabled_features.features.shaderFloat64=VK_FALSE;
 bool rejected=false;try{GeometryPreparation unavailable(context,gc,argv[2]);}catch(const std::runtime_error&){rejected=true;}require(rejected);context->enabled_features.features.shaderFloat64=VK_TRUE;
 // Opt-in software mode accepts Int64 without Float64; unsupported, merely
 // physical-but-disabled, and invalid precision profiles fail before allocation.
 context->enabled_features.features.shaderFloat64=VK_FALSE;
 context->enabled_features.features.shaderInt64=VK_FALSE;
 auto allocations_before_guard=buffer_allocations;
 rejected=false;try{GeometryPreparation unavailable(context,gc,argv[2],GeometryPrecision::SoftwareBinary64);}catch(const std::runtime_error& e){rejected=std::string(e.what()).find("shaderInt64 was not enabled")!=std::string::npos;}require(rejected&&buffer_allocations==allocations_before_guard);
 rejected=false;try{GeometryPreparation unavailable(context,gc,argv[2],static_cast<GeometryPrecision>(99));}catch(const std::invalid_argument&){rejected=true;}require(rejected&&buffer_allocations==allocations_before_guard);
 context->enabled_features.features.shaderInt64=VK_TRUE;
 {GeometryPreparation software(context,gc,argv[2],GeometryPrecision::SoftwareBinary64);
  auto lease=software.submit(source);auto copy=lease;require(software.ready(copy));require(software.finish(copy).calibration_generation==1);
 }
 context->enabled_features.features.shaderFloat64=VK_TRUE;
 context->enabled_features.features.shaderInt64=VK_FALSE;
 auto refs_before_failure=source.slot().references.load();submit_result=VK_ERROR_OUT_OF_HOST_MEMORY;
 rejected=false;try{geometry.submit(source);}catch(const std::runtime_error&){rejected=true;}require(rejected&&source.slot().references==refs_before_failure);submit_result=VK_SUCCESS;
 // Generation, compact source fault and source-completion admission stay strict.
 source.slot().calibration.generation=2;rejected=false;try{geometry.submit(source);}catch(const std::invalid_argument&){rejected=true;}require(rejected);source.slot().calibration.generation=1;
 auto* status=static_cast<uint32_t*>(source.slot().readback.mapped);status[1]=1;rejected=false;try{geometry.submit(source);}catch(const std::runtime_error&){rejected=true;}require(rejected);status[1]=0;
 complete[uintptr_t(source.slot().fence)]=false;rejected=false;try{geometry.submit(source);}catch(const std::logic_error&){rejected=true;}require(rejected);complete[uintptr_t(source.slot().fence)]=true;
 source.slot().metadata.frame_number=917;source.slot().metadata.depth_frame_number=916;source.slot().metadata.timestamp_ns=1234567;
 // Dropping an in-flight lease must retain the source and keep its slot busy.
 auto initial=source.slot().references.load();hold=true;{auto lease=geometry.submit(source);auto copy=lease;require(source.slot().references==initial+1);require(!geometry.ready(copy));}require(source.slot().references==initial+1);geometry.collect();require(source.slot().references==initial+1);
 hold=false;std::fill(std::begin(complete),std::end(complete),true);geometry.collect();require(source.slot().references==initial);
 {auto lease=geometry.submit(source);auto copy=lease;require(geometry.ready(copy));require(geometry.finish(copy).calibration_generation==1);}geometry.collect();
 // Three alignment slots stay unavailable until both refs and completion clear.
 hold=true;{auto a=alignment.acquire(),b=alignment.acquire(),c=alignment.acquire();require(a&&b&&c&&!alignment.acquire());alignment.submit(a);alignment.submit(b);alignment.submit(c);}require(!alignment.acquire());hold=false;std::fill(std::begin(complete),std::end(complete),true);
 {auto a=alignment.acquire();alignment.submit(a);require(alignment.ready(a));}
 auto before=heap_allocations,buffers_before=buffer_allocations,waits_before=wait_calls;
 double alignment_ns=measure([&]{auto frame=alignment.acquire();alignment.submit(frame);require(alignment.ready(frame));});
 double geometry_ns=measure([&]{auto lease=geometry.submit(source);require(geometry.ready(lease));auto counts=geometry.finish(lease);require(counts.support_points==0);});
 double collect_ns=measure([&]{geometry.collect();});
 auto allocations=heap_allocations-before,buffers=buffer_allocations-buffers_before,waits=wait_calls-waits_before;
 require(!allocations&&!buffers&&!waits);require(source.slot().references==initial);
 std::cout<<"{\"scope\":\"actual alignment/geometry host paths with synthetic Vulkan calls; excludes real driver and GPU\",\"iterations\":100000,\"alignment_acquire_submit_ready_release_ns\":"<<alignment_ns<<",\"geometry_submit_ready_finish_release_ns\":"<<geometry_ns<<",\"geometry_idle_collect_ns\":"<<collect_ns<<",\"hot_heap_allocations\":"<<allocations<<",\"hot_buffer_allocations\":"<<buffers<<",\"hot_wait_calls\":"<<waits<<",\"drop_pending_source_retention\":true,\"alignment_three_slot_backpressure\":true}\n";
}
