// Actual module host paths with an explicitly synthetic Vulkan driver. No loader,
// instance, physical device, queue or GPU work is used. Driver timings are excluded.
#include "so101_station_vulkan/alignment.hpp"
#include "so101_station_vulkan/raw_capture_bridge.hpp"
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
const Buffer* watched_source=nullptr; bool consumer_destroyed=false,watched_source_released=false;
unsigned producer_fault=0; bool core_fault=false,core_throw=false; unsigned core_drains=0;
void* mapped_buffers[4096]{};
bool hold=false;bool complete[4096]{};
template<class T>T handle(){return (T)(uintptr_t(next_handle++));}
void require(bool ok){if(!ok)throw std::runtime_error("raw bridge host contract failed");}
}
void* operator new(size_t n){if(auto p=std::malloc(n?n:1)){++heap_allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
namespace so101_station_vulkan {
Context::Context(){properties.limits.maxPushConstantsSize=128;properties.limits.maxComputeWorkGroupCount[0]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=128*1024*1024;enabled_features.features.shaderFloat64=VK_TRUE;}
Context::~Context()=default;
Buffer::~Buffer(){reset();}
void Buffer::reset()noexcept{if(this==watched_source){require(consumer_destroyed);watched_source_released=true;}std::free(mapped);mapped=nullptr;buffer=VK_NULL_HANDLE;size=0;}
void Buffer::create(Context& c,size_t n,VkBufferUsageFlags,VkMemoryPropertyFlags flags){++buffer_allocations;owner=&c;size=n;buffer=handle<VkBuffer>();if(flags&VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT){mapped=std::calloc(n,1);if(!mapped)throw std::bad_alloc();mapped_buffers[uintptr_t(buffer)]=mapped;}}
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
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer,VkBuffer dst,uint32_t,const VkBufferCopy* copy){if(copy->size==4&&mapped_buffers[uintptr_t(dst)])*static_cast<uint32_t*>(mapped_buffers[uintptr_t(dst)])=producer_fault;}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize,VkDeviceSize,uint32_t){}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags,VkPipelineStageFlags,VkDependencyFlags,uint32_t,const VkMemoryBarrier*,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t,const void*){}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline){}
void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo*,VkFence f){++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}
namespace so101_station_vulkan {
Pipeline::Pipeline(unsigned w,unsigned h,const std::string& mode,const std::string&,bool,bool){
 backend_=mode;if(mode=="vulkan")context_=std::make_shared<Context>();
 pool_=std::make_shared<FramePool>(w,h,context_,false);
}
Pipeline::~Pipeline(){consumer_destroyed=true;for(auto& s:pool_->slots)if(s.submitted){complete[uintptr_t(s.fence)]=true;++core_drains;}}
void Pipeline::submit(const FrameLease& f){if(!owns(f))throw std::invalid_argument("foreign");f.slot().submitted=true;}
void Pipeline::submit_device_depth(const FrameLease& f,const Buffer& b){
 require(submits%2==1);require(b.owner==context_.get());++submits;
 if(core_throw)throw std::runtime_error("core submit failed");
 submit(f);complete[uintptr_t(f.slot().fence)]=false;
}
void Pipeline::finish(const FrameLease& f){require(f.slot().submitted);if(core_fault)throw std::runtime_error("core fault");}
}
AlignmentCalibration calibration(){AlignmentCalibration c;c.generation=7;c.convention=AlignmentConvention::Scalar;c.depth.width=c.color.width=640;c.depth.height=c.color.height=480;c.depth.fx=c.depth.fy=c.color.fx=c.color.fy=600;c.depth.ppx=c.color.ppx=320;c.depth.ppy=c.color.ppy=240;c.depth_to_color.rotation[0]=c.depth_to_color.rotation[4]=c.depth_to_color.rotation[8]=1;return c;}
std::unique_ptr<RawCaptureBridge> bridge(const char* shaders){
 auto p=std::make_unique<Pipeline>(640,480,"vulkan","");
 auto a=std::make_unique<Alignment>(p->context(),calibration(),shaders);
 return std::make_unique<RawCaptureBridge>(std::move(p),std::move(a));
}
RawFrame frame(RawCaptureBridge& b,unsigned id){auto f=b.acquire();require(bool(f));auto& m=f.core.slot().metadata;
 require(m.width==640&&m.height==480&&!m.frame_number&&!m.depth_frame_number&&!m.timestamp_ns&&!m.calibration_generation&&!m.hardware_timestamp_ms&&!m.depth_hardware_timestamp_ms&&!m.timestamp_domain&&!m.depth_timestamp_domain&&!m.publication_session_token&&m.serial.empty());
 auto& d=f.depth.metadata();require(d.width==640&&d.height==480&&d.calibration_generation==7&&!d.frame_number&&!d.depth_frame_number&&!d.timestamp_ns&&!d.hardware_timestamp_ms&&!d.depth_hardware_timestamp_ms&&!d.timestamp_domain&&!d.depth_timestamp_domain&&d.serial.empty());
 m.publication_session_token=700+id;m.frame_number=id;m.depth_frame_number=id+9;m.timestamp_ns=id*1000;m.calibration_generation=7;m.hardware_timestamp_ms=id;m.depth_hardware_timestamp_ms=id+1;m.timestamp_domain=1;m.depth_timestamp_domain=2;m.serial="camera-serial-longer-than-small-string-capacity-012345678901234567890123456789";f.core.slot().calibration=b.expected_calibration();f.depth.metadata()=m;return f;}
template<class F>void rejects(F fn){bool rejected=false;try{fn();}catch(const std::exception&){rejected=true;}require(rejected);}
int main(int argc,char** argv){try{
 require(argc==2);
 // Pending replacements perform no work; active producer completion cannot
 // release a source still needed by its incomplete consuming copy.
 auto b=bridge(argv[1]);auto f=frame(*b,1);auto source=f.depth;auto* depth_buffer=&source.aligned();
 b->offer(std::move(f));b->dispatch_pending();require(submits==2);require(complete[uintptr_t(source.fence())]);
 auto pending=frame(*b,2);b->offer(std::move(pending));b->discard_pending();require(submits==2);
 source={};auto two=frame(*b,3),three=frame(*b,4);require(!b->acquire());
 require(&two.depth.aligned()!=depth_buffer&&&three.depth.aligned()!=depth_buffer);
 RawFrame done;require(!b->collect(done));require(!done);two={};three={};source={};
 complete[uintptr_t(b->active().core.slot().fence)]=true;require(b->collect(done));require(bool(done.depth));require(done.core.slot().metadata.publication_session_token==701&&done.depth.metadata().publication_session_token==701);
 auto late=done.core;done={};b.reset();require(late.slot().references==1);late={};
 // Foreign source, exact calibration and every provenance cohort are admitted
 // before the producer receives queue work.
 b=bridge(argv[1]);auto x=frame(*b,5);auto y=frame(*b,6);auto before=submits;
 auto swapped=x;swapped.depth=y.depth;rejects([&]{b->offer(swapped);});
 auto other_alignment=std::make_unique<Alignment>(b->pipeline().context(),calibration(),argv[1]);
 auto foreign_depth=other_alignment->acquire();foreign_depth.metadata()=x.core.slot().metadata;
 swapped.depth=foreign_depth;rejects([&]{b->offer(swapped);});
 x.core.slot().calibration.generation=8;rejects([&]{b->offer(x);});x.core.slot().calibration.generation=7;
 x.depth.metadata().publication_session_token++;rejects([&]{b->offer(x);});x.depth.metadata()=x.core.slot().metadata;
 x.depth.metadata().timestamp_ns++;rejects([&]{b->offer(x);});x.depth.metadata()=x.core.slot().metadata;
 auto foreign=bridge(argv[1]);auto z=frame(*foreign,7);rejects([&]{b->offer(z);});require(submits==before);
 x={};y={};swapped={};z={};foreign.reset();
 // Faults never publish and keep the paired active ownership until shutdown.
 for(unsigned fault=0;fault<4;fault++){
  b=bridge(argv[1]);producer_fault=fault==0;core_fault=fault==1;core_throw=fault==2;
  auto q=frame(*b,10+fault);b->offer(std::move(q));
  if(core_throw)rejects([&]{b->dispatch_pending();});else{b->dispatch_pending();complete[uintptr_t(b->active().core.slot().fence)]=true;if(fault==3)complete[uintptr_t(b->active().depth.fence())]=false;rejects([&]{b->collect(done);});}
  require(!done&&b->active().depth);rejects([&]{b->acquire();});
  watched_source=&b->active().depth.aligned();consumer_destroyed=false;watched_source_released=false;auto drains=core_drains;b.reset();
  require(watched_source_released);watched_source=nullptr;
  if(!core_throw)require(core_drains>drains);
  producer_fault=0;core_fault=false;core_throw=false;
 }
 // Failed producer submission cannot enqueue a consumer.
 b=bridge(argv[1]);auto producer=frame(*b,20);b->offer(std::move(producer));before=submits;
 submit_result=VK_ERROR_DEVICE_LOST;rejects([&]{b->dispatch_pending();});require(submits==before+1);submit_result=VK_SUCCESS;b.reset();submits=0;
 // No host waits, slot allocations or heap activity on repeated admitted cycles.
 b=bridge(argv[1]);{auto warm=frame(*b,99);b->offer(std::move(warm));b->dispatch_pending();complete[uintptr_t(b->active().core.slot().fence)]=true;require(b->collect(done));done={};}
 auto allocations=heap_allocations,buffers=buffer_allocations,waits=wait_calls;
 auto start=std::chrono::steady_clock::now();constexpr unsigned cycles=10000;
 for(unsigned i=0;i<cycles;i++){
  auto q=frame(*b,100+i);b->offer(std::move(q));b->dispatch_pending();
  complete[uintptr_t(b->active().core.slot().fence)]=true;require(b->collect(done));done={};
 }
 auto ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/cycles;
 require(heap_allocations==allocations&&buffer_allocations==buffers&&wait_calls==waits);
 std::cout<<"cpu_fake_driver_bridge_ns_per_cycle="<<ns<<"; cycles="<<cycles<<"; hot_heap_allocations=0; hot_buffer_allocations=0; hot_waits=0\n";
 b.reset();
 // The legacy CPU path needs no Alignment and keeps late core leases alive.
 {RawCaptureBridge cpu(std::make_unique<Pipeline>(8,8,"cpu",""));auto q=cpu.acquire();cpu.offer(std::move(q));cpu.dispatch_pending();require(cpu.collect(done));require(done.core&&!done.depth);done={};}
 std::cout<<"actual FrameLease/AlignmentLease: pending, three-slot retention, submit order, source/core faults, failed consumer, shutdown and late consumer PASS\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
