// Actual module host paths with an explicitly synthetic Vulkan driver. No loader,
// instance, physical device, queue or GPU work is used. Driver timings are excluded.
#include "so101_station_vulkan/raw_capture_bridge.hpp"
#include "so101_station_vulkan/raw_camera_admission.hpp"
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
unsigned live_command_pools=0; void* mapped_buffers[4096]{};uint32_t producer_fault=0;const Buffer* watched_source=nullptr;VkFence watched_consumer=VK_NULL_HANDLE;bool consumer_drained=false,source_released_after_consumer=false;
struct Event {unsigned kind=0,pipeline=0;VkBuffer src{},dst{};VkBufferCopy copy{};VkPipelineStageFlags source_stage=0,dest_stage=0;VkAccessFlags source_access=0,dest_access=0;};
Event events[1024];unsigned event_count=0,created_pipelines=0,current_pipeline=0;bool trace=false;VkPipeline pipeline_handles[128]{}; VkCommandPool command_pools[4096]{}; VkCommandBuffer submitted_commands[16]{}; unsigned submitted_count=0;
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
Context::Context(){properties.limits.maxPushConstantsSize=128;properties.limits.maxComputeWorkGroupCount[0]=65535;properties.limits.maxComputeWorkGroupCount[1]=65535;properties.limits.maxComputeWorkGroupCount[2]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeWorkGroupSize[1]=16;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=256*1024*1024;enabled_features.features.shaderFloat64=VK_TRUE;commands=handle<VkCommandPool>();}
Context::~Context()=default;
std::shared_ptr<Context> Context::shared(){return std::make_shared<Context>();}
Buffer::~Buffer(){reset();}
void Buffer::reset()noexcept{if(this==watched_source){require(consumer_drained);source_released_after_consumer=true;}std::free(mapped);mapped=nullptr;buffer=VK_NULL_HANDLE;size=0;}
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
VkResult vkResetFences(VkDevice,uint32_t count,const VkFence* f){if(trace)event(4);for(unsigned i=0;i<count;i++)complete[uintptr_t(f[i])]=false;return VK_SUCCESS;}
VkResult vkWaitForFences(VkDevice,uint32_t count,const VkFence* fences,VkBool32,uint64_t){++wait_calls;for(unsigned i=0;i<count;i++)if(fences[i]==watched_consumer)consumer_drained=true;return VK_SUCCESS;}
VkResult vkAllocateCommandBuffers(VkDevice,const VkCommandBufferAllocateInfo* info,VkCommandBuffer* out){for(unsigned i=0;i<info->commandBufferCount;i++){out[i]=handle<VkCommandBuffer>();command_pools[uintptr_t(out[i])]=info->commandPool;}return VK_SUCCESS;}
void vkFreeCommandBuffers(VkDevice,VkCommandPool,uint32_t,const VkCommandBuffer*){}
VkResult vkAllocateDescriptorSets(VkDevice,const VkDescriptorSetAllocateInfo* info,VkDescriptorSet* out){for(unsigned i=0;i<info->descriptorSetCount;i++)out[i]=handle<VkDescriptorSet>();return VK_SUCCESS;}
VkResult vkCreateComputePipelines(VkDevice,VkPipelineCache,uint32_t n,const VkComputePipelineCreateInfo*,const VkAllocationCallbacks*,VkPipeline* out){for(unsigned i=0;i<n;i++){out[i]=handle<VkPipeline>();pipeline_handles[created_pipelines++]=out[i];}return VK_SUCCESS;}
void vkUpdateDescriptorSets(VkDevice,uint32_t,const VkWriteDescriptorSet*,uint32_t,const VkCopyDescriptorSet*){}
VkResult vkResetCommandBuffer(VkCommandBuffer,VkCommandBufferResetFlags){if(trace)event(6);return VK_SUCCESS;}
VkResult vkBeginCommandBuffer(VkCommandBuffer,const VkCommandBufferBeginInfo*){return VK_SUCCESS;}
VkResult vkEndCommandBuffer(VkCommandBuffer){return VK_SUCCESS;}
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer src,VkBuffer dst,uint32_t n,const VkBufferCopy* copy){if(n==1&&copy->size==4&&mapped_buffers[uintptr_t(dst)])*static_cast<uint32_t*>(mapped_buffers[uintptr_t(dst)])=producer_fault;if(trace){require(n==1);auto& e=event(1);e.src=src;e.dst=dst;e.copy=*copy;}}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize,VkDeviceSize,uint32_t){}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags src,VkPipelineStageFlags dst,VkDependencyFlags,uint32_t n,const VkMemoryBarrier* b,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){if(trace){require(n==1);auto& e=event(2);e.source_stage=src;e.dest_stage=dst;e.source_access=b->srcAccessMask;e.dest_access=b->dstAccessMask;}}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t,const void*){}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline pipeline){for(unsigned i=0;i<created_pipelines;i++)if(pipeline==pipeline_handles[i])current_pipeline=i%8;}
void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){if(trace)event(3).pipeline=current_pipeline;}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo* info,VkFence f){if(trace){event(5);require(submitted_count<16);submitted_commands[submitted_count++]=*info->pCommandBuffers;}++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}

extern "C" {
void vkCmdResetQueryPool(VkCommandBuffer,VkQueryPool,uint32_t,uint32_t){}
void vkCmdWriteTimestamp(VkCommandBuffer,VkPipelineStageFlagBits,VkQueryPool,uint32_t){}
VkResult vkGetQueryPoolResults(VkDevice,VkQueryPool,uint32_t,uint32_t,size_t,void*,VkDeviceSize,VkQueryResultFlags){return VK_SUCCESS;}
}

// Actual Pipeline + Alignment + bridge sources link against this fake API.
// No Vulkan loader, SDK camera, shader execution or data movement is simulated.
AlignmentCalibration calibration(const CameraCaptureProfile& profile){
 AlignmentCalibration c;c.generation=7;c.convention=AlignmentConvention::Scalar;
 c.depth={int(profile.depth_width),int(profile.depth_height),profile.depth_width*.5f,profile.depth_height*.5f,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};
 c.color={int(profile.color_width),int(profile.color_height),profile.color_width*.5f,profile.color_height*.5f,900,900,RS2_DISTORTION_NONE,{0,0,0,0,0}};
 c.depth_to_color.rotation[0]=c.depth_to_color.rotation[4]=c.depth_to_color.rotation[8]=1;return c;
}
constexpr char serial[]="camera-serial-longer-than-small-string-capacity-012345678901234567890123456789";
RawFrame frame(RawCaptureBridge& b,uint64_t id){auto f=b.acquire();require(bool(f));auto& m=f.core.slot().metadata;m.frame_number=id;m.depth_frame_number=id+9;m.timestamp_ns=id*1000;m.calibration_generation=7;m.hardware_timestamp_ms=id;m.depth_hardware_timestamp_ms=id+1;m.timestamp_domain=1;m.depth_timestamp_domain=2;m.serial=serial;f.core.slot().calibration=b.expected_calibration();f.depth.metadata()=m;return f;}
struct FakeSourceFrame{uint64_t get_frame_number()const{return 101;}double get_timestamp()const{return 1234.5;}int get_frame_timestamp_domain()const{return 1;}};
struct FakeSourceFrames{FakeSourceFrame get_color_frame()const{return {};}FakeSourceFrame get_depth_frame()const{return {};}};
struct ActualMetadataBodyFixture{
 std::string serial_=serial;CameraSourceCalibration source_;
 void original_metadata(FrameMetadata& m,const FakeSourceFrames& native,uint64_t receipt)
 #include "camera_metadata_shared.h"
};
int main(int argc,char** argv){try{
 require(argc==3||argc==4);const char* profile_name=argc==4?argv[3]:"vga";const auto profile=camera_capture_profile(profile_name);
 const size_t color_n=size_t(profile.color_width)*profile.color_height,depth_n=size_t(profile.depth_width)*profile.depth_height;
 const auto k=calibration(profile);
 auto core=std::make_unique<Pipeline>(profile.color_width,profile.color_height,"vulkan",argv[1]);auto context=core->context();
 auto depth=std::make_unique<Alignment>(context,k,argv[2]);RawCaptureBridge bridge(std::move(core),std::move(depth));
 // Reserve all three actual slots exactly as the camera does before callbacks.
 {std::array<RawFrame,3> warm;for(auto& f:warm){f=bridge.acquire();require(bool(f));f.core.slot().metadata.serial.reserve(sizeof(serial));f.depth.metadata().serial.reserve(sizeof(serial));}}
 auto f=frame(bridge,1);auto src_buffer=f.depth.aligned().buffer;auto src_fence=f.depth.fence();auto consumer_fence=f.core.slot().fence;
 event_count=0;submitted_count=0;trace=true;hold=true;bridge.offer(std::move(f));bridge.dispatch_pending();trace=false;
 require(submitted_count==2);require(submitted_commands[0]!=submitted_commands[1]);
 require(command_pools[uintptr_t(submitted_commands[0])]!=context->commands);
 require(command_pools[uintptr_t(submitted_commands[1])]==context->commands);
 uint64_t upload_rgb=0,upload_raw=0,d2d=0,alignment_status=0,compact=0;unsigned copies=0,dispatches=0,barriers=0;
 for(unsigned i=0;i<event_count;i++){const auto& e=events[i];dispatches+=e.kind==3;barriers+=e.kind==2;if(e.kind!=1)continue;
  ++copies;if(e.src==src_buffer){d2d+=e.copy.size;require(e.copy.size==2*color_n&&e.copy.dstOffset==3*color_n);}
  else if(e.copy.size==3*color_n)upload_rgb+=e.copy.size;
  else if(e.copy.size==2*depth_n)upload_raw+=e.copy.size;
  else if(e.copy.size==4)alignment_status+=e.copy.size;
  else if(e.copy.size==compact_bytes())compact+=e.copy.size;
  else require(false);
 }
 require(copies==5&&upload_rgb==3*color_n&&upload_raw==2*depth_n&&d2d==2*color_n&&alignment_status==4&&compact==163856&&dispatches==73&&barriers==79);
 complete[uintptr_t(src_fence)]=true;RawFrame done;require(!bridge.collect(done));
 {auto two=frame(bridge,2),three=frame(bridge,3);require(!bridge.acquire());require(two.depth.aligned().buffer!=src_buffer&&three.depth.aligned().buffer!=src_buffer);}
 complete[uintptr_t(consumer_fence)]=true;require(bridge.collect(done));auto late=done.core;done={};
 // A late core consumer retains its own storage but can release raw source now.
 {auto two=frame(bridge,4),three=frame(bridge,5);require(!bridge.acquire());}late={};hold=false;
 // CPU-only helper admission cost. Buffers are cold allocated once, no SDK
 // frame getters, device work, image copies or ROS publication are included.
 std::vector<uint8_t> vendor(color_n*2);std::vector<uint16_t> native(depth_n),aligned(color_n);
 std::vector<uint32_t> reference(color_n);std::vector<float> xyz(color_n*4);
 for(unsigned i=0;i<24;i++){vendor[2*i]=1;native[i]=1;aligned[i]=1;reference[i]=1;}
 Slot cpu_slot,gpu_slot;FrameMetadata metadata;metadata.width=profile.color_width;metadata.height=profile.color_height;metadata.calibration_generation=7;metadata.serial=serial;
 std::array<double,7> cold_times{};
 for(auto& timing:cold_times){ColdCameraSelfcheck gate(7,serial,profile.color_width,profile.color_height,profile.depth_width,profile.depth_height);auto started=std::chrono::steady_clock::now();for(unsigned pass=1;pass<=3;++pass){metadata.frame_number=metadata.depth_frame_number=pass;gate.verify(metadata,native.data(),vendor.data(),aligned.data(),reference.data(),xyz.data(),xyz.data(),cpu_slot,gpu_slot);}gate.require_complete();timing=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-started).count()/3;}
 std::sort(cold_times.begin(),cold_times.end());double cold_ns=cold_times[3];
 CameraSourceCalibration camera_source;camera_source.alignment=k;
 camera_source.color={2,0,int(profile.color_width),int(profile.color_height),30,RS2_FORMAT_RGB8,RS2_STREAM_COLOR};
 camera_source.depth={1,0,int(profile.depth_width),int(profile.depth_height),30,RS2_FORMAT_Z16,RS2_STREAM_DEPTH};
 volatile unsigned matches=0;auto comparison_start=std::chrono::steady_clock::now();
 for(unsigned i=0;i<100000;++i)matches+=same_camera_source(camera_source,camera_source);
 double source_ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-comparison_start).count()/100000;require(matches==100000);
 ResidentPublicationGate publication;publication.reserve(&publication);publication.admit(&publication,7);unsigned publications=0;
 auto gate_start=std::chrono::steady_clock::now();
 for(unsigned i=0;i<100000;i++)require(publication.publish(7,[&]{++publications;}));
 double gate_ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-gate_start).count()/100000;require(publications==100000);
 ActualMetadataBodyFixture metadata_fixture;metadata_fixture.source_=camera_source;
 FrameMetadata current,failure,copied_core,copied_depth;for(auto* m:{&current,&failure,&copied_core,&copied_depth})m->serial.reserve(sizeof(serial));
 auto metadata_allocations=heap_allocations;auto metadata_start=std::chrono::steady_clock::now();
 for(unsigned i=0;i<100000;i++){metadata_fixture.original_metadata(current,FakeSourceFrames{},i);failure=current;copied_core=current;copied_depth=current;}
 double metadata_ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-metadata_start).count()/100000;require(heap_allocations==metadata_allocations&&copied_depth.serial==serial&&copied_depth.width==profile.color_width&&copied_depth.height==profile.color_height);
 auto before=heap_allocations,buffers=buffer_allocations,waits=wait_calls;constexpr unsigned warmup=1000,cycles=100000;
 auto cycle=[&](uint64_t id){auto q=frame(bridge,id);require(bridge.accept_source(id,id+9));bridge.offer(std::move(q));bridge.dispatch_pending();require(bridge.collect(done));done={};};
 for(unsigned i=0;i<warmup;i++)cycle(100+i);
 require(heap_allocations==before&&buffer_allocations==buffers&&wait_calls==waits);
 auto start=std::chrono::steady_clock::now();
 for(unsigned i=0;i<cycles;i++)cycle(10000+i);
 double ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/cycles;
 require(heap_allocations==before&&buffer_allocations==buffers&&wait_calls==waits);
 // The normal fake device emits zero components. Also time the bounded maximum
 // compact decode/sort on the same actual module path, without dynamic storage.
 {std::array<RawFrame,3> seeded;for(auto& q:seeded){q=bridge.acquire();require(bool(q));auto* words=static_cast<uint32_t*>(q.core.slot().readback.mapped);words[0]=kMaxComponents;words[1]=0;for(unsigned i=0;i<kMaxComponents;i++)words[4+10*i]=kMaxComponents-i;}}
 before=heap_allocations;buffers=buffer_allocations;waits=wait_calls;start=std::chrono::steady_clock::now();
 for(unsigned i=0;i<1000;i++){auto q=frame(bridge,200000+i);require(bridge.accept_source(200000+i,200009+i));bridge.offer(std::move(q));bridge.dispatch_pending();require(bridge.collect(done));require(done.core.slot().component_count==kMaxComponents);require(done.core.slot().cpu_components[0].root==1);done={};}
 double full_ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/1000;
 require(heap_allocations==before&&buffer_allocations==buffers&&wait_calls==waits);
 // Exercise real core decode faults and producer status faults through the
 // same bridge. Neither may return a completed frame or recycle active source.
 for(unsigned fault=0;fault<2;fault++){
  auto p=std::make_unique<Pipeline>(profile.color_width,profile.color_height,"vulkan",argv[1]);auto a=std::make_unique<Alignment>(p->context(),k,argv[2]);
  auto bad=std::make_unique<RawCaptureBridge>(std::move(p),std::move(a));auto q=frame(*bad,70000+fault);
  if(fault==0)static_cast<uint32_t*>(q.core.slot().readback.mapped)[1]=1;else producer_fault=1;
  bad->offer(std::move(q));bad->dispatch_pending();bool rejected=false;
  try{bad->collect(done);}catch(const std::exception&){rejected=true;}require(rejected&&!done&&bad->active().depth);
  rejected=false;try{bad->acquire();}catch(const std::exception&){rejected=true;}require(rejected);
  watched_source=&bad->active().depth.aligned();watched_consumer=bad->active().core.slot().fence;consumer_drained=false;source_released_after_consumer=false;
  bad.reset();require(source_released_after_consumer);watched_source=nullptr;watched_consumer=VK_NULL_HANDLE;producer_fault=0;
 }
 std::cout<<"{\"scope\":\"actual Pipeline+Alignment+RawCaptureBridge; synthetic Vulkan API; excludes driver, GPU, SDK and host image copies\",\"profile\":\""<<profile_name<<"\",\"color_pixels\":"<<color_n<<",\"depth_pixels\":"<<depth_n<<",\"warmup_iterations\":"<<warmup<<",\"iterations\":"<<cycles<<",\"host_ns_per_cycle\":"<<ns<<",\"cold_verify_ns_per_cohort\":"<<cold_ns<<",\"max4096_component_cycle_ns\":"<<full_ns<<",\"source_calibration_compare_ns\":"<<source_ns<<",\"publication_gate_noop_ns\":"<<gate_ns<<",\"metadata77_builder_and_copy_ns\":"<<metadata_ns<<",\"metadata77_hot_heap_allocations\":0,\"hot_heap_allocations\":0,\"hot_buffer_allocations\":0,\"hot_wait_calls\":0,\"rgb_upload_bytes\":"<<upload_rgb<<",\"raw_upload_bytes\":"<<upload_raw<<",\"aligned_d2d_bytes\":"<<d2d<<",\"alignment_readback_bytes\":"<<alignment_status<<",\"core_readback_bytes\":"<<compact<<",\"dispatches\":"<<dispatches<<",\"barriers\":"<<barriers<<",\"ordered_submissions\":2,\"distinct_command_pools\":true,\"source_retained_through_consumer_fence\":true,\"actual_source_core_faults_rejected\":true,\"actual_failure_shutdown_consumer_first\":true}\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
