// Actual child/parent host paths against synthetic Vulkan; no loader/device.
#include "so101_station_vulkan/children.hpp"
#include <cfenv>
#include <limits>
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
unsigned live_command_pools=0,dispatches=0;
struct FakeBuffer{void* data=nullptr;size_t size=0;VkBufferUsageFlags usage=0;};FakeBuffer fake_memory[4096];
VkBuffer compact_buffer=VK_NULL_HANDLE,last_readback=VK_NULL_HANDLE;
uint32_t parameters[14]{};
uint32_t child_parameters[32]{};bool child_mode=false,indirect_barrier=false;unsigned indirects=0;
VkBuffer child_compact=VK_NULL_HANDLE,child_parent_compact=VK_NULL_HANDLE,child_arguments=VK_NULL_HANDLE;
std::array<ParentChildStatus,4096> fake_status{};std::array<ChildRecord,4096> fake_records{};unsigned fake_children=0,fake_packed=0,fake_jobs=0,fake_fault=0;
bool hold=false;bool complete[4096]{};
template<class T>T handle(){return (T)(uintptr_t(next_handle++));}
void require(bool ok){if(!ok)throw std::runtime_error("preparation host contract failed");}
}
void* operator new(size_t n){if(auto p=std::malloc(n?n:1)){++heap_allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
namespace so101_station_vulkan {
Context::Context(){properties.limits.maxPushConstantsSize=128;properties.limits.maxComputeWorkGroupCount[0]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=128*1024*1024;properties.limits.maxPerStageDescriptorStorageBuffers=32;properties.limits.maxDescriptorSetStorageBuffers=32;properties.limits.minStorageBufferOffsetAlignment=256;enabled_features.features.shaderFloat64=VK_FALSE;enabled_features.features.shaderInt64=VK_FALSE;}
Context::~Context()=default;
Buffer::~Buffer(){reset();}
void Buffer::reset()noexcept{if(buffer){std::free(fake_memory[uintptr_t(buffer)].data);fake_memory[uintptr_t(buffer)]={};}mapped=nullptr;buffer=VK_NULL_HANDLE;size=0;}
void Buffer::create(Context& c,size_t n,VkBufferUsageFlags usage,VkMemoryPropertyFlags flags){++buffer_allocations;owner=&c;size=n;buffer=handle<VkBuffer>();auto& m=fake_memory[uintptr_t(buffer)];m={std::calloc(n,1),n,usage};if(!m.data)throw std::bad_alloc();if(flags&VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)mapped=m.data;}
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
void vkUpdateDescriptorSets(VkDevice,uint32_t n,const VkWriteDescriptorSet* writes,uint32_t,const VkCopyDescriptorSet*){if(n==3){child_mode=false;for(unsigned i=0;i<n;i++)if(writes[i].dstBinding==2)compact_buffer=writes[i].pBufferInfo->buffer;}else{require(n==11);child_mode=true;for(unsigned i=0;i<n;i++){require(writes[i].pBufferInfo->offset==0);if(writes[i].dstBinding==2)child_parent_compact=writes[i].pBufferInfo->buffer;if(writes[i].dstBinding==8)child_compact=writes[i].pBufferInfo->buffer;if(writes[i].dstBinding==10)child_arguments=writes[i].pBufferInfo->buffer;}}}
VkResult vkResetCommandBuffer(VkCommandBuffer,VkCommandBufferResetFlags){return VK_SUCCESS;}
VkResult vkBeginCommandBuffer(VkCommandBuffer,const VkCommandBufferBeginInfo*){return VK_SUCCESS;}
VkResult vkEndCommandBuffer(VkCommandBuffer){return VK_SUCCESS;}
void vkCmdDispatchIndirect(VkCommandBuffer,VkBuffer buffer,VkDeviceSize offset){require(buffer==child_arguments&&offset==0&&indirect_barrier);require((fake_memory[uintptr_t(buffer)].usage&VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT)!=0);++dispatches;++indirects;}
void emit_child(){auto* w=static_cast<uint32_t*>(fake_memory[uintptr_t(child_compact)].data);auto* p=child_parameters;auto* parents=static_cast<uint32_t*>(fake_memory[uintptr_t(child_parent_compact)].data);w[0]=1;w[1]=p[0];w[2]=p[1];w[3]=parents[7];w[4]=fake_children;w[5]=fake_packed;w[6]=fake_jobs;w[7]=fake_fault;w[8]=p[16];w[9]=p[17];w[10]=p[18];w[11]=p[19];w[12]=p[20];w[13]=p[21];w[14]=p[10];w[15]=p[11];w[16]=p[12];w[17]=p[5];w[18]=p[6];w[19]=p[7];w[20]=p[4];w[22]=p[22];w[23]=p[23];w[24]=p[24];w[25]=p[25];std::memcpy(w+32,fake_status.data(),size_t(p[4])*16);std::memcpy(w+32+4*p[4],fake_records.data(),size_t(std::min(fake_children,p[5]))*64);}
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer from,VkBuffer to,uint32_t n,const VkBufferCopy* copies){if(child_mode)emit_child();for(unsigned i=0;i<n;i++)std::memcpy(static_cast<char*>(fake_memory[uintptr_t(to)].data)+copies[i].dstOffset,static_cast<char*>(fake_memory[uintptr_t(from)].data)+copies[i].srcOffset,copies[i].size);last_readback=to;}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer b,VkDeviceSize offset,VkDeviceSize size,uint32_t){std::memset(static_cast<char*>(fake_memory[uintptr_t(b)].data)+offset,0,size);}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags src,VkPipelineStageFlags dst,VkDependencyFlags,uint32_t count,const VkMemoryBarrier* b,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){if((src&VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT)&&(dst&VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT)&&(dst&VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT)&&count&&((b[0].dstAccessMask&VK_ACCESS_INDIRECT_COMMAND_READ_BIT)!=0)&&((b[0].dstAccessMask&VK_ACCESS_SHADER_READ_BIT)!=0))indirect_barrier=true;}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t n,const void* data){child_mode=n==128;if(child_mode)std::memcpy(child_parameters,data,n);else{require(n==sizeof(parameters));std::memcpy(parameters,data,n);}}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline){}
void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){
 ++dispatches;if(child_mode)return;auto* words=static_cast<uint32_t*>(fake_memory[uintptr_t(compact_buffer)].data);
 // Synthetic EMPTY shader result exercises host admission/lifetime only.
 words[0]=1;words[1]=parameters[10];words[2]=parameters[11];words[3]=parameters[12];words[4]=parameters[13];words[5]=parameters[0];words[6]=parameters[1];
}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo*,VkFence f){++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}

template<class E=std::exception,class F>void rejects(F fn){bool rejected=false;try{fn();}catch(const E&){rejected=true;}require(rejected);}
void parent_result(unsigned count,unsigned width,unsigned height){
 auto* gpu=static_cast<uint32_t*>(fake_memory[uintptr_t(compact_buffer)].data);auto* host=static_cast<uint32_t*>(fake_memory[uintptr_t(last_readback)].data);gpu[7]=count;gpu[9]=count;
 auto* records=reinterpret_cast<ParentRecord*>(gpu+16);for(unsigned j=0;j<count;j++){unsigned y=count==2?j*(height/2):0,h=count==2?height/2:height;records[j]={0,j+1,(y/2)*((width+1)/2),width*h,0,y,width,h};}
 std::memcpy(host,gpu,64+32*count);
}
void source_metadata(FrameLease& source,const ParentConfig& config){auto& s=source.slot();s.submitted=true;s.metadata.calibration_generation=config.calibration_generation;s.calibration.generation=config.calibration_generation;s.metadata.frame_number=911;s.metadata.depth_frame_number=910;complete[uintptr_t(s.fence)]=true;}
void clear_result(){fake_status.fill({0,0,0,0});fake_records.fill({});fake_children=fake_packed=fake_jobs=fake_fault=0;}
int main(int argc,char** argv){
 require(argc==3);ChildConfig config;config.generation=0x300000004ull;config.min_child_pixels=8;
 ParentConfig expected;expected.width=8;expected.height=4;expected.min_parent_pixels=1;expected.record_capacity=2;expected.calibration_generation=0x100000002ull;expected.generation=0x200000003ull;config.record_capacity=2;
 config.validate(8,4,2);require(sizeof(child_detail::Parameters)==128);ChildGraphLayout max(640,480);require(max.total_bytes()==24538448&&3*max.total_bytes()==73615344);require(max.compact_bytes==327808);require(ChildGraphLayout(7,1025).count==7175);
 rejects<std::invalid_argument>([&]{ChildGraphLayout(0,1);});rejects<std::invalid_argument>([&]{ChildGraphLayout(2049,1);});rejects<std::invalid_argument>([&]{ChildGraphLayout(1024,1024);});rejects<std::invalid_argument>([&]{ChildGraphLayout(8,4,2,2,129);});
 auto bad=config;bad.generation=0;rejects<std::invalid_argument>([&]{bad.validate(8,4,2);});bad=config;bad.min_child_pixels=7;rejects<std::invalid_argument>([&]{bad.validate(8,4,2);});bad=config;bad.max_observation_depth_m=.10;rejects<std::invalid_argument>([&]{bad.validate(8,4,2);});bad=config;bad.max_neighbor_jump_m=0;rejects<std::invalid_argument>([&]{bad.validate(8,4,2);});bad=config;bad.max_neighbor_jump_m=std::numeric_limits<double>::infinity();rejects<std::invalid_argument>([&]{bad.validate(8,4,2);});
 bad=config;bad.max_observation_depth_m=1e300;bad.max_neighbor_jump_m=1e-50;bad.validate(8,4,2);require(child_f32_rne_bits(bad.max_observation_depth_m)==0x7f800000&&child_f32_rne_bits(bad.max_neighbor_jump_m)==0);
 for(int mode:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO}){require(std::fesetround(mode)==0);require(child_f32_rne_bits(1.0+0x1p-24)==0x3f800000);require(child_f32_rne_bits(1.0+0x1.8p-23)==0x3f800002);require(child_f32_rne_bits(0x1p-150)==0);require(child_f32_rne_bits(1e300)==0x7f800000);}std::fesetround(FE_TONEAREST);
 ChildLease empty;rejects<std::logic_error>([&]{empty.parent();});rejects<std::logic_error>([&]{empty.source_root_buffer();});rejects<std::logic_error>([&]{empty.packed_flags_buffer();});rejects<std::logic_error>([&]{empty.records();});rejects<std::logic_error>([&]{empty.parent_status();});
 auto context=std::make_shared<Context>();auto pool=std::make_shared<FramePool>(8,4,context,false);auto source=pool->acquire();source_metadata(source,expected);
 ParentPreparation parents(context,pool->source_token(),expected,argv[1]);auto parent=parents.submit(source);parent_result(1,8,4);auto parentcopy=parent;require(parent.configuration().generation==expected.generation&&parent.source_frame().belongs_to(pool->source_token()));
 ChildPreparation children(context,pool->source_token(),expected,config,argv[2]);clear_result();
 rejects<std::invalid_argument>([&]{children.ready(empty);});rejects<std::invalid_argument>([&]{children.submit({});});
 // Strict producer identity even with the same Context/slot/config dimensions.
 {auto other=std::make_shared<FramePool>(8,4,context,false);auto f=other->acquire();source_metadata(f,expected);ParentPreparation foreign(context,other->source_token(),expected,argv[1]);auto p=foreign.submit(f);parent_result(1,8,4);rejects<std::invalid_argument>([&]{children.submit(p);});}
 // Complete expected parent configuration, not merely its generation.
 {auto wrong=expected;wrong.min_parent_pixels=2;ParentPreparation other(context,pool->source_token(),wrong,argv[1]);auto p=other.submit(source);parent_result(1,8,4);rejects<std::invalid_argument>([&]{children.submit(p);});}
 auto& slot=source.slot();slot.fault=1;rejects<std::invalid_argument>([&]{children.submit(parent);});slot.fault=0;auto* core=static_cast<uint32_t*>(slot.readback.mapped);core[1]=1;rejects<std::runtime_error>([&]{children.submit(parent);});core[1]=0;
 ++slot.metadata.calibration_generation;rejects<std::invalid_argument>([&]{children.submit(parent);});--slot.metadata.calibration_generation;++slot.calibration.generation;rejects<std::invalid_argument>([&]{children.submit(parent);});--slot.calibration.generation;
 complete[uintptr_t(slot.fence)]=false;rejects<std::logic_error>([&]{children.submit(parent);});complete[uintptr_t(slot.fence)]=true;
 auto initial=slot.references.load();submit_result=VK_ERROR_OUT_OF_HOST_MEMORY;rejects<std::runtime_error>([&]{children.submit(parent);});require(slot.references==initial);submit_result=VK_SUCCESS;
 // Pending/dropped child retains BOTH its parent and exact source lease.
 hold=true;{auto a=children.submit(parent);auto b=a;auto moved=std::move(b);require(!b&&moved&&!children.ready(moved));require(slot.references==initial+1);rejects<std::logic_error>([&]{children.finish(moved);});rejects<std::logic_error>([&]{moved.parent();});rejects<std::logic_error>([&]{children.submit(parent);});}children.collect();require(slot.references==initial+1);std::fill(std::begin(complete),std::end(complete),true);children.collect();require(slot.references==initial);hold=false;
 {auto a=children.submit(parent);auto counts=children.finish(a);require(counts.parent_count==1&&counts.record_count==0&&counts.configuration_generation==config.generation);require(a.source_root_buffer()&&a.packed_flags_buffer()&&a.resident_records_buffer()&&a.source_flags_offset_bytes()==128);auto p=a.parent();require(p.index()==parent.index()&&p.source_frame().belongs_to(pool->source_token()));require(a.metadata().frame_number==911&&a.parent_status()[0].valid_depth_pixels==0);auto* data=a.records();require(a.records()==data);rejects<std::logic_error>([&]{children.submit(parent);});}
 for(unsigned fault:{1u,2u,4u,8u,16u,32u,64u}){clear_result();fake_fault=fault;auto a=children.submit(parent);rejects<std::runtime_error>([&]{children.finish(a);});rejects<std::runtime_error>([&]{a.parent();});rejects<std::runtime_error>([&]{a.source_root_buffer();});rejects<std::runtime_error>([&]{a.source_flags_offset_bytes();});rejects<std::runtime_error>([&]{a.packed_flags_buffer();});rejects<std::runtime_error>([&]{a.resident_records_buffer();});rejects<std::runtime_error>([&]{a.records();});rejects<std::runtime_error>([&]{a.parent_status();});}
 clear_result();fake_children=2;fake_packed=32;fake_jobs=2;fake_status[0]={16,16,2,2};fake_records[0]={1,0,0,8,0,0,2,4,0,0,4,4,0,0,8,0};fake_records[1]={7,0,3,8,6,0,2,4,4,0,4,4,16,0,8,0};
 {auto a=children.submit(parent);require(children.finish(a).record_count==2);require(a.records()[1].key==3&&a.parent_status()[0].kept_children==2);}
 fake_records[1].key=0;{auto a=children.submit(parent);rejects<std::runtime_error>([&]{children.finish(a);});}fake_records[1].key=3;
 // Parent status capacity P is independent of child record capacity C.
 {auto pc=expected;pc.record_capacity=2;ParentPreparation pp(context,pool->source_token(),pc,argv[1]);auto p=pp.submit(source);parent_result(2,8,4);auto cc=config;cc.record_capacity=1;ChildPreparation c(context,pool->source_token(),pc,cc,argv[2]);clear_result();fake_status[0]={16,0,0,0};fake_status[1]={16,0,0,0};auto a=c.submit(p);auto n=c.finish(a);require(n.parent_count==2&&n.record_count==0&&a.parent_status()[1].valid_depth_pixels==16);}
 // Many subminimum children can exceed C without overflowing kept records.
 clear_result();fake_status[0]={32,32,32,0};{auto a=children.submit(parent);require(children.finish(a).record_count==0&&a.parent_status()[0].all_children==32);}
 // P=1/C=2 accepts two children; retained parent bridge adds no source slot.
 {auto pc=expected;pc.record_capacity=1;ParentPreparation pp(context,pool->source_token(),pc,argv[1]);auto p=pp.submit(source);parent_result(1,8,4);ChildPreparation c(context,pool->source_token(),pc,config,argv[2]);clear_result();fake_children=2;fake_packed=32;fake_jobs=2;fake_status[0]={16,16,2,2};fake_records[0]={1,0,0,8,0,0,2,4,0,0,4,4,0,0,8,0};fake_records[1]={7,0,3,8,6,0,2,4,4,0,4,4,16,0,8,0};auto a=c.submit(p);require(c.finish(a).parent_count==1&&a.records()[1].root==7);}
 // Per-parent kept status must match sorted record ownership, not merely total R.
 {auto pc=expected;ParentPreparation pp(context,pool->source_token(),pc,argv[1]);auto p=pp.submit(source);parent_result(2,8,4);auto cc=config;cc.record_capacity=1;ChildPreparation c(context,pool->source_token(),pc,cc,argv[2]);clear_result();fake_children=1;fake_packed=12;fake_jobs=1;fake_status[0]={16,8,1,1};fake_status[1]={16,8,1,0};fake_records[0]={17,1,0,8,0,0,4,2,0,0,6,2,0,0,8,0};auto a=c.submit(p);bool mismatch=false;try{c.finish(a);}catch(const std::runtime_error& e){mismatch=std::string(e.what()).find("per-parent record/status mismatch")!=std::string::npos;}require(mismatch);}
 // Tiny odd parent label offset160 is not256aligned, descriptors remain offset0.
 {auto pc=expected;pc.width=pc.height=1;pc.record_capacity=1;auto tiny_pool=std::make_shared<FramePool>(1,1,context,false);auto f=tiny_pool->acquire();source_metadata(f,pc);ParentPreparation pp(context,tiny_pool->source_token(),pc,argv[1]);auto p=pp.submit(f);parent_result(1,1,1);auto cc=config;cc.record_capacity=1;ChildPreparation c(context,tiny_pool->source_token(),pc,cc,argv[2]);clear_result();auto a=c.submit(p);require(c.finish(a).record_count==0&&child_parameters[28]==40);auto bridge=a.parent();f={};p={};a={};c.collect();auto b=tiny_pool->acquire(),d=tiny_pool->acquire();require(b&&d&&!tiny_pool->acquire());bridge={};pp.collect();require(bool(tiny_pool->acquire()));}
 {auto pc=expected;pc.width=7;pc.height=1025;rejects<std::invalid_argument>([&]{ChildPreparation c(context,pool->source_token(),pc,config,argv[2]);});}
 // Every source-index arena stays busy through a downstream copied lease.
 clear_result();hold=true;{auto b=pool->acquire(),d=pool->acquire();require(b&&d&&!pool->acquire());source_metadata(b,expected);source_metadata(d,expected);hold=false;auto pb=parents.submit(b);parent_result(1,8,4);auto pd=parents.submit(d);parent_result(1,8,4);hold=true;auto a=children.submit(parent),bb=children.submit(pb),dd=children.submit(pd);b={};d={};a={};bb={};dd={};children.collect();require(!pool->acquire());std::fill(std::begin(complete),std::end(complete),true);children.collect();hold=false;}
 auto heaps=heap_allocations,buffers=buffer_allocations,waits=wait_calls;auto before=dispatches;auto indirect_before=indirects;
 for(unsigned i=0;i<100;i++){auto a=children.submit(parent);require(children.ready(a));require(children.finish(a).record_count==0);require(a.parent_status()&&a.records());}
 require(heap_allocations==heaps&&buffer_allocations==buffers&&wait_calls==waits);require(dispatches-before==16600&&indirects-indirect_before==6800);
 require(indirect_barrier);std::cout<<"child actual-module fakeVulkan contract PASS;100 cycles zero heap/Buffer/wait;166 dispatches,68 indirect percycle\n";
}
