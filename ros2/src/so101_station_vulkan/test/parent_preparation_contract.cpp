// CPU integer transcription of the recorded shader passes. Frozen OpenCV maps
// are the independent oracle. Host section uses synthetic Vulkan, no loader.
#include "so101_station_vulkan/parent_graph.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>
using namespace so101_station_vulkan;
static int graph(bool reverse=false){
 unsigned w,h,minimum,capacity,rounds;std::cin>>w>>h>>minimum>>capacity>>rounds;
 ParentGraphLayout p(w,h);std::vector<unsigned> mask(p.count),occupancy(p.nodes),a(p.nodes),b(p.nodes),labels(p.count),rank(p.nodes);
 for(auto& value:mask)std::cin>>value;
 unsigned fault=0;for(unsigned i=0;i<p.count;i++){
  auto value=mask[i];if(value!=0&&value!=1&&value!=2&&value!=4&&value!=8){fault|=1;continue;}
  for(unsigned c=0;c<4;c++)if(value==(1u<<c))occupancy[c*p.blocks+(i/w/2)*p.block_width+(i%w/2)]|=1u<<((i/w%2)*2+i%w%2);
 }
 for(unsigned j=0;j<p.nodes;j++)a[j]=b[j]=occupancy[j]?j+1:0;
 auto edges=[&](unsigned j,auto visit){
  unsigned key=j%p.blocks,x=key%p.block_width,y=key/p.block_width;
  if(x&&parent_block_edge(occupancy[j],occupancy[j-1],0))visit(j-1);
  if(y&&parent_block_edge(occupancy[j],occupancy[j-p.block_width],1))visit(j-p.block_width);
  if(x&&y&&parent_block_edge(occupancy[j],occupancy[j-p.block_width-1],2))visit(j-p.block_width-1);
  if(x+1<p.block_width&&y&&parent_block_edge(occupancy[j],occupancy[j-p.block_width+1],3))visit(j-p.block_width+1);
 };
 for(unsigned r=0;r<rounds;r++){
  for(unsigned item=0;item<p.nodes;item++){
   unsigned j=reverse?p.nodes-item-1:item;if(!occupancy[j])continue;
   a[j]=std::min(a[j],b[j]); // GPU uses InterlockedMin; never a plain Store.
   edges(j,[&](unsigned k){auto high=std::max(b[j],b[k]),low=std::min(b[j],b[k]);a[high-1]=std::min(a[high-1],low);});
  }
  // Separate pass: a is immutable throughout every root walk.
  for(unsigned j=0;j<p.nodes;j++)if(occupancy[j]){unsigned root=a[j],steps=0;while(root&&a[root-1]!=root&&steps++<p.nodes)root=a[root-1];b[j]=root;}
 }
 for(unsigned j=0;j<p.nodes;j++)if(occupancy[j]){
  if(!a[j]||a[j]>j+1||(a[j]-1)/p.blocks!=j/p.blocks||!b[j]||b[j]>j+1||!occupancy[b[j]-1]||b[b[j]-1]!=b[j])fault|=4;
  edges(j,[&](unsigned k){if(b[j]!=b[k])fault|=4;});
 }
 struct Stats{unsigned c,label,key,area=0,x0,y0,x1=0,y1=0,minpixel;};
 std::vector<Stats> components;
 // Same 256-lane ping-pong scan, per-color group totals, exclusive offsets
 // as the shader. Tail lanes participate with zero flags.
 for(unsigned c=0;c<4;c++){
  unsigned offset=0;
  for(unsigned group=0;group<p.scan_groups;group++){
   std::array<unsigned,256> scan{},other{};
   for(unsigned lane=0;lane<256;lane++){unsigned key=group*256+lane,j=c*p.blocks+key;if(key<p.blocks)scan[lane]=b[j]==j+1;}
   for(unsigned stride=1;stride<256;stride*=2){for(unsigned lane=0;lane<256;lane++)other[lane]=scan[lane]+(lane>=stride?scan[lane-stride]:0);scan.swap(other);}
   for(unsigned lane=0;lane<256;lane++){unsigned key=group*256+lane,j=c*p.blocks+key;if(key<p.blocks&&b[j]==j+1)rank[j]=offset+scan[lane];}
   offset+=scan[255];
  }
  for(unsigned j=c*p.blocks;j<(c+1)*p.blocks;j++)if(b[j]==j+1)components.push_back({c,rank[j],j%p.blocks,0,w,h,0,0,p.count});
 }
 for(unsigned i=0;i<p.count;i++)for(unsigned c=0;c<4;c++)if(mask[i]==1u<<c){
  auto j=c*p.blocks+(i/w/2)*p.block_width+i%w/2;labels[i]=rank[b[j]-1];
  auto pos=std::find_if(components.begin(),components.end(),[&](auto s){return s.c==c&&s.label==labels[i];});
  ++pos->area;pos->x0=std::min(pos->x0,i%w);pos->y0=std::min(pos->y0,i/w);pos->x1=std::max(pos->x1,i%w);pos->y1=std::max(pos->y1,i/w);pos->minpixel=std::min(pos->minpixel,i);
 }
 unsigned kept=std::count_if(components.begin(),components.end(),[&](auto s){return s.area>=minimum;});if(kept>capacity)fault|=2;
 std::cout<<"{\"fault\":"<<fault<<",\"labels\":[";for(unsigned y=0;y<h;y++){if(y)std::cout<<',';std::cout<<'[';for(unsigned x=0;x<w;x++){if(x)std::cout<<',';std::cout<<labels[y*w+x];}std::cout<<']';}std::cout<<"],\"components\":[";
 auto print=[](auto s){std::cout<<"{\"color\":"<<s.c<<",\"label\":"<<s.label<<",\"area\":"<<s.area<<",\"bounds_xywh\":["<<s.x0<<','<<s.y0<<','<<s.x1-s.x0+1<<','<<s.y1-s.y0+1<<"],\"min_flat_pixel\":"<<s.minpixel<<",\"min_block_key\":"<<s.key<<'}';};
 bool comma=false;for(auto s:components){if(comma)std::cout<<',';comma=true;print(s);}std::cout<<"],\"records\":[";comma=false;if(!fault)for(auto s:components)if(s.area>=minimum){if(comma)std::cout<<',';comma=true;print(s);}std::cout<<"]}\n";return 0;
}
static int walk_proof(){
 // Dense 64x64 red mask => all 32x32 blocks active. Count immutable-A reads,
 // excluding dispatch/init/occupancy traffic. Not a driver/GPU timing claim.
 auto run=[](bool compress,bool reverse){
  constexpr unsigned side=32,n=side*side;std::array<unsigned,n> a{},b{};
  for(unsigned j=0;j<n;j++)a[j]=b[j]=j+1;
  uint64_t reads=0;
  for(unsigned round=0;round<32;round++){
   for(unsigned item=0;item<n;item++){
    unsigned j=reverse?n-item-1:item;if(compress)a[j]=std::min(a[j],b[j]);
    auto edge=[&](unsigned k){unsigned hi=std::max(b[j],b[k]),lo=std::min(b[j],b[k]);a[hi-1]=std::min(a[hi-1],lo);};
    if(j%side)edge(j-1);if(j>=side)edge(j-side);if(j>=side&&j%side)edge(j-side-1);if(j>=side&&j%side+1<side)edge(j-side+1);
   }
   for(unsigned j=0;j<n;j++){unsigned root=a[j];++reads;for(;;){unsigned next=a[root-1];++reads;if(next==root)break;assert(next<root);root=next;}b[j]=root;}
  }
  for(auto root:b)assert(root==1);return reads;
 };
 auto baseline=run(false,false),fused=run(true,false),reverse=run(true,true);
 assert(fused<baseline&&reverse<baseline);
 // One legal adversarial interleaving: newer edge hook wins before stale
 // snapshot compression arrives. Plain Store loses it, atomic min preserves it.
 unsigned snapshot=3,forest=1;unsigned wrong_store=snapshot,atomic_min=std::min(forest,snapshot);assert(wrong_store!=forest&&atomic_min==forest);
 std::cout<<"{\"baseline_A_reads\":"<<baseline<<",\"fused_A_reads\":"<<fused<<",\"reverse_fused_A_reads\":"<<reverse<<",\"plain_store_mutant_rejected\":true}\n";return 0;
}
#ifdef PARENT_GRAPH_ONLY
int main(int argc,char** argv){
 if(argc==2&&std::string(argv[1])=="--walks")return walk_proof();
 if(argc==2&&std::string(argv[1])=="--graph-reverse")return graph(true);
 if(argc==2&&std::string(argv[1])=="--edges"){
  for(unsigned direction=0;direction<4;direction++)for(unsigned a=0;a<16;a++)for(unsigned b=0;b<16;b++)std::cout<<parent_block_edge(a,b,direction);std::cout<<'\n';return 0;
 }
 return graph();
}
#else
// Actual module host paths with an explicitly synthetic Vulkan driver. No loader,
// instance, physical device, queue or GPU work is used. Driver timings are excluded.
#include "so101_station_vulkan/parents.hpp"
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
bool hold=false;bool complete[4096]{};
template<class T>T handle(){return (T)(uintptr_t(next_handle++));}
void require(bool ok){if(!ok)throw std::runtime_error("preparation host contract failed");}
}
void* operator new(size_t n){if(auto p=std::malloc(n?n:1)){++heap_allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
namespace so101_station_vulkan {
Context::Context(){properties.limits.maxPushConstantsSize=128;properties.limits.maxComputeWorkGroupCount[0]=65535;properties.limits.maxComputeWorkGroupInvocations=256;properties.limits.maxComputeWorkGroupSize[0]=256;properties.limits.maxComputeSharedMemorySize=32768;properties.limits.maxStorageBufferRange=128*1024*1024;enabled_features.features.shaderFloat64=VK_FALSE;enabled_features.features.shaderInt64=VK_FALSE;}
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
void vkUpdateDescriptorSets(VkDevice,uint32_t n,const VkWriteDescriptorSet* writes,uint32_t,const VkCopyDescriptorSet*){for(unsigned i=0;i<n;i++)if(writes[i].dstBinding==2)compact_buffer=writes[i].pBufferInfo->buffer;}
VkResult vkResetCommandBuffer(VkCommandBuffer,VkCommandBufferResetFlags){return VK_SUCCESS;}
VkResult vkBeginCommandBuffer(VkCommandBuffer,const VkCommandBufferBeginInfo*){return VK_SUCCESS;}
VkResult vkEndCommandBuffer(VkCommandBuffer){return VK_SUCCESS;}
void vkCmdCopyBuffer(VkCommandBuffer,VkBuffer from,VkBuffer to,uint32_t n,const VkBufferCopy* copies){for(unsigned i=0;i<n;i++)std::memcpy(static_cast<char*>(fake_memory[uintptr_t(to)].data)+copies[i].dstOffset,static_cast<char*>(fake_memory[uintptr_t(from)].data)+copies[i].srcOffset,copies[i].size);last_readback=to;}
void vkCmdFillBuffer(VkCommandBuffer,VkBuffer b,VkDeviceSize offset,VkDeviceSize size,uint32_t){std::memset(static_cast<char*>(fake_memory[uintptr_t(b)].data)+offset,0,size);}
void vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags,VkPipelineStageFlags,VkDependencyFlags,uint32_t,const VkMemoryBarrier*,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){}
void vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t,const VkDescriptorSet*,uint32_t,const uint32_t*){}
void vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t,uint32_t n,const void* data){require(n==sizeof(parameters));std::memcpy(parameters,data,n);}
void vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline){}
void vkCmdDispatch(VkCommandBuffer,uint32_t,uint32_t,uint32_t){
 ++dispatches;auto* words=static_cast<uint32_t*>(fake_memory[uintptr_t(compact_buffer)].data);
 // Synthetic EMPTY shader result exercises host admission/lifetime only.
 words[0]=1;words[1]=parameters[10];words[2]=parameters[11];words[3]=parameters[12];words[4]=parameters[13];words[5]=parameters[0];words[6]=parameters[1];
}
VkResult vkQueueSubmit(VkQueue,uint32_t,const VkSubmitInfo*,VkFence f){++submits;if(submit_result!=VK_SUCCESS)return submit_result;complete[uintptr_t(f)]=!hold;return VK_SUCCESS;}
}

template<class E=std::exception,class F>void rejects(F fn){bool rejected=false;try{fn();}catch(const E&){rejected=true;}require(rejected);}
int main(int argc,char** argv){
 if(argc==2&&std::string(argv[1])=="--graph")return graph();
 require(argc==2);ParentConfig config;config.width=7;config.height=5;config.min_parent_pixels=1;config.generation=0x100000002ull;config.calibration_generation=0x300000004ull;config.validate();
 require(parent_arena_bytes(1,1)==196);require(parent_arena_bytes(640,480)==13526400);require(parent_compact_bytes(4096)==131136);
 rejects<std::invalid_argument>([&]{parent_arena_bytes(641,1);});rejects<std::invalid_argument>([&]{parent_arena_bytes(0,1);});rejects<std::invalid_argument>([&]{parent_compact_bytes(4097);});
 auto bad=config;bad.generation=0;rejects<std::invalid_argument>([&]{bad.validate();});bad=config;bad.calibration_generation=0;rejects<std::invalid_argument>([&]{bad.validate();});bad=config;bad.min_parent_pixels=0;rejects<std::invalid_argument>([&]{bad.validate();});bad=config;bad.record_capacity=0;rejects<std::invalid_argument>([&]{bad.validate();});
 ParentLease empty;rejects<std::logic_error>([&]{empty.resident_buffer();});rejects<std::logic_error>([&]{empty.records();});rejects<std::logic_error>([&]{empty.metadata();});
 auto context=std::make_shared<Context>();auto pool=std::make_shared<FramePool>(7,5,context,false);auto token=pool->source_token();
 auto source=pool->acquire();auto& slot=source.slot();slot.submitted=true;slot.metadata.calibration_generation=config.calibration_generation;slot.calibration.generation=config.calibration_generation;complete[uintptr_t(slot.fence)]=true;
 require(source.belongs_to(token)&&!FrameLease{}.belongs_to(token));
 // Token stores weak owner identity and consumes none of three source slots.
 {auto b=pool->acquire(),c=pool->acquire();require(b&&c&&!pool->acquire());}
 ParentPreparation parents(context,token,config,argv[1]);
 rejects<std::invalid_argument>([&]{parents.ready(empty);});rejects<std::invalid_argument>([&]{parents.finish(empty);});
 auto otherpool=std::make_shared<FramePool>(7,5,context,false);auto foreign=otherpool->acquire();require(foreign.index()==source.index()&&!foreign.belongs_to(token));rejects<std::invalid_argument>([&]{parents.submit(foreign);});
 {auto othercontext=std::make_shared<Context>();auto op=std::make_shared<FramePool>(7,5,othercontext,false);auto f=op->acquire();rejects<std::invalid_argument>([&]{parents.submit(f);});}
 rejects<std::invalid_argument>([&]{ParentPreparation unavailable(context,{},config,argv[1]);});
 auto initial=slot.references.load();slot.calibration.generation++;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.calibration.generation--;
 slot.metadata.calibration_generation++;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.metadata.calibration_generation--;
 slot.metadata.width++;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.metadata.width--;
 slot.fault=1;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.fault=0;
 auto source_bytes=slot.scratch.size;slot.scratch.size=4*35-1;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.scratch.size=source_bytes;
 auto* source_owner=slot.scratch.owner;slot.scratch.owner=nullptr;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.scratch.owner=source_owner;
 slot.submitted=false;rejects<std::invalid_argument>([&]{parents.submit(source);});slot.submitted=true;
 auto* core=static_cast<uint32_t*>(slot.readback.mapped);core[1]=1;rejects<std::runtime_error>([&]{parents.submit(source);});core[1]=0;core[0]=4097;rejects<std::runtime_error>([&]{parents.submit(source);});core[0]=0;
 complete[uintptr_t(slot.fence)]=false;rejects<std::logic_error>([&]{parents.submit(source);});complete[uintptr_t(slot.fence)]=true;
 submit_result=VK_ERROR_OUT_OF_HOST_MEMORY;rejects<std::runtime_error>([&]{parents.submit(source);});require(slot.references==initial);submit_result=VK_SUCCESS;
 auto pools=live_command_pools;fence_result=VK_ERROR_OUT_OF_HOST_MEMORY;rejects<std::runtime_error>([&]{ParentPreparation broken(context,token,config,argv[1]);});require(live_command_pools==pools);fence_result=VK_SUCCESS;
 slot.metadata.frame_number=917;slot.metadata.depth_frame_number=916;slot.metadata.serial="fake-source";
 hold=true;{auto lease=parents.submit(source);auto copy=lease;auto moved=std::move(copy);require(!copy&&moved&&slot.references==initial+1);require(!parents.ready(moved));require(moved.metadata().frame_number==917);rejects<std::logic_error>([&]{parents.finish(moved);});rejects<std::logic_error>([&]{moved.resident_buffer();});rejects<std::logic_error>([&]{parents.submit(source);});}
 require(slot.references==initial+1);parents.collect();require(slot.references==initial+1);std::fill(std::begin(complete),std::end(complete),true);parents.collect();require(slot.references==initial);hold=false;
 {auto lease=parents.submit(source);auto downstream=lease;lease={};require(slot.references==initial+1);auto counts=parents.finish(downstream);require(counts.configuration_generation==config.generation&&counts.calibration_generation==config.calibration_generation&&counts.record_count==0);require(downstream.resident_buffer()&&downstream.resident_records_buffer()&&downstream.records());require(downstream.original_mask_buffer()==slot.scratch.buffer);require(fake_memory[uintptr_t(downstream.resident_buffer())].usage==(VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT));require(downstream.label_offset_bytes()==1920);rejects<std::logic_error>([&]{parents.submit(source);});
 }parents.collect();require(slot.references==initial);
 auto result_fault=[&](auto mutate){auto lease=parents.submit(source);auto* words=static_cast<uint32_t*>(fake_memory[uintptr_t(last_readback)].data);mutate(words);rejects<std::runtime_error>([&]{parents.finish(lease);});rejects<std::runtime_error>([&]{lease.resident_buffer();});};
 result_fault([](auto* w){w[8]=2;});result_fault([](auto* w){w[8]=4;});result_fault([](auto* w){w[9]=13;});result_fault([](auto* w){w[2]++;});result_fault([&](auto* w){w[7]=config.record_capacity+1;});
 // Inject immutable synthetic completion BEFORE the first decode. Retained
 // label 2 remains 2; subsequent lease reads expose an immutable CPU array.
 auto record_result=[](auto* words){words[7]=1;words[9]=2;auto* record=reinterpret_cast<ParentRecord*>(words+16);record[0]={0,2,0,30,0,0,6,5};};
 {auto lease=parents.submit(source);auto* words=static_cast<uint32_t*>(fake_memory[uintptr_t(last_readback)].data);record_result(words);require(parents.finish(lease).record_count==1);auto* records=lease.records();require(records[0].label_id==2&&lease.records()==records);}
 result_fault([&](auto* w){record_result(w);reinterpret_cast<ParentRecord*>(w+16)[0].label_id=3;});
 result_fault([&](auto* w){record_result(w);reinterpret_cast<ParentRecord*>(w+16)[0].width=8;});
 result_fault([&](auto* w){record_result(w);w[7]=2;auto* r=reinterpret_cast<ParentRecord*>(w+16);r[1]=r[0];});
 parents.collect();require(slot.references==initial);
 // All source-index arenas enforce backpressure and release dropped work only
 // on fence completion. A retained downstream copy keeps the source alive.
 {auto b=pool->acquire(),c=pool->acquire();for(auto* f:{&b,&c}){f->slot().submitted=true;f->slot().metadata.calibration_generation=config.calibration_generation;f->slot().calibration.generation=config.calibration_generation;complete[uintptr_t(f->slot().fence)]=true;}
 hold=true;{auto a=parents.submit(source),bb=parents.submit(b),cc=parents.submit(c);require(a.index()!=bb.index()&&bb.index()!=cc.index());}b={};c={};require(!pool->acquire());parents.collect();require(!pool->acquire());std::fill(std::begin(complete),std::end(complete),true);parents.collect();require(bool(pool->acquire()));hold=false;
 }
 // Positive thresholds above N are valid retained-CPU semantics: all
 // parents are still counted/ranked, only compact records are filtered out.
 {auto small_pool=std::make_shared<FramePool>(2,2,context,false);auto small_source=small_pool->acquire();auto& small_slot=small_source.slot();small_slot.submitted=true;small_slot.metadata.calibration_generation=config.calibration_generation;small_slot.calibration.generation=config.calibration_generation;complete[uintptr_t(small_slot.fence)]=true;
  for(unsigned minimum:{30u,0xffffffffu}){ParentConfig small=config;small.width=small.height=2;small.min_parent_pixels=minimum;small.validate();ParentPreparation filtered(context,small_pool->source_token(),small,argv[1]);auto lease=filtered.submit(small_source);auto* words=static_cast<uint32_t*>(fake_memory[uintptr_t(last_readback)].data);words[9]=1;auto result=filtered.finish(lease);require(result.all_parents[0]==1&&result.record_count==0&&lease.resident_buffer());}
 }
 // Normal host paths allocate/map/wait zero times after cold construction.
 auto heaps=heap_allocations,buffers=buffer_allocations,waits=wait_calls;auto dispatch_before=dispatches;
 for(unsigned i=0;i<100;i++){auto lease=parents.submit(source);require(parents.ready(lease));require(parents.finish(lease).record_count==0);parents.collect();}
 require(heap_allocations==heaps&&buffer_allocations==buffers&&wait_calls==waits&&dispatches-dispatch_before==7300);
 FramePoolToken expired;{auto p=std::make_shared<FramePool>(1,1,context,false);expired=p->source_token();require(bool(expired));}require(!expired&&!source.belongs_to(expired));
 std::cout<<"parent fake-Vulkan host contracts: PASS; hot heap/buffer/wait=0; dispatches/frame=73; no Vulkan loader/device\n";
}

#endif
