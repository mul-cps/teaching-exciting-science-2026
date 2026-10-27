#include "so101_station_vulkan/parents.hpp"
#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>
namespace so101_station_vulkan {
namespace {
void checked(VkResult r,const char* operation){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(operation)+": "+std::to_string(r));}
struct Parameters {uint32_t width,height,count,blockWidth,blocks,nodes,scanGroups,minArea,capacity,phase,configLo,configHi,calibrationLo,calibrationHi;};
void barrier(VkCommandBuffer command,VkPipelineStageFlags from,VkPipelineStageFlags to,VkAccessFlags src,VkAccessFlags dst){VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=src;b.dstAccessMask=dst;vkCmdPipelineBarrier(command,from,to,0,1,&b,0,nullptr,0,nullptr);}
constexpr unsigned header_bytes=64;
}
size_t parent_arena_bytes(unsigned w,unsigned h){return ParentGraphLayout(w,h).arena_bytes();}
size_t parent_compact_bytes(unsigned capacity){if(!capacity||capacity>kMaxComponents)throw std::invalid_argument("parent record capacity must be 1..4096");return header_bytes+size_t(capacity)*sizeof(ParentRecord);}
void ParentConfig::validate()const{
 ParentGraphLayout{width,height};parent_compact_bytes(record_capacity);
 if(!generation||!calibration_generation)throw std::invalid_argument("parent configuration and calibration generations must be nonzero");
 if(!min_parent_pixels)throw std::invalid_argument("parent minimum area must be positive");
}
static_assert(sizeof(ParentRecord)==32,"parent compact ABI");
struct ParentStorage {
 struct Entry {
  Buffer arena,compact,readback;FrameLease source;std::array<ParentRecord,kMaxComponents> records{};
  std::atomic<unsigned> references{0};bool submitted=false,decoded=false;ParentCounts completed_counts;
  VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;VkDescriptorSet descriptors=VK_NULL_HANDLE;
 };
 std::shared_ptr<Context> context;FramePoolToken source_token;ParentConfig config;
 Parameters p{};std::array<Entry,kSlots> entries;std::mutex mutex;
 VkCommandPool commands=VK_NULL_HANDLE;VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;VkDescriptorPool descriptor_pool=VK_NULL_HANDLE;
 VkPipelineLayout layout=VK_NULL_HANDLE;std::array<VkPipeline,9> kernels{};
 ~ParentStorage(){
  if(!context)return;
  auto& c=*context;
  for(auto& e:entries){
   // Cold shutdown only. Retain each source through its own parent fence.
   if(e.submitted)vkWaitForFences(c.device,1,&e.fence,VK_TRUE,UINT64_MAX);
   if(e.fence)vkDestroyFence(c.device,e.fence,nullptr);
  }
  if(commands)vkDestroyCommandPool(c.device,commands,nullptr);
  for(auto k:kernels)if(k)vkDestroyPipeline(c.device,k,nullptr);
  if(layout)vkDestroyPipelineLayout(c.device,layout,nullptr);
  if(descriptor_pool)vkDestroyDescriptorPool(c.device,descriptor_pool,nullptr);
  if(descriptors)vkDestroyDescriptorSetLayout(c.device,descriptors,nullptr);
 }
 bool ready(unsigned i)const{const auto& e=entries[i];if(!e.submitted)return false;auto r=vkGetFenceStatus(context->device,e.fence);if(r==VK_NOT_READY)return false;checked(r,"parent fence status");return true;}
 void collect_locked(){for(unsigned i=0;i<kSlots;i++){auto& e=entries[i];if(!e.references.load()&&e.submitted&&ready(i)){e.source={};e.submitted=false;}}}
 void release(unsigned i)noexcept{if(--entries[i].references)return;try{std::lock_guard<std::mutex> lock(mutex);collect_locked();}catch(...){} }
 ParentCounts result(unsigned i){
  // Decode once under the storage lock. Later lease copies can borrow the
  // immutable preallocated CPU records concurrently without another writer.
  std::lock_guard<std::mutex> lock(mutex);auto& e=entries[i];
  if(e.decoded)return e.completed_counts;
  if(!ready(i))throw std::logic_error("parent preparation still pending");
  const auto* words=static_cast<const uint32_t*>(e.readback.mapped);
  if(words[0]!=kParentLayoutVersion||words[1]!=uint32_t(config.generation)||words[2]!=uint32_t(config.generation>>32)||words[3]!=uint32_t(config.calibration_generation)||words[4]!=uint32_t(config.calibration_generation>>32)||words[5]!=p.width||words[6]!=p.height||words[7]>p.capacity||words[8]||words[13]||words[14]||words[15])throw std::runtime_error("parent incomplete, overflow, nonconvergence or result fault: frame unknown");
  ParentCounts counts;counts.record_count=words[7];counts.calibration_generation=config.calibration_generation;counts.configuration_generation=config.generation;
  unsigned total=0;for(unsigned c=0;c<4;c++){counts.all_parents[c]=words[9+c];if(words[9+c]>p.blocks)throw std::runtime_error("parent count exceeds full graph");total+=words[9+c];}
  if(counts.record_count>total)throw std::runtime_error("parent record count exceeds all parents");
  std::memcpy(e.records.data(),words+16,size_t(counts.record_count)*sizeof(ParentRecord));
  for(unsigned j=0;j<counts.record_count;j++){
   const auto& r=e.records[j];
   if(r.color_ordinal>=4||!r.label_id||r.label_id>counts.all_parents[r.color_ordinal]||r.min_block_key>=p.blocks||r.parent_pixels<config.min_parent_pixels||r.parent_pixels>p.count||!r.width||!r.height||r.x>=p.width||r.y>=p.height||r.width>p.width-r.x||r.height>p.height-r.y||r.parent_pixels>r.width*r.height)throw std::runtime_error("parent compact record fault");
   if(j){const auto& previous=e.records[j-1];if(previous.color_ordinal>r.color_ordinal||(previous.color_ordinal==r.color_ordinal&&previous.label_id>=r.label_id))throw std::runtime_error("parent compact order fault");}
  }
  e.completed_counts=counts;e.decoded=true;return counts;
 }
};
ParentLease::ParentLease(std::shared_ptr<ParentStorage> s,unsigned i):storage_(std::move(s)),index_(i){++storage_->entries[i].references;}
ParentLease::ParentLease(const ParentLease& other):storage_(other.storage_),index_(other.index_){if(storage_)++storage_->entries[index_].references;}
ParentLease::ParentLease(ParentLease&& other)noexcept:storage_(std::move(other.storage_)),index_(other.index_){}
ParentLease& ParentLease::operator=(ParentLease other){swap(other);return *this;}
ParentLease::~ParentLease(){if(storage_)storage_->release(index_);}
void ParentLease::swap(ParentLease& other)noexcept{storage_.swap(other.storage_);std::swap(index_,other.index_);}
const FrameMetadata& ParentLease::metadata()const{if(!storage_)throw std::logic_error("empty parent lease");return storage_->entries[index_].source.slot().metadata;}
VkBuffer ParentLease::resident_buffer()const{if(!storage_)throw std::logic_error("empty parent lease");storage_->result(index_);return storage_->entries[index_].arena.buffer;}
VkBuffer ParentLease::original_mask_buffer()const{if(!storage_)throw std::logic_error("empty parent lease");storage_->result(index_);return storage_->entries[index_].source.slot().scratch.buffer;}
VkBuffer ParentLease::resident_records_buffer()const{if(!storage_)throw std::logic_error("empty parent lease");storage_->result(index_);return storage_->entries[index_].compact.buffer;}
size_t ParentLease::label_offset_bytes()const{if(!storage_)throw std::logic_error("empty parent lease");return size_t(10)*storage_->p.nodes*4;}
const ParentRecord* ParentLease::records()const{if(!storage_)throw std::logic_error("empty parent lease");storage_->result(index_);return storage_->entries[index_].records.data();}
FrameLease ParentLease::source_frame()const{if(!storage_)throw std::logic_error("empty parent lease");storage_->result(index_);return storage_->entries[index_].source;}
ParentConfig ParentLease::configuration()const{if(!storage_)throw std::logic_error("empty parent lease");storage_->result(index_);return storage_->config;}
ParentPreparation::ParentPreparation(std::shared_ptr<Context> context,FramePoolToken token,const ParentConfig& config,const std::string& directory){
 config.validate();if(!context||!token)throw std::invalid_argument("parents require resident Context and live producer token");
 ParentGraphLayout graph(config.width,config.height);auto& c=*context;
 if(c.properties.limits.maxComputeWorkGroupInvocations<256||c.properties.limits.maxComputeWorkGroupSize[0]<256||c.properties.limits.maxComputeSharedMemorySize<2048||c.properties.limits.maxComputeWorkGroupCount[0]<std::max((graph.nodes+255)/256,4*graph.scan_groups)||c.properties.limits.maxPushConstantsSize<sizeof(Parameters)||parent_arena_bytes(config.width,config.height)>c.properties.limits.maxStorageBufferRange||parent_compact_bytes(config.record_capacity)>c.properties.limits.maxStorageBufferRange)throw std::runtime_error("parents require bounded U32 storage and 256-lane compute");
 auto s=std::make_shared<ParentStorage>();s->context=std::move(context);s->source_token=std::move(token);s->config=config;
 s->p={graph.width,graph.height,graph.count,graph.block_width,graph.blocks,graph.nodes,graph.scan_groups,config.min_parent_pixels,config.record_capacity,0,uint32_t(config.generation),uint32_t(config.generation>>32),uint32_t(config.calibration_generation),uint32_t(config.calibration_generation>>32)};
 std::array<VkDescriptorSetLayoutBinding,3> bindings{};for(unsigned i=0;i<3;i++){bindings[i].binding=i;bindings[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;bindings[i].descriptorCount=1;bindings[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}
 VkDescriptorSetLayoutCreateInfo di{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};di.bindingCount=3;di.pBindings=bindings.data();checked(vkCreateDescriptorSetLayout(c.device,&di,nullptr,&s->descriptors),"parent descriptor layout");
 VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(Parameters)};VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};li.setLayoutCount=1;li.pSetLayouts=&s->descriptors;li.pushConstantRangeCount=1;li.pPushConstantRanges=&push;checked(vkCreatePipelineLayout(c.device,&li,nullptr,&s->layout),"parent pipeline layout");
 const char* names[]={"parents_init","parents_hook","parents_flatten","parents_validate","parents_scan","parents_offsets","parents_pixels","parents_labels","parents_records"};
 for(unsigned i=0;i<9;i++){
  std::ifstream file(directory+"/"+names[i]+".spv",std::ios::binary|std::ios::ate);if(!file)throw std::runtime_error(std::string("missing offline parent shader: ")+names[i]);
  auto bytes=file.tellg();if(bytes<=0||size_t(bytes)%4)throw std::runtime_error("invalid parent SPIR-V length");
  std::vector<uint32_t> code(size_t(bytes)/4);file.seekg(0);file.read(reinterpret_cast<char*>(code.data()),bytes);if(!file)throw std::runtime_error("incomplete parent SPIR-V");
  VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};mi.codeSize=size_t(bytes);mi.pCode=code.data();VkShaderModule module=VK_NULL_HANDLE;checked(vkCreateShaderModule(c.device,&mi,nullptr,&module),"parent shader module");
  VkComputePipelineCreateInfo pi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};pi.layout=s->layout;pi.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;pi.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pi.stage.module=module;pi.stage.pName="main";auto r=vkCreateComputePipelines(c.device,VK_NULL_HANDLE,1,&pi,nullptr,&s->kernels[i]);vkDestroyShaderModule(c.device,module,nullptr);checked(r,"parent compute pipeline");
 }
 VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,3*kSlots};VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};dpi.maxSets=kSlots;dpi.poolSizeCount=1;dpi.pPoolSizes=&ps;checked(vkCreateDescriptorPool(c.device,&dpi,nullptr,&s->descriptor_pool),"parent descriptor pool");
 VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};ci.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;ci.queueFamilyIndex=c.family;checked(vkCreateCommandPool(c.device,&ci,nullptr,&s->commands),"parent private command pool");
 for(auto& e:s->entries){
  auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  e.arena.create(c,graph.arena_bytes(),VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  e.compact.create(c,parent_compact_bytes(config.record_capacity),VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  e.readback.create(c,parent_compact_bytes(config.record_capacity),VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
  VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};fi.flags=VK_FENCE_CREATE_SIGNALED_BIT;checked(vkCreateFence(c.device,&fi,nullptr,&e.fence),"parent fence");
  VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};ai.commandPool=s->commands;ai.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;ai.commandBufferCount=1;checked(vkAllocateCommandBuffers(c.device,&ai,&e.command),"parent command");
  VkDescriptorSetAllocateInfo si{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};si.descriptorPool=s->descriptor_pool;si.descriptorSetCount=1;si.pSetLayouts=&s->descriptors;checked(vkAllocateDescriptorSets(c.device,&si,&e.descriptors),"parent descriptors");
 }
 storage_=std::move(s);
}
void ParentPreparation::collect(){std::lock_guard<std::mutex> lock(storage_->mutex);storage_->collect_locked();}
ParentLease ParentPreparation::submit(const FrameLease& frame){
 auto& s=*storage_;std::lock_guard<std::mutex> lock(s.mutex);s.collect_locked();
 if(!frame||frame.context()!=s.context||!frame.belongs_to(s.source_token))throw std::invalid_argument("parent source producer or Context mismatch");
 const auto& source=frame.slot();
 if(source.metadata.width!=s.p.width||source.metadata.height!=s.p.height||source.metadata.calibration_generation!=s.config.calibration_generation||source.calibration.generation!=s.config.calibration_generation)throw std::invalid_argument("parent source dimensions or calibration generation mismatch");
 if(!source.submitted||source.fault||!source.scratch.buffer||source.scratch.owner!=s.context.get()||source.scratch.size<size_t(s.p.count)*4)throw std::invalid_argument("parents require successful submitted resident source");
 auto r=vkGetFenceStatus(s.context->device,source.fence);if(r==VK_NOT_READY)throw std::logic_error("parent source is pending");checked(r,"parent source fence");
 if(!source.readback.mapped||source.readback.size<16)throw std::invalid_argument("parent source missing compact completion record");
 const auto* status=static_cast<const uint32_t*>(source.readback.mapped);if(status[0]>kMaxComponents||status[1])throw std::runtime_error("parent source Core fault: frame unknown");
 auto index=frame.index();if(index>=kSlots)throw std::invalid_argument("parent source index outside pool");auto& e=s.entries[index];if(e.submitted||e.references.load())throw std::logic_error("parent source index leased or pending");
 auto& c=*s.context;e.source=frame;e.decoded=false;struct Guard{ParentStorage::Entry& e;bool done=false;~Guard(){if(!done)e.source={};}}guard{e};
 std::array<VkDescriptorBufferInfo,3> buffers{{{source.scratch.buffer,0,size_t(s.p.count)*4},{e.arena.buffer,0,e.arena.size},{e.compact.buffer,0,e.compact.size}}};std::array<VkWriteDescriptorSet,3> writes{};
 for(unsigned i=0;i<3;i++){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=e.descriptors;writes[i].dstBinding=i;writes[i].descriptorCount=1;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;writes[i].pBufferInfo=&buffers[i];}vkUpdateDescriptorSets(c.device,3,writes.data(),0,nullptr);
 checked(vkResetCommandBuffer(e.command,0),"reset parent command");VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;checked(vkBeginCommandBuffer(e.command,&begin),"begin parent command");
 vkCmdFillBuffer(e.command,e.compact.buffer,0,header_bytes,0);
 barrier(e.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 vkCmdBindDescriptorSets(e.command,VK_PIPELINE_BIND_POINT_COMPUTE,s.layout,0,1,&e.descriptors,0,nullptr);auto p=s.p;
 auto dispatch=[&](unsigned kernel,unsigned groups){vkCmdBindPipeline(e.command,VK_PIPELINE_BIND_POINT_COMPUTE,s.kernels[kernel]);vkCmdDispatch(e.command,groups,1,1);barrier(e.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);};
 vkCmdPushConstants(e.command,s.layout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(p),&p);dispatch(0,(p.blocks+255)/256);
 for(unsigned round=0;round<32;round++){dispatch(1,(p.nodes+255)/256);dispatch(2,(p.nodes+255)/256);}
 dispatch(3,(p.nodes+255)/256);dispatch(4,4*p.scanGroups);dispatch(5,4);dispatch(6,(p.count+255)/256);dispatch(7,(p.nodes+255)/256);
 p.phase=1;vkCmdPushConstants(e.command,s.layout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(p),&p);dispatch(4,4*p.scanGroups);dispatch(5,4);dispatch(8,(p.nodes+255)/256);
 // Keep map/stats resident. Only the bounded compact header/records are copied.
 barrier(e.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);VkBufferCopy copy{0,0,e.compact.size};vkCmdCopyBuffer(e.command,e.compact.buffer,e.readback.buffer,1,&copy);
 barrier(e.command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);
 checked(vkEndCommandBuffer(e.command),"end parent command");checked(vkResetFences(c.device,1,&e.fence),"reset parent fence");VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};info.commandBufferCount=1;info.pCommandBuffers=&e.command;
 {std::lock_guard<std::mutex> queue_lock(c.queue_mutex);checked(vkQueueSubmit(c.queue,1,&info,e.fence),"submit parents");++c.submission_count;}
 e.submitted=true;guard.done=true;return ParentLease(storage_,index);
}
bool ParentPreparation::ready(const ParentLease& lease)const{if(lease.storage_!=storage_)throw std::invalid_argument("foreign or empty parent lease");return storage_->ready(lease.index_);}
ParentCounts ParentPreparation::finish(const ParentLease& lease){if(lease.storage_!=storage_)throw std::invalid_argument("foreign or empty parent lease");return storage_->result(lease.index_);}
}
