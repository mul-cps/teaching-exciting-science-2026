#include "so101_station_vulkan/observed_children.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>
namespace so101_station_vulkan {
namespace {
void checked(VkResult r,const char* op){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(op)+": "+std::to_string(r));}
void barrier(VkCommandBuffer cmd,VkPipelineStageFlags from,VkPipelineStageFlags to,VkAccessFlags src,VkAccessFlags dst){VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=src;b.dstAccessMask=dst;vkCmdPipelineBarrier(cmd,from,to,0,1,&b,0,nullptr,0,nullptr);}
bool parent_equal(const ParentConfig& a,const ParentConfig& b){return a.width==b.width&&a.height==b.height&&a.min_parent_pixels==b.min_parent_pixels&&a.record_capacity==b.record_capacity&&a.calibration_generation==b.calibration_generation&&a.generation==b.generation;}
bool child_equal(const ChildConfig& a,const ChildConfig& b){return a.generation==b.generation&&a.max_observation_depth_m==b.max_observation_depth_m&&a.max_neighbor_jump_m==b.max_neighbor_jump_m&&a.min_child_pixels==b.min_child_pixels&&a.record_capacity==b.record_capacity&&a.packed_pixel_capacity==b.packed_pixel_capacity;}
}
static_assert(sizeof(ObservedRecord)==80&&sizeof(observed_detail::Parameters)==64,"observed compact/push ABI");
void ObservedConfig::validate(unsigned count)const{
 if(!generation||closed_membership_capacity>4*uint64_t(count))throw std::invalid_argument("observed requires generation and bounded4N membership capacity");
 if(observation_roi)for(unsigned i=0;i<6;i+=2)if(!std::isfinite((*observation_roi)[i])||!std::isfinite((*observation_roi)[i+1])||(*observation_roi)[i]>(*observation_roi)[i+1])throw std::invalid_argument("observation ROI must have finite nondecreasing inclusive bounds");
}
namespace observed_detail {
const std::array<const char*,kernel_count> kernel_names{{"observed_clear","observed_records","observed_observe","observed_eligible","observed_close","observed_finalize"}};
void record_graph(VkCommandBuffer cmd,VkPipelineLayout layout,const std::array<VkPipeline,kernel_count>& kernels,VkDescriptorSet descriptors,const Parameters& p,VkBuffer compact,VkBuffer readback,size_t bytes){
 vkCmdFillBuffer(cmd,compact,0,bytes,0);
 barrier(cmd,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT|VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,layout,0,1,&descriptors,0,nullptr);vkCmdPushConstants(cmd,layout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(p),&p);
 const unsigned groups[]={ (p.count+255)/256,(p.capacity+255)/256,(p.count+255)/256,(p.capacity+255)/256,(p.count+255)/256,1};
 for(unsigned i=0;i<kernel_count;i++){vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,kernels[i]);vkCmdDispatch(cmd,groups[i],1,1);barrier(cmd,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,i+1==kernel_count?VK_PIPELINE_STAGE_TRANSFER_BIT:VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,i+1==kernel_count?VK_ACCESS_TRANSFER_READ_BIT:VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);}
 VkBufferCopy copy{0,0,bytes};vkCmdCopyBuffer(cmd,compact,readback,1,&copy);barrier(cmd,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);
}
}
struct ObservedStorage {
 struct Entry {Buffer arena,compact,readback;ChildLease child;GeometryLease geometry;std::atomic<unsigned> references{0};bool submitted=false;VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;VkDescriptorSet descriptors=VK_NULL_HANDLE;};
 std::shared_ptr<Context> context;FramePoolToken source_token;GeometryProducerToken geometry_token;ParentConfig parent;ChildConfig child;ObservedConfig config;GeometryConfiguration geometry_config;ObservedLayout graph;observed_detail::Parameters p{};Buffer roi;std::array<Entry,kSlots> entries;std::mutex mutex;
 VkCommandPool commands=VK_NULL_HANDLE;VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;VkDescriptorPool descriptor_pool=VK_NULL_HANDLE;VkPipelineLayout layout=VK_NULL_HANDLE;std::array<VkPipeline,observed_detail::kernel_count> kernels{};
 ObservedStorage(const ParentConfig& a,const ChildConfig& b,const ObservedConfig& c):parent(a),child(b),config(c),graph(a.width,a.height,b.record_capacity,a.record_capacity,c.closed_membership_capacity){}
 ~ObservedStorage(){if(!context)return;auto& c=*context;for(auto& e:entries){if(e.submitted)vkWaitForFences(c.device,1,&e.fence,VK_TRUE,UINT64_MAX);if(e.fence)vkDestroyFence(c.device,e.fence,nullptr);}if(commands)vkDestroyCommandPool(c.device,commands,nullptr);for(auto k:kernels)if(k)vkDestroyPipeline(c.device,k,nullptr);if(layout)vkDestroyPipelineLayout(c.device,layout,nullptr);if(descriptor_pool)vkDestroyDescriptorPool(c.device,descriptor_pool,nullptr);if(descriptors)vkDestroyDescriptorSetLayout(c.device,descriptors,nullptr);}
 bool ready(unsigned i)const{const auto& e=entries[i];if(!e.submitted)return false;auto r=vkGetFenceStatus(context->device,e.fence);if(r==VK_NOT_READY)return false;checked(r,"observed fence status");return true;}
 void collect_locked(){for(unsigned i=0;i<kSlots;i++){auto& e=entries[i];if(e.references.load()==0&&e.submitted&&ready(i)){e.child={};e.geometry={};e.submitted=false;}}}
 void release(unsigned i)noexcept{if(--entries[i].references)return;try{std::lock_guard<std::mutex> lock(mutex);collect_locked();}catch(...){}}
 ObservedCounts result(unsigned i)const{
  if(!ready(i))throw std::logic_error("observed preparation still pending");const auto* h=static_cast<const uint32_t*>(entries[i].readback.mapped);
  if(h[0]!=1||h[1]!=p.width||h[2]!=p.height||h[3]>p.capacity||h[4]>h[3]||h[5]>p.count||h[6]>p.closedCapacity||h[7]||h[8]!=p.calLo||h[9]!=p.calHi||h[10]!=p.childLo||h[11]!=p.childHi||h[12]!=p.observedLo||h[13]!=p.observedHi||h[14]!=p.closedCapacity||h[15]!=p.capacity)throw std::runtime_error("observed identity, capacity or result fault: frame unknown");
  return {h[3],h[4],h[5],h[6],parent.calibration_generation,child.generation,config.generation};
 }
};
ObservedLease::ObservedLease(std::shared_ptr<ObservedStorage> s,unsigned i):storage_(std::move(s)),index_(i){++storage_->entries[i].references;}
ObservedLease::ObservedLease(const ObservedLease& b):storage_(b.storage_),index_(b.index_){if(storage_)++storage_->entries[index_].references;}
ObservedLease::ObservedLease(ObservedLease&& b)noexcept:storage_(std::move(b.storage_)),index_(b.index_){}
ObservedLease& ObservedLease::operator=(ObservedLease b){swap(b);return *this;}
ObservedLease::~ObservedLease(){if(storage_)storage_->release(index_);}
void ObservedLease::swap(ObservedLease& b)noexcept{storage_.swap(b.storage_);std::swap(index_,b.index_);}
VkBuffer ObservedLease::resident_buffer()const{if(!storage_)throw std::logic_error("empty observed lease");storage_->result(index_);return storage_->entries[index_].arena.buffer;}
const ObservedRecord* ObservedLease::records()const{if(!storage_)throw std::logic_error("empty observed lease");storage_->result(index_);return reinterpret_cast<const ObservedRecord*>(static_cast<const uint32_t*>(storage_->entries[index_].readback.mapped)+16);}
ChildLease ObservedLease::children()const{if(!storage_)throw std::logic_error("empty observed lease");storage_->result(index_);return storage_->entries[index_].child;}
GeometryLease ObservedLease::geometry()const{if(!storage_)throw std::logic_error("empty observed lease");storage_->result(index_);return storage_->entries[index_].geometry;}
ObservedPreparation::ObservedPreparation(std::shared_ptr<Context> context,FramePoolToken source,GeometryProducerToken geometry,const ParentConfig& parent,const ChildConfig& child,const ObservedConfig& config,const std::string& directory){
 parent.validate();child.validate(parent.width,parent.height,parent.record_capacity);ObservedLayout g(parent.width,parent.height,child.record_capacity,parent.record_capacity,config.closed_membership_capacity);config.validate(g.count);
 if(!context||!source||!geometry)throw std::invalid_argument("observed requires live source/Geometry identity and shared Context");
 const auto gc=geometry.configuration();if(gc.context!=context||gc.width!=g.width||gc.height!=g.height||gc.generation!=parent.calibration_generation||gc.layout_version!=1||gc.max_observation_depth_m!=child.max_observation_depth_m)throw std::invalid_argument("observed Geometry configuration/Context/depth domain mismatch");
 if(gc.precision==GeometryPrecision::NativeFloat64){if(!context->enabled_features.features.shaderFloat64)throw std::runtime_error("observed requires enabled native shaderFloat64");if(!context->enabled_features.features.shaderInt64)throw std::runtime_error("observed native byte-address Float64 requires enabled shaderInt64");}
 else if(gc.precision==GeometryPrecision::SoftwareBinary64){if(!context->enabled_features.features.shaderInt64)throw std::runtime_error("observed requires enabled software-binary64 shaderInt64");}
 else throw std::invalid_argument("observed unknown Geometry precision");
 const auto& limits=context->properties.limits;
 if(limits.maxComputeWorkGroupInvocations<256||limits.maxComputeWorkGroupSize[0]<256||limits.maxComputeWorkGroupCount[0]<std::max((g.count+255)/256,(g.capacity+255)/256)||limits.maxPushConstantsSize<sizeof(observed_detail::Parameters)||limits.maxPerStageDescriptorStorageBuffers<7||limits.maxDescriptorSetStorageBuffers<7)throw std::runtime_error("observed requires bounded256lane/7storage/64byte push compute");
 for(size_t bytes:{g.arena_bytes,g.compact_bytes,size_t(8)*g.count,size_t(128)+16*g.parent_capacity+64*g.capacity,size_t(64)+32*g.parent_capacity,gc.arena_bytes,size_t(48)})if(bytes>limits.maxStorageBufferRange)throw std::runtime_error("observed bounded storage range unsupported");
 auto s=std::make_shared<ObservedStorage>(parent,child,config);s->context=std::move(context);s->source_token=std::move(source);s->geometry_token=std::move(geometry);s->geometry_config=gc;auto& c=*s->context;
 s->p={g.width,g.height,g.count,g.capacity,g.parent_capacity,g.closed_capacity,uint32_t(bool(config.observation_roi)),uint32_t(parent.calibration_generation),uint32_t(parent.calibration_generation>>32),uint32_t(parent.generation),uint32_t(parent.generation>>32),uint32_t(child.generation),uint32_t(child.generation>>32),uint32_t(config.generation),uint32_t(config.generation>>32),child.min_child_pixels};
 auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;s->roi.create(c,48,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,host);auto* values=static_cast<double*>(s->roi.mapped);for(unsigned i=0;i<6;i++)values[i]=config.observation_roi?(*config.observation_roi)[i]:0;
 std::array<VkDescriptorSetLayoutBinding,7> bindings{};for(unsigned i=0;i<7;i++){bindings[i].binding=i;bindings[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;bindings[i].descriptorCount=1;bindings[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}VkDescriptorSetLayoutCreateInfo di{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};di.bindingCount=7;di.pBindings=bindings.data();checked(vkCreateDescriptorSetLayout(c.device,&di,nullptr,&s->descriptors),"observed descriptor layout");
 VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(s->p)};VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};li.setLayoutCount=1;li.pSetLayouts=&s->descriptors;li.pushConstantRangeCount=1;li.pPushConstantRanges=&push;checked(vkCreatePipelineLayout(c.device,&li,nullptr,&s->layout),"observed pipeline layout");
 for(unsigned i=0;i<observed_detail::kernel_count;i++){
  std::ifstream file(directory+"/"+observed_detail::kernel_names[i]+".spv",std::ios::binary|std::ios::ate);if(!file)throw std::runtime_error("missing offline observed shader");auto bytes=file.tellg();if(bytes<=0||size_t(bytes)%4)throw std::runtime_error("invalid observed SPIR-V length");std::vector<uint32_t> code(size_t(bytes)/4);file.seekg(0);file.read(reinterpret_cast<char*>(code.data()),bytes);if(!file)throw std::runtime_error("incomplete observed SPIR-V");VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};mi.codeSize=size_t(bytes);mi.pCode=code.data();VkShaderModule module=VK_NULL_HANDLE;checked(vkCreateShaderModule(c.device,&mi,nullptr,&module),"observed shader module");VkComputePipelineCreateInfo pi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};pi.layout=s->layout;pi.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;pi.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pi.stage.module=module;pi.stage.pName="main";auto r=vkCreateComputePipelines(c.device,VK_NULL_HANDLE,1,&pi,nullptr,&s->kernels[i]);vkDestroyShaderModule(c.device,module,nullptr);checked(r,"observed pipeline");
 }
 VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,7*kSlots};VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};dpi.maxSets=kSlots;dpi.poolSizeCount=1;dpi.pPoolSizes=&ps;checked(vkCreateDescriptorPool(c.device,&dpi,nullptr,&s->descriptor_pool),"observed descriptor pool");VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};ci.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;ci.queueFamilyIndex=c.family;checked(vkCreateCommandPool(c.device,&ci,nullptr,&s->commands),"observed private command pool");
 for(auto& e:s->entries){e.arena.create(c,g.arena_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.compact.create(c,g.compact_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.readback.create(c,g.compact_bytes,VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};fi.flags=VK_FENCE_CREATE_SIGNALED_BIT;checked(vkCreateFence(c.device,&fi,nullptr,&e.fence),"observed fence");VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};ai.commandPool=s->commands;ai.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;ai.commandBufferCount=1;checked(vkAllocateCommandBuffers(c.device,&ai,&e.command),"observed command");VkDescriptorSetAllocateInfo si{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};si.descriptorPool=s->descriptor_pool;si.descriptorSetCount=1;si.pSetLayouts=&s->descriptors;checked(vkAllocateDescriptorSets(c.device,&si,&e.descriptors),"observed descriptors");}
 storage_=std::move(s);
}
void ObservedPreparation::collect(){std::lock_guard<std::mutex> lock(storage_->mutex);storage_->collect_locked();}
ObservedLease ObservedPreparation::submit(const ChildLease& child,const GeometryLease& geometry){
 auto& s=*storage_;std::lock_guard<std::mutex> lock(s.mutex);s.collect_locked();
 if(!child||!geometry||!geometry.belongs_to(s.geometry_token))throw std::invalid_argument("observed empty/foreign Geometry producer epoch");
 const auto input=child.resident_view();const auto view=geometry.resident_view();const auto source=geometry.source_frame();
 if(!parent_equal(input.parent_config,s.parent)||!child_equal(input.configuration,s.child)||input.source.context()!=s.context||source.context()!=s.context||!source.belongs_to(s.source_token)||!input.source.belongs_to(s.source_token)||source.index()!=input.source.index()||geometry.index()!=source.index()||child.index()!=source.index()||&source.slot()!=&input.source.slot())throw std::invalid_argument("observed mismatched child/Geometry source/configuration/session");
 const auto& m=source.slot().metadata;if(m.width!=s.p.width||m.height!=s.p.height||m.calibration_generation!=s.parent.calibration_generation||source.slot().calibration.generation!=m.calibration_generation||source.slot().calibration.depth_scale!=float(s.geometry_config.depth_scale_m)||source.slot().fault||!source.slot().submitted)throw std::invalid_argument("observed stale source dimensions/calibration/depthscale or fault");
 auto index=source.index();if(index>=kSlots)throw std::invalid_argument("observed source outside pool");auto& e=s.entries[index];if(e.submitted||e.references.load())throw std::logic_error("observed source index still leased/pending");
 auto parent_records=input.parent.resident_records_buffer();auto& c=*s.context;e.child=child;e.geometry=geometry;
 struct Guard{ObservedStorage::Entry& e;bool done=false;~Guard(){if(!done){e.child={};e.geometry={};}}}guard{e};
 std::array<VkDescriptorBufferInfo,7> buffers{{{input.roots,0,size_t(8)*s.p.count},{input.records,0,128+size_t(16)*s.p.parentCapacity+size_t(64)*s.p.capacity},{parent_records,0,64+size_t(32)*s.p.parentCapacity},{view.coordinates,0,view.configuration.arena_bytes},{s.roi.buffer,0,48},{e.arena.buffer,0,e.arena.size},{e.compact.buffer,0,e.compact.size}}};std::array<VkWriteDescriptorSet,7> writes{};
 for(unsigned i=0;i<7;i++){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=e.descriptors;writes[i].dstBinding=i;writes[i].descriptorCount=1;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;writes[i].pBufferInfo=&buffers[i];}vkUpdateDescriptorSets(c.device,7,writes.data(),0,nullptr);
 checked(vkResetCommandBuffer(e.command,0),"reset observed command");VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;checked(vkBeginCommandBuffer(e.command,&begin),"begin observed command");observed_detail::record_graph(e.command,s.layout,s.kernels,e.descriptors,s.p,e.compact.buffer,e.readback.buffer,e.compact.size);checked(vkEndCommandBuffer(e.command),"end observed command");checked(vkResetFences(c.device,1,&e.fence),"reset observed fence");VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};info.commandBufferCount=1;info.pCommandBuffers=&e.command;{std::lock_guard<std::mutex> queue_lock(c.queue_mutex);checked(vkQueueSubmit(c.queue,1,&info,e.fence),"submit observed children");++c.submission_count;}e.submitted=true;guard.done=true;return ObservedLease(storage_,index);
}
bool ObservedPreparation::ready(const ObservedLease& a)const{if(a.storage_!=storage_)throw std::invalid_argument("empty/foreign observed lease");return storage_->ready(a.index_);}
ObservedCounts ObservedPreparation::finish(const ObservedLease& a){if(a.storage_!=storage_)throw std::invalid_argument("empty/foreign observed lease");return storage_->result(a.index_);}
}
