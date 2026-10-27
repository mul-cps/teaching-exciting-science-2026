#include "so101_station_vulkan/children.hpp"
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>
namespace so101_station_vulkan {
namespace {
void checked(VkResult r,const char* operation){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(operation)+": "+std::to_string(r));}
void barrier(VkCommandBuffer cmd,VkPipelineStageFlags from,VkPipelineStageFlags to,VkAccessFlags src,VkAccessFlags dst){VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=src;b.dstAccessMask=dst;vkCmdPipelineBarrier(cmd,from,to,0,1,&b,0,nullptr,0,nullptr);}
bool same_parent_config(const ParentConfig& a,const ParentConfig& b){return a.width==b.width&&a.height==b.height&&a.min_parent_pixels==b.min_parent_pixels&&a.record_capacity==b.record_capacity&&a.calibration_generation==b.calibration_generation&&a.generation==b.generation;}
}
static_assert(sizeof(ChildRecord)==64&&sizeof(ParentChildStatus)==16,"child compact ABI");
static_assert(sizeof(child_detail::Parameters)==128,"minimum Vulkan push-constant ABI");
static_assert(sizeof(double)==8&&sizeof(float)==4&&std::numeric_limits<double>::is_iec559&&std::numeric_limits<float>::is_iec559,"IEEE comparison bits required");
void ChildConfig::validate(unsigned width,unsigned height,unsigned parent_capacity)const{
 ChildGraphLayout{width,height,parent_capacity,record_capacity,packed_pixel_capacity};
 if(!generation||!std::isfinite(max_observation_depth_m)||max_observation_depth_m<=.10||!std::isfinite(max_neighbor_jump_m)||max_neighbor_jump_m<=0||min_child_pixels<8)throw std::invalid_argument("child configuration requires finite positive original thresholds, max>.10, minimum>=8 and nonzero generation");
 // Rounded F32zero/inf are valid; do NOT revalidate derived bits as original values.
}
namespace child_detail {
const std::array<const char*,kernel_count> kernel_names{{"child_lookup","child_init","child_hook","child_jump","child_validate","child_reduce","child_gather","child_radix_hist","child_radix_offsets","child_radix_scatter","child_scatter","child_pack_scan","child_pack_offsets","child_jobs","child_packed_init","child_fill_hook","child_fill_jump","child_fill_validate","child_boundary","child_support","child_finalize"}};
void record_graph(VkCommandBuffer command,VkPipelineLayout layout,const std::array<VkPipeline,kernel_count>& kernels,VkDescriptorSet descriptors,const Parameters& original,VkBuffer lookup,VkBuffer compact,VkBuffer readback,size_t compact_bytes,VkBuffer arguments){
 auto p=original;
 vkCmdFillBuffer(command,lookup,0,size_t(4)*p.lookupCount,0);
 vkCmdFillBuffer(command,compact,0,128+size_t(16)*p.parentCapacity,0);
 vkCmdFillBuffer(command,arguments,0,64,0);
 // Shared graph owns lookup/header/argument initialization for literal adapters too.
 barrier(command,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT|VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 vkCmdBindDescriptorSets(command,VK_PIPELINE_BIND_POINT_COMPUTE,layout,0,1,&descriptors,0,nullptr);
 auto push=[&]{vkCmdPushConstants(command,layout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(p),&p);};
 auto dispatch=[&](unsigned kernel,unsigned groups){vkCmdBindPipeline(command,VK_PIPELINE_BIND_POINT_COMPUTE,kernels[kernel]);vkCmdDispatch(command,groups,1,1);barrier(command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);};
 auto indirect=[&](unsigned kernel){vkCmdBindPipeline(command,VK_PIPELINE_BIND_POINT_COMPUTE,kernels[kernel]);vkCmdDispatchIndirect(command,arguments,0);barrier(command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);};
 push();dispatch(0,(p.parentCapacity+255)/256);dispatch(1,(p.count+255)/256);
 for(unsigned round=0;round<32;round++){dispatch(2,(p.count+255)/256);dispatch(3,(p.count+255)/256);}
 dispatch(4,(p.count+255)/256);dispatch(5,(p.count+255)/256);dispatch(6,(p.count+255)/256);
 // Completed roots/sourceFlags persist separately. Histograms reuse dead graph
 // stats, with draftWords padding proving that live drafts cannot be overwritten.
 for(unsigned pass=0;pass<8;pass++){p.radixShift=4*pass;p.radixSide=pass&1;push();dispatch(7,p.groups);dispatch(8,1);dispatch(9,p.groups);}
 dispatch(10,p.groups);dispatch(11,p.groups);dispatch(12,1);dispatch(13,p.capacity);
 // Both args AND jobs/records were written by compute. Indirect consumption
 // requires DRAW_INDIRECT/INDIRECT_COMMAND_READ, not a shader-only dependency.
 barrier(command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_INDIRECT_COMMAND_READ_BIT|VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 indirect(14); // Alias transition to complement A/B/boundary[S] after final records.
 for(unsigned round=0;round<32;round++){indirect(15);indirect(16);}
 indirect(17);indirect(18);indirect(19);dispatch(20,1);
 barrier(command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);VkBufferCopy copy{0,0,compact_bytes};vkCmdCopyBuffer(command,compact,readback,1,&copy);
 barrier(command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);
}
}
struct ChildStorage {
 struct Entry {
  Buffer workspace,persistent,lookup,packed,jobs,compact,pairs,arguments,readback;
  ParentLease parent;FrameLease source;std::vector<ChildRecord> records;std::vector<ParentChildStatus> statuses;
  std::atomic<unsigned> references{0};bool submitted=false,decoded=false;ChildCounts counts;
  VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;VkDescriptorSet descriptors=VK_NULL_HANDLE;
 };
 std::shared_ptr<Context> context;FramePoolToken source_token;ParentConfig expected;ChildConfig config;
 ChildGraphLayout graph;child_detail::Parameters p{};std::array<Entry,kSlots> entries;std::mutex mutex;
 VkCommandPool commands=VK_NULL_HANDLE;VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;VkDescriptorPool descriptor_pool=VK_NULL_HANDLE;VkPipelineLayout layout=VK_NULL_HANDLE;
 std::array<VkPipeline,child_detail::kernel_count> kernels{};
 explicit ChildStorage(const ParentConfig& parent,const ChildConfig& child):expected(parent),config(child),graph(parent.width,parent.height,parent.record_capacity,child.record_capacity,child.packed_pixel_capacity){}
 ~ChildStorage(){
  if(!context)return;auto& c=*context;
  // Only cold teardown waits. Normal drop/collect/finish never waits on device.
  for(auto& e:entries){if(e.submitted)vkWaitForFences(c.device,1,&e.fence,VK_TRUE,UINT64_MAX);if(e.fence)vkDestroyFence(c.device,e.fence,nullptr);}
  if(commands)vkDestroyCommandPool(c.device,commands,nullptr);for(auto k:kernels)if(k)vkDestroyPipeline(c.device,k,nullptr);if(layout)vkDestroyPipelineLayout(c.device,layout,nullptr);if(descriptor_pool)vkDestroyDescriptorPool(c.device,descriptor_pool,nullptr);if(descriptors)vkDestroyDescriptorSetLayout(c.device,descriptors,nullptr);
 }
 bool ready(unsigned i)const{const auto& e=entries[i];if(!e.submitted)return false;auto r=vkGetFenceStatus(context->device,e.fence);if(r==VK_NOT_READY)return false;checked(r,"child fence status");return true;}
 void collect_locked(){for(unsigned i=0;i<kSlots;i++){auto& e=entries[i];if(e.references.load()==0&&e.submitted&&ready(i)){e.parent={};e.source={};e.submitted=false;}}}
 void release(unsigned i)noexcept{if(--entries[i].references)return;try{std::lock_guard<std::mutex> lock(mutex);collect_locked();}catch(...){} }
 ChildCounts header(unsigned i)const{
  const auto& e=entries[i];
  if(!ready(i))throw std::logic_error("child preparation still pending");const auto* words=static_cast<const uint32_t*>(e.readback.mapped);
  if(words[0]!=1||words[1]!=p.width||words[2]!=p.height||words[3]>p.parentCapacity||words[4]>p.capacity||words[5]>p.packedCapacity||words[6]>p.jobCapacity||words[7]||words[8]!=p.calLo||words[9]!=p.calHi||words[10]!=p.parentLo||words[11]!=p.parentHi||words[12]!=p.childLo||words[13]!=p.childHi||words[14]!=p.maxBits||words[15]!=p.jumpBits||words[16]!=p.minChild||words[17]!=p.capacity||words[18]!=p.packedCapacity||words[19]!=p.jobCapacity||words[20]!=p.parentCapacity||words[21]||words[22]!=p.maxLo||words[23]!=p.maxHi||words[24]!=p.jumpLo||words[25]!=p.jumpHi)throw std::runtime_error("child capacity, identity, overflow, nonconvergence or result fault: frame unknown");
  for(unsigned j=26;j<32;j++)if(words[j])throw std::runtime_error("child reserved result fault");
  return {words[3],words[4],words[5],words[6],expected.calibration_generation,expected.generation,config.generation};
 }
 ChildCounts result(unsigned i){
  std::lock_guard<std::mutex> lock(mutex);auto& e=entries[i];if(e.decoded)return e.counts;
  const auto counts=header(i);const auto* words=static_cast<const uint32_t*>(e.readback.mapped);
  if(!counts.record_count&&(counts.packed_pixels||counts.jobs))throw std::runtime_error("empty child result has packed work");
  const auto* parents=e.parent.records();uint32_t valid=0,candidate=0,all=0,kept=0;
  std::memcpy(e.statuses.data(),words+32,size_t(counts.parent_count)*sizeof(ParentChildStatus));
  for(unsigned j=0;j<counts.parent_count;j++){const auto& v=e.statuses[j];if(v.valid_depth_pixels>parents[j].parent_pixels||v.candidate_pixels>v.valid_depth_pixels||v.all_children>v.candidate_pixels||v.kept_children>v.all_children)throw std::runtime_error("child parent status fault");valid+=v.valid_depth_pixels;candidate+=v.candidate_pixels;all+=v.all_children;kept+=v.kept_children;}
  if(valid>p.count||candidate>valid||all>candidate||kept!=counts.record_count)throw std::runtime_error("child status total fault");
  std::memcpy(e.records.data(),words+32+4*p.parentCapacity,size_t(counts.record_count)*sizeof(ChildRecord));
  uint32_t packed_end=0,jobs=0;
  for(unsigned j=0;j<counts.record_count;j++){
   const auto& r=e.records[j];if(r.parent_record_index>=counts.parent_count||!r.root||r.root>p.count||r.region_pixels<p.minChild||r.region_pixels>p.count||!r.width||!r.height||r.reserved)throw std::runtime_error("child compact record fault");
   const auto& par=parents[r.parent_record_index];uint32_t bw=(par.width+1)/2,bh=(par.height+1)/2;
   if(r.key>=bw*bh||r.x>=par.width||r.y>=par.height||r.width>par.width-r.x||r.height>par.height-r.y||r.region_pixels>r.width*r.height||r.left!=(r.x>2?r.x-2:0)||r.top!=(r.y>2?r.y-2:0)||r.halo_width!=std::min(par.width,r.x+r.width+2)-r.left||r.halo_height!=std::min(par.height,r.y+r.height+2)-r.top||r.packed_offset!=packed_end)throw std::runtime_error("child local bbox/halo/packed fault");
   auto area=r.halo_width*r.halo_height;auto rootx=(r.root-1)%p.width,rooty=(r.root-1)/p.width;
   if(area>p.packedCapacity-packed_end||r.support_pixels<r.region_pixels||r.support_pixels>std::min(area,par.parent_pixels)||r.depth_cut_pixels>r.support_pixels||r.depth_cut_pixels>area-r.region_pixels||rootx<par.x+r.x||rooty<par.y+r.y||rootx>=par.x+r.x+r.width||rooty>=par.y+r.y+r.height)throw std::runtime_error("child root/support capacity fault");
   packed_end+=area;jobs+=(area+255)/256;
   if(j){const auto& before=e.records[j-1];if(before.parent_record_index>r.parent_record_index||(before.parent_record_index==r.parent_record_index&&before.key>=r.key))throw std::runtime_error("child order fault");}
  }
  // Sorted records permit a linear P+R consistency check without another
  // counter array. all_children may exceed C when subminimum roots filter out.
  unsigned cursor=0;
  for(unsigned parent=0;parent<counts.parent_count;parent++){unsigned start=cursor;uint32_t area=0;while(cursor<counts.record_count&&e.records[cursor].parent_record_index==parent){area+=e.records[cursor].region_pixels;++cursor;}if(cursor-start!=e.statuses[parent].kept_children||area>e.statuses[parent].candidate_pixels)throw std::runtime_error("child per-parent record/status mismatch");}
  if(packed_end!=counts.packed_pixels||jobs!=counts.jobs)throw std::runtime_error("child packed job total fault");
  e.counts=counts;e.decoded=true;return counts;
 }
};
ChildLease::ChildLease(std::shared_ptr<ChildStorage> s,unsigned i):storage_(std::move(s)),index_(i){++storage_->entries[i].references;}
ChildLease::ChildLease(const ChildLease& other):storage_(other.storage_),index_(other.index_){if(storage_)++storage_->entries[index_].references;}
ChildLease::ChildLease(ChildLease&& other)noexcept:storage_(std::move(other.storage_)),index_(other.index_){}
ChildLease& ChildLease::operator=(ChildLease other){swap(other);return *this;}
ChildLease::~ChildLease(){if(storage_)storage_->release(index_);}
void ChildLease::swap(ChildLease& other)noexcept{storage_.swap(other.storage_);std::swap(index_,other.index_);}
const FrameMetadata& ChildLease::metadata()const{if(!storage_)throw std::logic_error("empty child lease");return storage_->entries[index_].source.slot().metadata;}
ParentLease ChildLease::parent()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return storage_->entries[index_].parent;}
VkBuffer ChildLease::source_root_buffer()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return storage_->entries[index_].persistent.buffer;}
size_t ChildLease::source_flags_offset_bytes()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return size_t(storage_->p.count)*4;}
VkBuffer ChildLease::packed_flags_buffer()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return storage_->entries[index_].packed.buffer;}
VkBuffer ChildLease::resident_records_buffer()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return storage_->entries[index_].compact.buffer;}
const ChildRecord* ChildLease::records()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return storage_->entries[index_].records.data();}
const ParentChildStatus* ChildLease::parent_status()const{if(!storage_)throw std::logic_error("empty child lease");storage_->result(index_);return storage_->entries[index_].statuses.data();}
ChildResidentView ChildLease::resident_view()const{
 if(!storage_)throw std::logic_error("empty child lease");
 auto& s=*storage_;std::lock_guard<std::mutex> lock(s.mutex);auto counts=s.header(index_);const auto& e=s.entries[index_];
 return {e.source,e.parent,s.expected,s.config,counts,e.persistent.buffer,e.compact.buffer};
}
ChildPreparation::ChildPreparation(std::shared_ptr<Context> context,FramePoolToken token,const ParentConfig& expected,const ChildConfig& config,const std::string& directory){
 expected.validate();config.validate(expected.width,expected.height,expected.record_capacity);if(!context||!token)throw std::invalid_argument("children require resident Context and live producer token");
 auto s=std::make_shared<ChildStorage>(expected,config);s->context=std::move(context);s->source_token=std::move(token);auto& c=*s->context;const auto& g=s->graph;auto& limits=c.properties.limits;
 if(limits.maxComputeWorkGroupInvocations<256||limits.maxComputeWorkGroupSize[0]<256||limits.maxComputeSharedMemorySize<16384||limits.maxComputeWorkGroupCount[0]<std::max({(g.count+255)/256,g.job_capacity,g.record_capacity})||limits.maxPushConstantsSize<sizeof(child_detail::Parameters)||limits.maxPerStageDescriptorStorageBuffers<11||limits.maxDescriptorSetStorageBuffers<11)throw std::runtime_error("children require bounded256lane/16KiB/11storage/128byte push compute");
 for(size_t size:{g.workspace_bytes,g.persistent_bytes,g.lookup_bytes,g.packed_bytes,g.jobs_bytes,g.pairs_bytes,g.compact_bytes,size_t(8)*g.count,parent_arena_bytes(expected.width,expected.height)})if(size>limits.maxStorageBufferRange)throw std::runtime_error("child bounded storage range unsupported");
 auto max=child_double_bits(config.max_observation_depth_m),jump=child_double_bits(config.max_neighbor_jump_m);
 s->p={g.width,g.height,g.count,g.blocks,g.parent_capacity,g.record_capacity,g.packed_capacity,g.job_capacity,g.key_bits,g.groups,child_f32_rne_bits(config.max_observation_depth_m),child_f32_rne_bits(config.max_neighbor_jump_m),config.min_child_pixels,0,0,0,uint32_t(expected.calibration_generation),uint32_t(expected.calibration_generation>>32),uint32_t(expected.generation),uint32_t(expected.generation>>32),uint32_t(config.generation),uint32_t(config.generation>>32),uint32_t(max),uint32_t(max>>32),uint32_t(jump),uint32_t(jump>>32),uint32_t(g.draft_offset/4),g.lookup_count,0,{0,0,0}};
 std::array<VkDescriptorSetLayoutBinding,11> bindings{};for(unsigned i=0;i<11;i++){bindings[i].binding=i;bindings[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;bindings[i].descriptorCount=1;bindings[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}
 VkDescriptorSetLayoutCreateInfo di{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};di.bindingCount=11;di.pBindings=bindings.data();checked(vkCreateDescriptorSetLayout(c.device,&di,nullptr,&s->descriptors),"child descriptor layout");
 VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(s->p)};VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};li.setLayoutCount=1;li.pSetLayouts=&s->descriptors;li.pushConstantRangeCount=1;li.pPushConstantRanges=&push;checked(vkCreatePipelineLayout(c.device,&li,nullptr,&s->layout),"child layout");
 for(unsigned i=0;i<child_detail::kernel_count;i++){
  std::ifstream file(directory+"/"+child_detail::kernel_names[i]+".spv",std::ios::binary|std::ios::ate);if(!file)throw std::runtime_error("missing offline child shader");auto bytes=file.tellg();if(bytes<=0||size_t(bytes)%4)throw std::runtime_error("invalid child SPIR-V length");std::vector<uint32_t> code(size_t(bytes)/4);file.seekg(0);file.read(reinterpret_cast<char*>(code.data()),bytes);if(!file)throw std::runtime_error("incomplete child SPIR-V");VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};mi.codeSize=size_t(bytes);mi.pCode=code.data();VkShaderModule module=VK_NULL_HANDLE;checked(vkCreateShaderModule(c.device,&mi,nullptr,&module),"child shader module");VkComputePipelineCreateInfo pi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};pi.layout=s->layout;pi.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;pi.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pi.stage.module=module;pi.stage.pName="main";auto result=vkCreateComputePipelines(c.device,VK_NULL_HANDLE,1,&pi,nullptr,&s->kernels[i]);vkDestroyShaderModule(c.device,module,nullptr);checked(result,"child pipeline");
 }
 VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,11*kSlots};VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};dpi.maxSets=kSlots;dpi.poolSizeCount=1;dpi.pPoolSizes=&ps;checked(vkCreateDescriptorPool(c.device,&dpi,nullptr,&s->descriptor_pool),"child descriptor pool");VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};ci.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;ci.queueFamilyIndex=c.family;checked(vkCreateCommandPool(c.device,&ci,nullptr,&s->commands),"child private command pool");
 for(auto& e:s->entries){VkBufferUsageFlags storage=VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,read=storage|VK_BUFFER_USAGE_TRANSFER_SRC_BIT;auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  e.workspace.create(c,g.workspace_bytes,storage,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.persistent.create(c,g.persistent_bytes,read,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.lookup.create(c,g.lookup_bytes,storage|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.packed.create(c,g.packed_bytes,read,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.jobs.create(c,g.jobs_bytes,storage,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.pairs.create(c,g.pairs_bytes,storage,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.compact.create(c,g.compact_bytes,read|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.arguments.create(c,64,storage|VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);e.readback.create(c,g.compact_bytes,VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);e.records.resize(g.record_capacity);e.statuses.resize(g.parent_capacity);
  VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};fi.flags=VK_FENCE_CREATE_SIGNALED_BIT;checked(vkCreateFence(c.device,&fi,nullptr,&e.fence),"child fence");VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};ai.commandPool=s->commands;ai.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;ai.commandBufferCount=1;checked(vkAllocateCommandBuffers(c.device,&ai,&e.command),"child command");VkDescriptorSetAllocateInfo si{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};si.descriptorPool=s->descriptor_pool;si.descriptorSetCount=1;si.pSetLayouts=&s->descriptors;checked(vkAllocateDescriptorSets(c.device,&si,&e.descriptors),"child descriptors");
 }
 storage_=std::move(s);
}
void ChildPreparation::collect(){std::lock_guard<std::mutex> lock(storage_->mutex);storage_->collect_locked();}
ChildLease ChildPreparation::submit(const ParentLease& parent){
 auto& s=*storage_;std::lock_guard<std::mutex> lock(s.mutex);s.collect_locked();
 if(!parent)throw std::invalid_argument("empty parent input");
 auto configuration=parent.configuration();auto source_frame=parent.source_frame();
 if(!same_parent_config(configuration,s.expected)||source_frame.context()!=s.context||!source_frame.belongs_to(s.source_token)||parent.index()!=source_frame.index())throw std::invalid_argument("child parent configuration/source owner/Context/index mismatch");
 const auto& source=source_frame.slot();
 if(source.metadata.width!=s.p.width||source.metadata.height!=s.p.height||source.metadata.calibration_generation!=s.expected.calibration_generation||source.calibration.generation!=s.expected.calibration_generation||!source.submitted||source.fault||!source.scratch.buffer||source.scratch.owner!=s.context.get()||source.scratch.size<size_t(s.p.count)*8)throw std::invalid_argument("child stale/invalid resident source");
 auto result=vkGetFenceStatus(s.context->device,source.fence);if(result==VK_NOT_READY)throw std::logic_error("child source pending");checked(result,"child source fence");if(!source.readback.mapped||source.readback.size<16)throw std::invalid_argument("child source missing compact completion");const auto* status=static_cast<const uint32_t*>(source.readback.mapped);if(status[0]>kMaxComponents||status[1])throw std::runtime_error("child source Core fault: unknown frame");
 auto index=source_frame.index();if(index>=kSlots)throw std::invalid_argument("child source index outside pool");auto& e=s.entries[index];if(e.submitted||e.references.load())throw std::logic_error("child source index still leased or pending");
 auto parent_buffer=parent.resident_buffer(),parent_records=parent.resident_records_buffer();auto label_offset=parent.label_offset_bytes();if(label_offset%4)throw std::invalid_argument("child parent label offset not U32 aligned");
 auto p=s.p;p.parentLabelWords=uint32_t(label_offset/4);auto& c=*s.context;e.parent=parent;e.source=source_frame;e.decoded=false;
 struct Guard{ChildStorage::Entry& e;bool done=false;~Guard(){if(!done){e.parent={};e.source={};}}}guard{e};
 // Offset0 keeps all descriptors aligned even for tiny odd parent images.
 std::array<VkDescriptorBufferInfo,11> buffers{{{source.scratch.buffer,0,size_t(p.count)*8},{parent_buffer,0,label_offset+size_t(p.count)*4},{parent_records,0,64+size_t(s.p.parentCapacity)*32},{e.workspace.buffer,0,e.workspace.size},{e.persistent.buffer,0,e.persistent.size},{e.lookup.buffer,0,e.lookup.size},{e.packed.buffer,0,e.packed.size},{e.jobs.buffer,0,e.jobs.size},{e.compact.buffer,0,e.compact.size},{e.pairs.buffer,0,e.pairs.size},{e.arguments.buffer,0,e.arguments.size}}};std::array<VkWriteDescriptorSet,11> writes{};
 for(unsigned i=0;i<11;i++){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=e.descriptors;writes[i].dstBinding=i;writes[i].descriptorCount=1;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;writes[i].pBufferInfo=&buffers[i];}vkUpdateDescriptorSets(c.device,11,writes.data(),0,nullptr);
 checked(vkResetCommandBuffer(e.command,0),"reset child command");VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;checked(vkBeginCommandBuffer(e.command,&begin),"begin child command");
 child_detail::record_graph(e.command,s.layout,s.kernels,e.descriptors,p,e.lookup.buffer,e.compact.buffer,e.readback.buffer,e.compact.size,e.arguments.buffer);
 checked(vkEndCommandBuffer(e.command),"end child command");checked(vkResetFences(c.device,1,&e.fence),"reset child fence");VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};info.commandBufferCount=1;info.pCommandBuffers=&e.command;{std::lock_guard<std::mutex> queue_lock(c.queue_mutex);checked(vkQueueSubmit(c.queue,1,&info,e.fence),"submit children");++c.submission_count;}
 e.submitted=true;guard.done=true;return ChildLease(storage_,index);
}
bool ChildPreparation::ready(const ChildLease& lease)const{if(lease.storage_!=storage_)throw std::invalid_argument("foreign or empty child lease");return storage_->ready(lease.index_);}
ChildCounts ChildPreparation::finish(const ChildLease& lease){if(lease.storage_!=storage_)throw std::invalid_argument("foreign or empty child lease");return storage_->result(lease.index_);}
}
