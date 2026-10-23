#include "so101_station_vulkan/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>
namespace so101_station_vulkan {
namespace {
void checked(VkResult result,const char* operation){
 if(result!=VK_SUCCESS)throw std::runtime_error(std::string(operation)+": "+std::to_string(result));
}
struct Parameters{uint32_t width,height,count,grid_count,grid_width,blocks;};
Parameters dimensions(unsigned w,unsigned h){
 if(!w||!h||w>640||h>480)throw std::invalid_argument("geometry requires dimensions within 640x480");
 unsigned gw=(w+7)/8,gh=(h+7)/8;return {w,h,w*h,gw*gh,gw,(gw*gh+255)/256};
}
void memory_barrier(VkCommandBuffer command,VkPipelineStageFlags source,VkPipelineStageFlags dest,VkAccessFlags read,VkAccessFlags write){
 VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};barrier.srcAccessMask=read;barrier.dstAccessMask=write;
 vkCmdPipelineBarrier(command,source,dest,0,1,&barrier,0,nullptr,0,nullptr);
}
}
size_t geometry_arena_bytes(unsigned width,unsigned height){
 auto p=dimensions(width,height);
 // Matches documented shader arena byte offsets. Pad selected double array.
 size_t selected=(size_t(p.count)*28+size_t(p.grid_count)*4+size_t(p.blocks)*8+7)&~size_t(7);
 return selected+size_t(p.grid_count)*28;
}
void GeometryCalibration::validate()const{
 auto p=dimensions(width,height);
 if(!generation)throw std::invalid_argument("geometry calibration generation must be nonzero");
 if(camera_rays.size()!=p.count)throw std::invalid_argument("geometry ray LUT dimensions mismatch");
 for(const auto& ray:camera_rays)for(double v:ray)if(!std::isfinite(v))throw std::invalid_argument("geometry ray LUT must be finite");
 for(double v:board_from_camera)if(!std::isfinite(v))throw std::invalid_argument("geometry transform must be finite");
 for(unsigned i=0;i<6;i+=2)if(!std::isfinite(support_roi[i])||!std::isfinite(support_roi[i+1])||support_roi[i]>=support_roi[i+1])throw std::invalid_argument("geometry support ROI must have finite increasing bounds");
 if(!std::isfinite(depth_scale_m)||depth_scale_m<=0||!std::isfinite(float(depth_scale_m))||float(depth_scale_m)<=0)throw std::invalid_argument("geometry depth scale invalid");
 if(!std::isfinite(max_observation_depth_m)||max_observation_depth_m<.1||!std::isfinite(float(max_observation_depth_m)))throw std::invalid_argument("geometry depth upper bound invalid");
}
struct GeometryStorage {
 struct Entry {
  Buffer arena,counts;
  FrameLease source;
  std::atomic<unsigned> references{0};
  VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;
  VkDescriptorSet descriptors=VK_NULL_HANDLE;bool submitted=false;
 };
 std::shared_ptr<Context> context;
 Parameters p;uint64_t generation=0;float depth_scale=0;
 Buffer calibration;std::array<Entry,kSlots> entries;
 std::mutex mutex;
 VkCommandPool commands=VK_NULL_HANDLE; // private pool, serialized by mutex through recording
 VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;VkDescriptorPool descriptor_pool=VK_NULL_HANDLE;
 VkPipelineLayout layout=VK_NULL_HANDLE;std::array<VkPipeline,5> kernels{};
 ~GeometryStorage(){
  if(!context)return;
  auto& c=*context;
  for(auto& e:entries){
   // Shutdown only; normal submit/poll/finish never wait for a device/queue.
   if(e.submitted)vkWaitForFences(c.device,1,&e.fence,VK_TRUE,UINT64_MAX);
   if(e.fence)vkDestroyFence(c.device,e.fence,nullptr);

  }
  if(commands)vkDestroyCommandPool(c.device,commands,nullptr);
  for(auto k:kernels)if(k)vkDestroyPipeline(c.device,k,nullptr);
  if(layout)vkDestroyPipelineLayout(c.device,layout,nullptr);
  if(descriptor_pool)vkDestroyDescriptorPool(c.device,descriptor_pool,nullptr);
  if(descriptors)vkDestroyDescriptorSetLayout(c.device,descriptors,nullptr);
 }
 bool ready(unsigned i)const{
  const auto& e=entries[i];if(!e.submitted)return false;
  VkResult r=vkGetFenceStatus(context->device,e.fence);if(r==VK_NOT_READY)return false;
  checked(r,"geometry fence status");return true;
 }
 void collect_locked(){
  for(unsigned i=0;i<kSlots;i++){
   auto& e=entries[i];if(e.references.load()==0&&e.submitted&&ready(i)){e.source={};e.submitted=false;}
  }
 }
 void release(unsigned i)noexcept{
  if(--entries[i].references!=0)return;
  // Never block a frame-drop destructor. In-flight dropped leases are reaped
  // by collect(), called on every submit and scheduler idle tick.
  try{std::lock_guard<std::mutex> lock(mutex);collect_locked();}catch(...){}
 }
};
GeometryLease::GeometryLease(std::shared_ptr<GeometryStorage> s,unsigned i):storage_(std::move(s)),index_(i){++storage_->entries[i].references;}
GeometryLease::GeometryLease(const GeometryLease& other):storage_(other.storage_),index_(other.index_){if(storage_)++storage_->entries[index_].references;}
GeometryLease::GeometryLease(GeometryLease&& other)noexcept:storage_(std::move(other.storage_)),index_(other.index_){}
GeometryLease& GeometryLease::operator=(GeometryLease other){swap(other);return *this;}
GeometryLease::~GeometryLease(){if(storage_)storage_->release(index_);}
void GeometryLease::swap(GeometryLease& other)noexcept{storage_.swap(other.storage_);std::swap(index_,other.index_);}
VkBuffer GeometryLease::resident_buffer()const{
 if(!storage_)throw std::logic_error("empty geometry lease");
 if(!storage_->ready(index_))throw std::logic_error("geometry buffer not complete");
 return storage_->entries[index_].arena.buffer;
}
GeometryPreparation::GeometryPreparation(std::shared_ptr<Context> context,const GeometryCalibration& cal,const std::string& directory){
 cal.validate();if(!context)throw std::invalid_argument("geometry requires the resident Vulkan device");
 if(!context->enabled_features.features.shaderFloat64)throw std::runtime_error("geometry unavailable: shaderFloat64 was not enabled on shared device");
 if(context->properties.limits.maxComputeWorkGroupInvocations<256||context->properties.limits.maxComputeWorkGroupSize[0]<256||context->properties.limits.maxComputeSharedMemorySize<2048||context->properties.limits.maxComputeWorkGroupCount[0]<(cal.width*cal.height+255)/256||context->properties.limits.maxPushConstantsSize<sizeof(Parameters))throw std::runtime_error("geometry requires 256-lane FP64 preparation workgroups");
 auto s=std::make_shared<GeometryStorage>();s->context=std::move(context);s->p=dimensions(cal.width,cal.height);s->generation=cal.generation;s->depth_scale=float(cal.depth_scale_m);auto& c=*s->context;
 size_t calibration_bytes=(size_t(s->p.count)*2+19)*sizeof(double);
 if(geometry_arena_bytes(cal.width,cal.height)>c.properties.limits.maxStorageBufferRange||calibration_bytes>c.properties.limits.maxStorageBufferRange)throw std::runtime_error("geometry unavailable: bounded arena exceeds device storage range");
 auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
 s->calibration.create(c,calibration_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,host);
 auto* values=static_cast<double*>(s->calibration.mapped);
 std::memcpy(values,cal.camera_rays.data(),cal.camera_rays.size()*sizeof(cal.camera_rays[0]));
 values+=2*s->p.count;std::copy(cal.board_from_camera.begin(),cal.board_from_camera.end(),values);
 std::copy(cal.support_roi.begin(),cal.support_roi.end(),values+12);values[18]=double(float(cal.max_observation_depth_m));
 std::array<VkDescriptorSetLayoutBinding,4> bindings{};
 for(unsigned i=0;i<4;i++){bindings[i].binding=i;bindings[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;bindings[i].descriptorCount=1;bindings[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}
 VkDescriptorSetLayoutCreateInfo descriptor_info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};descriptor_info.bindingCount=4;descriptor_info.pBindings=bindings.data();
 checked(vkCreateDescriptorSetLayout(c.device,&descriptor_info,nullptr,&s->descriptors),"geometry descriptor layout");
 VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(Parameters)};
 VkPipelineLayoutCreateInfo layout_info{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};layout_info.setLayoutCount=1;layout_info.pSetLayouts=&s->descriptors;layout_info.pushConstantRangeCount=1;layout_info.pPushConstantRanges=&push;
 checked(vkCreatePipelineLayout(c.device,&layout_info,nullptr,&s->layout),"geometry pipeline layout");
 const char* names[]={"geometry_coordinates","geometry_support_scan","geometry_support_offsets","geometry_support_scatter","geometry_valid_count"};
 for(unsigned i=0;i<5;i++){
  std::ifstream file(directory+"/"+names[i]+".spv",std::ios::binary|std::ios::ate);
  if(!file)throw std::runtime_error(std::string("geometry missing offline shader: ")+names[i]);
  auto bytes=file.tellg();if(bytes<=0||size_t(bytes)%4)throw std::runtime_error("geometry invalid SPIR-V length");
  std::vector<uint32_t> code(size_t(bytes)/4);file.seekg(0);file.read(reinterpret_cast<char*>(code.data()),bytes);
  if(!file)throw std::runtime_error("geometry failed reading complete SPIR-V");
  VkShaderModuleCreateInfo shader_info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};shader_info.codeSize=size_t(bytes);shader_info.pCode=code.data();VkShaderModule module=VK_NULL_HANDLE;
  checked(vkCreateShaderModule(c.device,&shader_info,nullptr,&module),"geometry shader module");
  VkComputePipelineCreateInfo info{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};info.layout=s->layout;info.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;info.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;info.stage.module=module;info.stage.pName="main";
  auto result=vkCreateComputePipelines(c.device,VK_NULL_HANDLE,1,&info,nullptr,&s->kernels[i]);vkDestroyShaderModule(c.device,module,nullptr);checked(result,"geometry compute pipeline");
 }
 VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,4*kSlots};
 VkDescriptorPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};pool_info.maxSets=kSlots;pool_info.poolSizeCount=1;pool_info.pPoolSizes=&pool_size;
 checked(vkCreateDescriptorPool(c.device,&pool_info,nullptr,&s->descriptor_pool),"geometry descriptor pool");
 VkCommandPoolCreateInfo command_pool{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
 command_pool.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;command_pool.queueFamilyIndex=c.family;
 checked(vkCreateCommandPool(c.device,&command_pool,nullptr,&s->commands),"geometry private command pool");
 for(auto& e:s->entries){
  e.arena.create(c,geometry_arena_bytes(cal.width,cal.height),VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  e.counts.create(c,16,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
  VkFenceCreateInfo fence_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};fence_info.flags=VK_FENCE_CREATE_SIGNALED_BIT;
  checked(vkCreateFence(c.device,&fence_info,nullptr,&e.fence),"geometry fence");
  VkCommandBufferAllocateInfo cmd_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};cmd_info.commandPool=s->commands;cmd_info.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;cmd_info.commandBufferCount=1;
  checked(vkAllocateCommandBuffers(c.device,&cmd_info,&e.command),"geometry command");
  VkDescriptorSetAllocateInfo set_info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};set_info.descriptorPool=s->descriptor_pool;set_info.descriptorSetCount=1;set_info.pSetLayouts=&s->descriptors;
  checked(vkAllocateDescriptorSets(c.device,&set_info,&e.descriptors),"geometry descriptor set");
 }
 storage_=std::move(s);
}
void GeometryPreparation::collect(){std::lock_guard<std::mutex> lock(storage_->mutex);storage_->collect_locked();}
GeometryLease GeometryPreparation::submit(const FrameLease& frame){
 auto& s=*storage_;std::lock_guard<std::mutex> lock(s.mutex);s.collect_locked();
 if(!frame||frame.context()!=s.context)throw std::invalid_argument("geometry requires a frame from the same shared Vulkan device");
 const auto& source=frame.slot();
 if(source.metadata.width!=s.p.width||source.metadata.height!=s.p.height||source.metadata.calibration_generation!=s.generation||source.calibration.generation!=s.generation)throw std::invalid_argument("geometry stale calibration generation or dimensions");
 if(source.calibration.depth_scale!=s.depth_scale)throw std::invalid_argument("geometry depth scale differs from calibration generation");
 if(!source.submitted||!source.scratch.buffer||source.scratch.size<size_t(s.p.count)*8||source.fault)throw std::invalid_argument("geometry requires a successful submitted resident input");
 auto r=vkGetFenceStatus(s.context->device,source.fence);if(r==VK_NOT_READY)throw std::logic_error("geometry source is not complete");checked(r,"geometry source fence");
 if(!source.readback.mapped||source.readback.size<16)throw std::invalid_argument("geometry source has no compact completion record");
 const auto* input_status=static_cast<const uint32_t*>(source.readback.mapped);
 if(input_status[0]>kMaxComponents||input_status[1])throw std::runtime_error("geometry input reduction fault: frame unknown");
 auto index=frame.index();auto& e=s.entries[index];if(e.submitted||e.references.load())throw std::logic_error("geometry slot still leased or pending");
 auto& c=*s.context;e.source=frame;
 struct SubmissionGuard { GeometryStorage::Entry& entry;bool done=false;~SubmissionGuard(){if(!done)entry.source={};} } guard{e};
 std::array<VkDescriptorBufferInfo,4> buffers{{{source.scratch.buffer,0,source.scratch.size},{s.calibration.buffer,0,s.calibration.size},{e.arena.buffer,0,e.arena.size},{e.counts.buffer,0,e.counts.size}}};
 std::array<VkWriteDescriptorSet,4> writes{};
 for(unsigned i=0;i<4;i++){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=e.descriptors;writes[i].dstBinding=i;writes[i].descriptorCount=1;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;writes[i].pBufferInfo=&buffers[i];}
 vkUpdateDescriptorSets(c.device,4,writes.data(),0,nullptr);
 checked(vkResetCommandBuffer(e.command,0),"reset geometry command");
 VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
 checked(vkBeginCommandBuffer(e.command,&begin),"begin geometry command");
 vkCmdFillBuffer(e.command,e.counts.buffer,0,16,0);
 memory_barrier(e.command,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT|VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 vkCmdBindDescriptorSets(e.command,VK_PIPELINE_BIND_POINT_COMPUTE,s.layout,0,1,&e.descriptors,0,nullptr);
 vkCmdPushConstants(e.command,s.layout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(s.p),&s.p);
 const unsigned dispatches[]={(s.p.count+255)/256,s.p.blocks,1,s.p.blocks,(s.p.count+255)/256};
 for(unsigned i=0;i<5;i++){
  vkCmdBindPipeline(e.command,VK_PIPELINE_BIND_POINT_COMPUTE,s.kernels[i]);vkCmdDispatch(e.command,dispatches[i],1,1);
  if(i+1<5)memory_barrier(e.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 }
 memory_barrier(e.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);
 checked(vkEndCommandBuffer(e.command),"end geometry command");
 checked(vkResetFences(c.device,1,&e.fence),"reset geometry fence");
 VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};info.commandBufferCount=1;info.pCommandBuffers=&e.command;
 {std::lock_guard<std::mutex> queue_lock(c.queue_mutex);checked(vkQueueSubmit(c.queue,1,&info,e.fence),"submit geometry");++c.submission_count;}
 e.submitted=true;guard.done=true;return GeometryLease(storage_,index);
}
bool GeometryPreparation::ready(const GeometryLease& lease)const{
 if(lease.storage_!=storage_)throw std::invalid_argument("foreign or empty geometry lease");
 return storage_->ready(lease.index_);
}
GeometryCounts GeometryPreparation::finish(const GeometryLease& lease){
 if(!ready(lease))throw std::logic_error("geometry preparation still pending");
 const auto* counts=static_cast<const uint32_t*>(storage_->entries[lease.index_].counts.mapped);
 if(counts[0]>storage_->p.count||counts[1]>storage_->p.grid_count||counts[1]>4800||counts[2]||counts[3])throw std::runtime_error("geometry capacity or result fault: frame unknown");
 return {counts[0],counts[1],storage_->generation};
}
}
