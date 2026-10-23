#include "so101_station_vulkan/pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>
namespace so101_station_vulkan {
namespace {
void check(VkResult r,const char* op){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(op)+": "+std::to_string(r));}
struct Parameters{uint32_t width,height,count,capacity;float scale,fx,fy,cx,cy,maxDepth,jump;uint32_t round;};
void barrier(VkCommandBuffer cmd,VkPipelineStageFlags src,VkPipelineStageFlags dst,VkAccessFlags sa,VkAccessFlags da){VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=sa;b.dstAccessMask=da;vkCmdPipelineBarrier(cmd,src,dst,0,1,&b,0,nullptr,0,nullptr);}
}
Pipeline::Pipeline(unsigned w,unsigned h,const std::string& mode,const std::string& shader_dir,bool debug,bool profile):shader_dir_(shader_dir){
 if(mode!="auto"&&mode!="vulkan"&&mode!="cpu")throw std::invalid_argument("backend must be auto, vulkan or cpu");
 backend_="cpu";
 if(mode!="cpu"){
  try{context_=Context::shared();pool_=std::make_shared<FramePool>(w,h,context_,debug,profile);create_gpu();backend_="vulkan";}
  catch(const std::exception& e){destroy_gpu();pool_.reset();divisions_.reset();context_.reset();if(mode=="vulkan")throw;reason_=e.what();}
 }
 if(!pool_)pool_=std::make_shared<FramePool>(w,h,nullptr,debug);
}
Pipeline::~Pipeline(){destroy_gpu();}
void Pipeline::destroy_gpu() noexcept{
 // Leases may retain frame storage after Pipeline destruction. Drain those
 // submissions before destroying the executable resources referenced by them.
 if(context_){if(pool_)for(auto& s:pool_->slots)if(s.submitted)vkWaitForFences(context_->device,1,&s.fence,VK_TRUE,UINT64_MAX);
  for(auto p:kernels_)if(p)vkDestroyPipeline(context_->device,p,nullptr);if(layout_)vkDestroyPipelineLayout(context_->device,layout_,nullptr);
  if(descriptor_pool_)vkDestroyDescriptorPool(context_->device,descriptor_pool_,nullptr);if(descriptors_)vkDestroyDescriptorSetLayout(context_->device,descriptors_,nullptr);
  kernels_.fill(VK_NULL_HANDLE);layout_=VK_NULL_HANDLE;descriptor_pool_=VK_NULL_HANDLE;descriptors_=VK_NULL_HANDLE;
 }
}
void Pipeline::create_gpu(){
 auto& c=*context_;
 if(c.properties.limits.maxComputeWorkGroupInvocations<256||c.properties.limits.maxComputeWorkGroupSize[0]<256||c.properties.limits.maxComputeWorkGroupSize[1]<16||c.properties.limits.maxComputeSharedMemorySize<3076)throw std::runtime_error("Vulkan device lacks bounded 16x16 tiled CC workgroup/shared-memory capabilities");
 std::array<VkDescriptorSetLayoutBinding,4> bs{};for(unsigned i=0;i<4;i++){bs[i].binding=i;bs[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;bs[i].descriptorCount=1;bs[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}
 VkDescriptorSetLayoutCreateInfo dl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};dl.bindingCount=bs.size();dl.pBindings=bs.data();check(vkCreateDescriptorSetLayout(c.device,&dl,nullptr,&descriptors_),"descriptor layout");
 VkPushConstantRange push{};push.stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;push.size=sizeof(Parameters);
 VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};pl.setLayoutCount=1;pl.pSetLayouts=&descriptors_;pl.pushConstantRangeCount=1;pl.pPushConstantRanges=&push;check(vkCreatePipelineLayout(c.device,&pl,nullptr,&layout_),"pipeline layout");
 const char* entries[]={"convert","candidates","connect","compress","reduce","gather","morphology","tiled_components"};
 for(unsigned i=0;i<kernels_.size();i++){
  std::ifstream f(shader_dir_+"/"+entries[i]+".spv",std::ios::binary|std::ios::ate);if(!f)throw std::runtime_error("missing offline SPIR-V: "+shader_dir_+"/"+entries[i]+".spv");
  auto bytes=f.tellg();if(bytes<=0||size_t(bytes)%4)throw std::runtime_error("invalid SPIR-V length");std::vector<uint32_t> code(size_t(bytes)/4);f.seekg(0);f.read(reinterpret_cast<char*>(code.data()),bytes);
  VkShaderModuleCreateInfo sm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};sm.codeSize=size_t(bytes);sm.pCode=code.data();VkShaderModule module;check(vkCreateShaderModule(c.device,&sm,nullptr,&module),"shader module");
  VkComputePipelineCreateInfo k{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};k.layout=layout_;k.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;k.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;k.stage.module=module;k.stage.pName="main";auto result=vkCreateComputePipelines(c.device,VK_NULL_HANDLE,1,&k,nullptr,&kernels_[i]);vkDestroyShaderModule(c.device,module,nullptr);check(result,"compute pipeline");
 }
 divisions_.create(c,512*4,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
 auto* t=static_cast<int*>(divisions_.mapped);std::memset(t,0,512*4);for(int i=1;i<256;i++){t[i]=int(std::nearbyint(double(255<<12)/i));t[256+i]=int(std::nearbyint(double(180<<12)/(6*i)));}
 VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,4*kSlots};VkDescriptorPoolCreateInfo dp{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};dp.maxSets=kSlots;dp.poolSizeCount=1;dp.pPoolSizes=&size;check(vkCreateDescriptorPool(c.device,&dp,nullptr,&descriptor_pool_),"descriptor pool");
 for(auto& s:pool_->slots){VkDescriptorSetAllocateInfo a{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};a.descriptorPool=descriptor_pool_;a.descriptorSetCount=1;a.pSetLayouts=&descriptors_;check(vkAllocateDescriptorSets(c.device,&a,&s.descriptors),"allocate descriptor set");
  std::array<VkDescriptorBufferInfo,4> infos{{{s.input.buffer,0,s.input.size},{s.scratch.buffer,0,s.scratch.size},{divisions_.buffer,0,divisions_.size},{s.compact.buffer,0,s.compact.size}}};std::array<VkWriteDescriptorSet,4> ws{};
  for(unsigned i=0;i<4;i++){ws[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;ws[i].dstSet=s.descriptors;ws[i].dstBinding=i;ws[i].descriptorCount=1;ws[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;ws[i].pBufferInfo=&infos[i];}vkUpdateDescriptorSets(c.device,ws.size(),ws.data(),0,nullptr);
 }
}
void Pipeline::submit(const FrameLease& f){submit_impl(f,nullptr);}
void Pipeline::submit_device_depth(const FrameLease& f,const Buffer& packed_z16){submit_impl(f,&packed_z16);}
void Pipeline::submit_impl(const FrameLease& f,const Buffer* packed_z16){
 if(!pool_->owns(f))throw std::invalid_argument("foreign or empty frame lease");auto& s=f.slot();if(s.submitted)throw std::logic_error("frame already submitted");
 const auto& calibration=s.calibration;
 if(!std::isfinite(calibration.fx)||!std::isfinite(calibration.fy)||!std::isfinite(calibration.depth_scale)||calibration.fx<=0||calibration.fy<=0||calibration.depth_scale<=0||!std::isfinite(calibration.cx)||!std::isfinite(calibration.cy))throw std::invalid_argument("invalid frame calibration");
 if(s.metadata.calibration_generation!=calibration.generation)throw std::invalid_argument("frame calibration generation mismatch");
 if(packed_z16){
  if(backend_!="vulkan")throw std::invalid_argument("device depth requires Vulkan backend");
  const size_t n=size_t(width())*height(),rgb_bytes=3*n,depth_bytes=2*n;
  if(n%4)throw std::invalid_argument("device depth copies require a four-byte aligned pixel count");
  if(s.metadata.width!=width()||s.metadata.height!=height())throw std::invalid_argument("device depth frame dimensions mismatch");
  if(packed_z16->owner!=context_.get()||packed_z16->buffer==VK_NULL_HANDLE||packed_z16->size!=depth_bytes)throw std::invalid_argument("invalid device depth source");
  if(packed_z16->buffer==s.input.buffer||packed_z16->buffer==s.upload.buffer||
     (packed_z16->memory!=VK_NULL_HANDLE&&(packed_z16->memory==s.input.memory||packed_z16->memory==s.upload.memory)))throw std::invalid_argument("device depth source aliases core storage");
  if(rgb_bytes>s.upload.size||rgb_bytes>s.input.size||depth_bytes>s.input.size-rgb_bytes)throw std::invalid_argument("device depth input range exceeds storage");
 }
 if(backend_=="cpu"){cpu_reference(s,width(),height());s.submitted=true;return;}
 // The shared command pool requires host serialization while recording. This
 // mutex never waits for the device; independent frame fences retain overlap.
 auto& c=*context_;std::lock_guard<std::mutex> command_lock(c.command_mutex);check(vkResetFences(c.device,1,&s.fence),"reset frame fence");check(vkResetCommandBuffer(s.command,0),"reset frame command");
 VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;check(vkBeginCommandBuffer(s.command,&begin),"begin frame");
 if(s.timestamps)vkCmdResetQueryPool(s.command,s.timestamps,0,9);
 auto stamp=[&](unsigned i){if(s.timestamps)vkCmdWriteTimestamp(s.command,VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,s.timestamps,i);};stamp(0);
 VkBufferCopy copy{0,0,packed_z16?size_t(width())*height()*3:s.input.size};vkCmdCopyBuffer(s.command,s.upload.buffer,s.input.buffer,1,&copy);
 if(packed_z16){VkBufferCopy depth_copy{0,size_t(width())*height()*3,packed_z16->size};vkCmdCopyBuffer(s.command,packed_z16->buffer,s.input.buffer,1,&depth_copy);}
 vkCmdFillBuffer(s.command,s.compact.buffer,0,s.compact.size,0);
 barrier(s.command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);
 Parameters p{width(),height(),width()*height(),kMaxComponents,s.calibration.depth_scale,s.calibration.fx,s.calibration.fy,s.calibration.cx,s.calibration.cy,1.2f,.012f,pool_->diagnostics?1u:0u};
 vkCmdBindDescriptorSets(s.command,VK_PIPELINE_BIND_POINT_COMPUTE,layout_,0,1,&s.descriptors,0,nullptr);vkCmdPushConstants(s.command,layout_,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(p),&p);
 auto dispatch=[&](unsigned i){vkCmdBindPipeline(s.command,VK_PIPELINE_BIND_POINT_COMPUTE,kernels_[i]);if(i==7)vkCmdDispatch(s.command,(p.width+15)/16,(p.height+15)/16,1);else vkCmdDispatch(s.command,(p.count+255)/256,1,1);barrier(s.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT);};
 // Compress reads immutable A and writes B. Connect reads frozen B and performs
 // only atomic min writes to A: active-pixel flattening plus boundary hooks.
 stamp(1);dispatch(0);stamp(2);dispatch(1);stamp(3);dispatch(7);for(unsigned i=0;i<32;i++){dispatch(3);dispatch(2);}dispatch(3);stamp(4);dispatch(4);stamp(5);dispatch(6);stamp(6);dispatch(5);stamp(7);
 // Retain the conservative compute/transfer source dependency for readback.
 barrier(s.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);
 copy.size=s.compact.size;vkCmdCopyBuffer(s.command,s.compact.buffer,s.readback.buffer,1,&copy);if(pool_->diagnostics){copy.size=s.scratch.size;vkCmdCopyBuffer(s.command,s.scratch.buffer,s.diagnostic.buffer,1,&copy);}
 barrier(s.command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);
 stamp(8);check(vkEndCommandBuffer(s.command),"end frame");VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};submit.commandBufferCount=1;submit.pCommandBuffers=&s.command;{std::lock_guard<std::mutex> queue_lock(c.queue_mutex);check(vkQueueSubmit(c.queue,1,&submit,s.fence),"submit frame");++c.submission_count;}s.submitted=true;
}
void Pipeline::finish(const FrameLease& f){
 if(!pool_->owns(f)||!f.slot().submitted)throw std::logic_error("finish requires an owned submitted frame");
 if(!ready(f))throw std::logic_error("frame completion not yet available");auto& s=f.slot();
 if(backend_=="cpu"){if(s.fault)throw std::runtime_error("CPU component reduction fault: "+std::to_string(s.fault));return;}
 decode_compact(s);
 if(s.timestamps){std::array<uint64_t,9> ticks{};check(vkGetQueryPoolResults(context_->device,s.timestamps,0,ticks.size(),sizeof(ticks),ticks.data(),sizeof(uint64_t),VK_QUERY_RESULT_64_BIT),"completed timestamp results");uint64_t mask=context_->timestamp_bits==64?UINT64_MAX:(uint64_t(1)<<context_->timestamp_bits)-1;for(unsigned i=0;i<8;i++)s.gpu_stage_ms[i]=double((ticks[i+1]-ticks[i])&mask)*context_->properties.limits.timestampPeriod/1e6;}

}
void decode_compact(Slot& s){
 auto* data=static_cast<uint32_t*>(s.readback.mapped);s.component_count=std::min(data[0],kMaxComponents);s.fault=data[1];
 if(data[0]>kMaxComponents)s.fault|=2;
 if(s.fault)throw std::runtime_error("Vulkan component reduction fault: "+std::to_string(s.fault));
 std::memcpy(s.cpu_components.data(),data+4,s.component_count*sizeof(Component));std::sort(s.cpu_components.begin(),s.cpu_components.begin()+s.component_count,[](const auto& a,const auto& b){return a.root<b.root;});
}
}
