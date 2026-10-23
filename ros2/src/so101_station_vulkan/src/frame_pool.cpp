#include "so101_station_vulkan/pipeline.hpp"
#include <stdexcept>
#include <utility>
namespace so101_station_vulkan {
FrameLease::FrameLease(std::shared_ptr<FramePool> p,unsigned i):pool_(std::move(p)),index_(i){}
FrameLease::FrameLease(const FrameLease& f):pool_(f.pool_),index_(f.index_){if(pool_)++slot().references;}
FrameLease::FrameLease(FrameLease&& f)noexcept:pool_(std::move(f.pool_)),index_(f.index_){}
FrameLease& FrameLease::operator=(FrameLease f){swap(f);return *this;}
FrameLease::~FrameLease(){if(pool_)--slot().references;}
void FrameLease::swap(FrameLease& f)noexcept{pool_.swap(f.pool_);std::swap(index_,f.index_);}
Slot& FrameLease::slot()const{if(!pool_)throw std::logic_error("empty frame lease");return pool_->slots[index_];}
std::shared_ptr<Context> FrameLease::context()const{return pool_?pool_->context:nullptr;}
FramePool::FramePool(unsigned w,unsigned h,std::shared_ptr<Context> c,bool debug,bool profile):context(std::move(c)),width(w),height(h),diagnostics(debug){
  if(!w||!h||size_t(w)*h>640*480)throw std::invalid_argument("resident frame dimensions must fit 640x480");
  try{for(auto& s:slots){s.metadata.width=w;s.metadata.height=h;size_t n=size_t(w)*h;s.cpu_masks.resize(n);s.cpu_parents.resize(n);s.cpu_depth.resize(n);s.cpu_xyz.resize(n*4);s.cpu_work.resize(n);s.cpu_dilated.resize(n);s.cpu_morph_counts.resize(n);if(!context)s.cpu_morph_roots.resize(n*9);
    if(context){
      auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
      s.upload.create(*context,input_bytes(w,h),VK_BUFFER_USAGE_TRANSFER_SRC_BIT,host);
      s.input.create(*context,input_bytes(w,h),VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
      s.scratch.create(*context,scratch_bytes(w,h)+(debug?morphology_bytes(w,h):0),VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
      s.compact.create(*context,compact_bytes(),VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
      s.readback.create(*context,compact_bytes(),VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
      if(debug)s.diagnostic.create(*context,scratch_bytes(w,h)+morphology_bytes(w,h),VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
      VkFenceCreateInfo f{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};f.flags=VK_FENCE_CREATE_SIGNALED_BIT;
      if(vkCreateFence(context->device,&f,nullptr,&s.fence)!=VK_SUCCESS)throw std::runtime_error("frame fence creation failed");
      if(profile){if(!context->timestamp_bits)throw std::runtime_error("queue timestamps unsupported");VkQueryPoolCreateInfo q{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};q.queryType=VK_QUERY_TYPE_TIMESTAMP;q.queryCount=9;if(vkCreateQueryPool(context->device,&q,nullptr,&s.timestamps)!=VK_SUCCESS)throw std::runtime_error("timestamp query creation failed");}
      VkCommandBufferAllocateInfo a{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};a.commandPool=context->commands;a.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;a.commandBufferCount=1;
      std::lock_guard<std::mutex> lock(context->command_mutex);
      if(vkAllocateCommandBuffers(context->device,&a,&s.command)!=VK_SUCCESS)throw std::runtime_error("frame command creation failed");
    }else{s.upload.size=input_bytes(w,h);s.upload.mapped=new uint8_t[s.upload.size]();}
  }}catch(...){release_resources();throw;}
}
FramePool::~FramePool(){release_resources();}
void FramePool::release_resources() noexcept{for(auto& s:slots){if(context){if(s.submitted)vkWaitForFences(context->device,1,&s.fence,VK_TRUE,UINT64_MAX);if(s.timestamps)vkDestroyQueryPool(context->device,s.timestamps,nullptr);s.timestamps=VK_NULL_HANDLE;if(s.fence)vkDestroyFence(context->device,s.fence,nullptr);if(s.command){std::lock_guard<std::mutex> lock(context->command_mutex);vkFreeCommandBuffers(context->device,context->commands,1,&s.command);}s.fence=VK_NULL_HANDLE;s.command=VK_NULL_HANDLE;}else {delete[] static_cast<uint8_t*>(s.upload.mapped);s.upload.mapped=nullptr;}}}

bool FramePool::ready(const FrameLease& f)const{if(!owns(f))throw std::invalid_argument("foreign or empty frame lease");auto& s=f.slot();if(!context||!s.submitted)return true;auto r=vkGetFenceStatus(context->device,s.fence);if(r==VK_NOT_READY)return false;if(r!=VK_SUCCESS)throw std::runtime_error("Vulkan frame fence error: "+std::to_string(r));return true;}
FrameLease FramePool::acquire(){for(unsigned i=0;i<kSlots;i++){auto& s=slots[i];unsigned expected=0;if(!s.references.compare_exchange_strong(expected,1))continue;
  if(context&&s.submitted){auto r=vkGetFenceStatus(context->device,s.fence);if(r==VK_NOT_READY){s.references=0;continue;}if(r!=VK_SUCCESS){s.references=0;throw std::runtime_error("Vulkan completed frame error");}}
  s.submitted=false;s.component_count=0;s.fault=0;return FrameLease(shared_from_this(),i);
}return {};}
bool Component::operator==(const Component& b)const{return root==b.root&&color==b.color&&count==b.count&&x0==b.x0&&y0==b.y0&&x1==b.x1&&y1==b.y1&&min_z_bits==b.min_z_bits&&max_z_bits==b.max_z_bits&&closed_count==b.closed_count;}
}
