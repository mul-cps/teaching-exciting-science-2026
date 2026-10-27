#pragma once
#include <vulkan/vulkan.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
namespace so101_station_vulkan {
constexpr unsigned kSlots=3, kMaxComponents=4096;
constexpr size_t kMaxFramePixels=size_t(1920)*1080;
struct Calibration { float fx=600,fy=600,cx=320,cy=240,depth_scale=.001f; uint64_t generation=0; };
struct FrameMetadata { uint32_t width=0,height=0; uint64_t frame_number=0, timestamp_ns=0, calibration_generation=0; uint64_t depth_frame_number=0; double hardware_timestamp_ms=0,depth_hardware_timestamp_ms=0; int timestamp_domain=0,depth_timestamp_domain=0; std::string serial; };
struct Component { uint32_t root,color,count,x0,y0,x1,y1,min_z_bits,max_z_bits,closed_count; bool operator==(const Component& b) const; };
struct Context;
struct Buffer {
  VkBuffer buffer=VK_NULL_HANDLE; VkDeviceMemory memory=VK_NULL_HANDLE; void* mapped=nullptr; size_t size=0; Context* owner=nullptr;
  Buffer()=default; Buffer(const Buffer&)=delete; Buffer& operator=(const Buffer&)=delete; ~Buffer(); void reset() noexcept;
  void create(Context&,size_t,VkBufferUsageFlags,VkMemoryPropertyFlags);
};
struct VideoQueue { uint32_t family; VkQueueFlags flags; VkVideoCodecOperationFlagsKHR codecs; };
struct Context {
  VkInstance instance=VK_NULL_HANDLE; VkPhysicalDevice physical=VK_NULL_HANDLE; VkDevice device=VK_NULL_HANDLE;
  VkQueue queue=VK_NULL_HANDLE; uint32_t family=0,timestamp_bits=0; VkCommandPool commands=VK_NULL_HANDLE; VkPhysicalDeviceProperties properties{};
  // One host lock serializes every shared queue, including FFmpeg submissions.
  std::mutex queue_mutex, command_mutex;
  uint32_t instance_version=VK_API_VERSION_1_1;
  VkPhysicalDeviceFeatures2 enabled_features{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
  VkPhysicalDeviceVulkan11Features enabled11{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};
  VkPhysicalDeviceVulkan12Features enabled12{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};
  VkPhysicalDeviceVulkan13Features enabled13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
  VkPhysicalDeviceVideoMaintenance1FeaturesKHR enabled_video_maintenance1{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR};
  std::vector<std::string> enabled_extensions; std::vector<VideoQueue> video_queues;
  bool video_capable=false; std::string video_reason;
  std::atomic<uint64_t> allocation_count{0},submission_count{0}; Context(); ~Context(); Context(const Context&)=delete;
  static std::shared_ptr<Context> shared();
};
struct Slot {
  std::atomic<unsigned> references{0}; bool submitted=false; VkFence fence=VK_NULL_HANDLE; VkCommandBuffer command=VK_NULL_HANDLE;
  VkQueryPool timestamps=VK_NULL_HANDLE; std::array<double,8> gpu_stage_ms{};
  VkDescriptorSet descriptors=VK_NULL_HANDLE;
  Buffer upload, input, scratch, compact, readback, diagnostic;
  std::vector<uint8_t> cpu_masks; std::vector<uint32_t> cpu_parents; std::vector<float> cpu_depth, cpu_xyz;
  std::vector<uint8_t> cpu_work,cpu_dilated,cpu_morph_counts; std::vector<uint32_t> cpu_morph_roots;
  std::array<Component,kMaxComponents> cpu_components{}; unsigned component_count=0; uint32_t fault=0;
  FrameMetadata metadata; Calibration calibration;
};
class FramePool;
// Weak control-block identity only: reserves no frame slot and keeps no pool alive.
class FramePoolToken {
 friend class FramePool;friend class FrameLease;
 std::weak_ptr<FramePool> pool_;
 explicit FramePoolToken(std::weak_ptr<FramePool> p):pool_(std::move(p)){}
public:
 FramePoolToken()=default;explicit operator bool()const{return !pool_.expired();}
};
class FrameLease {
  friend class FramePool; std::shared_ptr<FramePool> pool_; unsigned index_=0;
  FrameLease(std::shared_ptr<FramePool>,unsigned);
public:
  FrameLease()=default; FrameLease(const FrameLease&); FrameLease(FrameLease&&) noexcept;
  FrameLease& operator=(FrameLease); ~FrameLease(); void swap(FrameLease&) noexcept;
  bool belongs_to(const FramePoolToken& token)const{
   return pool_&&!token.pool_.expired()&&!pool_.owner_before(token.pool_)&&!token.pool_.owner_before(pool_);
  }
  explicit operator bool() const { return bool(pool_); } Slot& slot() const; unsigned index()const{return index_;} std::shared_ptr<Context> context()const;
};
class FramePool:public std::enable_shared_from_this<FramePool> {
  friend class FrameLease;
public:
  std::shared_ptr<Context> context; std::array<Slot,kSlots> slots; unsigned width,height; bool diagnostics;
  FramePoolToken source_token(){return FramePoolToken(weak_from_this());}
  void release_resources() noexcept; bool owns(const FrameLease& f)const{return f.pool_.get()==this;}
  FramePool(unsigned,unsigned,std::shared_ptr<Context>,bool,bool profile=false); ~FramePool(); FrameLease acquire(); bool ready(const FrameLease&) const;
};
class Pipeline {
  std::shared_ptr<Context> context_; std::shared_ptr<FramePool> pool_;
  VkDescriptorSetLayout descriptors_=VK_NULL_HANDLE; VkDescriptorPool descriptor_pool_=VK_NULL_HANDLE;
  VkPipelineLayout layout_=VK_NULL_HANDLE; std::array<VkPipeline,8> kernels_{}; Buffer divisions_;
  std::string backend_,reason_,shader_dir_; void create_gpu(); void destroy_gpu() noexcept;
  void submit_impl(const FrameLease&,const Buffer* packed_z16);
public:
  Pipeline(unsigned width,unsigned height,const std::string& mode,const std::string& shader_dir,bool diagnostics=false,bool profile=false);
  ~Pipeline(); FrameLease acquire(){return pool_->acquire();} void submit(const FrameLease&); bool ready(const FrameLease& f)const{return pool_->ready(f);}
  // Low-level borrowed device buffer. The producer must already have submitted
  // its compute-write -> transfer-read dependency on this Context's shared queue.
  // Retain the actual source lease (not just this Buffer reference) through the
  // core frame fence, then check producer validity/faults before using results.
  // Copies only RGB from upload; tightly packed Z16 stays device resident.
  // Pixel count must be divisible by four for Vulkan buffer-copy alignment.
  void submit_device_depth(const FrameLease&,const Buffer& packed_z16);
  FramePoolToken source_token()const{return pool_->source_token();}
  bool owns(const FrameLease& f)const{return pool_->owns(f);}
  void finish(const FrameLease&); const std::string& backend()const{return backend_;} const std::string& fallback_reason()const{return reason_;}
  std::shared_ptr<Context> context()const{return context_;} unsigned width()const{return pool_->width;} unsigned height()const{return pool_->height;}
};
void decode_compact(Slot&);
void cpu_morphology(Slot&,unsigned,unsigned);
void cpu_reference(Slot&,unsigned,unsigned);
void cpu_complete_reference(Slot&,unsigned,unsigned);
const std::array<int,512>& cpu_divisions();
#ifdef RESIDENT_SLANG_CPU
void cpu_slang(Slot&,unsigned,unsigned);
#endif
// Input layout is tightly packed native RGB8 followed by little-endian native Z16.
inline size_t input_bytes(unsigned w,unsigned h){return (size_t(w)*h*5+3)&~size_t(3);}
inline size_t scratch_bytes(unsigned w,unsigned h){return size_t(w)*h*72;}
inline size_t morphology_bytes(unsigned w,unsigned h){return size_t(w)*h*10*4;}
inline size_t compact_bytes(){return 16+kMaxComponents*sizeof(Component);}
}
