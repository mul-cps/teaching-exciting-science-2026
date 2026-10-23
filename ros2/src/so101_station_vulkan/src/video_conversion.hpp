#pragma once
#include "so101_station_vulkan/video.hpp"
extern "C" {
#include <libavutil/hwcontext_vulkan.h>
}
namespace so101_station_vulkan {
// Owns GPU conversion resources; does not own any FFmpeg image/semaphore.
class VideoConversion {
 struct Resources {Buffer nv12;VkDescriptorSet descriptors=VK_NULL_HANDLE;VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;bool submitted=false;FrameLease input;};
 std::shared_ptr<Context> context_;unsigned width_,height_;
 std::array<Resources,kSlots> slots_;
 VkCommandPool commands_=VK_NULL_HANDLE;
 VkDescriptorSetLayout descriptors_=VK_NULL_HANDLE;VkDescriptorPool descriptor_pool_=VK_NULL_HANDLE;
 VkPipelineLayout layout_=VK_NULL_HANDLE;VkPipeline kernel_=VK_NULL_HANDLE;
 void cleanup()noexcept;
public:
 VideoConversion(std::shared_ptr<Context>,unsigned,unsigned);
 ~VideoConversion();bool ready(unsigned);
 int submit(unsigned,const FrameLease&,AVFrame*);
};
}
