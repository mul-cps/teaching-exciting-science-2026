// Temporary diagnostic fault injection only. Never install or use for production.
#include <vulkan/vulkan.h>
#include <dlfcn.h>
#include <stdatomic.h>
#include <unistd.h>
static PFN_vkGetPhysicalDeviceFeatures2 real_query;
static _Atomic unsigned query_count, mask_count;
// Resolve once at process load, before any measured/query operation. No GPU call.
__attribute__((constructor)) static void resolve_real_query(void) {
  real_query = (PFN_vkGetPhysicalDeviceFeatures2)dlsym(RTLD_NEXT,"vkGetPhysicalDeviceFeatures2");
  if (!real_query) _exit(127);
}
__attribute__((visibility("default"))) unsigned workshop_video_query_count(void) {
  return atomic_load_explicit(&query_count,memory_order_relaxed);
}
__attribute__((visibility("default"))) unsigned workshop_video_mask_count(void) {
  return atomic_load_explicit(&mask_count,memory_order_relaxed);
}
__attribute__((visibility("default"))) VKAPI_ATTR void VKAPI_CALL
vkGetPhysicalDeviceFeatures2(VkPhysicalDevice physical,VkPhysicalDeviceFeatures2 *features) {
  real_query(physical,features);
  atomic_fetch_add_explicit(&query_count,1,memory_order_relaxed);
  for (VkBaseOutStructure *node=(VkBaseOutStructure*)features->pNext;node;node=node->pNext) {
    if (node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR) {
      VkPhysicalDeviceVideoMaintenance1FeaturesKHR *video=(VkPhysicalDeviceVideoMaintenance1FeaturesKHR*)node;
      if (video->videoMaintenance1) {
        video->videoMaintenance1=VK_FALSE;
        atomic_fetch_add_explicit(&mask_count,1,memory_order_relaxed);
      }
    }
  }
}
