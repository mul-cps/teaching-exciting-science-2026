// Executes actual cold Context negotiation using a synthetic Vulkan API only.
#include "so101_station_vulkan/pipeline.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace so101_station_vulkan;
namespace {
bool advertised=true,feature=true,fail_first=false;unsigned create_calls=0,query_calls=0;
void require(bool value){if(!value)throw std::runtime_error("cold Context feature contract failed");}
const char* extensions[]={"VK_KHR_video_queue","VK_KHR_video_encode_queue","VK_KHR_video_decode_queue","VK_KHR_video_encode_h264","VK_KHR_video_decode_h264","VK_KHR_video_maintenance1"};
VkResult version(uint32_t* result){*result=VK_API_VERSION_1_3;return VK_SUCCESS;}
}
extern "C" {
PFN_vkVoidFunction vkGetInstanceProcAddr(VkInstance,const char* name){require(std::strcmp(name,"vkEnumerateInstanceVersion")==0);return reinterpret_cast<PFN_vkVoidFunction>(version);}
VkResult vkCreateInstance(const VkInstanceCreateInfo*,const VkAllocationCallbacks*,VkInstance* out){*out=(VkInstance)1;return VK_SUCCESS;}
void vkDestroyInstance(VkInstance,const VkAllocationCallbacks*){}
VkResult vkEnumeratePhysicalDevices(VkInstance,uint32_t* count,VkPhysicalDevice* out){*count=1;if(out)*out=(VkPhysicalDevice)2;return VK_SUCCESS;}
void vkGetPhysicalDeviceProperties(VkPhysicalDevice,VkPhysicalDeviceProperties* out){*out={};out->apiVersion=VK_API_VERSION_1_3;out->deviceType=VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;std::strcpy(out->deviceName,"synthetic cold context fixture");}
void vkGetPhysicalDeviceQueueFamilyProperties(VkPhysicalDevice,uint32_t* count,VkQueueFamilyProperties* out){*count=1;if(out){*out={};out->queueFlags=VK_QUEUE_COMPUTE_BIT;out->timestampValidBits=64;out->queueCount=1;}}
VkResult vkEnumerateDeviceExtensionProperties(VkPhysicalDevice,const char*,uint32_t* count,VkExtensionProperties* out){*count=advertised?6:5;if(out)for(unsigned i=0;i<*count;i++){out[i]={};std::strcpy(out[i].extensionName,extensions[i]);}return VK_SUCCESS;}
void vkGetPhysicalDeviceFeatures2(VkPhysicalDevice,VkPhysicalDeviceFeatures2* root){++query_calls;root->features.shaderFloat64=VK_TRUE;bool saw_maintenance=false;
 for(auto* node=static_cast<VkBaseOutStructure*>(root->pNext);node;node=node->pNext){
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES)reinterpret_cast<VkPhysicalDeviceVulkan12Features*>(node)->timelineSemaphore=VK_TRUE;
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES)reinterpret_cast<VkPhysicalDeviceVulkan13Features*>(node)->synchronization2=VK_TRUE;
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR){saw_maintenance=true;reinterpret_cast<VkPhysicalDeviceVideoMaintenance1FeaturesKHR*>(node)->videoMaintenance1=feature;}
 }require(saw_maintenance==advertised);
}
void vkGetPhysicalDeviceQueueFamilyProperties2(VkPhysicalDevice,uint32_t* count,VkQueueFamilyProperties2* out){*count=1;if(out){out->queueFamilyProperties.queueFlags=VK_QUEUE_COMPUTE_BIT|VK_QUEUE_VIDEO_ENCODE_BIT_KHR|VK_QUEUE_VIDEO_DECODE_BIT_KHR;auto* video=static_cast<VkQueueFamilyVideoPropertiesKHR*>(out->pNext);video->videoCodecOperations=VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR|VK_VIDEO_CODEC_OPERATION_DECODE_H264_BIT_KHR;}}
VkResult vkCreateDevice(VkPhysicalDevice,const VkDeviceCreateInfo* info,const VkAllocationCallbacks*,VkDevice* out){++create_calls;bool video=advertised&&feature&&(!fail_first||create_calls==1);
 require(bool(info->pNext)==video);if(video){require(info->enabledExtensionCount==6);bool enabled=false;for(unsigned i=0;i<info->enabledExtensionCount;i++)enabled|=std::strcmp(info->ppEnabledExtensionNames[i],"VK_KHR_video_maintenance1")==0;require(enabled);}else require(!info->enabledExtensionCount&&info->queueCreateInfoCount==1);
 if(fail_first&&create_calls==1){*out=VK_NULL_HANDLE;return VK_ERROR_FEATURE_NOT_PRESENT;}*out=(VkDevice)3;return VK_SUCCESS;
}
void vkDestroyDevice(VkDevice,const VkAllocationCallbacks*){}
void vkGetDeviceQueue(VkDevice,uint32_t,uint32_t,VkQueue* out){*out=(VkQueue)4;}
VkResult vkCreateCommandPool(VkDevice,const VkCommandPoolCreateInfo*,const VkAllocationCallbacks*,VkCommandPool* out){*out=(VkCommandPool)5;return VK_SUCCESS;}
void vkDestroyCommandPool(VkDevice,VkCommandPool,const VkAllocationCallbacks*){}
VkResult vkDeviceWaitIdle(VkDevice){return VK_SUCCESS;}
}
int main(){
 for(unsigned scenario=0;scenario<4;scenario++){
  advertised=scenario!=2;feature=scenario!=1;fail_first=scenario==3;create_calls=query_calls=0;
  Context context;require(query_calls==1&&create_calls==(fail_first?2u:1u));bool enabled=scenario==0;require(context.video_capable==enabled);
  if(enabled){require(context.enabled_features.pNext==&context.enabled11&&context.enabled11.pNext==&context.enabled12&&context.enabled12.pNext==&context.enabled13&&context.enabled13.pNext==&context.enabled_video_maintenance1);require(context.enabled_video_maintenance1.videoMaintenance1==VK_TRUE);auto borrowed=context.enabled_features;require(borrowed.pNext==&context.enabled11);}
  else{require(context.enabled_features.pNext==nullptr&&!context.enabled_features.features.shaderFloat64&&context.enabled13.pNext==nullptr&&!context.enabled_video_maintenance1.videoMaintenance1&&context.enabled_extensions.empty()&&context.video_queues.empty());}
 }
 std::cout<<"PASS actual cold Context: maintenance1 success, absent feature, absent extension, device-create fallback; no Vulkan driver\n";
}
