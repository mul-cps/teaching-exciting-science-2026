// Executes actual cold Context negotiation using a synthetic Vulkan API only.
#include "so101_station_vulkan/pipeline.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace so101_station_vulkan;
namespace {
bool advertised=true,feature=true,fail_first=false,fp64_supported=true,int64_supported=true;unsigned create_calls=0,query_calls=0,compute_queries=0;
void require(bool value,const char* message="cold Context feature contract failed"){if(!value)throw std::runtime_error(message);}
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
void vkGetPhysicalDeviceFeatures(VkPhysicalDevice,VkPhysicalDeviceFeatures* out){++compute_queries;*out={};out->shaderFloat64=fp64_supported;out->shaderInt64=int64_supported;}
void vkGetPhysicalDeviceFeatures2(VkPhysicalDevice,VkPhysicalDeviceFeatures2* root){++query_calls;root->features.shaderFloat64=fp64_supported;root->features.shaderInt64=int64_supported;bool saw_maintenance=false;
 for(auto* node=static_cast<VkBaseOutStructure*>(root->pNext);node;node=node->pNext){
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES)reinterpret_cast<VkPhysicalDeviceVulkan12Features*>(node)->timelineSemaphore=VK_TRUE;
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES)reinterpret_cast<VkPhysicalDeviceVulkan13Features*>(node)->synchronization2=VK_TRUE;
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR){saw_maintenance=true;reinterpret_cast<VkPhysicalDeviceVideoMaintenance1FeaturesKHR*>(node)->videoMaintenance1=feature;}
 }require(saw_maintenance==advertised);
}
void vkGetPhysicalDeviceQueueFamilyProperties2(VkPhysicalDevice,uint32_t* count,VkQueueFamilyProperties2* out){*count=1;if(out){out->queueFamilyProperties.queueFlags=VK_QUEUE_COMPUTE_BIT|VK_QUEUE_VIDEO_ENCODE_BIT_KHR|VK_QUEUE_VIDEO_DECODE_BIT_KHR;auto* video=static_cast<VkQueueFamilyVideoPropertiesKHR*>(out->pNext);video->videoCodecOperations=VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR|VK_VIDEO_CODEC_OPERATION_DECODE_H264_BIT_KHR;}}
VkResult vkCreateDevice(VkPhysicalDevice,const VkDeviceCreateInfo* info,const VkAllocationCallbacks*,VkDevice* out){++create_calls;bool video=advertised&&feature&&(!fail_first||create_calls==1);
 require(bool(info->pNext)==video);if(video){require(!info->pEnabledFeatures,"Features2 chain must not also use legacy enabled features");require(static_cast<const VkPhysicalDeviceFeatures2*>(info->pNext)->features.shaderFloat64==fp64_supported);require(static_cast<const VkPhysicalDeviceFeatures2*>(info->pNext)->features.shaderInt64==int64_supported,"video create lost supported Int64 or enabled unsupported Int64");require(info->enabledExtensionCount==6);bool enabled=false;for(unsigned i=0;i<info->enabledExtensionCount;i++)enabled|=std::strcmp(info->ppEnabledExtensionNames[i],"VK_KHR_video_maintenance1")==0;require(enabled);}else{require(!info->enabledExtensionCount&&info->queueCreateInfoCount==1);require(info->pEnabledFeatures,"compute fallback omitted baseline features");require(info->pEnabledFeatures->shaderFloat64==fp64_supported,"compute fallback lost supported FP64 or enabled unsupported FP64");require(info->pEnabledFeatures->shaderInt64==int64_supported,"compute fallback lost supported Int64 or enabled unsupported Int64");require(!info->pEnabledFeatures->robustBufferAccess,"compute fallback enabled unrelated features");}
 if(fail_first&&create_calls==1){*out=VK_NULL_HANDLE;return VK_ERROR_FEATURE_NOT_PRESENT;}*out=(VkDevice)3;return VK_SUCCESS;
}
void vkDestroyDevice(VkDevice,const VkAllocationCallbacks*){}
void vkGetDeviceQueue(VkDevice,uint32_t,uint32_t,VkQueue* out){*out=(VkQueue)4;}
VkResult vkCreateCommandPool(VkDevice,const VkCommandPoolCreateInfo*,const VkAllocationCallbacks*,VkCommandPool* out){*out=(VkCommandPool)5;return VK_SUCCESS;}
void vkDestroyCommandPool(VkDevice,VkCommandPool,const VkAllocationCallbacks*){}
VkResult vkDeviceWaitIdle(VkDevice){return VK_SUCCESS;}
}
int main(){
 for(unsigned scenario:{4u,5u,2u,3u,0u,1u,6u}){
  advertised=scenario!=2&&scenario!=4&&scenario!=5;feature=scenario!=1;fail_first=scenario==3;fp64_supported=scenario!=4;int64_supported=scenario!=5&&scenario!=6;create_calls=query_calls=compute_queries=0;
  Context context;require(compute_queries==1&&query_calls==1&&create_calls==(fail_first?2u:1u));bool enabled=scenario==0||scenario==6;require(context.video_capable==enabled);
  require(bool(context.enabled_features.features.shaderFloat64)==fp64_supported&&bool(context.enabled_features.features.shaderInt64)==int64_supported,"reported compute features differ from supported enabled features");
  if(enabled){require(context.enabled_features.pNext==&context.enabled11&&context.enabled11.pNext==&context.enabled12&&context.enabled12.pNext==&context.enabled13&&context.enabled13.pNext==&context.enabled_video_maintenance1);require(context.enabled_video_maintenance1.videoMaintenance1==VK_TRUE);auto borrowed=context.enabled_features;require(borrowed.pNext==&context.enabled11);}
  else{require(context.enabled_features.pNext==nullptr&&bool(context.enabled_features.features.shaderFloat64)==fp64_supported&&bool(context.enabled_features.features.shaderInt64)==int64_supported&&!context.enabled_features.features.robustBufferAccess&&context.enabled13.pNext==nullptr&&!context.enabled_video_maintenance1.videoMaintenance1&&context.enabled_extensions.empty()&&context.video_queues.empty());}
 }
 std::cout<<"PASS actual cold Context: maintenance1 success, absent feature/extension, device-create fallback, supported/unsupported compute FP64 and Int64; no Vulkan driver\n";
}
