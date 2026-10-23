// Actual Context implementation, synthetic Vulkan API: never link a loader.
#include "so101_station_vulkan/pipeline.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>

using namespace so101_station_vulkan;
namespace {
struct Calls { uint64_t total=0,features=0,features2=0,creates=0,waits=0; } calls;
uint64_t heap=0;
struct Scenario { const char* name; bool api13,advertised,maintenance,fail_first,fp64; };
Scenario scenario{};
constexpr unsigned hot_acquisitions=200000, cold_contexts=1000, trials=7;
const char* extensions[]={"VK_KHR_video_queue","VK_KHR_video_encode_queue","VK_KHR_video_decode_queue","VK_KHR_video_encode_h264","VK_KHR_video_decode_h264","VK_KHR_video_maintenance1"};
void require(bool value,const char* why="synthetic Context performance contract failed") {
  if(!value)throw std::runtime_error(why);
}
VkResult version(uint32_t* result){++calls.total;*result=VK_API_VERSION_1_3;return VK_SUCCESS;}
bool video_possible(){return scenario.api13&&scenario.advertised&&scenario.maintenance;}
}
void* operator new(size_t size){if(void* p=std::malloc(size?size:1)){++heap;return p;}throw std::bad_alloc();}
void* operator new[](size_t size){return ::operator new(size);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
void* operator new(size_t size,std::align_val_t align){void* p=nullptr;if(posix_memalign(&p,size_t(align),size?size:1))throw std::bad_alloc();++heap;return p;}
void* operator new[](size_t size,std::align_val_t align){return ::operator new(size,align);}
void operator delete(void* p,std::align_val_t)noexcept{std::free(p);}void operator delete[](void* p,std::align_val_t)noexcept{std::free(p);}
void operator delete(void* p,size_t,std::align_val_t)noexcept{std::free(p);}void operator delete[](void* p,size_t,std::align_val_t)noexcept{std::free(p);}

extern "C" {
PFN_vkVoidFunction vkGetInstanceProcAddr(VkInstance,const char* name){++calls.total;require(std::strcmp(name,"vkEnumerateInstanceVersion")==0);return reinterpret_cast<PFN_vkVoidFunction>(version);}
VkResult vkCreateInstance(const VkInstanceCreateInfo*,const VkAllocationCallbacks*,VkInstance* out){++calls.total;*out=(VkInstance)1;return VK_SUCCESS;}
void vkDestroyInstance(VkInstance,const VkAllocationCallbacks*){++calls.total;}
VkResult vkEnumeratePhysicalDevices(VkInstance,uint32_t* count,VkPhysicalDevice* out){++calls.total;*count=1;if(out)*out=(VkPhysicalDevice)2;return VK_SUCCESS;}
void vkGetPhysicalDeviceProperties(VkPhysicalDevice,VkPhysicalDeviceProperties* out){++calls.total;*out={};out->apiVersion=scenario.api13?VK_API_VERSION_1_3:VK_API_VERSION_1_2;out->deviceType=VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;std::strcpy(out->deviceName,"synthetic Context fixture");}
void vkGetPhysicalDeviceQueueFamilyProperties(VkPhysicalDevice,uint32_t* count,VkQueueFamilyProperties* out){++calls.total;*count=1;if(out){*out={};out->queueFlags=VK_QUEUE_COMPUTE_BIT;out->timestampValidBits=64;out->queueCount=1;}}
VkResult vkEnumerateDeviceExtensionProperties(VkPhysicalDevice,const char*,uint32_t* count,VkExtensionProperties* out){++calls.total;*count=scenario.advertised?6:5;if(out)for(unsigned i=0;i<*count;i++){out[i]={};std::strcpy(out[i].extensionName,extensions[i]);}return VK_SUCCESS;}
void vkGetPhysicalDeviceFeatures(VkPhysicalDevice,VkPhysicalDeviceFeatures* out){++calls.total;++calls.features;*out={};out->shaderFloat64=scenario.fp64;}
void vkGetPhysicalDeviceFeatures2(VkPhysicalDevice,VkPhysicalDeviceFeatures2* root){++calls.total;++calls.features2;root->features.shaderFloat64=scenario.fp64;bool saw=false;
 for(auto* node=static_cast<VkBaseOutStructure*>(root->pNext);node;node=node->pNext){
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES)reinterpret_cast<VkPhysicalDeviceVulkan12Features*>(node)->timelineSemaphore=VK_TRUE;
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES)reinterpret_cast<VkPhysicalDeviceVulkan13Features*>(node)->synchronization2=VK_TRUE;
  if(node->sType==VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR){saw=true;reinterpret_cast<VkPhysicalDeviceVideoMaintenance1FeaturesKHR*>(node)->videoMaintenance1=scenario.maintenance;}
 }require(saw==scenario.advertised);
}
void vkGetPhysicalDeviceQueueFamilyProperties2(VkPhysicalDevice,uint32_t* count,VkQueueFamilyProperties2* out){++calls.total;*count=1;if(out){out->queueFamilyProperties.queueFlags=VK_QUEUE_COMPUTE_BIT|VK_QUEUE_VIDEO_ENCODE_BIT_KHR|VK_QUEUE_VIDEO_DECODE_BIT_KHR;auto* video=static_cast<VkQueueFamilyVideoPropertiesKHR*>(out->pNext);video->videoCodecOperations=VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR|VK_VIDEO_CODEC_OPERATION_DECODE_H264_BIT_KHR;}}
VkResult vkCreateDevice(VkPhysicalDevice,const VkDeviceCreateInfo* info,const VkAllocationCallbacks*,VkDevice* out){++calls.total;++calls.creates;
 const bool video=video_possible()&&(!scenario.fail_first||calls.creates%2==1);
 require(bool(info->pNext)==video,"wrong Features2/legacy branch");
 if(video){require(!info->pEnabledFeatures,"Features2 and legacy enabled feature pointers overlap");require(static_cast<const VkPhysicalDeviceFeatures2*>(info->pNext)->features.shaderFloat64==scenario.fp64);require(info->enabledExtensionCount==6);}
 else{VkPhysicalDeviceFeatures expected{};expected.shaderFloat64=scenario.fp64;require(info->pEnabledFeatures,"missing legacy feature request");require(std::memcmp(info->pEnabledFeatures,&expected,sizeof(expected))==0,"enabled unsupported/unrelated feature or omitted FP64");require(!info->enabledExtensionCount&&info->queueCreateInfoCount==1);}
 if(scenario.fail_first&&calls.creates%2==1){*out=VK_NULL_HANDLE;return VK_ERROR_FEATURE_NOT_PRESENT;}*out=(VkDevice)3;return VK_SUCCESS;
}
void vkDestroyDevice(VkDevice,const VkAllocationCallbacks*){++calls.total;}
void vkGetDeviceQueue(VkDevice,uint32_t,uint32_t,VkQueue* out){++calls.total;*out=(VkQueue)4;}
VkResult vkCreateCommandPool(VkDevice,const VkCommandPoolCreateInfo*,const VkAllocationCallbacks*,VkCommandPool* out){++calls.total;*out=(VkCommandPool)5;return VK_SUCCESS;}
void vkDestroyCommandPool(VkDevice,VkCommandPool,const VkAllocationCallbacks*){++calls.total;}
VkResult vkDeviceWaitIdle(VkDevice){++calls.total;++calls.waits;return VK_SUCCESS;}
}

int main(){try{
 const std::array<Scenario,5> scenarios{{
  {"physical_api12_fp64",false,false,false,false,true},
  {"physical_api12_no_fp64",false,false,false,false,false},
  {"missing_video_extension_fp64",true,false,true,false,true},
  {"video_fp64",true,true,true,false,true},
  {"video_create_fallback_fp64",true,true,true,true,true}}};
 std::cout<<"{\"schema\":\"context-features-perf-v1\",\"trials\":"<<trials<<",\"scenarios\":[";
 bool first=true;
 for(const auto& chosen:scenarios){
  scenario=chosen;calls={};auto owner=Context::shared();
  const bool video=video_possible()&&!scenario.fail_first;
  require(owner->video_capable==video);require(bool(owner->enabled_features.features.shaderFloat64)==scenario.fp64,"reported FP64 differs from supported/enabled feature");
  require(calls.features==1&&calls.features2==unsigned(scenario.api13)&&calls.creates==(scenario.fail_first?2u:1u));
  const auto cold_initial=calls;
  // Retain the owner; cache-hit acquisitions must never reconstruct Context.
  for(unsigned i=0;i<1000;++i)require(Context::shared().get()==owner.get());
  std::array<double,trials> hot{},cold{};
  uint64_t hot_heap=0,hot_api_calls=0,hot_feature_queries=0,hot_device_creates=0,hot_waits=0;
  for(auto& ns:hot){const auto before=calls;const auto allocations=heap;const auto start=std::chrono::steady_clock::now();
   for(unsigned i=0;i<hot_acquisitions;++i)require(Context::shared().get()==owner.get());
   ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/hot_acquisitions;
   hot_heap+=heap-allocations;hot_api_calls+=calls.total-before.total;hot_feature_queries+=calls.features-before.features+calls.features2-before.features2;hot_device_creates+=calls.creates-before.creates;hot_waits+=calls.waits-before.waits;
  }
  require(!hot_heap&&!hot_api_calls&&!hot_feature_queries&&!hot_device_creates&&!hot_waits,"hot cache acquisition allocates, queries, creates or waits");
  owner.reset();
  // Constructor/destructor cost under the synthetic API, not driver latency.
  for(auto& ns:cold){const auto before=calls;const auto start=std::chrono::steady_clock::now();
   for(unsigned i=0;i<cold_contexts;++i){Context context;require(bool(context.enabled_features.features.shaderFloat64)==scenario.fp64);}
   ns=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/cold_contexts;
   require(calls.features-before.features==cold_contexts);require(calls.features2-before.features2==cold_contexts*unsigned(scenario.api13));require(calls.creates-before.creates==cold_contexts*(scenario.fail_first?2u:1u));require(calls.waits-before.waits==cold_contexts);
  }
  auto sorted_hot=hot,sorted_cold=cold;std::sort(sorted_hot.begin(),sorted_hot.end());std::sort(sorted_cold.begin(),sorted_cold.end());
  if(!first)std::cout<<',';first=false;
  std::cout<<"{\"name\":\""<<scenario.name<<"\",\"fp64_supported_and_enabled\":"<<(scenario.fp64?"true":"false")<<",\"video_enabled\":"<<(video?"true":"false")
   <<",\"cold_feature_queries\":"<<cold_initial.features<<",\"cold_features2_queries\":"<<cold_initial.features2<<",\"cold_device_creates\":"<<cold_initial.creates
   <<",\"hot_acquisitions_per_trial\":"<<hot_acquisitions<<",\"hot_heap_allocations\":"<<hot_heap<<",\"hot_vulkan_calls\":"<<hot_api_calls<<",\"hot_feature_queries\":"<<hot_feature_queries<<",\"hot_device_creates\":"<<hot_device_creates<<",\"hot_waits\":"<<hot_waits
   <<",\"median_hot_ns\":"<<sorted_hot[trials/2]<<",\"median_cold_context_ns\":"<<sorted_cold[trials/2]<<",\"cold_contexts_per_trial\":"<<cold_contexts<<",\"hot_samples_ns\":[";
  for(unsigned i=0;i<trials;++i){if(i)std::cout<<',';std::cout<<hot[i];}std::cout<<"],\"cold_samples_ns\":[";
  for(unsigned i=0;i<trials;++i){if(i)std::cout<<',';std::cout<<cold[i];}std::cout<<"]}";
 }
 std::cout<<"],\"scope\":\"actual Context; synthetic Vulkan only; no loader, driver, SDK, GPU, image work or contention\"}\n";
 return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
