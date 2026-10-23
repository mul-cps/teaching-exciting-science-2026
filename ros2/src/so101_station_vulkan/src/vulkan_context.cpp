#include "so101_station_vulkan/pipeline.hpp"
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <algorithm>
namespace so101_station_vulkan {
static void check(VkResult r,const char* op){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(op)+": "+std::to_string(r));}
Buffer::~Buffer(){reset();}
void Buffer::reset() noexcept{if(owner){if(mapped)vkUnmapMemory(owner->device,memory);if(buffer)vkDestroyBuffer(owner->device,buffer,nullptr);if(memory)vkFreeMemory(owner->device,memory,nullptr);}buffer=VK_NULL_HANDLE;memory=VK_NULL_HANDLE;mapped=nullptr;size=0;owner=nullptr;}

void Buffer::create(Context& c,size_t bytes,VkBufferUsageFlags usage,VkMemoryPropertyFlags flags){
  owner=&c;size=bytes;VkBufferCreateInfo b{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};b.size=bytes;b.usage=usage;b.sharingMode=VK_SHARING_MODE_EXCLUSIVE;
  check(vkCreateBuffer(c.device,&b,nullptr,&buffer),"vkCreateBuffer");VkMemoryRequirements req;vkGetBufferMemoryRequirements(c.device,buffer,&req);
  VkPhysicalDeviceMemoryProperties m;vkGetPhysicalDeviceMemoryProperties(c.physical,&m);uint32_t t=0;
  for(;t<m.memoryTypeCount;t++)if((req.memoryTypeBits&(1u<<t))&&(m.memoryTypes[t].propertyFlags&flags)==flags)break;
  if(t==m.memoryTypeCount)throw std::runtime_error("required Vulkan memory type unavailable");
  VkMemoryAllocateInfo a{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};a.allocationSize=req.size;a.memoryTypeIndex=t;
  check(vkAllocateMemory(c.device,&a,nullptr,&memory),"vkAllocateMemory");++c.allocation_count;check(vkBindBufferMemory(c.device,buffer,memory,0),"vkBindBufferMemory");
  if(flags&VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)check(vkMapMemory(c.device,memory,0,bytes,0,&mapped),"vkMapMemory");
}
Context::Context(){
 try{
  VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};app.pApplicationName="so101 resident RGBD";uint32_t loader=VK_API_VERSION_1_0;
  auto enumerate_version=reinterpret_cast<PFN_vkEnumerateInstanceVersion>(vkGetInstanceProcAddr(nullptr,"vkEnumerateInstanceVersion"));
  if(enumerate_version)check(enumerate_version(&loader),"loader version");
  instance_version=loader>=VK_API_VERSION_1_3?VK_API_VERSION_1_3:VK_API_VERSION_1_1;app.apiVersion=instance_version;
  VkInstanceCreateInfo i{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};i.pApplicationInfo=&app;check(vkCreateInstance(&i,nullptr,&instance),"vkCreateInstance");
  uint32_t n=0;check(vkEnumeratePhysicalDevices(instance,&n,nullptr),"enumerate devices");std::vector<VkPhysicalDevice> devices(n);check(vkEnumeratePhysicalDevices(instance,&n,devices.data()),"enumerate devices");
  for(auto p:devices){VkPhysicalDeviceProperties props;vkGetPhysicalDeviceProperties(p,&props);
    if(props.deviceType==VK_PHYSICAL_DEVICE_TYPE_CPU)continue;
    std::string name=props.deviceName;std::transform(name.begin(),name.end(),name.begin(),[](unsigned char c){return std::tolower(c);});
    if(name.find("llvmpipe")!=std::string::npos||name.find("lavapipe")!=std::string::npos||name.find("swiftshader")!=std::string::npos)continue;
    uint32_t qn=0;vkGetPhysicalDeviceQueueFamilyProperties(p,&qn,nullptr);std::vector<VkQueueFamilyProperties> qs(qn);vkGetPhysicalDeviceQueueFamilyProperties(p,&qn,qs.data());
    for(uint32_t q=0;q<qn;q++)if(qs[q].queueFlags&VK_QUEUE_COMPUTE_BIT){physical=p;family=q;timestamp_bits=qs[q].timestampValidBits;properties=props;break;}if(physical)break;
  }
  if(!physical)throw std::runtime_error("no hardware Vulkan compute device (software adapters refused)");
  float priority=1;
  // Optional video is negotiated on this same device before any compute consumer
  // obtains it. Unsupported or failed negotiation retains the compute lane.
  std::vector<VkDeviceQueueCreateInfo> queues;
  std::vector<const char*> extensions;
  const auto add_queue=[&](uint32_t index){for(const auto& q:queues)if(q.queueFamilyIndex==index)return;VkDeviceQueueCreateInfo q{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};q.queueFamilyIndex=index;q.queueCount=1;q.pQueuePriorities=&priority;queues.push_back(q);};
  add_queue(family);
  if(instance_version>=VK_API_VERSION_1_3&&properties.apiVersion>=VK_API_VERSION_1_3){
    uint32_t en=0;check(vkEnumerateDeviceExtensionProperties(physical,nullptr,&en,nullptr),"video extension count");std::vector<VkExtensionProperties> available(en);check(vkEnumerateDeviceExtensionProperties(physical,nullptr,&en,available.data()),"video extensions");
    const auto has=[&](const char* name){return std::any_of(available.begin(),available.end(),[&](const auto& e){return std::strcmp(name,e.extensionName)==0;});};
    enabled_features.pNext=&enabled11;enabled11.pNext=&enabled12;enabled12.pNext=&enabled13;
    if(has("VK_KHR_video_maintenance1"))enabled13.pNext=&enabled_video_maintenance1;
    vkGetPhysicalDeviceFeatures2(physical,&enabled_features);
    const char* optional[]={"VK_KHR_video_maintenance1","VK_KHR_video_queue","VK_KHR_video_encode_queue","VK_KHR_video_decode_queue","VK_KHR_video_encode_h264","VK_KHR_video_decode_h264","VK_KHR_push_descriptor","VK_KHR_external_memory_fd","VK_KHR_external_semaphore_fd","VK_EXT_external_memory_dma_buf","VK_EXT_image_drm_format_modifier"};
    if(has("VK_KHR_video_queue")&&has("VK_KHR_video_maintenance1")&&enabled_video_maintenance1.videoMaintenance1&&enabled12.timelineSemaphore&&enabled13.synchronization2){
      for(auto name:optional)if(has(name))enabled_extensions.emplace_back(name);
      uint32_t count=0;vkGetPhysicalDeviceQueueFamilyProperties2(physical,&count,nullptr);std::vector<VkQueueFamilyProperties2> qs(count);std::vector<VkQueueFamilyVideoPropertiesKHR> vs(count);
      for(uint32_t j=0;j<count;j++){qs[j].sType=VK_STRUCTURE_TYPE_QUEUE_FAMILY_PROPERTIES_2;vs[j].sType=VK_STRUCTURE_TYPE_QUEUE_FAMILY_VIDEO_PROPERTIES_KHR;qs[j].pNext=&vs[j];}
      vkGetPhysicalDeviceQueueFamilyProperties2(physical,&count,qs.data());
      for(uint32_t j=0;j<count;j++){auto flags=qs[j].queueFamilyProperties.queueFlags;auto codecs=vs[j].videoCodecOperations;
        // Advertise only operations whose extensions are actually enabled.
        if(!has("VK_KHR_video_encode_h264"))codecs&=~VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR;
        if(!has("VK_KHR_video_decode_h264"))codecs&=~VK_VIDEO_CODEC_OPERATION_DECODE_H264_BIT_KHR;
        codecs&=VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR|VK_VIDEO_CODEC_OPERATION_DECODE_H264_BIT_KHR;
        if(j==family||codecs){add_queue(j);video_queues.push_back({j,flags,codecs});}
      }
      video_capable=std::any_of(video_queues.begin(),video_queues.end(),[](const auto& q){return q.codecs!=0;});
    }
  }
  if(!video_capable){enabled_extensions.clear();video_queues.clear();queues.resize(1);video_reason="Vulkan 1.3 timeline/synchronization2, video maintenance1 feature and H.264 video queue unavailable";}
  for(auto& extension:enabled_extensions)extensions.push_back(extension.c_str());
  VkDeviceCreateInfo d{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};d.queueCreateInfoCount=queues.size();d.pQueueCreateInfos=queues.data();
  if(video_capable){d.pNext=&enabled_features;d.enabledExtensionCount=extensions.size();d.ppEnabledExtensionNames=extensions.data();}
  auto result=vkCreateDevice(physical,&d,nullptr,&device);
  if(result!=VK_SUCCESS&&video_capable){video_reason="optional video device negotiation failed: "+std::to_string(result);video_capable=false;enabled_extensions.clear();video_queues.clear();d.pNext=nullptr;d.enabledExtensionCount=0;d.ppEnabledExtensionNames=nullptr;d.queueCreateInfoCount=1;result=vkCreateDevice(physical,&d,nullptr,&device);}
  if(!video_capable){enabled_features={VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};enabled11={VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES};enabled12={VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};enabled13={VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};enabled_video_maintenance1={VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR};}
  check(result,"vkCreateDevice");vkGetDeviceQueue(device,family,0,&queue);
  VkCommandPoolCreateInfo cp{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};cp.queueFamilyIndex=family;cp.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;check(vkCreateCommandPool(device,&cp,nullptr,&commands),"vkCreateCommandPool");
 }catch(...){if(device)vkDestroyDevice(device,nullptr);if(instance)vkDestroyInstance(instance,nullptr);throw;}
}
Context::~Context(){if(device){vkDeviceWaitIdle(device);if(commands)vkDestroyCommandPool(device,commands,nullptr);vkDestroyDevice(device,nullptr);}if(instance)vkDestroyInstance(instance,nullptr);}
std::shared_ptr<Context> Context::shared(){static std::mutex mutex;static std::weak_ptr<Context> weak;std::lock_guard<std::mutex> lock(mutex);auto c=weak.lock();if(!c){c=std::make_shared<Context>();weak=c;}return c;}
}
