// Explicit helper qualification, separate from the SDK alignment oracle.
// No camera, timing claim, normal-path integration or automatic GPU test.
#include "so101_station_vulkan/pipeline.hpp"
#include <array>
#include <cfenv>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
using namespace so101_station_vulkan;
namespace {
constexpr uint32_t kCount=65536, kGroup=256;
constexpr uint64_t kDeadlineNs=10000000000ULL;
constexpr size_t kMaxShaderBytes=4*1024*1024;
struct Parameters { float projection[17]{}; uint32_t corners=kCount,convention=0; };
static_assert(sizeof(Parameters)==76 && offsetof(Parameters,corners)==68);
static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
static_assert(kCount%kGroup==0 && uint64_t(kCount)*8<=UINT32_MAX);
using Pair=std::array<uint32_t,2>;
uint32_t bits(float value) { uint32_t u; std::memcpy(&u,&value,4);return u; }
float value(uint32_t u) { float f;std::memcpy(&f,&u,4);return f; }
bool nan(uint32_t u) {return (u&0x7fffffff)>0x7f800000;}
bool matches(uint32_t a,uint32_t b) {return a==b || (nan(a)&&nan(b));}
uint32_t reference(Pair pair) {
  volatile float a=value(pair[0]),b=value(pair[1]);
  volatile float result=a/b;
  return bits(result);
}
std::vector<Pair> cases() {
  std::vector<Pair> result;result.reserve(kCount);
  // Both signs, NaNs, infinities, normals, adjacent boundaries and subnormals.
  constexpr uint32_t edges[]={0,1,2,3,0x3fffff,0x400000,0x7ffffe,0x7fffff,
    0x800000,0x800001,0xffffff,0x33800000,0x3effffff,0x3f000000,
    0x3f000001,0x3f7fffff,0x3f800000,0x3f800001,0x3fffffff,
    0x40000000,0x40400000,0x7f000000,0x7f7ffffe,0x7f7fffff,
    0x7f800000,0x7fc00000,0x7f800001,0x7fffffff};
  for(auto a:edges) for(auto b:edges) for(uint32_t sign=0;sign<4;++sign)
    result.push_back({a|((sign&1)<<31),b|((sign>>1)<<31)});
  // Exact halfway underflow ties, including even/odd retained significands.
  for(uint32_t a=1;a<4096;++a) {
    result.push_back({a,0x40000000});
    result.push_back({a|0x80000000,0x40000000});
  }
  uint32_t rng=0x83d2e691;
  auto random=[&](){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;};
  while(result.size()<kCount) {const auto a=random(),b=random();result.push_back({a,b});}
  if(result.size()!=kCount) throw std::runtime_error("corpus exceeds fixed cap");
  return result;
}
void cpu_contract() {
  if(std::fesetround(FE_TONEAREST)!=0 || std::fegetround()!=FE_TONEAREST)
    throw std::runtime_error("CPU reference requires FE_TONEAREST");
  const std::array<std::array<uint32_t,3>,9> known{{
    {{0x3f800000,0x40400000,0x3eaaaaab}},{{1,0x40000000,0}},
    {{3,0x40000000,2}},{{0x80000001,0x40000000,0x80000000}},
    {{0x00800000,0x40000000,0x00400000}},{{0x7f7fffff,0x3f000000,0x7f800000}},
    {{0,0,0x7fc00000}},{{0x80000000,0x3f800000,0x80000000}},
    {{0x3f800000,0x7f800000,0}}}};
  for(auto c:known) if(!matches(reference({c[0],c[1]}),c[2]))
    throw std::runtime_error("CPU binary32 reference contract failed (rounding/FTZ/DAZ)");
  if(matches(0,0x80000000) || !matches(0x7fc00001,0xff800001) || matches(0x7f800000,0x7fc00000))
    throw std::runtime_error("bitwise/NaN comparator contract failed");
}
void check(VkResult result,const char* operation) {
  if(result!=VK_SUCCESS) throw std::runtime_error(std::string(operation)+": "+std::to_string(result));
}
std::string json_string(const char* text) {
  std::string out="\"";
  for(const unsigned char c:std::string(text)) {
    if(c=='"'||c=='\\') {out+='\\';out+=char(c);}
    else if(c<32) out+='?'; else out+=char(c);
  }
  return out+'"';
}
struct Probe {
  std::shared_ptr<Context> context=Context::shared();
  Buffer input,output;
  VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;
  VkDescriptorPool pool=VK_NULL_HANDLE;
  VkPipelineLayout layout=VK_NULL_HANDLE;
  VkShaderModule module=VK_NULL_HANDLE;
  VkPipeline pipeline=VK_NULL_HANDLE;
  VkCommandPool commands=VK_NULL_HANDLE;
  VkFence fence=VK_NULL_HANDLE;
  ~Probe() {
    // Only reached before submission, after completion or after device loss.
    auto d=context->device;
    if(fence)vkDestroyFence(d,fence,nullptr);
    if(commands)vkDestroyCommandPool(d,commands,nullptr);
    if(pipeline)vkDestroyPipeline(d,pipeline,nullptr);
    if(module)vkDestroyShaderModule(d,module,nullptr);
    if(layout)vkDestroyPipelineLayout(d,layout,nullptr);
    if(pool)vkDestroyDescriptorPool(d,pool,nullptr);
    if(descriptors)vkDestroyDescriptorSetLayout(d,descriptors,nullptr);
  }
};
int run(const std::string& path,const std::vector<Pair>& pairs) {
  // Bound the input file before allocating or creating Vulkan objects.
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file)throw std::runtime_error("cannot open probe SPIR-V");
  const auto length=file.tellg();
  if(length<=0 || uint64_t(length)>kMaxShaderBytes || uint64_t(length)%4)
    throw std::runtime_error("invalid/beyond-cap probe SPIR-V size");
  std::vector<uint32_t> code(size_t(length)/4);
  file.seekg(0);file.read(reinterpret_cast<char*>(code.data()),length);
  if(!file || code[0]!=0x07230203)throw std::runtime_error("invalid probe SPIR-V");
  Probe p;auto d=p.context->device;
  const auto& limits=p.context->properties.limits;
  if(pairs.size()!=kCount || limits.maxStorageBufferRange<kCount*8 ||
     limits.maxPushConstantsSize<sizeof(Parameters) || limits.maxComputeWorkGroupInvocations<kGroup ||
     limits.maxComputeWorkGroupSize[0]<kGroup || limits.maxComputeWorkGroupCount[0]<kCount/kGroup ||
     limits.maxPerStageDescriptorStorageBuffers<2 || limits.maxDescriptorSetStorageBuffers<2)
    throw std::runtime_error("probe device/layout/corpus limits exceeded");
  const auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  p.input.create(*p.context,kCount*8,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,host);
  p.output.create(*p.context,kCount*4,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,host);
  std::memcpy(p.input.mapped,pairs.data(),kCount*8);
  std::array<VkDescriptorSetLayoutBinding,2> bindings{};
  for(unsigned i=0;i<2;++i) bindings[i]={i?3u:0u,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr};
  VkDescriptorSetLayoutCreateInfo desc{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
  desc.bindingCount=2;desc.pBindings=bindings.data();
  check(vkCreateDescriptorSetLayout(d,&desc,nullptr,&p.descriptors),"descriptor layout");
  VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(Parameters)};
  VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  layout.setLayoutCount=1;layout.pSetLayouts=&p.descriptors;layout.pushConstantRangeCount=1;layout.pPushConstantRanges=&range;
  check(vkCreatePipelineLayout(d,&layout,nullptr,&p.layout),"pipeline layout");
  VkShaderModuleCreateInfo module{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  module.codeSize=code.size()*4;module.pCode=code.data();
  check(vkCreateShaderModule(d,&module,nullptr,&p.module),"shader module");
  VkComputePipelineCreateInfo pipeline{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
  pipeline.layout=p.layout;pipeline.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  pipeline.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pipeline.stage.module=p.module;pipeline.stage.pName="main";
  check(vkCreateComputePipelines(d,VK_NULL_HANDLE,1,&pipeline,nullptr,&p.pipeline),"compute pipeline");
  VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,2};
  VkDescriptorPoolCreateInfo pool{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
  pool.maxSets=1;pool.poolSizeCount=1;pool.pPoolSizes=&pool_size;
  check(vkCreateDescriptorPool(d,&pool,nullptr,&p.pool),"descriptor pool");
  VkDescriptorSetAllocateInfo alloc{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
  alloc.descriptorPool=p.pool;alloc.descriptorSetCount=1;alloc.pSetLayouts=&p.descriptors;
  VkDescriptorSet set;check(vkAllocateDescriptorSets(d,&alloc,&set),"descriptor set");
  std::array<VkDescriptorBufferInfo,2> buffers{{{p.input.buffer,0,p.input.size},{p.output.buffer,0,p.output.size}}};
  std::array<VkWriteDescriptorSet,2> writes{};
  for(unsigned i=0;i<2;++i) {
    writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=set;
    writes[i].dstBinding=bindings[i].binding;writes[i].descriptorCount=1;
    writes[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;writes[i].pBufferInfo=&buffers[i];
  }
  vkUpdateDescriptorSets(d,2,writes.data(),0,nullptr);
  VkCommandPoolCreateInfo commands{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};commands.queueFamilyIndex=p.context->family;
  check(vkCreateCommandPool(d,&commands,nullptr,&p.commands),"command pool");
  VkCommandBufferAllocateInfo command_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  command_info.commandPool=p.commands;command_info.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;command_info.commandBufferCount=1;
  VkCommandBuffer command;check(vkAllocateCommandBuffers(d,&command_info,&command),"command buffer");
  VkFenceCreateInfo fence{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};check(vkCreateFence(d,&fence,nullptr,&p.fence),"fence");
  VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  check(vkBeginCommandBuffer(command,&begin),"begin");
  VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
  barrier.srcAccessMask=VK_ACCESS_HOST_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_SHADER_READ_BIT;
  vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,1,&barrier,0,nullptr,0,nullptr);
  vkCmdBindPipeline(command,VK_PIPELINE_BIND_POINT_COMPUTE,p.pipeline);
  vkCmdBindDescriptorSets(command,VK_PIPELINE_BIND_POINT_COMPUTE,p.layout,0,1,&set,0,nullptr);
  Parameters parameters;vkCmdPushConstants(command,p.layout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(parameters),&parameters);
  vkCmdDispatch(command,kCount/kGroup,1,1);
  barrier.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_HOST_READ_BIT;
  vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&barrier,0,nullptr,0,nullptr);
  check(vkEndCommandBuffer(command),"end");
  VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};submit.commandBufferCount=1;submit.pCommandBuffers=&command;
  {std::lock_guard<std::mutex> lock(p.context->queue_mutex);check(vkQueueSubmit(p.context->queue,1,&submit,p.fence),"submit");}
  const auto completion=vkWaitForFences(d,1,&p.fence,VK_TRUE,kDeadlineNs);
  if(completion!=VK_SUCCESS && completion!=VK_ERROR_DEVICE_LOST) {
    // Pending Vulkan objects cannot be destroyed safely. Exit the isolated
    // qualification process without running destructors or unbounded idle waits.
    std::cout<<"{\"physical\":false,\"status\":\"completion_failed\",\"vk_result\":"<<completion
      <<",\"case_count\":"<<kCount<<",\"deadline_ns\":"<<kDeadlineNs<<"}"<<std::endl;
    std::_Exit(2);
  }
  check(completion,"bounded completion");
  const auto* output=static_cast<const uint32_t*>(p.output.mapped);
  size_t mismatches=0;std::string examples;
  for(size_t i=0;i<pairs.size();++i) {
    const auto expected=reference(pairs[i]);
    if(matches(output[i],expected))continue;
    if(mismatches++<8) {
      if(!examples.empty())examples+=',';
      examples+="{\"case\":"+std::to_string(i)+",\"a_bits\":"+std::to_string(pairs[i][0])+
        ",\"b_bits\":"+std::to_string(pairs[i][1])+",\"expected_bits\":"+std::to_string(expected)+
        ",\"actual_bits\":"+std::to_string(output[i])+"}";
    }
  }
  const auto& props=p.context->properties;
  std::cout<<"{\"physical\":false,\"qualification\":\"alignment_divide_bits\",\"device\":"<<json_string(props.deviceName)
    <<",\"runtime_key\":{\"vendor_id\":"<<props.vendorID<<",\"device_id\":"<<props.deviceID
    <<",\"driver_version\":"<<props.driverVersion<<",\"api_version\":"<<props.apiVersion
    <<"},\"case_count\":"<<kCount<<",\"mismatch_count\":"<<mismatches<<",\"first_mismatches\":["<<examples<<"]}"<<std::endl;
  return mismatches?1:0;
}
}
int main(int argc,char** argv) {
  try {
    if(argc>2)throw std::invalid_argument("usage: qualify_alignment_divide [--cpu|probe.spv]");
    cpu_contract();const auto pairs=cases();
    if(argc==2 && std::string(argv[1])=="--cpu") {
      for(auto pair:pairs)(void)reference(pair);
      std::cout<<"{\"physical\":false,\"mode\":\"cpu_reference_contract\",\"case_count\":"<<pairs.size()<<",\"gpu_executed\":false}"<<std::endl;
      return 0;
    }
    return run(argc==2?argv[1]:ALIGNMENT_DIVIDE_PROBE_PATH,pairs);
  } catch(const std::exception& error) {
    std::cerr<<"{\"physical\":false,\"status\":\"error\",\"error\":"<<json_string(error.what())<<"}"<<std::endl;
    return 1;
  }
}
