// Explicit GPU-only qualification probe. NEVER registered in ordinary CTest.
// Exercises the production reduce SPIR-V with a deliberately nonconverged B
// snapshot and then a correct one, without weakening production round limits.
#include "so101_station_vulkan/pipeline.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace so101_station_vulkan;
namespace {
void check(VkResult result, const char *operation) {
  if (result != VK_SUCCESS)
    throw std::runtime_error(std::string(operation) + ": " +
                             std::to_string(result));
}
struct Probe {
  std::shared_ptr<Context> context = Context::shared();
  Buffer input, scratch, divisions, compact;
  VkDescriptorSetLayout descriptors = VK_NULL_HANDLE;
  VkPipelineLayout layout = VK_NULL_HANDLE;
  VkPipeline pipeline = VK_NULL_HANDLE;
  VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
  VkCommandPool commands = VK_NULL_HANDLE;
  VkFence fence = VK_NULL_HANDLE;
  bool submitted = false;
  ~Probe() {
    auto device = context->device;
    if (submitted)
      vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX);
    if (fence)
      vkDestroyFence(device, fence, nullptr);
    if (commands)
      vkDestroyCommandPool(device, commands, nullptr);
    if (pipeline)
      vkDestroyPipeline(device, pipeline, nullptr);
    if (layout)
      vkDestroyPipelineLayout(device, layout, nullptr);
    if (descriptor_pool)
      vkDestroyDescriptorPool(device, descriptor_pool, nullptr);
    if (descriptors)
      vkDestroyDescriptorSetLayout(device, descriptors, nullptr);
  }
};
struct Parameters {
  uint32_t width, height, count, capacity;
  float scale, fx, fy, cx, cy, maxDepth, jump;
  uint32_t diagnostics;
};
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc > 2)
      throw std::invalid_argument(
          "usage: verify_cc_convergence_fault [shader-directory]");
    const std::string shader_dir = argc == 2 ? argv[1] : DEFAULT_SHADER_DIR;
    Probe p;
    auto device = p.context->device;
    constexpr unsigned n = 32;
    const auto host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    p.input.create(*p.context, input_bytes(n, 1),
                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, host);
    p.scratch.create(*p.context, scratch_bytes(n, 1),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, host);
    p.divisions.create(*p.context, 512 * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                       host);
    p.compact.create(*p.context, compact_bytes(),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, host);
    std::array<VkDescriptorSetLayoutBinding, 4> bindings{};
    for (unsigned i = 0; i < 4; i++) {
      bindings[i].binding = i;
      bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      bindings[i].descriptorCount = 1;
      bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo descriptor_info{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    descriptor_info.bindingCount = 4;
    descriptor_info.pBindings = bindings.data();
    check(vkCreateDescriptorSetLayout(device, &descriptor_info, nullptr,
                                      &p.descriptors),
          "probe descriptor layout");
    VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                              sizeof(Parameters)};
    VkPipelineLayoutCreateInfo layout_info{
        VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layout_info.setLayoutCount = 1;
    layout_info.pSetLayouts = &p.descriptors;
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &range;
    check(vkCreatePipelineLayout(device, &layout_info, nullptr, &p.layout),
          "probe pipeline layout");
    std::ifstream shader(shader_dir + "/reduce.spv",
                         std::ios::binary | std::ios::ate);
    if (!shader)
      throw std::runtime_error("missing production reduce.spv");
    auto bytes = shader.tellg();
    if (bytes <= 0 || size_t(bytes) % 4)
      throw std::runtime_error("invalid reduce SPIR-V");
    std::vector<uint32_t> code(size_t(bytes) / 4);
    shader.seekg(0);
    shader.read(reinterpret_cast<char *>(code.data()), bytes);
    VkShaderModuleCreateInfo module_info{
        VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    module_info.codeSize = size_t(bytes);
    module_info.pCode = code.data();
    VkShaderModule module;
    check(vkCreateShaderModule(device, &module_info, nullptr, &module),
          "probe shader module");
    VkComputePipelineCreateInfo pipeline_info{
        VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipeline_info.layout = p.layout;
    pipeline_info.stage.sType =
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pipeline_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline_info.stage.module = module;
    pipeline_info.stage.pName = "main";
    auto pipeline_result = vkCreateComputePipelines(
        device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &p.pipeline);
    vkDestroyShaderModule(device, module, nullptr);
    check(pipeline_result, "probe compute pipeline");
    VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4};
    VkDescriptorPoolCreateInfo pool_info{
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    pool_info.maxSets = 1;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    check(
        vkCreateDescriptorPool(device, &pool_info, nullptr, &p.descriptor_pool),
        "probe descriptor pool");
    VkDescriptorSetAllocateInfo allocate{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocate.descriptorPool = p.descriptor_pool;
    allocate.descriptorSetCount = 1;
    allocate.pSetLayouts = &p.descriptors;
    VkDescriptorSet set;
    check(vkAllocateDescriptorSets(device, &allocate, &set),
          "probe descriptor set");
    std::array<VkDescriptorBufferInfo, 4> buffers{
        {{p.input.buffer, 0, p.input.size},
         {p.scratch.buffer, 0, p.scratch.size},
         {p.divisions.buffer, 0, p.divisions.size},
         {p.compact.buffer, 0, p.compact.size}}};
    std::array<VkWriteDescriptorSet, 4> writes{};
    for (unsigned i = 0; i < 4; i++) {
      writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      writes[i].dstSet = set;
      writes[i].dstBinding = i;
      writes[i].descriptorCount = 1;
      writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      writes[i].pBufferInfo = &buffers[i];
    }
    vkUpdateDescriptorSets(device, 4, writes.data(), 0, nullptr);
    VkCommandPoolCreateInfo commands_info{
        VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    commands_info.queueFamilyIndex = p.context->family;
    commands_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    check(vkCreateCommandPool(device, &commands_info, nullptr, &p.commands),
          "probe command pool");
    VkCommandBufferAllocateInfo command_info{
        VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    command_info.commandPool = p.commands;
    command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_info.commandBufferCount = 1;
    VkCommandBuffer command;
    check(vkAllocateCommandBuffers(device, &command_info, &command),
          "probe command buffer");
    VkFenceCreateInfo fence_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    check(vkCreateFence(device, &fence_info, nullptr, &p.fence), "probe fence");
    for (bool broken : {true, false}) {
      auto *scratch = static_cast<uint32_t *>(p.scratch.mapped);
      std::memset(scratch, 0, p.scratch.size);
      std::memset(p.compact.mapped, 0, p.compact.size);
      for (unsigned i = 0; i < n; i++) {
        scratch[i] = 1;
        float z = .5f;
        std::memcpy(scratch + n + i, &z, 4);
        scratch[17 * n + i] = broken && i >= 16 ? 17 : 1;
      }
      check(vkResetCommandBuffer(command, 0), "probe reset command");
      VkCommandBufferBeginInfo begin{
          VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
      check(vkBeginCommandBuffer(command, &begin), "probe begin");
      VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
      barrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
      barrier.dstAccessMask =
          VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_HOST_BIT,
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &barrier,
                           0, nullptr, 0, nullptr);
      Parameters parameters{n,   1,  n, kMaxComponents, .001f, 600,
                            600, 16, 0, 1.2f,           .012f, 0};
      vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, p.pipeline);
      vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_COMPUTE, p.layout,
                              0, 1, &set, 0, nullptr);
      vkCmdPushConstants(command, p.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                         sizeof(parameters), &parameters);
      vkCmdDispatch(command, 1, 1, 1);
      barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
      barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
      vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &barrier, 0,
                           nullptr, 0, nullptr);
      check(vkEndCommandBuffer(command), "probe end");
      VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
      submit.commandBufferCount = 1;
      submit.pCommandBuffers = &command;
      {
        std::lock_guard<std::mutex> lock(p.context->queue_mutex);
        check(vkQueueSubmit(p.context->queue, 1, &submit, p.fence),
              "probe submit");
        p.submitted = true;
      }
      check(vkWaitForFences(device, 1, &p.fence, VK_TRUE, UINT64_MAX),
            "probe completion");
      p.submitted = false;
      check(vkResetFences(device, 1, &p.fence), "probe reset fence");
      const auto fault = static_cast<uint32_t *>(p.compact.mapped)[1];
      if (fault != (broken ? 1u : 0u))
        throw std::runtime_error("production convergence fault mismatch: " +
                                 std::to_string(fault));
    }
    std::cout << "production reduce: deliberately nonconverged snapshot "
                 "fault1; canonical snapshot fault0: PASS\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
