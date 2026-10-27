// Explicit procedural parent diagnostic. --offline never creates a VkInstance.
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#ifndef CHILD_QUALIFIER_CPU_ONLY
#include "so101_station_vulkan/children.hpp"
#include <chrono>
#include <thread>
#include <dlfcn.h>
#endif
#ifndef CHILD_QUALIFIER_SOURCE_SHA256
#define CHILD_QUALIFIER_SOURCE_SHA256 "unrecorded"
#endif
#ifndef CHILD_MODULE_SOURCE_SHA256
#define CHILD_MODULE_SOURCE_SHA256 "unrecorded"
#endif
namespace {
using u32=uint32_t;using u64=uint64_t;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c);else o<<c;}o<<'"';return o.str();}
// Streaming standard SHA-256; CPU/parser tests compare it with hashlib.
struct SHA256 {
 std::array<u32,8> h{{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19}};
 std::array<unsigned char,64> block{};size_t used=0;u64 total=0;
 static u32 r(u32 x,unsigned n){return (x>>n)|(x<<(32-n));}
 void transform(){
  static constexpr u32 k[]={0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
  u32 w[64];for(unsigned j=0;j<16;j++)w[j]=(u32(block[4*j])<<24)|(u32(block[4*j+1])<<16)|(u32(block[4*j+2])<<8)|block[4*j+3];
  for(unsigned j=16;j<64;j++)w[j]=w[j-16]+(r(w[j-15],7)^r(w[j-15],18)^(w[j-15]>>3))+w[j-7]+(r(w[j-2],17)^r(w[j-2],19)^(w[j-2]>>10));
  auto a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],v=h[7];
  for(unsigned j=0;j<64;j++){u32 t1=v+(r(e,6)^r(e,11)^r(e,25))+((e&f)^(~e&g))+k[j]+w[j],t2=(r(a,2)^r(a,13)^r(a,22))+((a&b)^(a&c)^(b&c));v=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;}
  h[0]+=a;h[1]+=b;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=v;
 }
 void put(unsigned char c){block[used++]=c;++total;if(used==64){transform();used=0;}}
 std::string finish(){u64 bits=total*8;put(128);while(used!=56)put(0);for(int i=7;i>=0;i--)put(static_cast<unsigned char>(bits>>(8*i)));std::ostringstream out;for(auto x:h)out<<std::hex<<std::setw(8)<<std::setfill('0')<<x;return out.str();}
};
std::string hash_file(const std::string& path){std::ifstream f(path,std::ios::binary);require(bool(f),"provenance file unavailable");SHA256 h;std::array<char,16384> chunk;while(f){f.read(chunk.data(),chunk.size());for(std::streamsize i=0;i<f.gcount();i++)h.put(static_cast<unsigned char>(chunk[size_t(i)]));}require(f.eof(),"provenance file read failed");return h.finish();}
u32 le32(const unsigned char* p){return u32(p[0])|(u32(p[1])<<8)|(u32(p[2])<<16)|(u32(p[3])<<24);}
#ifndef CHILD_SHADER_SOURCE_SHA256
#define CHILD_SHADER_SOURCE_SHA256 "unrecorded"
#endif
struct Fixture {
 u32 width,height,profile,flags,P,C,S,minimum,parent_minimum,parent_count;u64 calibration,parent_generation,generation,max_bits,jump_bits;
 double maximum,jump;std::vector<u32> mask,depth,labels,parents;std::string hash;
 size_t n()const{return size_t(width)*height;}
};
Fixture load(const std::string& path){
 std::ifstream file(path,std::ios::binary|std::ios::ate);require(bool(file),"fixture missing");auto length=file.tellg();require(length>=128&&length<=128+12*307200+32*4096,"fixture file bound");
 std::array<unsigned char,128> header{};file.seekg(0);file.read(reinterpret_cast<char*>(header.data()),128);require(bool(file),"short header");const char magic[12]="CHILDQv1";require(std::memcmp(header.data(),magic,12)==0,"fixture magic/endian");
 std::array<u32,29> v{};for(unsigned j=0;j<29;j++)v[j]=le32(header.data()+12+4*j);
 Fixture q;q.width=v[0];q.height=v[1];q.profile=v[2];q.flags=v[3];q.P=v[4];q.C=v[5];q.S=v[6];q.minimum=v[7];q.parent_minimum=v[8];q.calibration=u64(v[9])|(u64(v[10])<<32);q.parent_generation=u64(v[11])|(u64(v[12])<<32);q.generation=u64(v[13])|(u64(v[14])<<32);q.max_bits=u64(v[15])|(u64(v[16])<<32);q.jump_bits=u64(v[17])|(u64(v[18])<<32);q.parent_count=v[19];
 std::memcpy(&q.maximum,&q.max_bits,8);std::memcpy(&q.jump,&q.jump_bits,8);
 require(q.width&&q.height&&q.width<=2048&&q.height<=2048&&q.n()<=307200&&v[20]==q.n(),"dimensions/count");require(q.profile==1||q.profile==2,"profile");require(q.profile!=1||(q.width<=640&&q.height<=480),"typed parent dimensions");require(q.flags<=1&&q.P&&q.P<=4096&&q.C&&q.C<=4096&&q.S&&q.S<=4*q.n()&&q.minimum>=8&&q.parent_minimum&&q.parent_count<=q.P,"capacity/flags");require(q.calibration==17&&q.parent_generation==23&&q.generation==29&&std::isfinite(q.maximum)&&q.maximum>.10&&std::isfinite(q.jump)&&q.jump>0,"generation/original thresholds");for(unsigned j=21;j<29;j++)require(v[j]==0,"reserved header");
 require(size_t(length)==128+12*q.n()+32*size_t(q.parent_count),"exact file length");
 // All scalar dimensions, capacities, flags and exact file size checked BEFORE allocation.
 q.mask.resize(q.n());q.depth.resize(q.n());q.labels.resize(q.n());q.parents.resize(size_t(q.parent_count)*8);
 auto words=[&](std::vector<u32>& values){std::array<unsigned char,4> word{};for(auto& x:values){file.read(reinterpret_cast<char*>(word.data()),4);require(bool(file),"short payload");x=le32(word.data());}};words(q.mask);words(q.depth);words(q.labels);words(q.parents);
 u32 blocks=((q.width+1)/2)*((q.height+1)/2);for(size_t j=0;j<q.n();j++){require(q.flags||q.mask[j]==0||q.mask[j]==1||q.mask[j]==2||q.mask[j]==4||q.mask[j]==8,"unflagged mask");require(q.labels[j]<=blocks,"parent label bound");}
 for(unsigned j=0;j<q.parent_count;j++){auto* r=q.parents.data()+8*j;require(r[0]<4&&r[1]&&r[1]<=blocks&&r[2]<blocks&&r[3]&&r[3]<=q.n()&&r[4]<q.width&&r[5]<q.height&&r[6]&&r[7]&&r[6]<=q.width-r[4]&&r[7]<=q.height-r[5]&&r[3]<=r[6]*r[7],"parent record bounds");if(j)require(q.parents[8*(j-1)]<r[0]||(q.parents[8*(j-1)]==r[0]&&q.parents[8*(j-1)+1]<r[1]),"parent order");}
 q.hash=hash_file(path);return q;
}
// Cold failure observation only: already-completed mapped compact header.
// Fixed128-byte snapshot, exactly32 U32 values; no GPU copy or ordinary API.
[[maybe_unused]] void compact_failure_header(std::ostream& out,const u32* mapped,u32 expected_parent_count){
 std::array<u32,32> snapshot{};std::memcpy(snapshot.data(),mapped,sizeof(snapshot));
 out<<std::dec<<"literal_compact_header32=[";for(unsigned i=0;i<snapshot.size();i++){if(i)out<<',';out<<snapshot[i];}out<<"] expected_parent_count="<<expected_parent_count<<'\n';
}
// Cold raw observation of the one failed35-pixel literal; never usable output.
[[maybe_unused]] size_t candidate_snapshot_part_bytes(size_t n,size_t capacity){
 require(n&&n<=307200,"raw candidate count bound");size_t part=8*n;require(part<=capacity&&part<=capacity-part,"raw candidate download capacity");return part;
}
[[maybe_unused]] void candidate_unusable_observation(std::ostream& out,const Fixture& q,const u32* header,const u32* raw,size_t capacity){
 require(q.width==7&&q.height==5&&q.n()==35,"raw candidate observation fixture bound");auto part=candidate_snapshot_part_bytes(q.n(),capacity);require(2*part==560,"raw candidate snapshot bound");
 std::array<u32,36> compact{};std::array<u32,140> snapshot{};std::memcpy(compact.data(),header,sizeof(compact));std::memcpy(snapshot.data(),raw,sizeof(snapshot));
 out<<"{\"scope\":\"raw_candidate_observation_only\",\"status\":\"UNUSABLE\",\"normal_result_published\":false,\"physical_camera\":false,\"objects\":false,\"world_qualified\":false,\"N\":35,\"fixture_sha256\":"<<quote(q.hash)<<",\"qualifier_source_sha256\":"<<quote(CHILD_QUALIFIER_SOURCE_SHA256)<<",\"expected_parent_count\":"<<q.parent_count<<",\"compact_header32\":[";
 for(unsigned i=0;i<32;i++){if(i)out<<',';out<<compact[i];}out<<"],\"first_parent_status\":[";for(unsigned i=0;i<4;i++){if(i)out<<',';out<<compact[32+i];}out<<']';
 const char* names[]={"A","B","persistent_roots","source_flags"};for(unsigned a=0;a<4;a++){out<<','<<quote(names[a])<<":[";for(unsigned i=0;i<35;i++){if(i)out<<',';out<<snapshot[35*a+i];}out<<']';}out<<"}\n";
}
std::string common(const Fixture& q){std::ostringstream o;o<<"\"scope\":\"childprep_only\",\"physical_camera\":false,\"objects\":false,\"world_qualified\":false,\"profile\":"<<quote(q.profile==1?"typed_parent":"literal_kernel")<<",\"width\":"<<q.width<<",\"height\":"<<q.height<<",\"calibration_generation\":"<<q.calibration<<",\"parent_generation\":"<<q.parent_generation<<",\"configuration_generation\":"<<q.generation<<",\"fixture_sha256\":"<<quote(q.hash)<<",\"qualifier_source_sha256\":"<<quote(CHILD_QUALIFIER_SOURCE_SHA256)<<",\"child_module_source_sha256\":"<<quote(CHILD_MODULE_SOURCE_SHA256)<<",\"compiler\":"<<quote(__VERSION__)<<",\"executable_sha256\":"<<quote(hash_file("/proc/self/exe"));return o.str();}
void fresh_file(const std::string& path,const void* data,size_t bytes){int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);require(fd>=0,"output exists or cannot be created");auto* p=static_cast<const char*>(data);while(bytes){auto n=write(fd,p,bytes);if(n<=0){close(fd);throw std::runtime_error("output write failed");}p+=n;bytes-=size_t(n);}require(close(fd)==0,"output close failed");}
void fresh_text(const std::string& path,const std::string& text){fresh_file(path,text.data(),text.size());}
#ifndef CHILD_QUALIFIER_CPU_ONLY
using namespace so101_station_vulkan;
[[noreturn]] void fatal(const char* message,int code=124){std::cerr<<message<<'\n';std::cerr.flush();std::_Exit(code);}
void check(VkResult r,const char* message){if(r!=VK_SUCCESS)throw std::runtime_error(message);}
template<class F>bool rejects(F fn){try{fn();return false;}catch(const std::exception&){return true;}}
struct Transfer {
 std::shared_ptr<Context> context;Buffer upload,download;VkCommandPool pool=VK_NULL_HANDLE;VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE,pending=VK_NULL_HANDLE;
 Transfer(std::shared_ptr<Context> c,size_t bytes):context(std::move(c)){
  auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;upload.create(*context,bytes,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,host);download.create(*context,bytes,VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
  VkCommandPoolCreateInfo p{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};p.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;p.queueFamilyIndex=context->family;check(vkCreateCommandPool(context->device,&p,nullptr,&pool),"transfer pool");VkCommandBufferAllocateInfo a{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};a.commandPool=pool;a.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;a.commandBufferCount=1;check(vkAllocateCommandBuffers(context->device,&a,&command),"transfer command");VkFenceCreateInfo f{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};check(vkCreateFence(context->device,&f,nullptr,&fence),"transfer fence");
 }
 ~Transfer(){if(pending)fatal("in-flight diagnostic teardown refused");if(fence)vkDestroyFence(context->device,fence,nullptr);if(pool)vkDestroyCommandPool(context->device,pool,nullptr);}
 void barrier(VkPipelineStageFlags src,VkPipelineStageFlags dst,VkAccessFlags a,VkAccessFlags b){VkMemoryBarrier m{VK_STRUCTURE_TYPE_MEMORY_BARRIER};m.srcAccessMask=a;m.dstAccessMask=b;vkCmdPipelineBarrier(command,src,dst,0,1,&m,0,nullptr,0,nullptr);}
 void begin(){check(vkResetCommandBuffer(command,0),"transfer reset");VkCommandBufferBeginInfo b{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};b.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;check(vkBeginCommandBuffer(command,&b),"transfer begin");}
 void run(VkFence target){check(vkEndCommandBuffer(command),"transfer end");check(vkResetFences(context->device,1,&target),"transfer reset fence");VkSubmitInfo s{VK_STRUCTURE_TYPE_SUBMIT_INFO};s.commandBufferCount=1;s.pCommandBuffers=&command;{std::lock_guard<std::mutex> lock(context->queue_mutex);if(vkQueueSubmit(context->queue,1,&s,target)!=VK_SUCCESS)fatal("submit failed; exit before potentially in-flight resources free",125);pending=target;}if(vkWaitForFences(context->device,1,&target,VK_TRUE,10000000000ULL)!=VK_SUCCESS)fatal("diagnostic10s fence deadline; exit before resources free");pending=VK_NULL_HANDLE;}
 void copy(VkBuffer from,VkBuffer to,size_t bytes,size_t source=0,size_t destination=0){require(bytes&&source+bytes>=source&&destination+bytes>=destination,"copy size");VkBufferCopy c{source,destination,bytes};vkCmdCopyBuffer(command,from,to,1,&c);}
 void source(FrameLease& frame,size_t n){auto& s=frame.slot();begin();barrier(VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);copy(upload.buffer,s.scratch.buffer,8*n);vkCmdFillBuffer(command,s.compact.buffer,0,16,0);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT|VK_ACCESS_SHADER_READ_BIT);copy(s.compact.buffer,s.readback.buffer,16);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(s.fence);s.submitted=true;}
 void candidate_snapshot(const Buffer& work,const Buffer& persistent,size_t n){auto part=candidate_snapshot_part_bytes(n,download.size);require(work.size>=part&&persistent.size>=part,"raw candidate source capacity");begin();barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);copy(work.buffer,download.buffer,part);copy(persistent.buffer,download.buffer,part,0,part);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(fence);}
 void snapshot(VkBuffer roots,VkBuffer packed,size_t n,size_t pixels){require(8*n+4*pixels<=download.size,"snapshot bound");begin();barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);copy(roots,download.buffer,8*n);if(pixels)copy(packed,download.buffer,4*pixels,0,8*n);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(fence);}
};
child_detail::Parameters parameters(const Fixture& q,const ChildGraphLayout& g){auto m=q.max_bits,j=q.jump_bits;return {g.width,g.height,g.count,g.blocks,g.parent_capacity,g.record_capacity,g.packed_capacity,g.job_capacity,g.key_bits,g.groups,child_f32_rne_bits(q.maximum),child_f32_rne_bits(q.jump),q.minimum,0,0,0,17,0,23,0,29,0,u32(m),u32(m>>32),u32(j),u32(j>>32),u32(g.draft_offset/4),g.lookup_count,0,{0,0,0}};}
// Fixture-only bound-buffer adapter. No ParentLease construction, casts or
// private owner state. The production encoder and exact21 SPVs do all work.
struct LiteralKernel {
 std::shared_ptr<Context> c;ChildGraphLayout graph;child_detail::Parameters p;std::array<Buffer,11> buffers;Buffer readback;VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;VkDescriptorPool pool=VK_NULL_HANDLE;VkDescriptorSet set=VK_NULL_HANDLE;VkPipelineLayout layout=VK_NULL_HANDLE;std::array<VkPipeline,child_detail::kernel_count> kernels{};
 LiteralKernel(std::shared_ptr<Context> context,const Fixture& q,const std::string& shaders):c(std::move(context)),graph(q.width,q.height,q.P,q.C,q.S),p(parameters(q,graph)){
  auto& g=graph;const auto& l=c->properties.limits;require(l.maxComputeWorkGroupInvocations>=256&&l.maxComputeWorkGroupSize[0]>=256&&l.maxComputeSharedMemorySize>=16384&&l.maxPushConstantsSize>=128&&l.maxPerStageDescriptorStorageBuffers>=11&&l.maxDescriptorSetStorageBuffers>=11&&l.maxComputeWorkGroupCount[0]>=std::max({(g.count+255)/256,g.job_capacity,g.record_capacity}),"literal compute limits");
  std::array<size_t,11> sizes{{8*q.n(),4*q.n(),64+32*size_t(q.P),g.workspace_bytes,g.persistent_bytes,g.lookup_bytes,g.packed_bytes,g.jobs_bytes,g.compact_bytes,g.pairs_bytes,64}};
  for(unsigned j=0;j<11;j++){require(sizes[j]<=l.maxStorageBufferRange,"literal storage range");auto usage=VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT;if(j==10)usage|=VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;buffers[j].create(*c,sizes[j],usage,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);}readback.create(*c,g.compact_bytes,VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  std::array<VkDescriptorSetLayoutBinding,11> bindings{};for(unsigned j=0;j<11;j++){bindings[j].binding=j;bindings[j].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;bindings[j].descriptorCount=1;bindings[j].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}VkDescriptorSetLayoutCreateInfo di{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};di.bindingCount=11;di.pBindings=bindings.data();check(vkCreateDescriptorSetLayout(c->device,&di,nullptr,&descriptors),"literal descriptors");VkPushConstantRange pc{VK_SHADER_STAGE_COMPUTE_BIT,0,128};VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};li.setLayoutCount=1;li.pSetLayouts=&descriptors;li.pushConstantRangeCount=1;li.pPushConstantRanges=&pc;check(vkCreatePipelineLayout(c->device,&li,nullptr,&layout),"literal layout");
  for(unsigned j=0;j<kernels.size();j++){std::ifstream f(shaders+"/"+child_detail::kernel_names[j]+".spv",std::ios::binary|std::ios::ate);require(bool(f)&&f.tellg()>0&&f.tellg()<=1048576&&size_t(f.tellg())%4==0,"literal shader size");std::vector<u32> code(size_t(f.tellg())/4);f.seekg(0);f.read(reinterpret_cast<char*>(code.data()),code.size()*4);require(bool(f),"literal shader read");VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};mi.codeSize=code.size()*4;mi.pCode=code.data();VkShaderModule module=VK_NULL_HANDLE;check(vkCreateShaderModule(c->device,&mi,nullptr,&module),"literal module");VkComputePipelineCreateInfo pi{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};pi.layout=layout;pi.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;pi.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pi.stage.module=module;pi.stage.pName="main";auto r=vkCreateComputePipelines(c->device,VK_NULL_HANDLE,1,&pi,nullptr,&kernels[j]);vkDestroyShaderModule(c->device,module,nullptr);check(r,"literal pipeline");}
  VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,11};VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};dpi.maxSets=1;dpi.poolSizeCount=1;dpi.pPoolSizes=&ps;check(vkCreateDescriptorPool(c->device,&dpi,nullptr,&pool),"literal descriptor pool");VkDescriptorSetAllocateInfo a{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};a.descriptorPool=pool;a.descriptorSetCount=1;a.pSetLayouts=&descriptors;check(vkAllocateDescriptorSets(c->device,&a,&set),"literal descriptor set");std::array<VkDescriptorBufferInfo,11> bi{};std::array<VkWriteDescriptorSet,11> wr{};for(unsigned j=0;j<11;j++){bi[j]={buffers[j].buffer,0,buffers[j].size};wr[j].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;wr[j].dstSet=set;wr[j].dstBinding=j;wr[j].descriptorCount=1;wr[j].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;wr[j].pBufferInfo=&bi[j];}vkUpdateDescriptorSets(c->device,11,wr.data(),0,nullptr);
 }
 ~LiteralKernel(){for(auto k:kernels)if(k)vkDestroyPipeline(c->device,k,nullptr);if(layout)vkDestroyPipelineLayout(c->device,layout,nullptr);if(pool)vkDestroyDescriptorPool(c->device,pool,nullptr);if(descriptors)vkDestroyDescriptorSetLayout(c->device,descriptors,nullptr);}
 void execute(Transfer& t,const Fixture& q){
  auto* bytes=static_cast<unsigned char*>(t.upload.mapped);std::memcpy(bytes,q.mask.data(),4*q.n());std::memcpy(bytes+4*q.n(),q.depth.data(),4*q.n());std::memcpy(bytes+8*q.n(),q.labels.data(),4*q.n());auto* header=reinterpret_cast<u32*>(bytes+12*q.n());std::memset(header,0,64+32*size_t(q.P));header[0]=1;header[1]=23;header[3]=17;header[5]=q.width;header[6]=q.height;header[7]=q.parent_count;std::memcpy(header+16,q.parents.data(),32*size_t(q.parent_count));
  t.begin();t.barrier(VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);t.copy(t.upload.buffer,buffers[0].buffer,8*q.n());t.copy(t.upload.buffer,buffers[1].buffer,4*q.n(),8*q.n());t.copy(t.upload.buffer,buffers[2].buffer,64+32*size_t(q.P),12*q.n());t.barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT);
  child_detail::record_graph(t.command,layout,kernels,set,p,buffers[5].buffer,buffers[8].buffer,readback.buffer,graph.compact_bytes,buffers[10].buffer);t.run(t.fence);
 }
};
void metadata(FrameLease& frame,const Fixture& q,unsigned number){auto& s=frame.slot();s.calibration.generation=q.calibration;s.metadata.calibration_generation=q.calibration;s.metadata.frame_number=number;s.metadata.depth_frame_number=number-1;s.metadata.timestamp_ns=12345678+number;s.metadata.serial="procedural-child-diagnostic-only";s.metadata.hardware_timestamp_ms=123.25;s.metadata.depth_hardware_timestamp_ms=122.5;s.metadata.timestamp_domain=1;s.metadata.depth_timestamp_domain=2;}
template<class Processor,class Lease>void complete(Processor& processor,const Lease& lease){auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(!processor.ready(lease)){if(std::chrono::steady_clock::now()>=deadline)fatal("preparation10s deadline; exit before potentially in-flight resources free");std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
std::string small_file(const std::string& path){std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f)&&f.tellg()>=0&&f.tellg()<=16384,"missing/oversized shader provenance");std::string text(size_t(f.tellg()),'\0');f.seekg(0);f.read(text.data(),text.size());require(bool(f),"provenance read");return text;}
std::string shader_identity(const std::string& directory,const std::vector<std::string>& names,const std::string& expected=""){
 auto source=small_file(directory+"/source.sha256");require(source.size()==65&&source.back()=='\n'&&source.find_first_not_of("0123456789abcdef")==64,"shader source hash");source.pop_back();require(expected.empty()||source==expected,"shader source/build tag mismatch");auto provenance=small_file(directory+"/provenance.txt");require(provenance.find("source_sha256="+source+"\n")!=std::string::npos,"shader provenance mismatch");std::ostringstream o;o<<"{\"source_sha256\":"<<quote(source)<<",\"provenance\":"<<quote(provenance)<<",\"provenance_sha256\":"<<quote(hash_file(directory+"/provenance.txt"))<<",\"spirv\":{";bool comma=false;
 for(const auto& name:names){auto filename=name+".spv";auto hash=hash_file(directory+"/"+filename);require(small_file(directory+"/"+filename+".sha256")==hash+"  "+filename+"\n","shader binary digest mismatch");std::ifstream f(directory+"/"+filename,std::ios::binary|std::ios::ate);require(bool(f)&&f.tellg()>=20&&f.tellg()<=1048576&&size_t(f.tellg())%4==0,"SPIRV bound");std::vector<u32> words(size_t(f.tellg())/4);f.seekg(0);f.read(reinterpret_cast<char*>(words.data()),words.size()*4);require(bool(f)&&words[0]==0x07230203,"SPIRV header");unsigned caps=0;for(size_t at=5;at<words.size();){unsigned n=words[at]>>16;require(n&&n<=words.size()-at,"SPIRV instruction");if((words[at]&65535)==17){require(n==2&&words[at+1]==1,"requires Shader capability only");++caps;}at+=n;}require(caps==1,"SPIRV capability list");if(comma)o<<',';comma=true;o<<quote(filename)<<':'<<quote(hash);}
 o<<"}}";return o.str();
}
std::string library(const void* symbol){Dl_info info{};require(dladdr(symbol,&info)&&info.dli_fname,"loaded library unavailable");auto p=std::filesystem::canonical(info.dli_fname).string();return "{\"path\":"+quote(p)+",\"sha256\":"+quote(hash_file(p))+"}";}
struct GPUState {std::shared_ptr<Context> context;std::shared_ptr<FramePool> pool;std::unique_ptr<ParentPreparation> parent;std::unique_ptr<ChildPreparation> child;std::unique_ptr<Transfer> transfer;std::unique_ptr<LiteralKernel> literal;};
void publish(const std::string& prefix,const Fixture& q,Transfer& t,const u32* records,const u32* statuses,const std::array<u32,4>& counts){auto* data=static_cast<unsigned char*>(t.download.mapped);fresh_file(prefix+".roots.bin",data,4*q.n());fresh_file(prefix+".flags.bin",data+4*q.n(),4*q.n());fresh_file(prefix+".packed.bin",data+8*q.n(),4*size_t(counts[2]));fresh_file(prefix+".records.bin",records,64*size_t(counts[1]));fresh_file(prefix+".statuses.bin",statuses,16*size_t(counts[0]));fresh_file(prefix+".counts.bin",counts.data(),16);}
int gpu(const Fixture& q,const std::string& parent_shaders,const std::string& shaders,const std::string& prefix){
 require(std::strlen(CHILD_QUALIFIER_SOURCE_SHA256)==64&&std::strlen(CHILD_MODULE_SOURCE_SHA256)==64&&std::strlen(CHILD_SHADER_SOURCE_SHA256)==64,"missing source/build SHA256 tags");
 std::vector<std::string> child_names;for(auto n:child_detail::kernel_names)child_names.emplace_back(n);auto child_identity=shader_identity(shaders,child_names,CHILD_SHADER_SOURCE_SHA256);std::string parent_identity="null";if(q.profile==1)parent_identity=shader_identity(parent_shaders,{"parents_init","parents_hook","parents_flatten","parents_validate","parents_scan","parents_offsets","parents_pixels","parents_labels","parents_records"});
 // Validate every output and source/build/shader identity BEFORE creating Context.
 for(const std::string cycle:{"",".empty",".restore"})for(const std::string suffix:{".json",".roots.bin",".flags.bin",".records.bin",".statuses.bin",".packed.bin",".counts.bin",".running",".offline.json",".candidate-unusable.json"})require(!std::filesystem::exists(prefix+cycle+suffix),"output prefix not fresh");
 auto directory=std::filesystem::path(prefix).parent_path();require(std::filesystem::is_directory(directory.empty()?".":directory),"output directory missing");auto child_library=library(reinterpret_cast<const void*>(&child_detail::record_graph)),parent_library=library(reinterpret_cast<const void*>(&parent_arena_bytes)),core_library=library(reinterpret_cast<const void*>(&cpu_reference));fresh_text(prefix+".running","synthetic child diagnostic; requires a REAL external process-group watchdog, acknowledgment alone does not implement it\n");
 // Deliberately heap-retained. Every unexpected GPU-path exception exits before
 // member destruction; driver cold calls require the external90s PG watchdog.
 auto* state=new GPUState;
 try{
  auto& s=*state;s.context=Context::shared();auto c=s.context;require(c->properties.deviceType!=VK_PHYSICAL_DEVICE_TYPE_CPU,"software device refused");VkPhysicalDeviceFeatures supported{};vkGetPhysicalDeviceFeatures(c->physical,&supported);size_t bytes=std::max(12*q.n()+64+32*size_t(q.P),8*q.n()+4*size_t(q.S));s.transfer=std::make_unique<Transfer>(c,bytes);auto& t=*s.transfer;
  std::ostringstream extra;extra<<",\"external_watchdog_acknowledged\":true,\"shader_source_sha256\":"<<quote(CHILD_SHADER_SOURCE_SHA256)<<",\"child_shader_identity\":"<<child_identity<<",\"parent_shader_identity\":"<<parent_identity<<",\"requires_float64\":false,\"requires_int64\":false,\"loaded_libraries\":["<<child_library<<','<<parent_library<<','<<core_library<<"],\"device\":{\"name\":"<<quote(c->properties.deviceName)<<",\"vendor_id\":"<<c->properties.vendorID<<",\"device_id\":"<<c->properties.deviceID<<",\"driver_version\":"<<c->properties.driverVersion<<",\"api_version\":"<<c->properties.apiVersion<<",\"float64_supported\":"<<(supported.shaderFloat64?"true":"false")<<",\"int64_supported\":"<<(supported.shaderInt64?"true":"false")<<",\"float64_enabled\":"<<(c->enabled_features.features.shaderFloat64?"true":"false")<<",\"int64_enabled\":"<<(c->enabled_features.features.shaderInt64?"true":"false")<<'}';
  if(q.profile==2){
   s.literal=std::make_unique<LiteralKernel>(c,q,shaders);auto& k=*s.literal;k.execute(t,q);auto* w=static_cast<const u32*>(k.readback.mapped);require(w[0]==1&&w[1]==q.width&&w[2]==q.height&&w[3]<=q.P&&w[4]<=q.C&&w[5]<=q.S&&w[6]<=k.graph.job_capacity,"literal completion header bounds");
   auto p=k.p;std::array<u32,24> identity{{p.calLo,p.calHi,p.parentLo,p.parentHi,p.childLo,p.childHi,p.maxBits,p.jumpBits,p.minChild,p.capacity,p.packedCapacity,p.jobCapacity,p.parentCapacity,0,p.maxLo,p.maxHi,p.jumpLo,p.jumpHi,0,0,0,0,0,0}};require(std::equal(identity.begin(),identity.end(),w+8),"literal header config/generation identity");
   if(q.flags){require(w[7]!=0&&w[4]==0&&w[5]==0&&w[6]==0,"literal negative must fault/zero jobs");fresh_text(prefix+".json","{"+common(q)+extra.str()+",\"status\":\"unusable\",\"production_views_used\":false,\"fault\":"+std::to_string(w[7])+"}\n");}
   else{const bool compact_valid=!w[7]&&w[3]==q.parent_count&&w[4]<=q.C&&w[5]<=q.S&&w[6]<=k.graph.job_capacity;if(!compact_valid){compact_failure_header(std::cerr,w,q.parent_count);if(w[7]==ChildNonconvergence&&q.hash=="e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd"){t.candidate_snapshot(k.buffers[3],k.buffers[4],q.n());std::ostringstream raw;candidate_unusable_observation(raw,q,w,static_cast<const u32*>(t.download.mapped),t.download.size);fresh_text(prefix+".candidate-unusable.json",raw.str());}}require(compact_valid,"literal compact bounds/fault");t.snapshot(k.buffers[4].buffer,k.buffers[6].buffer,q.n(),w[5]);std::array<u32,4> counts{{w[3],w[4],w[5],w[6]}};publish(prefix,q,t,w+32+4*q.P,w+32,counts);fresh_text(prefix+".json","{"+common(q)+extra.str()+",\"status\":\"valid\",\"production_views_used\":false,\"fault\":0}\n");}
  }else{
   s.pool=std::make_shared<FramePool>(q.width,q.height,c,false);ParentConfig pc;pc.width=q.width;pc.height=q.height;pc.min_parent_pixels=q.parent_minimum;pc.record_capacity=q.P;pc.calibration_generation=17;pc.generation=23;ChildConfig cc;cc.generation=29;cc.max_observation_depth_m=q.maximum;cc.max_neighbor_jump_m=q.jump;cc.min_child_pixels=q.minimum;cc.record_capacity=q.C;cc.packed_pixel_capacity=q.S;s.parent=std::make_unique<ParentPreparation>(c,s.pool->source_token(),pc,parent_shaders);s.child=std::make_unique<ChildPreparation>(c,s.pool->source_token(),pc,cc,shaders);auto& parent=*s.parent;auto& child=*s.child;
   const unsigned cycles=q.flags?1:3;for(unsigned cycle=0;cycle<cycles;cycle++){
    auto source=s.pool->acquire();require(bool(source),"source slot missing");metadata(source,q,91+cycle);auto* upload=static_cast<u32*>(t.upload.mapped);if(cycle==1)std::memset(upload,0,4*q.n());else std::memcpy(upload,q.mask.data(),4*q.n());std::memcpy(upload+q.n(),q.depth.data(),4*q.n());t.source(source,q.n());auto par=parent.submit(source);complete(parent,par);auto parent_counts=parent.finish(par);require(parent_counts.record_count==(cycle==1?0:q.parent_count),"typed parent oracle count");if(cycle!=1)require(std::memcmp(par.records(),q.parents.data(),32*size_t(q.parent_count))==0,"typed parent oracle records");
    auto& slot=source.slot();
    if(cycle==0){
     slot.fault=1;require(rejects([&]{child.submit(par);}),"child source fault accepted");slot.fault=0;static_cast<u32*>(slot.readback.mapped)[1]=1;require(rejects([&]{child.submit(par);}),"child Core fault accepted");static_cast<u32*>(slot.readback.mapped)[1]=0;
     ++slot.calibration.generation;require(rejects([&]{child.submit(par);}),"child calibration accepted");--slot.calibration.generation;++slot.metadata.calibration_generation;require(rejects([&]{child.submit(par);}),"child metadata generation accepted");--slot.metadata.calibration_generation;
     auto wrong=pc;++wrong.generation;auto* wrong_parent=new ParentPreparation(c,s.pool->source_token(),wrong,parent_shaders);auto wp=wrong_parent->submit(source);complete(*wrong_parent,wp);wrong_parent->finish(wp);require(rejects([&]{child.submit(wp);}),"complete parent configuration generation accepted");wp={};wrong_parent->collect();delete wrong_parent;
     check(vkResetFences(c->device,1,&slot.fence),"pending source fence");require(rejects([&]{child.submit(par);}),"child pending source accepted");t.source(source,q.n());
     // Foreign producer uses the SAME Context and SAME source index. Retain its
     // heap owners until completion so negative paths never unwind live state.
     auto* foreign_pool=new std::shared_ptr<FramePool>(std::make_shared<FramePool>(q.width,q.height,c,false));auto* foreign_parent=new ParentPreparation(c,(*foreign_pool)->source_token(),pc,parent_shaders);auto f=(*foreign_pool)->acquire();metadata(f,q,91);t.source(f,q.n());auto fp=foreign_parent->submit(f);complete(*foreign_parent,fp);foreign_parent->finish(fp);require(f.index()==source.index()&&rejects([&]{child.submit(fp);}),"same-index wrong producer accepted");fp={};f={};foreign_parent->collect();delete foreign_parent;delete foreign_pool;
    }
    auto lease=child.submit(par);require(rejects([&]{child.submit(par);}),"busy child index accepted");auto retained=lease;lease={};complete(child,retained);
    const auto& m=retained.metadata();require(m.width==q.width&&m.height==q.height&&m.calibration_generation==17&&m.frame_number==91+cycle&&m.depth_frame_number==90+cycle&&m.timestamp_ns==12345769+cycle&&m.serial=="procedural-child-diagnostic-only"&&m.hardware_timestamp_ms==123.25&&m.depth_hardware_timestamp_ms==122.5&&m.timestamp_domain==1&&m.depth_timestamp_domain==2,"metadata changed");
    if(q.flags){
     bool finish=rejects([&]{child.finish(retained);}),p=rejects([&]{retained.parent();}),roots=rejects([&]{retained.source_root_buffer();}),offset=rejects([&]{retained.source_flags_offset_bytes();}),packed=rejects([&]{retained.packed_flags_buffer();}),buffer=rejects([&]{retained.resident_records_buffer();}),records=rejects([&]{retained.records();}),status=rejects([&]{retained.parent_status();});require(finish&&p&&roots&&offset&&packed&&buffer&&records&&status,"typed negative exposed computed output");fresh_text(prefix+".json","{"+common(q)+extra.str()+",\"status\":\"unusable\",\"production_views_used\":true,\"fault_views_rejected\":{\"finish\":true,\"parent\":true,\"source_root_buffer\":true,\"source_flags_offset_bytes\":true,\"packed_flags_buffer\":true,\"resident_records_buffer\":true,\"records\":true,\"parent_status\":true}}\n");
    }else{
     auto counts=child.finish(retained);require(counts.calibration_generation==17&&counts.parent_generation==23&&counts.configuration_generation==29,"child generations changed");require(retained.source_flags_offset_bytes()==4*q.n(),"source flags offset");auto bridge=retained.parent();require(bridge.index()==par.index()&&bridge.source_frame().belongs_to(s.pool->source_token()),"parent bridge owner");bridge={};unsigned index=source.index();source={};par={};require(slot.references.load()>=2,"actual source not retained by child+parent");auto b=s.pool->acquire(),d=s.pool->acquire();require(b&&d&&!s.pool->acquire(),"three slot retention lost");b={};d={};
     // Retained copy protects parent/source and all diagnostic arrays THROUGH
     // the finite diagnostic copy fence; publish only after snapshot completes.
     t.snapshot(retained.source_root_buffer(),retained.packed_flags_buffer(),q.n(),counts.packed_pixels);std::string suffix=cycle==1?".empty":cycle==2?".restore":"";publish(prefix+suffix,q,t,reinterpret_cast<const u32*>(retained.records()),reinterpret_cast<const u32*>(retained.parent_status()),{{counts.parent_count,counts.record_count,counts.packed_pixels,counts.jobs}});retained={};child.collect();parent.collect();auto recycled=s.pool->acquire();require(recycled&&recycled.index()==index,"source not recycled");recycled={};
    }
    retained={};par={};source={};child.collect();parent.collect();
   }
   if(!q.flags){
    // A dropped submitted child must retain both upstream owners until its own
    // fence completes. Collect is nonblocking and bounded by an explicit timer.
    auto source=s.pool->acquire();metadata(source,q,94);auto* upload=static_cast<u32*>(t.upload.mapped);std::memcpy(upload,q.mask.data(),4*q.n());std::memcpy(upload+q.n(),q.depth.data(),4*q.n());t.source(source,q.n());auto par=parent.submit(source);complete(parent,par);parent.finish(par);auto dropped=child.submit(par);auto& slot=source.slot();source={};par={};dropped={};auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(slot.references.load()){child.collect();parent.collect();if(std::chrono::steady_clock::now()>=deadline)fatal("dropped child10s deadline");std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    fresh_text(prefix+".json","{"+common(q)+extra.str()+",\"status\":\"valid\",\"production_views_used\":true,\"metadata_preserved\":true,\"source_retained\":true,\"source_pool_collision_rejected\":true,\"generation_rejected\":true,\"source_fault_rejected\":true,\"pending_source_rejected\":true,\"copied_lease\":true,\"three_slot_retention\":true,\"dropped_source_reaped\":true,\"reuse_empty_restore\":true,\"frame_numbers\":[91,92,93]}\n");
   }
  }
  delete state;std::cout<<"child diagnostic output written; physical_camera=false\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';fatal("child diagnostic exception; exit before potentially in-flight member teardown",1);}catch(...){fatal("unknown child diagnostic exception; exit before potentially in-flight member teardown",1);}
}
#endif
}
int main(int argc,char** argv){try{
 require(argc>=3,"usage: --offline FIXTURE [NEWPREFIX] | --vulkan FIXTURE PARENT_SHADERDIR CHILD_SHADERDIR NEWPREFIX --external-watchdog-ack");std::string mode=argv[1];
 if(mode=="--vulkan"){
  require(argc==7&&std::string(argv[6])=="--external-watchdog-ack","real external watchdog acknowledgment required BEFORE Context; ack alone is not a watchdog");
#ifdef CHILD_QUALIFIER_CPU_ONLY
  throw std::runtime_error("CPU-only build cannot create a Vulkan device");
#else
  u32 endian=1;require(*reinterpret_cast<unsigned char*>(&endian)==1,"little-endian host required");auto q=load(argv[2]);return gpu(q,argv[3],argv[4],argv[5]);
#endif
 }
 require(mode=="--offline"&&(argc==3||argc==4),"invalid qualifier mode/arguments");auto q=load(argv[2]);std::string text="{"+common(q)+",\"gpu_executed\":false,\"expected_unusable\":"+(q.flags?"true":"false")+",\"parent_count\":"+std::to_string(q.parent_count)+",\"minimum\":"+std::to_string(q.minimum)+",\"capacity\":"+std::to_string(q.C)+",\"packed_capacity\":"+std::to_string(q.S)+"}\n";if(argc==4)fresh_text(std::string(argv[3])+".offline.json",text);std::cout<<text;return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
