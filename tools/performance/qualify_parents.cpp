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
#include <fcntl.h>
#include <unistd.h>
#ifndef PARENT_QUALIFIER_CPU_ONLY
#include "so101_station_vulkan/parents.hpp"
#include <chrono>
#include <thread>
#include <dlfcn.h>
#endif
#ifndef PARENT_QUALIFIER_SOURCE_SHA256
#define PARENT_QUALIFIER_SOURCE_SHA256 "unrecorded"
#endif
#ifndef PARENT_MODULE_SOURCE_SHA256
#define PARENT_MODULE_SOURCE_SHA256 "unrecorded"
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
u64 le64(const unsigned char* p){return u64(le32(p))|(u64(le32(p+4))<<32);}
struct Fixture {u32 width,height,minimum,capacity,flags;u64 calibration,generation;std::vector<u32> mask;std::string hash;};
Fixture load(const std::string& path){
 std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f),"fixture missing");auto bytes=f.tellg();require(bytes>=48&&bytes<=48+640*480*4,"fixture file bound");
 std::array<unsigned char,48> header{};f.seekg(0);f.read(reinterpret_cast<char*>(header.data()),48);const char magic[12]="PARENTQv1";require(std::memcmp(header.data(),magic,12)==0,"fixture magic");
 Fixture q;q.width=le32(header.data()+12);q.height=le32(header.data()+16);q.calibration=le64(header.data()+20);q.generation=le64(header.data()+28);q.minimum=le32(header.data()+36);q.capacity=le32(header.data()+40);q.flags=le32(header.data()+44);
 require(q.width&&q.width<=640&&q.height&&q.height<=480&&q.calibration==17&&q.generation==23&&q.minimum&&q.capacity&&q.capacity<=4096&&q.flags<=1,"fixture configuration");
 size_t n=size_t(q.width)*q.height;require(size_t(bytes)==48+4*n,"fixture exact file length");q.mask.resize(n);std::array<unsigned char,4> word;
 for(auto& value:q.mask){f.read(reinterpret_cast<char*>(word.data()),4);require(bool(f),"short mask");value=le32(word.data());require(q.flags||value==0||value==1||value==2||value==4||value==8,"unflagged unsupported mask");}
 q.hash=hash_file(path);return q;
}
std::string common(const Fixture& q){std::ostringstream out;out<<"\"scope\":\"parentprep_only\",\"physical_camera\":false,\"objects\":false,\"world_qualified\":false,\"width\":"<<q.width<<",\"height\":"<<q.height<<",\"calibration_generation\":"<<q.calibration<<",\"configuration_generation\":"<<q.generation<<",\"fixture_sha256\":"<<quote(q.hash)<<",\"qualifier_source_sha256\":"<<quote(PARENT_QUALIFIER_SOURCE_SHA256)<<",\"parent_module_source_sha256\":"<<quote(PARENT_MODULE_SOURCE_SHA256)<<",\"compiler\":"<<quote(__VERSION__)<<",\"executable_sha256\":"<<quote(hash_file("/proc/self/exe"));return out.str();}
void fresh_file(const std::string& path,const void* data,size_t bytes){int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);require(fd>=0,"output exists or cannot be created");auto* p=static_cast<const char*>(data);while(bytes){auto n=write(fd,p,bytes);if(n<=0){close(fd);throw std::runtime_error("output write failed");}p+=n;bytes-=size_t(n);}require(close(fd)==0,"output close failed");}
void fresh_text(const std::string& path,const std::string& text){fresh_file(path,text.data(),text.size());}
#ifndef PARENT_QUALIFIER_CPU_ONLY
using namespace so101_station_vulkan;
[[noreturn]] void fatal(const char* message,int code=124){std::cerr<<message<<'\n';std::cerr.flush();std::_Exit(code);}
void check(VkResult r,const char* message){if(r!=VK_SUCCESS)throw std::runtime_error(message);}
template<class F>bool rejects(F fn){try{fn();return false;}catch(const std::exception&){return true;}}
struct Transfer {
 std::shared_ptr<Context> context;Buffer upload,download;VkCommandPool pool=VK_NULL_HANDLE;VkCommandBuffer command=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE,pending=VK_NULL_HANDLE;
 Transfer(std::shared_ptr<Context> c,size_t n):context(std::move(c)){
  try{auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;upload.create(*context,n,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,host);download.create(*context,n,VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
   VkCommandPoolCreateInfo p{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};p.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;p.queueFamilyIndex=context->family;check(vkCreateCommandPool(context->device,&p,nullptr,&pool),"transfer pool");VkCommandBufferAllocateInfo a{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};a.commandPool=pool;a.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;a.commandBufferCount=1;check(vkAllocateCommandBuffers(context->device,&a,&command),"transfer command");VkFenceCreateInfo f{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};check(vkCreateFence(context->device,&f,nullptr,&fence),"transfer fence");
  }catch(...){if(fence)vkDestroyFence(context->device,fence,nullptr);if(pool)vkDestroyCommandPool(context->device,pool,nullptr);throw;}
 }
 ~Transfer(){if(pending)fatal("in-flight diagnostic teardown refused");if(fence)vkDestroyFence(context->device,fence,nullptr);if(pool)vkDestroyCommandPool(context->device,pool,nullptr);}
 void barrier(VkPipelineStageFlags src,VkPipelineStageFlags dst,VkAccessFlags a,VkAccessFlags b){VkMemoryBarrier m{VK_STRUCTURE_TYPE_MEMORY_BARRIER};m.srcAccessMask=a;m.dstAccessMask=b;vkCmdPipelineBarrier(command,src,dst,0,1,&m,0,nullptr,0,nullptr);}
 void begin(){check(vkResetCommandBuffer(command,0),"transfer reset");VkCommandBufferBeginInfo b{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};b.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;check(vkBeginCommandBuffer(command,&b),"transfer begin");}
 void run(VkFence target){check(vkEndCommandBuffer(command),"transfer end");check(vkResetFences(context->device,1,&target),"transfer reset fence");VkSubmitInfo s{VK_STRUCTURE_TYPE_SUBMIT_INFO};s.commandBufferCount=1;s.pCommandBuffers=&command;{std::lock_guard<std::mutex> lock(context->queue_mutex);auto result=vkQueueSubmit(context->queue,1,&s,target);if(result!=VK_SUCCESS)fatal("diagnostic submit failed; process exit before GPU resources free",125);pending=target;}auto r=vkWaitForFences(context->device,1,&target,VK_TRUE,10000000000ULL);if(r!=VK_SUCCESS)fatal("diagnostic fence failed/timed out; process exit before GPU resources free");pending=VK_NULL_HANDLE;}
 void source(FrameLease& frame){auto& s=frame.slot();begin();barrier(VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);VkBufferCopy copy{0,0,upload.size};vkCmdCopyBuffer(command,upload.buffer,s.scratch.buffer,1,&copy);vkCmdFillBuffer(command,s.compact.buffer,0,16,0);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT|VK_ACCESS_SHADER_READ_BIT);copy.size=16;vkCmdCopyBuffer(command,s.compact.buffer,s.readback.buffer,1,&copy);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(s.fence);s.submitted=true;}
 void snapshot(const ParentLease& lease){begin();barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);VkBufferCopy copy{lease.label_offset_bytes(),0,download.size};vkCmdCopyBuffer(command,lease.resident_buffer(),download.buffer,1,&copy);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(fence);}
};
void metadata(FrameLease& source,const Fixture& q){auto& s=source.slot();s.calibration.generation=q.calibration;s.metadata.calibration_generation=q.calibration;s.metadata.frame_number=91;s.metadata.depth_frame_number=90;s.metadata.timestamp_ns=12345678;s.metadata.serial="procedural-parent-diagnostic-only";s.metadata.hardware_timestamp_ms=123.25;s.metadata.depth_hardware_timestamp_ms=122.5;s.metadata.timestamp_domain=1;s.metadata.depth_timestamp_domain=2;}
void complete(ParentPreparation& parent,const ParentLease& lease){auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(!parent.ready(lease)){if(std::chrono::steady_clock::now()>=deadline)fatal("parent deadline; process exit before GPU resources free");std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
std::string library(const void* symbol){Dl_info info{};require(dladdr(symbol,&info)&&info.dli_fname,"loaded library provenance unavailable");auto p=std::filesystem::canonical(info.dli_fname).string();return "{\"path\":"+quote(p)+",\"sha256\":"+quote(hash_file(p))+"}";}
// Heap state intentionally survives every unexpected exception. Catching an
// in-flight failure must exit the process before any pool/Buffer/device frees.
struct GPUState {std::shared_ptr<Context> context;std::shared_ptr<FramePool> pool;std::unique_ptr<ParentPreparation> parent;std::unique_ptr<Transfer> transfer;};
std::string small_file(const std::string& path){std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f)&&f.tellg()>=0&&f.tellg()<=16384,"missing/oversized shader provenance");std::string text(size_t(f.tellg()),'\0');f.seekg(0);f.read(text.data(),text.size());require(bool(f),"shader provenance read failed");return text;}
int gpu(const Fixture& q,const std::string& shaders,const std::string& prefix){
 require(std::strlen(PARENT_QUALIFIER_SOURCE_SHA256)==64&&std::strlen(PARENT_MODULE_SOURCE_SHA256)==64,"missing source/build SHA256 tags");
 auto shader_source=small_file(shaders+"/source.sha256");require(shader_source.size()==65&&shader_source.back()=='\n'&&shader_source.find_first_not_of("0123456789abcdef")==64,"invalid shader source identity");shader_source.pop_back();
 auto shader_provenance=small_file(shaders+"/provenance.txt");require(shader_provenance.find("source_sha256="+shader_source+"\n")!=std::string::npos,"shader source/provenance disagree");
 for(auto suffix:{".json",".labels.bin",".records.bin",".counts.bin",".running",".offline.json"})require(!std::filesystem::exists(prefix+suffix),"output prefix is not fresh");
 require(std::filesystem::is_directory(std::filesystem::path(prefix).parent_path().empty()?".":std::filesystem::path(prefix).parent_path()),"output directory missing");
 fresh_text(prefix+".running","procedural parent diagnostic; external watchdog acknowledged\n");
 auto* state=new GPUState;
 try{
  auto& s=*state;s.context=Context::shared();auto c=s.context;require(c->properties.deviceType!=VK_PHYSICAL_DEVICE_TYPE_CPU,"software device refused");VkPhysicalDeviceFeatures supported{};vkGetPhysicalDeviceFeatures(c->physical,&supported);
  s.pool=std::make_shared<FramePool>(q.width,q.height,c,false);ParentConfig config;config.width=q.width;config.height=q.height;config.calibration_generation=q.calibration;config.generation=q.generation;config.min_parent_pixels=q.minimum;config.record_capacity=q.capacity;
  s.parent=std::make_unique<ParentPreparation>(c,s.pool->source_token(),config,shaders);s.transfer=std::make_unique<Transfer>(c,q.mask.size()*4);auto& parent=*s.parent;auto& transfer=*s.transfer;std::memcpy(transfer.upload.mapped,q.mask.data(),q.mask.size()*4);
  auto source=s.pool->acquire();metadata(source,q);auto& slot=source.slot();transfer.source(source);
  {auto foreign=std::make_shared<FramePool>(q.width,q.height,c,false);auto f=foreign->acquire();metadata(f,q);transfer.source(f);require(f.index()==source.index()&&rejects([&]{parent.submit(f);}),"producer pool collision accepted");}
  slot.fault=1;require(rejects([&]{parent.submit(source);}),"source fault accepted");slot.fault=0;
  static_cast<u32*>(slot.readback.mapped)[1]=1;require(rejects([&]{parent.submit(source);}),"source Core completion fault accepted");static_cast<u32*>(slot.readback.mapped)[1]=0;
  ++slot.metadata.calibration_generation;require(rejects([&]{parent.submit(source);}),"metadata generation accepted");--slot.metadata.calibration_generation;
  ++slot.calibration.generation;require(rejects([&]{parent.submit(source);}),"calibration generation accepted");--slot.calibration.generation;
  check(vkResetFences(c->device,1,&slot.fence),"reset source pending test");require(rejects([&]{parent.submit(source);}),"pending source accepted");transfer.source(source);
  auto result=parent.submit(source);require(rejects([&]{parent.submit(source);}),"busy parent index accepted");auto retained=result;result={};unsigned index=source.index();source={};require(slot.references==1,"source not retained");complete(parent,retained);
  const auto& m=retained.metadata();require(m.width==q.width&&m.height==q.height&&m.calibration_generation==17&&m.frame_number==91&&m.depth_frame_number==90&&m.timestamp_ns==12345678&&m.serial=="procedural-parent-diagnostic-only"&&m.hardware_timestamp_ms==123.25&&m.depth_hardware_timestamp_ms==122.5&&m.timestamp_domain==1&&m.depth_timestamp_domain==2,"metadata changed");
  std::ostringstream extra;extra<<",\"source_metadata\":{\"width\":"<<m.width<<",\"height\":"<<m.height<<",\"calibration_generation\":"<<m.calibration_generation<<",\"frame_number\":"<<m.frame_number<<",\"depth_frame_number\":"<<m.depth_frame_number<<",\"timestamp_ns\":"<<m.timestamp_ns<<",\"serial\":"<<quote(m.serial)<<",\"hardware_timestamp_ms\":"<<m.hardware_timestamp_ms<<",\"depth_hardware_timestamp_ms\":"<<m.depth_hardware_timestamp_ms<<",\"timestamp_domain\":"<<m.timestamp_domain<<",\"depth_timestamp_domain\":"<<m.depth_timestamp_domain<<'}';extra<<",\"external_watchdog_acknowledged\":true,\"shader_source_sha256\":"<<quote(shader_source)<<",\"shader_build_provenance\":"<<quote(shader_provenance)<<",\"shader_build_provenance_sha256\":"<<quote(hash_file(shaders+"/provenance.txt"))<<",\"requires_float64\":false,\"requires_int64\":false,\"device\":{\"name\":"<<quote(c->properties.deviceName)<<",\"vendor_id\":"<<c->properties.vendorID<<",\"device_id\":"<<c->properties.deviceID<<",\"driver_version\":"<<c->properties.driverVersion<<",\"api_version\":"<<c->properties.apiVersion<<",\"float64_supported\":"<<(supported.shaderFloat64?"true":"false")<<",\"int64_supported\":"<<(supported.shaderInt64?"true":"false")<<",\"float64_enabled\":"<<(c->enabled_features.features.shaderFloat64?"true":"false")<<",\"int64_enabled\":"<<(c->enabled_features.features.shaderInt64?"true":"false")<<"},\"loaded_libraries\":["<<library(reinterpret_cast<const void*>(&parent_arena_bytes))<<','<<library(reinterpret_cast<const void*>(&cpu_reference))<<"],\"shader_sha256\":{";
  bool comma=false;for(const char* name:{"parents_init","parents_hook","parents_flatten","parents_validate","parents_scan","parents_offsets","parents_pixels","parents_labels","parents_records"}){if(comma)extra<<',';comma=true;extra<<quote(std::string(name)+".spv")<<':'<<quote(hash_file(shaders+"/"+name+".spv"));}extra<<'}';
  if(q.flags){
   bool finish=rejects([&]{parent.finish(retained);}),arena=rejects([&]{retained.resident_buffer();}),records_buffer=rejects([&]{retained.resident_records_buffer();}),mask=rejects([&]{retained.original_mask_buffer();}),records=rejects([&]{retained.records();});require(finish&&arena&&records_buffer&&mask&&records,"unusable fixture exposed partial output");
   fresh_text(prefix+".json","{"+common(q)+extra.str()+",\"status\":\"unusable\",\"fault_views_rejected\":{\"finish\":true,\"resident_buffer\":true,\"resident_records_buffer\":true,\"original_mask_buffer\":true,\"records\":true}}\n");retained={};parent.collect();
  }else{
   auto counts=parent.finish(retained);require(counts.calibration_generation==17&&counts.configuration_generation==23,"result generation changed");transfer.snapshot(retained);
   fresh_file(prefix+".labels.bin",transfer.download.mapped,q.mask.size()*4);fresh_file(prefix+".records.bin",retained.records(),size_t(counts.record_count)*sizeof(ParentRecord));fresh_file(prefix+".counts.bin",counts.all_parents.data(),16);
   auto b=s.pool->acquire(),d=s.pool->acquire();require(b&&d&&!s.pool->acquire(),"retained source pool not saturated");metadata(b,q);transfer.source(b);auto pb=parent.submit(b);metadata(d,q);transfer.source(d);auto pd=parent.submit(d);complete(parent,pb);complete(parent,pd);auto cb=parent.finish(pb),cd=parent.finish(pd);require(cb.all_parents==counts.all_parents&&cd.all_parents==counts.all_parents&&cb.record_count==counts.record_count&&cd.record_count==counts.record_count,"three slots changed counts");pb={};pd={};parent.collect();b={};d={};retained={};parent.collect();
   auto recycled=s.pool->acquire();require(recycled&&recycled.index()==index,"source index not recycled");metadata(recycled,q);transfer.source(recycled);auto dropped=parent.submit(recycled);recycled={};dropped={};auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(slot.references.load()){parent.collect();if(std::chrono::steady_clock::now()>end)fatal("dropped source deadline");std::this_thread::sleep_for(std::chrono::milliseconds(1));}
   fresh_text(prefix+".json","{"+common(q)+extra.str()+",\"status\":\"valid\",\"record_count\":"+std::to_string(counts.record_count)+",\"metadata_preserved\":true,\"source_retained\":true,\"source_pool_collision_rejected\":true,\"source_fault_rejected\":true,\"pending_source_rejected\":true,\"generation_rejected\":true,\"copied_lease\":true,\"three_slot_retention\":true,\"dropped_source_reaped\":true}\n");
  }
  delete state;std::cout<<"parent diagnostic output written; physical_camera=false\n";return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';fatal("parent diagnostic failed; exit before freeing potentially in-flight resources",1);}
}
#endif
}
int main(int argc,char** argv){try{
 require(argc>=3,"usage: --offline FIXTURE | --vulkan FIXTURE SHADERDIR NEWPREFIX --external-watchdog-ack");std::string mode=argv[1];
 if(mode=="--vulkan"){
  require(argc==6&&std::string(argv[5])=="--external-watchdog-ack","real external watchdog acknowledgment required BEFORE Context");
#ifdef PARENT_QUALIFIER_CPU_ONLY
  throw std::runtime_error("CPU-only build cannot create a Vulkan device");
#else
  u32 endian=1;require(*reinterpret_cast<unsigned char*>(&endian)==1,"diagnostic requires little-endian host");auto q=load(argv[2]);return gpu(q,argv[3],argv[4]);
#endif
 }
 require(mode=="--offline"&&(argc==3||argc==4),"invalid qualifier mode/arguments");auto q=load(argv[2]);std::ostringstream output;output<<'{'<<common(q)<<",\"minimum\":"<<q.minimum<<",\"capacity\":"<<q.capacity<<",\"expected_unusable\":"<<(q.flags?"true":"false")<<"}\n";if(argc==4)fresh_text(std::string(argv[3])+".offline.json",output.str());std::cout<<output.str();return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
