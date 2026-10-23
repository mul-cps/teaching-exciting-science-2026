// Diagnostic microbenchmark: repeated resident source, no acquisition/unique delivery.
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cerrno>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#ifndef GEOMETRY_BENCHMARK_CPU_ONLY
#include "so101_station_vulkan/geometry.hpp"
#endif
#ifdef GEOMETRY_BENCHMARK_BUILD_HEADER
#include "geometry-benchmark-build.hpp"
#endif
#ifndef GEOMETRY_BENCHMARK_PROVENANCE
#define GEOMETRY_BENCHMARK_PROVENANCE "unrecorded"
#endif
namespace { std::atomic<uint64_t> cpp_allocations{0}; }
void* operator new(size_t n){if(auto p=std::malloc(n?n:1)){cpp_allocations.fetch_add(1,std::memory_order_relaxed);return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p) noexcept {std::free(p);}void operator delete[](void* p) noexcept {std::free(p);}
void operator delete(void* p,size_t) noexcept {std::free(p);}void operator delete[](void* p,size_t) noexcept {std::free(p);}
namespace {
constexpr uint64_t kDeadlineNs=10000000000ULL;
constexpr const char* kShaderNames[]={"geometry_coordinates","geometry_support_scan","geometry_support_offsets","geometry_support_scatter","geometry_valid_count"};
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
std::string quote(const std::string& s){std::string out="\"";for(unsigned char c:s){if(c=='"'||c=='\\'){out+='\\';out+=c;}else if(c=='\n')out+="\\n";else if(c<32)out+='?';else out+=c;}return out+'"';}
// Small self-contained SHA-256 for immutable artifact identity; no subprocesses.
std::string sha256(std::vector<uint8_t> bytes) {
 static constexpr uint32_t k[]={0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
 uint32_t h[]={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
 auto rotate=[](uint32_t x,unsigned n){return(x>>n)|(x<<(32-n));};
 uint64_t length=uint64_t(bytes.size())*8;bytes.push_back(128);while(bytes.size()%64!=56)bytes.push_back(0);
 for(int i=7;i>=0;--i)bytes.push_back(uint8_t(length>>(i*8)));
 for(size_t off=0;off<bytes.size();off+=64) {
  uint32_t w[64];for(unsigned i=0;i<16;++i)w[i]=(uint32_t(bytes[off+i*4])<<24)|(uint32_t(bytes[off+i*4+1])<<16)|(uint32_t(bytes[off+i*4+2])<<8)|bytes[off+i*4+3];
  for(unsigned i=16;i<64;++i){auto a=w[i-15],b=w[i-2];w[i]=w[i-16]+(rotate(a,7)^rotate(a,18)^(a>>3))+w[i-7]+(rotate(b,17)^rotate(b,19)^(b>>10));}
  uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
  for(unsigned i=0;i<64;++i){uint32_t t=hh+(rotate(e,6)^rotate(e,11)^rotate(e,25))+((e&f)^(~e&g))+k[i]+w[i];uint32_t u=(rotate(a,2)^rotate(a,13)^rotate(a,22))+((a&b)^(a&c)^(b&c));hh=g;g=f;f=e;e=d+t;d=c;c=b;b=a;a=t+u;}
  h[0]+=a;h[1]+=b;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=hh;
 }
 std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto v:h)out<<std::setw(8)<<v;return out.str();
}
uint32_t le32(const uint8_t* p){uint32_t v=0;for(unsigned i=0;i<4;++i)v|=uint32_t(p[i])<<(8*i);return v;}
uint64_t le64(const uint8_t* p){return uint64_t(le32(p))|uint64_t(le32(p+4))<<32;}
double real64(const uint8_t* p){auto bits=le64(p);double value;std::memcpy(&value,&bits,8);return value;}
std::vector<uint8_t> read_file(const std::string& path,size_t cap){std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f),"cannot open input");auto n=f.tellg();require(n>=0&&uint64_t(n)<=cap,"input length exceeds bound");std::vector<uint8_t> b(static_cast<size_t>(n));f.seekg(0);f.read(reinterpret_cast<char*>(b.data()),n);require(bool(f),"input read failed");return b;}
void fresh_file(const std::string& path,const std::string& text){int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);require(fd>=0,"output must be a fresh file");size_t offset=0;while(offset<text.size()){auto n=::write(fd,text.data()+offset,text.size()-offset);if(n<=0){::close(fd);throw std::runtime_error("output write failed; partial artifact retained");}offset+=size_t(n);}require(::close(fd)==0,"output close failed");}
[[maybe_unused]] uint64_t clock_ns(clockid_t id){timespec t{};require(clock_gettime(id,&t)==0,"clock_gettime failed");return uint64_t(t.tv_sec)*1000000000ULL+uint64_t(t.tv_nsec);}
uint64_t number(const std::string& s){size_t end=0;require(!s.empty()&&s[0]!='-',"expected unsigned decimal");auto n=std::stoull(s,&end,10);require(end==s.size(),"invalid unsigned decimal");return n;}
struct Config {bool offline=false,software=false;std::string fixture,shaders,output;uint32_t iterations=32,trials=3,warmup=8,poll_us=50;uint64_t cpu_budget_ms=2000,expected_valid=UINT64_MAX,expected_support=UINT64_MAX;};
Config options(int argc,char** argv){
 require(argc>=8,"usage: benchmark_geometry --offline|--run --software-binary64|--native-float64 FIXTURE SHADERS NEW_JSON --expected-valid N --expected-support N [--iterations 32 --trials 3 --warmup 8 --poll-us 50 --cpu-budget-ms 2000]");
 Config c;std::string mode=argv[1],precision=argv[2];require(mode=="--offline"||mode=="--run","explicit offline/run mode required");require(precision=="--software-binary64"||precision=="--native-float64","explicit precision required");c.offline=mode=="--offline";c.software=precision=="--software-binary64";c.fixture=argv[3];c.shaders=argv[4];c.output=argv[5];
 for(int i=6;i<argc;i+=2){require(i+1<argc,"missing option value");std::string key=argv[i];uint64_t n=number(argv[i+1]);if(key=="--expected-valid")c.expected_valid=n;else if(key=="--expected-support")c.expected_support=n;else if(key=="--cpu-budget-ms")c.cpu_budget_ms=n;else {require(n<=UINT32_MAX,"option exceeds uint32 range");if(key=="--iterations")c.iterations=uint32_t(n);else if(key=="--trials")c.trials=uint32_t(n);else if(key=="--warmup")c.warmup=uint32_t(n);else if(key=="--poll-us")c.poll_us=uint32_t(n);else throw std::runtime_error("unknown option");}}
 require(c.iterations>=1&&c.iterations<=512&&c.trials>=1&&c.trials<=8&&uint64_t(c.iterations)*c.trials<=2048&&c.warmup<=32&&c.poll_us>=1&&c.poll_us<=10000&&c.cpu_budget_ms>=1&&c.cpu_budget_ms<=10000,"configuration outside fixed bounds");require(c.expected_valid!=UINT64_MAX&&c.expected_support!=UINT64_MAX,"independent expected counts required");return c;
}
struct Fixture {uint32_t width=0,height=0;uint64_t generation=0;double depth_scale=0,maximum=0;std::array<double,12> transform{};std::array<double,6> roi{};std::vector<std::array<double,2>> rays;std::vector<uint8_t> source;std::string sha;};
Fixture fixture(const Config& c){
 auto b=read_file(c.fixture,184+640*480*24);require(b.size()>=184&&std::memcmp(b.data(),"GEOQv1\0\0",8)==0,"bad GEOQv1 header");Fixture f;f.width=le32(b.data()+8);f.height=le32(b.data()+12);f.generation=le64(b.data()+16);f.depth_scale=real64(b.data()+24);f.maximum=real64(b.data()+32);require(f.width&&f.width<=640&&f.height&&f.height<=480&&f.generation,"invalid dimensions/generation");size_t n=size_t(f.width)*f.height;require(b.size()==184+n*24,"fixture length mismatch");require(std::isfinite(f.depth_scale)&&f.depth_scale>0&&std::isfinite(float(f.depth_scale))&&float(f.depth_scale)>0&&std::isfinite(f.maximum)&&f.maximum>=.1&&std::isfinite(float(f.maximum)),"invalid scale/depth limit");
 for(unsigned i=0;i<12;++i){f.transform[i]=real64(b.data()+40+i*8);require(std::isfinite(f.transform[i]),"nonfinite transform");}for(unsigned i=0;i<6;++i)f.roi[i]=real64(b.data()+136+i*8);for(unsigned i=0;i<6;i+=2)require(std::isfinite(f.roi[i])&&std::isfinite(f.roi[i+1])&&f.roi[i]<f.roi[i+1],"invalid support ROI");
 f.rays.resize(n);for(size_t i=0;i<n;++i)for(unsigned j=0;j<2;++j){f.rays[i][j]=real64(b.data()+184+i*16+j*8);require(std::isfinite(f.rays[i][j]),"nonfinite ray");}f.source.assign(b.begin()+184+n*16,b.end());f.sha=sha256(b);require(c.expected_valid<=n&&c.expected_support<=((f.width+7)/8)*((f.height+7)/8)&&c.expected_support<=4800,"expected counts exceed fixture bounds");return f;
}
struct ShaderIdentity {std::array<std::string,5> sha;};
ShaderIdentity shaders(const Config& c){ShaderIdentity result;for(unsigned i=0;i<5;++i){auto b=read_file(c.shaders+"/"+kShaderNames[i]+".spv",4*1024*1024);require(b.size()>=20&&b.size()%4==0&&le32(b.data())==0x07230203,"invalid SPIR-V header");bool int64=false,float64=false;for(size_t w=5;w<b.size()/4;){auto instruction=le32(b.data()+w*4);unsigned words=instruction>>16,op=instruction&65535;require(words&&w+words<=b.size()/4,"bad SPIR-V instruction");if(op==17){require(words==2,"bad capability");auto cap=le32(b.data()+(w+1)*4);int64|=cap==11;float64|=cap==10;}w+=words;}if(c.software){require(!float64,"software shader declares Float64");if(i==0||i==3)require(int64,"software coordinate/scatter shader lacks Int64");}else if(i==0||i==3)require(float64,"native coordinate/scatter shader lacks Float64");result.sha[i]=sha256(b);}return result;}
struct Sample {uint64_t latency_ns=0,cpu_ns=0,sleep_wall_ns=0,sleep_calls=0,polls=0;uint32_t valid=0,support=0,arena=0;};
struct Trial {uint64_t cpu_ns=0,cpp_new_calls=0,buffer_allocations=0;};
struct Measurement {std::vector<Sample> samples;std::vector<Trial> trials;uint64_t checksum=1469598103934665603ULL,cpu_ns=0;std::string device="not_queried";uint32_t vendor=0,id=0,driver=0,api=0;bool physical_int64=false,enabled_int64=false,physical_float64=false,enabled_float64=false;uint32_t source_slot=0;};
#ifndef GEOMETRY_BENCHMARK_CPU_ONLY
using namespace so101_station_vulkan;
void checked(VkResult r,const char* message){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(message)+": "+std::to_string(r));}
struct PendingExit {bool active=false;~PendingExit(){if(active){std::cerr<<"pending benchmark GPU work retained until process exit"<<std::endl;std::_Exit(2);}}};
struct SourceUpload {
 std::shared_ptr<Context> context;Buffer upload;VkCommandPool pool=VK_NULL_HANDLE;VkCommandBuffer command=VK_NULL_HANDLE;bool pending=false;
 explicit SourceUpload(std::shared_ptr<Context> c,size_t size):context(std::move(c)){
  try{upload.create(*context,size,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);VkCommandPoolCreateInfo p{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};p.queueFamilyIndex=context->family;checked(vkCreateCommandPool(context->device,&p,nullptr,&pool),"upload pool");VkCommandBufferAllocateInfo a{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};a.commandPool=pool;a.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;a.commandBufferCount=1;checked(vkAllocateCommandBuffers(context->device,&a,&command),"upload command");}catch(...){if(pool)vkDestroyCommandPool(context->device,pool,nullptr);throw;}
 }
 ~SourceUpload(){if(pending){std::cerr<<"pending source upload retained until process exit"<<std::endl;std::_Exit(2);}if(pool)vkDestroyCommandPool(context->device,pool,nullptr);}
 void complete(FrameLease& source,const std::vector<uint8_t>& bytes){
  auto& s=source.slot();std::memcpy(upload.mapped,bytes.data(),bytes.size());VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;checked(vkBeginCommandBuffer(command,&begin),"upload begin");VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};barrier.srcAccessMask=VK_ACCESS_HOST_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_TRANSFER_READ_BIT;vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,0,1,&barrier,0,nullptr,0,nullptr);
  VkBufferCopy copy{0,0,bytes.size()};vkCmdCopyBuffer(command,upload.buffer,s.scratch.buffer,1,&copy);vkCmdFillBuffer(command,s.compact.buffer,0,16,0);barrier.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_TRANSFER_READ_BIT|VK_ACCESS_SHADER_READ_BIT;vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,1,&barrier,0,nullptr,0,nullptr);copy.size=16;vkCmdCopyBuffer(command,s.compact.buffer,s.readback.buffer,1,&copy);barrier.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_HOST_READ_BIT;vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&barrier,0,nullptr,0,nullptr);checked(vkEndCommandBuffer(command),"upload end");checked(vkResetFences(context->device,1,&s.fence),"upload reset fence");VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};submit.commandBufferCount=1;submit.pCommandBuffers=&command;pending=true;{std::lock_guard<std::mutex> lock(context->queue_mutex);checked(vkQueueSubmit(context->queue,1,&submit,s.fence),"upload submit");}checked(vkWaitForFences(context->device,1,&s.fence,VK_TRUE,kDeadlineNs),"bounded source completion");pending=false;s.submitted=true;
 }
};
Sample one(GeometryPreparation& geometry,const FrameLease& source,const Config& c){
 Sample sample;PendingExit pending;auto cpu_start=clock_ns(CLOCK_PROCESS_CPUTIME_ID);auto start=clock_ns(CLOCK_MONOTONIC);pending.active=true;auto lease=geometry.submit(source);sample.arena=lease.index();
 while(!geometry.ready(lease)){
  ++sample.polls;require(clock_ns(CLOCK_MONOTONIC)-start<kDeadlineNs,"geometry operation exceeded 10-second deadline");
  timespec remaining{0,long(c.poll_us)*1000};int rc;
  do{auto before=clock_ns(CLOCK_MONOTONIC);rc=nanosleep(&remaining,&remaining);int sleep_errno=errno;auto after=clock_ns(CLOCK_MONOTONIC);sample.sleep_wall_ns+=after-before;++sample.sleep_calls;require(rc==0||sleep_errno==EINTR,"poll nanosleep failed");require(after-start<kDeadlineNs,"geometry operation exceeded 10-second deadline");}while(rc!=0);
 }
 pending.active=false;auto counts=geometry.finish(lease);sample.latency_ns=clock_ns(CLOCK_MONOTONIC)-start;require(sample.latency_ns<kDeadlineNs,"geometry operation exceeded 10-second deadline");sample.valid=counts.valid_pixels;sample.support=counts.support_points;require(sample.valid==c.expected_valid&&sample.support==c.expected_support&&counts.calibration_generation==source.slot().metadata.calibration_generation,"counts differ from independently supplied fixture expectations");lease={};sample.cpu_ns=clock_ns(CLOCK_PROCESS_CPUTIME_ID)-cpu_start;return sample;
}
Measurement run(const Config& c,const Fixture& f){
 require(std::string(GEOMETRY_BENCHMARK_PROVENANCE)!="unrecorded","meaningful build provenance required");const char* watchdog=std::getenv("GEOMETRY_BENCHMARK_EXTERNAL_WATCHDOG");require(watchdog&&std::string(watchdog)=="1","real external process watchdog required before Context");Measurement out;out.samples.resize(size_t(c.iterations)*c.trials);out.trials.resize(c.trials);
 auto context=Context::shared();VkPhysicalDeviceFeatures physical{};vkGetPhysicalDeviceFeatures(context->physical,&physical);auto enabled=context->enabled_features.features;
 require(c.software?(physical.shaderInt64&&enabled.shaderInt64):(physical.shaderFloat64&&enabled.shaderFloat64),"physical and actually enabled precision feature required (native Float64 is unavailable on tested Intel)");
 out.device=context->properties.deviceName;out.vendor=context->properties.vendorID;out.id=context->properties.deviceID;out.driver=context->properties.driverVersion;out.api=context->properties.apiVersion;out.physical_int64=physical.shaderInt64;out.enabled_int64=enabled.shaderInt64;out.physical_float64=physical.shaderFloat64;out.enabled_float64=enabled.shaderFloat64;
 GeometryCalibration cal;cal.width=f.width;cal.height=f.height;cal.generation=f.generation;cal.depth_scale_m=f.depth_scale;cal.max_observation_depth_m=f.maximum;cal.camera_rays=f.rays;cal.board_from_camera=f.transform;cal.support_roi=f.roi;cal.validate();
 auto pool=std::make_shared<FramePool>(f.width,f.height,context,false);GeometryPreparation geometry(context,cal,c.shaders,c.software?GeometryPrecision::SoftwareBinary64:GeometryPrecision::NativeFloat64);auto source=pool->acquire();require(bool(source),"source allocation failed");out.source_slot=source.index();source.slot().metadata.calibration_generation=f.generation;source.slot().calibration.generation=f.generation;source.slot().calibration.depth_scale=float(f.depth_scale);SourceUpload upload(context,f.source.size());upload.complete(source,f.source);
 for(uint32_t i=0;i<c.warmup;++i)(void)one(geometry,source,c);
 for(uint32_t trial=0;trial<c.trials;++trial){auto allocations=cpp_allocations.load(std::memory_order_relaxed),buffers=context->allocation_count.load();auto cpu_start=clock_ns(CLOCK_PROCESS_CPUTIME_ID);
  for(uint32_t i=0;i<c.iterations;++i){auto& sample=out.samples[size_t(trial)*c.iterations+i];sample=one(geometry,source,c);out.checksum=(out.checksum^((uint64_t(sample.valid)<<32)|sample.support))*1099511628211ULL;require(out.cpu_ns+clock_ns(CLOCK_PROCESS_CPUTIME_ID)-cpu_start<=c.cpu_budget_ms*1000000ULL,"measured process CPU budget exceeded");}
  auto& measured=out.trials[trial];measured.cpu_ns=clock_ns(CLOCK_PROCESS_CPUTIME_ID)-cpu_start;measured.cpp_new_calls=cpp_allocations.load(std::memory_order_relaxed)-allocations;measured.buffer_allocations=context->allocation_count.load()-buffers;out.cpu_ns+=measured.cpu_ns;require(!measured.cpp_new_calls&&!measured.buffer_allocations,"steady benchmark allocated C++ heap or station GPU buffers");
 }
 return out;
}
#endif
#ifdef GEOMETRY_BENCHMARK_SYNTHETIC_DRIVER
constexpr bool kSynthetic=true;
#else
constexpr bool kSynthetic=false;
#endif
std::string report(const Config& c,const Fixture& f,const ShaderIdentity& identity,const Measurement& m){
 std::ostringstream out;out<<"{\"scope\":\"preparation_only_repeated_single_resident_source\",\"offline_validation_only\":"<<(c.offline?"true":"false")<<",\"synthetic_driver\":"<<(kSynthetic?"true":"false")<<",\"gpu_executed\":"<<(!c.offline&&!kSynthetic?"true":"false")<<",\"precision\":"<<quote(c.software?"software_binary64":"native_float64")<<",\"qualified\":false,\"unique_delivery\":false,\"frame_age_qualified\":false,\"capture_qualified\":false,\"full_geometry_qualified\":false,\"throughput_qualified\":false,\"performance_acceptance\":false,\"full_station_qualified\":false,\"cold_setup_upload_warmup_teardown_excluded\":true,\"resident_source_uploads\":"<<(c.offline?0:1)<<",\"geometry_arena_capacity\":3,\"max_in_flight\":1,\"source_slot\":"<<m.source_slot<<",\"fixture_sha256\":"<<quote(f.sha)<<",\"build_provenance\":"<<quote(GEOMETRY_BENCHMARK_PROVENANCE)<<",\"width\":"<<f.width<<",\"height\":"<<f.height<<",\"generation\":"<<f.generation<<",\"expected_valid\":"<<c.expected_valid<<",\"expected_support\":"<<c.expected_support<<",\"expected_counts_externally_supplied\":true,\"iterations_per_trial\":"<<c.iterations<<",\"trials\":"<<c.trials<<",\"warmup_iterations\":"<<c.warmup<<",\"requested_poll_sleep_us\":"<<c.poll_us<<",\"operation_deadline_ns\":"<<kDeadlineNs<<",\"process_cpu_budget_ms\":"<<c.cpu_budget_ms<<",\"measured_process_cpu_ns\":"<<m.cpu_ns<<",\"checksum\":"<<m.checksum<<",\"cpp_allocation_counter_excludes_driver_malloc\":true,\"latency_clock\":\"CLOCK_MONOTONIC\",\"cpu_clock\":\"CLOCK_PROCESS_CPUTIME_ID\",\"latency_interval\":\"before_submit_to_after_finish\",\"cpu_interval\":\"submit_poll_finish_validate_release_including_instrumentation\",\"device\":"<<quote(m.device)<<",\"vendor_id\":"<<m.vendor<<",\"device_id\":"<<m.id<<",\"driver_version\":"<<m.driver<<",\"api_version\":"<<m.api<<",\"physical_shader_int64\":"<<(m.physical_int64?"true":"false")<<",\"enabled_shader_int64\":"<<(m.enabled_int64?"true":"false")<<",\"physical_shader_float64\":"<<(m.physical_float64?"true":"false")<<",\"enabled_shader_float64\":"<<(m.enabled_float64?"true":"false")<<",\"shaders\":{";
 for(unsigned i=0;i<5;++i){if(i)out<<',';out<<quote(kShaderNames[i])<<':'<<quote(identity.sha[i]);}out<<"},\"trial_samples\":[";for(size_t i=0;i<m.trials.size();++i){if(i)out<<',';auto& t=m.trials[i];out<<"{\"process_cpu_ns\":"<<t.cpu_ns<<",\"cpp_new_calls\":"<<t.cpp_new_calls<<",\"station_buffer_allocations\":"<<t.buffer_allocations<<"}";}out<<"],\"samples\":[";
 for(size_t i=0;i<m.samples.size();++i){if(i)out<<',';auto& s=m.samples[i];out<<"{\"trial\":"<<i/c.iterations<<",\"iteration\":"<<i%c.iterations<<",\"submit_to_finish_ns\":"<<s.latency_ns<<",\"process_cpu_ns\":"<<s.cpu_ns<<",\"observed_sleep_wall_ns\":"<<s.sleep_wall_ns<<",\"sleep_calls\":"<<s.sleep_calls<<",\"not_ready_polls\":"<<s.polls<<",\"valid\":"<<s.valid<<",\"support\":"<<s.support<<",\"arena\":"<<s.arena<<"}";}out<<"]}";return out.str();
}
}
int main(int argc,char** argv){try{auto config=options(argc,argv);auto f=fixture(config);auto identity=shaders(config);Measurement measured;if(!config.offline){
#ifdef GEOMETRY_BENCHMARK_CPU_ONLY
 throw std::runtime_error("CPU-only build cannot construct a Vulkan Context");
#else
 measured=run(config,f);
#endif
 }auto text=report(config,f,identity,measured);fresh_file(config.output,text+"\n");std::cout<<text<<std::endl;return 0;}catch(const std::exception& e){std::cerr<<"{\"status\":\"error\",\"error\":"<<quote(e.what())<<"}"<<std::endl;return 1;}}
