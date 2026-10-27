// Offline diagnostic. CPU expectations never include or call the software helper.
// CPU-only build: c++ -std=c++17 -O2 -frounding-math -fno-fast-math
//   -DSOFT_BINARY64_CPU_ONLY qualify_soft_binary64.cpp -o qualify_soft_binary64_cpu
// Full build additionally links the station Vulkan Context/Buffer implementation.
#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif
#ifndef SOFT_BINARY64_CPU_ONLY
#include "so101_station_vulkan/pipeline.hpp"
#endif
#ifndef SOFT_BINARY64_BUILD_PROVENANCE
#define SOFT_BINARY64_BUILD_PROVENANCE "unrecorded"
#endif
namespace {
constexpr uint64_t kNaN=0xfff8000000000000ULL, kMaxRows=4000000;
constexpr char kMagic[8]={'S','F','6','4','P','0','0','1'};
constexpr char kDivSqrtMagic[8]={'S','F','6','4','D','0','0','1'};
constexpr const char* kDivSqrtGenerator="native-volatile-ieee754-rne-divsqrt-v1";
constexpr const char* kGenerator="native-volatile-ieee754-rne-v1";
using Row=std::array<uint64_t,7>;
struct Corpus { bool divsqrt=false; uint64_t seed=0; std::vector<Row> rows; std::vector<uint8_t> bytes; };
std::string quote(const std::string& s) {
 std::string r="\"";for(unsigned char c:s) { if(c=='"'||c=='\\') {r+='\\';r+=c;} else if(c<32)r+='?';else r+=c;}return r+'"';
}
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
void put64(std::vector<uint8_t>& b,uint64_t n){for(unsigned i=0;i<8;++i)b.push_back(uint8_t(n>>(8*i)));}
uint64_t get64(const uint8_t* b){uint64_t n=0;for(unsigned i=0;i<8;++i)n|=uint64_t(b[i])<<(8*i);return n;}
std::vector<uint8_t> read(const std::string& path,size_t cap) {
 std::ifstream in(path,std::ios::binary|std::ios::ate);if(!in)throw std::runtime_error("cannot open "+path);
 auto length=in.tellg();if(length<0||uint64_t(length)>cap)throw std::runtime_error("artifact exceeds size cap");
 std::vector<uint8_t> b(static_cast<size_t>(length));in.seekg(0);in.read(reinterpret_cast<char*>(b.data()),length);if(!in)throw std::runtime_error("artifact read failed");return b;
}
void fresh(const std::string& path,const std::vector<uint8_t>& bytes) {
 int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);if(fd<0)throw std::runtime_error("output must be a fresh file: "+path);
 size_t done=0;while(done<bytes.size()){ssize_t n=::write(fd,bytes.data()+done,bytes.size()-done);if(n<=0){::close(fd);throw std::runtime_error("output write failed (partial artifact retained)");}done+=size_t(n);}
 if(::close(fd))throw std::runtime_error("output close failed");
}
[[maybe_unused]] void fresh_text(const std::string& path,const std::string& s){fresh(path,std::vector<uint8_t>(s.begin(),s.end()));}
uint64_t canonical(uint64_t b){return(b&0x7fffffffffffffffULL)>0x7ff0000000000000ULL?kNaN:b;}
double value(uint64_t b){double d;std::memcpy(&d,&b,8);return d;}
uint64_t bits(double d){uint64_t b;std::memcpy(&b,&d,8);return b;}
struct Environment {
 fenv_t saved{};
#if defined(__SSE__)
 unsigned csr=0;
#endif
 Environment(){if(std::fegetenv(&saved))throw std::runtime_error("fegetenv failed");
#if defined(__SSE__)
 csr=_mm_getcsr();
#endif
 // Mask traps and clear prior exception flags for signalling-NaN test inputs.
 if(std::feholdexcept(&saved)||std::fesetround(FE_TONEAREST))throw std::runtime_error("cannot set RNE environment");
#if defined(__SSE__)
 _mm_setcsr((_mm_getcsr()|0x1f80u)&~((1u<<15)|(1u<<6)));
#endif
 }
 ~Environment(){std::fesetenv(&saved);
#if defined(__SSE__)
 _mm_setcsr(csr);
#endif
 }
};
Row oracle(uint64_t a,uint64_t b,uint32_t f,bool divsqrt=false) {
 volatile double x=value(a),y=value(b);volatile double sum=x+y,product=x*y;
 float fv;std::memcpy(&fv,&f,4);volatile float narrow=fv;volatile double wide=narrow;
 uint64_t flags=(x<y?1u:0u)|(x<=y?2u:0u)|(std::isfinite(x)?4u:0u)|(std::isfinite(y)?8u:0u);
 if(divsqrt){volatile double quotient=x/y,root=std::sqrt(x);return {a,b,f,canonical(bits(quotient)),canonical(bits(root)),canonical(bits(wide)),flags};}
 return {a,b,f,canonical(bits(sum)),canonical(bits(product)),canonical(bits(wide)),flags};
}
void oracle_contract() {
 static_assert(sizeof(double)==8&&sizeof(float)==4&&std::numeric_limits<double>::is_iec559&&std::numeric_limits<float>::is_iec559);
 if(std::fegetround()!=FE_TONEAREST||oracle(1,0x3fe0000000000000ULL,1)[4]!=0||oracle(3,0x3fe0000000000000ULL,1)[4]!=2||oracle(0x8000000000000000ULL,0x3ff0000000000000ULL,0x80000000)[4]!=0x8000000000000000ULL||oracle(0,0,1)[5]!=0x36a0000000000000ULL||oracle(0x7ff0000000000000ULL,0,0)[4]!=kNaN)
  throw std::runtime_error("native oracle contract failed (RNE/FTZ/DAZ/NaN/signed zero)");
}
Corpus parse(const std::string& path) {
 Corpus c;c.bytes=read(path,24+kMaxRows*56);
 if(c.bytes.size()<24)throw std::runtime_error("bad corpus magic/header");
 c.divsqrt=std::memcmp(c.bytes.data(),kDivSqrtMagic,8)==0;
 if(!c.divsqrt&&std::memcmp(c.bytes.data(),kMagic,8))throw std::runtime_error("bad corpus magic/header");
 uint64_t n=get64(c.bytes.data()+8);c.seed=get64(c.bytes.data()+16);
 if(!n||n>kMaxRows||c.bytes.size()!=24+n*56)throw std::runtime_error("bad corpus count/length");
 c.rows.resize(n);for(size_t i=0;i<n;++i){for(unsigned w=0;w<7;++w)c.rows[i][w]=get64(c.bytes.data()+24+i*56+w*8);if(c.rows[i][2]>UINT32_MAX||c.rows[i][6]>15)throw std::runtime_error("bad corpus word range");}
 return c;
}
void validate(const Corpus& c) {
 for(size_t i=0;i<c.rows.size();++i){auto& r=c.rows[i];if(oracle(r[0],r[1],uint32_t(r[2]),c.divsqrt)!=r)throw std::runtime_error("corpus expected mismatch at row "+std::to_string(i));}
}
uint64_t number(const std::string& s){size_t end=0;auto n=std::stoull(s,&end,10);if(end!=s.size()||s.empty()||s[0]=='-')throw std::runtime_error("invalid unsigned decimal");return n;}
uint64_t hexword(const std::string& s){size_t end=0;if(s.empty()||s.size()>16||s[0]=='-')throw std::runtime_error("bad TestFloat hex word");auto n=std::stoull(s,&end,16);if(end!=s.size())throw std::runtime_error("bad TestFloat hex word");return n;}
uint64_t append_testfloat(Corpus& c,const std::string& dir) {
 uint64_t total=0;
 for(const auto& name:(c.divsqrt?std::vector<std::string>{"f64_div","f64_sqrt","f32_to_f64"}:std::vector<std::string>{"f64_add","f64_mul","f32_to_f64"})) {
  std::ifstream in(std::filesystem::path(dir)/(std::string(name)+".txt"));if(!in)throw std::runtime_error("missing pre-generated TestFloat file: "+std::string(name));
  std::string line;uint64_t line_no=0;
  while(std::getline(in,line)){++line_no;std::istringstream stream(line);std::vector<std::string> words;std::string w;while(stream>>w){if(w[0]=='#')break;words.push_back(w);}if(words.empty())continue;
   bool conversion=name=="f32_to_f64",unary=conversion||name=="f64_sqrt";if(words.size()!=(unary?3u:4u))throw std::runtime_error("bad TestFloat columns");
   uint64_t a=hexword(words[0]),b=unary?0x3ff0000000000000ULL:hexword(words[1]);uint64_t expected=hexword(words[unary?1:2]),flags=hexword(words.back());
   if(flags>31||(conversion&&a>UINT32_MAX))throw std::runtime_error("bad TestFloat range");
   Row row=conversion?oracle(oracle(0,0,uint32_t(a))[5],b,uint32_t(a),c.divsqrt):oracle(a,b,uint32_t(a),c.divsqrt);
   unsigned field=conversion?5:(name=="f64_add"||name=="f64_div")?3:4;
   if(row[field]!=canonical(expected))throw std::runtime_error("native oracle disagrees with TestFloat "+std::string(name)+" line "+std::to_string(line_no));
   if(c.rows.size()==kMaxRows)throw std::runtime_error("TestFloat corpus exceeds row cap");
   c.rows.push_back(row);++total;
  }
  if(!in.eof())throw std::runtime_error("TestFloat read failed");
 }
 return total;
}
Corpus generate(uint64_t count,uint64_t seed,const std::string& testfloat,uint64_t& upstream,bool divsqrt=false) {
 if(count<16384||count>kMaxRows||!seed)throw std::runtime_error("count must be 16384..4000000 and seed nonzero");
 Corpus c;c.divsqrt=divsqrt;c.seed=seed;c.rows.reserve(count);auto add=[&](uint64_t a,uint64_t b,uint32_t f){if(c.rows.size()<count)c.rows.push_back(oracle(a,b,f,c.divsqrt));};
 const uint64_t edges[]={0,1,2,3,0x000ffffffffffffeULL,0x000fffffffffffffULL,0x0010000000000000ULL,0x0010000000000001ULL,0x3c90000000000000ULL,0x3ca0000000000000ULL,0x3cb0000000000000ULL,0x3fdfffffffffffffULL,0x3fe0000000000000ULL,0x3fefffffffffffffULL,0x3ff0000000000000ULL,0x3ff0000000000001ULL,0x3ff0000000000002ULL,0x4000000000000000ULL,0x7fdfffffffffffffULL,0x7feffffffffffffeULL,0x7fefffffffffffffULL,0x7ff0000000000000ULL,0x7ff0000000000001ULL,0x7ff8000000000000ULL,0x7fffffffffffffffULL};
 const uint32_t f32[]={0,1,2,0x007fffff,0x00800000,0x3f000000,0x3f800000,0x3f800001,0x7f7fffff,0x7f800000,0x7f800001,0x7fc00000,0x80000000,0x80000001,0xff800000,0xffffffff};
 size_t fi=0;for(auto a:edges)for(auto b:edges)for(unsigned sign=0;sign<4;++sign)add(a|uint64_t(sign&1)<<63,b|uint64_t(sign>>1)<<63,f32[fi++%16]);
 // Exact half-way ties at normal exponents and underflow ties, both signs.
 for(uint64_t e=54;e<2047;++e)for(uint64_t parity=0;parity<2;++parity)add((e<<52)|parity,(e-53)<<52,f32[fi++%16]);
 for(uint64_t a=1;a<2048;++a){add(a,0x3fe0000000000000ULL,1);add(a|0x8000000000000000ULL,0x3fe0000000000000ULL,0x80000001);}
 if(divsqrt)for(unsigned i=1;i<4096;++i){double x=double(i)/32;uint64_t square=bits(x*x);add(square-1,bits(x),uint32_t(i));add(square,bits(x),uint32_t(i));add(square+1,bits(x),uint32_t(i));}
 uint64_t rng=seed;auto random=[&](){rng^=rng<<13;rng^=rng>>7;rng^=rng<<17;return rng;};
 while(c.rows.size()<count){auto a=random(),b=random();auto f=uint32_t(random());if(c.rows.size()%3==0){a=oracle(0,0,uint32_t(a))[5];b=oracle(0,0,uint32_t(b))[5];}add(a,b,f);}
 upstream=testfloat.empty()?0:append_testfloat(c,testfloat);
 const char* magic=c.divsqrt?kDivSqrtMagic:kMagic;c.bytes.assign(magic,magic+8);put64(c.bytes,c.rows.size());put64(c.bytes,seed);for(auto& r:c.rows)for(auto w:r)put64(c.bytes,w);return c;
}
std::string summary(const Corpus& c,const char* mode,uint64_t upstream=0) {
 return "{\"mode\":"+quote(mode)+",\"generator\":"+quote(c.divsqrt?kDivSqrtGenerator:kGenerator)+(c.divsqrt?",\"corpus_kind\":\"SF64D001\"":"")+",\"corpus_sha256\":"+quote(sha256(c.bytes))+",\"case_count\":"+std::to_string(c.rows.size())+",\"seed\":"+std::to_string(c.seed)+",\"testfloat_rows\":"+std::to_string(upstream)+",\"gpu_executed\":false,\"geometry_qualified\":false,\"full_station_qualified\":false,\"exception_flags_qualified\":false}";
}
uint32_t load32(const uint8_t* b){return uint32_t(b[0])|uint32_t(b[1])<<8|uint32_t(b[2])<<16|uint32_t(b[3])<<24;}
void spirv_contract(const std::vector<uint8_t>& b) {
 if(b.size()<20||b.size()%4||load32(b.data())!=0x07230203)throw std::runtime_error("invalid SPIR-V header");
 bool int64=false,float64=false;
 for(size_t i=5;i<b.size()/4;){uint32_t word=load32(b.data()+i*4);unsigned n=word>>16,op=word&65535;
  if(!n||i+n>b.size()/4)throw std::runtime_error("invalid SPIR-V instruction");
  if(op==17){if(n!=2)throw std::runtime_error("invalid SPIR-V capability");uint32_t cap=load32(b.data()+(i+1)*4);int64|=cap==11;float64|=cap==10;}i+=n;
 }
 if(!int64||float64)throw std::runtime_error("SPIR-V must declare Int64 and must not declare Float64");
}
bool divsqrt_spirv(const std::vector<uint8_t>& b) {
 for(size_t i=5;i<b.size()/4;){unsigned n=load32(b.data()+i*4)>>16,op=load32(b.data()+i*4)&65535;
  if(!n||i+n>b.size()/4)throw std::runtime_error("invalid SPIR-V kind instruction");
  if(op==5&&n>=3){const char* text=reinterpret_cast<const char*>(b.data()+(i+2)*4);size_t size=(n-2)*4;
   const void* end=std::memchr(text,0,size);if(!end)throw std::runtime_error("unterminated SPIR-V name");
   if(std::string(text,static_cast<const char*>(end)-text)=="probe_input_SF64D001")return true;
  }i+=n;
 }return false;
}
void shader_kind(const std::vector<uint8_t>& spv,bool divsqrt) {
 if(divsqrt_spirv(spv)!=divsqrt)throw std::runtime_error("corpus/shader arithmetic kind mismatch");
}
#ifndef SOFT_BINARY64_CPU_ONLY
using namespace so101_station_vulkan;
constexpr uint32_t kBatch=16384,kGroup=256;
void check(VkResult r,const char* op){if(r!=VK_SUCCESS)throw std::runtime_error(std::string(op)+": "+std::to_string(r));}
struct Probe {
 std::shared_ptr<Context> context=Context::shared();Buffer input,output;
 VkDescriptorSetLayout descriptors=VK_NULL_HANDLE;VkDescriptorPool pool=VK_NULL_HANDLE;
 VkPipelineLayout layout=VK_NULL_HANDLE;VkShaderModule module=VK_NULL_HANDLE;VkPipeline pipeline=VK_NULL_HANDLE;
 VkCommandPool commands=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;bool in_flight=false;
 ~Probe(){
  // Never free pending objects or enter Context's destructor after an exception.
  // The independently owned process watchdog must terminate any driver hang.
  if(in_flight)std::_Exit(2);
  auto d=context->device;if(fence)vkDestroyFence(d,fence,nullptr);if(commands)vkDestroyCommandPool(d,commands,nullptr);
  if(pipeline)vkDestroyPipeline(d,pipeline,nullptr);
  if(module)vkDestroyShaderModule(d,module,nullptr);
  if(layout)vkDestroyPipelineLayout(d,layout,nullptr);
  if(pool)vkDestroyDescriptorPool(d,pool,nullptr);
  if(descriptors)vkDestroyDescriptorSetLayout(d,descriptors,nullptr);
 }
};
int run(const Corpus& c,const std::string& shader,const std::string& directory) {
 if(std::string(SOFT_BINARY64_BUILD_PROVENANCE)=="unrecorded")throw std::runtime_error("full build requires source/helper hashes and compile flags in SOFT_BINARY64_BUILD_PROVENANCE");
 const char* watchdog=std::getenv("SOFT_BINARY64_EXTERNAL_WATCHDOG");if(!watchdog||std::string(watchdog)!="1")throw std::runtime_error("external process watchdog required; launcher must set SOFT_BINARY64_EXTERNAL_WATCHDOG=1");
 auto spv=read(shader,4*1024*1024);spirv_contract(spv);
 shader_kind(spv,c.divsqrt);
 if(!std::filesystem::create_directory(directory))throw std::runtime_error("output directory must be fresh");
 // All output storage and Vulkan resources are allocated before the first batch.
 std::vector<uint8_t> output_bytes(c.rows.size()*32);std::vector<uint32_t> code(spv.size()/4);
 for(size_t i=0;i<code.size();++i)code[i]=uint32_t(spv[i*4])|uint32_t(spv[i*4+1])<<8|uint32_t(spv[i*4+2])<<16|uint32_t(spv[i*4+3])<<24;
 Probe p;auto d=p.context->device;VkPhysicalDeviceFeatures physical{};vkGetPhysicalDeviceFeatures(p.context->physical,&physical);
 const auto& enabled=p.context->enabled_features.features;
 if(!physical.shaderInt64||!enabled.shaderInt64)throw std::runtime_error("physical and enabled shaderInt64 required");
 const auto& limits=p.context->properties.limits;
 if(limits.maxStorageBufferRange<kBatch*32||limits.maxPushConstantsSize<4||limits.maxComputeWorkGroupInvocations<kGroup||limits.maxComputeWorkGroupSize[0]<kGroup||limits.maxComputeWorkGroupCount[0]<kBatch/kGroup||limits.maxPerStageDescriptorStorageBuffers<2||limits.maxDescriptorSetStorageBuffers<2)throw std::runtime_error("device probe limits exceeded");
 auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
 p.input.create(*p.context,kBatch*24,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,host);p.output.create(*p.context,kBatch*32,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,host);
 std::array<VkDescriptorSetLayoutBinding,2> bindings{};for(unsigned i=0;i<2;++i)bindings[i]={i,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr};
 VkDescriptorSetLayoutCreateInfo desc{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};desc.bindingCount=2;desc.pBindings=bindings.data();check(vkCreateDescriptorSetLayout(d,&desc,nullptr,&p.descriptors),"descriptor layout");
 VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT,0,4};VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};layout.setLayoutCount=1;layout.pSetLayouts=&p.descriptors;layout.pushConstantRangeCount=1;layout.pPushConstantRanges=&range;check(vkCreatePipelineLayout(d,&layout,nullptr,&p.layout),"pipeline layout");
 VkShaderModuleCreateInfo module{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};module.codeSize=spv.size();module.pCode=code.data();check(vkCreateShaderModule(d,&module,nullptr,&p.module),"shader module");
 VkComputePipelineCreateInfo pipeline{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};pipeline.layout=p.layout;pipeline.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;pipeline.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pipeline.stage.module=p.module;pipeline.stage.pName="main";check(vkCreateComputePipelines(d,VK_NULL_HANDLE,1,&pipeline,nullptr,&p.pipeline),"pipeline");
 VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,2};VkDescriptorPoolCreateInfo pool{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};pool.maxSets=1;pool.poolSizeCount=1;pool.pPoolSizes=&size;check(vkCreateDescriptorPool(d,&pool,nullptr,&p.pool),"descriptor pool");
 VkDescriptorSetAllocateInfo alloc{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};alloc.descriptorPool=p.pool;alloc.descriptorSetCount=1;alloc.pSetLayouts=&p.descriptors;VkDescriptorSet set;check(vkAllocateDescriptorSets(d,&alloc,&set),"descriptor set");
 std::array<VkDescriptorBufferInfo,2> buffers{{{p.input.buffer,0,p.input.size},{p.output.buffer,0,p.output.size}}};std::array<VkWriteDescriptorSet,2> writes{};
 for(unsigned i=0;i<2;++i){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=set;writes[i].dstBinding=i;writes[i].descriptorCount=1;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;writes[i].pBufferInfo=&buffers[i];}vkUpdateDescriptorSets(d,2,writes.data(),0,nullptr);
 VkCommandPoolCreateInfo commands{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};commands.queueFamilyIndex=p.context->family;check(vkCreateCommandPool(d,&commands,nullptr,&p.commands),"command pool");
 VkCommandBufferAllocateInfo command_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};command_info.commandPool=p.commands;command_info.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;command_info.commandBufferCount=1;VkCommandBuffer command;check(vkAllocateCommandBuffers(d,&command_info,&command),"command buffer");
 VkFenceCreateInfo fence{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};check(vkCreateFence(d,&fence,nullptr,&p.fence),"fence");
 uint64_t batches=0,mismatches=0,first_row=UINT64_MAX,first_word=0,first_actual=0,first_expected=0;
 for(size_t base=0;base<c.rows.size();base+=kBatch){uint32_t count=uint32_t(std::min<size_t>(kBatch,c.rows.size()-base));auto* input=static_cast<uint64_t*>(p.input.mapped);
  for(size_t i=0;i<count;++i)for(unsigned w=0;w<3;++w)input[i*3+w]=c.rows[base+i][w];
  std::memset(p.output.mapped,0xcd,p.output.size);check(vkResetCommandPool(d,p.commands,0),"reset command pool");check(vkResetFences(d,1,&p.fence),"reset fence");
  VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};begin.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;check(vkBeginCommandBuffer(command,&begin),"begin");
  VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};barrier.srcAccessMask=VK_ACCESS_HOST_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT;vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,1,&barrier,0,nullptr,0,nullptr);
  vkCmdBindPipeline(command,VK_PIPELINE_BIND_POINT_COMPUTE,p.pipeline);vkCmdBindDescriptorSets(command,VK_PIPELINE_BIND_POINT_COMPUTE,p.layout,0,1,&set,0,nullptr);vkCmdPushConstants(command,p.layout,VK_SHADER_STAGE_COMPUTE_BIT,0,4,&count);vkCmdDispatch(command,(count+kGroup-1)/kGroup,1,1);
  barrier.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_HOST_READ_BIT;vkCmdPipelineBarrier(command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&barrier,0,nullptr,0,nullptr);check(vkEndCommandBuffer(command),"end");
  VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};submit.commandBufferCount=1;submit.pCommandBuffers=&command;{std::lock_guard<std::mutex> lock(p.context->queue_mutex);check(vkQueueSubmit(p.context->queue,1,&submit,p.fence),"submit");}p.in_flight=true;
  auto completion=vkWaitForFences(d,1,&p.fence,VK_TRUE,10000000000ULL);
  if(completion!=VK_SUCCESS&&completion!=VK_ERROR_DEVICE_LOST){std::cerr<<"bounded fence wait failed; pending resources retained until process exit"<<std::endl;std::_Exit(2);}p.in_flight=false;check(completion,"bounded fence completion");++batches;
  auto* output=static_cast<const uint64_t*>(p.output.mapped);
  for(size_t i=0;i<count;++i)for(unsigned w=0;w<4;++w){uint64_t actual=output[i*4+w],expected=c.rows[base+i][w+3];size_t offset=((base+i)*4+w)*8;for(unsigned byte=0;byte<8;++byte)output_bytes[offset+byte]=uint8_t(actual>>(byte*8));
   if(actual!=expected){if(!mismatches){first_row=base+i;first_word=w;first_actual=actual;first_expected=expected;}++mismatches;}}
 }
 const auto& props=p.context->properties;
 std::ostringstream result;result<<"{\"mode\":\"software_binary64_gpu_bits\",\"gpu_executed\":true,\"device\":"<<quote(props.deviceName)<<",\"vendor_id\":"<<props.vendorID<<",\"device_id\":"<<props.deviceID<<",\"driver_version\":"<<props.driverVersion<<",\"api_version\":"<<props.apiVersion<<",\"physical_shader_float64\":"<<physical.shaderFloat64<<",\"enabled_shader_float64\":"<<enabled.shaderFloat64<<",\"physical_shader_int64\":"<<physical.shaderInt64<<",\"enabled_shader_int64\":"<<enabled.shaderInt64<<",\"case_count\":"<<c.rows.size()<<",\"seed\":"<<c.seed<<",\"batch_cap\":"<<kBatch<<",\"batch_count\":"<<batches<<",\"mismatch_word_count\":"<<mismatches<<",\"corpus_sha256\":"<<quote(sha256(c.bytes))<<",\"spirv_sha256\":"<<quote(sha256(spv))<<",\"output_sha256\":"<<quote(sha256(output_bytes))<<",\"build_provenance\":"<<quote(SOFT_BINARY64_BUILD_PROVENANCE)<<",\"generator\":"<<quote(c.divsqrt?kDivSqrtGenerator:kGenerator)<<(c.divsqrt?",\"corpus_kind\":\"SF64D001\"":"")<<",\"corpus_artifact\":"<<quote(std::filesystem::absolute(directory+"/corpus.bin").string())<<",\"output_artifact\":"<<quote(std::filesystem::absolute(directory+"/actual.bin").string())<<",\"geometry_qualified\":false,\"full_station_qualified\":false,\"performance_claimed\":false,\"exception_flags_qualified\":false";
 if(mismatches)result<<",\"first_failure\":{\"row\":"<<first_row<<",\"word\":"<<first_word<<",\"expected\":"<<first_expected<<",\"actual\":"<<first_actual<<"}";
 result<<"}";
 fresh(directory+"/corpus.bin",c.bytes);fresh(directory+"/probe.spv",spv);fresh(directory+"/actual.bin",output_bytes);fresh_text(directory+"/result.json",result.str()+"\n");std::cout<<result.str()<<std::endl;return mismatches?1:0;
}
#endif
}
int main(int argc,char** argv) {
 try {
  // Crucially this dispatch precedes any construction of a Vulkan Context.
  if(argc>=3&&(std::string(argv[1])=="--generate"||std::string(argv[1])=="--generate-divsqrt")){
   uint64_t count=65536,seed=17;std::string testfloat;
   for(int i=3;i<argc;i+=2){if(i+1>=argc)throw std::runtime_error("missing option value");std::string option=argv[i];if(option=="--count")count=number(argv[i+1]);else if(option=="--seed")seed=number(argv[i+1]);else if(option=="--testfloat-dir")testfloat=argv[i+1];else throw std::runtime_error("unknown generation option");}
   Environment environment;oracle_contract();uint64_t upstream=0;auto c=generate(count,seed,testfloat,upstream,std::string(argv[1])=="--generate-divsqrt");fresh(argv[2],c.bytes);std::cout<<summary(c,"cpu_generate",upstream)<<std::endl;return 0;
  }
  if(argc==3&&std::string(argv[1])=="--validate"){Environment environment;oracle_contract();auto c=parse(argv[2]);validate(c);std::cout<<summary(c,"cpu_validate")<<std::endl;return 0;}
  if((argc==3||argc==5)&&std::string(argv[1])=="--validate-spirv"){auto b=read(argv[2],4*1024*1024);spirv_contract(b);if(argc==5){std::string kind=argv[4];if(std::string(argv[3])!="--kind"||(kind!="SF64P001"&&kind!="SF64D001"))throw std::runtime_error("invalid explicit corpus kind");shader_kind(b,kind=="SF64D001");}std::cout<<"{\"mode\":\"offline_spirv_validate\",\"int64\":true,\"float64\":false,\"gpu_executed\":false,\"spirv_sha256\":"<<quote(sha256(b))<<(divsqrt_spirv(b)?",\"corpus_kind\":\"SF64D001\"":"")<<"}"<<std::endl;return 0;}
#ifndef SOFT_BINARY64_CPU_ONLY
  if(argc==5&&std::string(argv[1])=="--vulkan"){auto c=parse(argv[2]);return run(c,argv[3],argv[4]);}
#endif
  throw std::runtime_error("usage: --generate|--generate-divsqrt NEW_CORPUS [--count 65536 --seed 17 --testfloat-dir DIR] | --validate CORPUS | --validate-spirv SPIRV | --vulkan CORPUS SPIRV NEW_OUTPUT_DIR");
 }catch(const std::exception& e){std::cerr<<"{\"status\":\"error\",\"error\":"<<quote(e.what())<<"}"<<std::endl;return 1;}
}
