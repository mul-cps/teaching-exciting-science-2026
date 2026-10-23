#include "so101_station_vulkan/pipeline.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sys/resource.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
using namespace so101_station_vulkan;
using json=nlohmann::json;
using ns=std::chrono::nanoseconds;
using Clock=std::chrono::steady_clock;
static int64_t now(){return std::chrono::duration_cast<ns>(Clock::now().time_since_epoch()).count();}
static int64_t cpu(clockid_t id){timespec t{};if(clock_gettime(id,&t))throw std::runtime_error("clock_gettime");return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
static void sleep_until(int64_t t){timespec at{t/1000000000,t%1000000000};while(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&at,nullptr)==EINTR){} }
static json quantiles(std::vector<double> v){if(v.empty())return nullptr;std::sort(v.begin(),v.end());auto p=[&](double q){return v[size_t((v.size()-1)*q)];};return{{"n",v.size()},{"p50",p(.5)},{"p95",p(.95)},{"p99",p(.99)},{"max",v.back()}};}
static void read_file(const std::string& path,void* p,size_t n){std::ifstream f(path,std::ios::binary);if(!f.read(static_cast<char*>(p),n)||f.peek()!=EOF)throw std::runtime_error("fixture bytes");}
static std::vector<unsigned> affinity(){cpu_set_t set;CPU_ZERO(&set);sched_getaffinity(0,sizeof(set),&set);std::vector<unsigned> r;for(unsigned i=0;i<CPU_SETSIZE;i++)if(CPU_ISSET(i,&set))r.push_back(i);return r;}
static void write_json(const std::string& path,const json& j){std::ofstream f(path);f<<j.dump(2)<<'\n';if(!f)throw std::runtime_error("write failed");}
int main(int argc,char** argv)try{
 if(argc!=2)throw std::runtime_error("config json argument required");std::ifstream cfg(argv[1]);json c;cfg>>c;
 const std::string kind=c.at("kind");const int64_t begin=c.at("start_ns"),duration=int64_t(c.at("seconds").get<double>()*1e9),end=begin+duration;
 const auto p0=cpu(CLOCK_PROCESS_CPUTIME_ID);const auto wall0=now();json result={{"schema","station-headroom-v1"},{"kind",kind},{"physical",false},{"pid",getpid()},{"affinity",affinity()}};
 if(kind=="control"){
  constexpr int64_t period=8000000; // actual native teleop timer125Hz, synthetic workload only
  std::vector<double> lateness,work;lateness.reserve(duration/period+1);work.reserve(duration/period+1);
  uint32_t state=0x8c32a5u;volatile uint32_t sink=0;uint64_t skipped=0,missed=0;int64_t due=begin;
  sleep_until(begin);auto pcpu=cpu(CLOCK_PROCESS_CPUTIME_ID);auto st=now();
  while(due<end){sleep_until(due);auto awake=now();if(awake>=end)break;auto t=cpu(CLOCK_THREAD_CPUTIME_ID);
   // Fixed2048 integer-control surrogate, no ROS, serial, actuators or devices.
   for(unsigned k=0;k<2048;k++){state^=state<<13;state^=state>>17;state^=state<<5;}sink=state;
   auto stop=now();lateness.push_back((awake-due)/1e6);work.push_back((cpu(CLOCK_THREAD_CPUTIME_ID)-t)/1e6);missed+=stop>=due+period;
   due+=period;if(due<stop){auto n=(stop-due)/period+1;skipped+=n;due+=n*period;}
  }
  sleep_until(end);if(lateness.capacity()!=size_t(duration/period+1)||work.capacity()!=size_t(duration/period+1))throw std::runtime_error("control vector grew");result["measurement_valid"]=true;result["timing_vector_capacities_unchanged"]=true;result.update({{"steady_wall_s",(now()-st)/1e9},{"steady_process_cpu_ms",(cpu(CLOCK_PROCESS_CPUTIME_ID)-pcpu)/1e6},{"frequency_hz",125},{"fixed_work_iterations",2048},{"lateness_ms",quantiles(lateness)},{"work_cpu_ms",quantiles(work)},{"missed_deadlines",missed},{"skipped_ticks",skipped},{"checksum",uint32_t(sink)}});
 }else if(kind=="perception"){
  const unsigned w=640,h=480;const size_t n=size_t(w)*h;const double hz=c.at("hz");const int64_t frame_period=int64_t(1e9/hz),poll_period=2000000;const std::string mode=c.at("mode"),workload=c.at("workload");
  Pipeline pipeline(w,h,mode,c.at("shaders"),false,true);std::vector<uint8_t> fixture(input_bytes(w,h));Calibration k;
  if(workload=="recorded"){
   const std::string root=c.at("fixture");std::ifstream f(root+"/case.json");json j;f>>j;k.fx=j.at("fx");k.fy=j.at("fy");k.cx=j.at("cx");k.cy=j.at("cy");k.depth_scale=j.at("depth_scale");read_file(root+"/rgb.bin",fixture.data(),n*3);read_file(root+"/depth.bin",fixture.data()+n*3,n*2);
  }else if(workload=="dense"){
   k.cx=w/2;k.cy=h/2;uint32_t random=0x1012026u;
   for(unsigned i=0;i<n;i++){unsigned x=i%w,y=i/w;auto* color=fixture.data()+3*i;random=random*1664525u+1013904223u;if(y<h/3){color[0]=uint8_t(random>>24);color[1]=uint8_t(random>>16);color[2]=uint8_t(random>>8);}else{unsigned group=(x/(w/8))%4;color[group%3]=255;if(group==3)color[0]=color[1]=255;}uint16_t z=(x+y)%71==0?0:((x>w/2&&y>h/2)?520:500);fixture[3*n+2*i]=z&255;fixture[3*n+2*i+1]=z>>8;}
  }else throw std::runtime_error("workload");
  // Reference and warmup are outside the steady measurement. NativeCPU oracle checks compact output.
  Slot reference;reference.cpu_masks.resize(n);reference.cpu_parents.resize(n);reference.cpu_depth.resize(n);reference.cpu_xyz.resize(n*4);reference.cpu_work.resize(n);reference.cpu_dilated.resize(n);reference.cpu_morph_counts.resize(n);reference.cpu_morph_roots.resize(n*9);reference.upload.mapped=fixture.data();reference.calibration=k;cpu_reference(reference,w,h);
  auto verify=[&](const FrameLease& f){const auto& s=f.slot();if(s.component_count!=reference.component_count)throw std::runtime_error("component count mismatch");for(unsigned i=0;i<s.component_count;i++)if(!(s.cpu_components[i]==reference.cpu_components[i]))throw std::runtime_error("compact output mismatch");};
  for(unsigned i=0;i<10;i++){auto f=pipeline.acquire();std::memcpy(f.slot().upload.mapped,fixture.data(),fixture.size());f.slot().calibration=k;pipeline.submit(f);while(!pipeline.ready(f))std::this_thread::sleep_for(std::chrono::milliseconds(2));pipeline.finish(f);verify(f);}
  const auto prepared=now();if(prepared>=begin)throw std::runtime_error("startup/warmup crossed frozen measurement start");
  const auto alloc0=pipeline.context()?pipeline.context()->allocation_count.load():0;
  std::vector<double> lat,age,submit,copy,finish,poll;std::array<std::vector<double>,8> stages;const size_t reserve=size_t(duration/frame_period)+50;for(auto* v:{&lat,&age,&submit,&copy,&finish,&poll})v->reserve(reserve);for(auto& v:stages)v.reserve(reserve);poll.reserve(size_t(duration/poll_period)+100);
  const auto poll_capacity=poll.capacity(),lat_capacity=lat.capacity(),age_capacity=age.capacity(),submit_capacity=submit.capacity(),copy_capacity=copy.capacity(),finish_capacity=finish.capacity();std::array<size_t,8> stage_capacity{};for(unsigned i=0;i<8;i++)stage_capacity[i]=stages[i].capacity();
  FrameLease active,pending;int64_t active_capture=0,active_submit=0,pending_capture=0,last_arrival=-1;uint64_t offered=0,source_skipped=0,replaced=0,dropped=0,completed=0,polls=0;unsigned comp_last=0;
  sleep_until(begin);const auto pcpu=cpu(CLOCK_PROCESS_CPUTIME_ID),tcpu=cpu(CLOCK_THREAD_CPUTIME_ID);rusage ru0{};getrusage(RUSAGE_SELF,&ru0);const auto st=now();int64_t tick=begin;
  auto dispatch=[&](){if(!active&&pending){active=std::move(pending);active_capture=pending_capture;active_submit=now();auto t=cpu(CLOCK_THREAD_CPUTIME_ID);pipeline.submit(active);submit.push_back((cpu(CLOCK_THREAD_CPUTIME_ID)-t)/1e6);}};
  while(now()<end){
   if(active){auto t=cpu(CLOCK_THREAD_CPUTIME_ID);bool ready=pipeline.ready(active);poll.push_back((cpu(CLOCK_THREAD_CPUTIME_ID)-t)/1e6);polls++;if(ready){auto ft=cpu(CLOCK_THREAD_CPUTIME_ID);pipeline.finish(active);finish.push_back((cpu(CLOCK_THREAD_CPUTIME_ID)-ft)/1e6);auto done=now();if(done<end){verify(active);lat.push_back((done-active_submit)/1e6);age.push_back((done-active_capture)/1e6);for(unsigned s=0;s<8;s++)stages[s].push_back(active.slot().gpu_stage_ms[s]);completed++;comp_last=active.slot().component_count;}active={};dispatch();}}
   auto tnow=now();const int64_t arrival=(tnow-begin)/frame_period;
   if(tnow<end&&arrival>last_arrival){const uint64_t delta=arrival-last_arrival;offered+=delta;source_skipped+=delta-1;last_arrival=arrival;if(pending){pending={};replaced++;}auto f=pipeline.acquire();if(!f)dropped++;else{auto t=cpu(CLOCK_THREAD_CPUTIME_ID);std::memcpy(f.slot().upload.mapped,fixture.data(),fixture.size());copy.push_back((cpu(CLOCK_THREAD_CPUTIME_ID)-t)/1e6);f.slot().calibration=k;f.slot().metadata.frame_number=arrival;pending_capture=begin+arrival*frame_period;pending=std::move(f);dispatch();}}
   tick+=poll_period;auto tn=now();if(tick<=tn)tick+=((tn-tick)/poll_period+1)*poll_period;sleep_until(std::min(tick,end));
  }
  const auto wall_end=now(),pcpu_end=cpu(CLOCK_PROCESS_CPUTIME_ID),tcpu_end=cpu(CLOCK_THREAD_CPUTIME_ID);
  bool grew=poll.capacity()!=poll_capacity||lat.capacity()!=lat_capacity||age.capacity()!=age_capacity||submit.capacity()!=submit_capacity||copy.capacity()!=copy_capacity||finish.capacity()!=finish_capacity;for(unsigned i=0;i<8;i++)grew|=stages[i].capacity()!=stage_capacity[i];if(grew)throw std::runtime_error("diagnostic timing vector grew during steady window");result["timing_vector_capacities_unchanged"]=true;result["measurement_valid"]=true;result["poll_reserved_capacity"]=poll_capacity;rusage ru1{};getrusage(RUSAGE_SELF,&ru1);
  result.update({{"mode",mode},{"backend",pipeline.backend()},{"workload",workload},{"input_hz",hz},{"poll_period_ms",2},{"source_policy","one active, one latest replaceable pending; skipped native arrivals coalesced"},{"setup_and_warmup_wall_ms",(prepared-wall0)/1e6},{"setup_and_warmup_process_cpu_ms",(pcpu-p0)/1e6},{"steady_wall_s",(wall_end-st)/1e9},{"steady_process_cpu_ms",(pcpu_end-pcpu)/1e6},{"steady_main_thread_cpu_ms",(tcpu_end-tcpu)/1e6},{"offered",offered},{"completed",completed},{"replaced_pending",replaced},{"dropped_no_slot",dropped},{"skipped_source_arrivals",source_skipped},{"inflight_at_end",bool(active)},{"pending_at_end",bool(pending)},{"components",comp_last},{"latency_ms",quantiles(lat)},{"frame_age_ms",quantiles(age)},{"host_copy_cpu_ms",quantiles(copy)},{"host_submit_cpu_ms",quantiles(submit)},{"host_finish_cpu_ms",quantiles(finish)},{"host_poll_cpu_ms",quantiles(poll)},{"poll_calls",polls},{"voluntary_context_switches",ru1.ru_nvcsw-ru0.ru_nvcsw},{"involuntary_context_switches",ru1.ru_nivcsw-ru0.ru_nivcsw}});
  const char* names[]={"upload","convert","cuts","cc_with_copyback","reduce","morphology","gather","readback"};for(unsigned i=0;i<8;i++)result["gpu_stage_ms"][names[i]]=quantiles(stages[i]);
  if(pipeline.context()&&pipeline.context()->allocation_count.load()!=alloc0)throw std::runtime_error("hot Vulkan allocation");result["hot_vulkan_allocations"]=0;
  // Deliberately excluded drain after fixed measurement window; clean up submitted work.
  while(active&&!pipeline.ready(active))std::this_thread::sleep_for(std::chrono::milliseconds(2));if(active)pipeline.finish(active);
 }else throw std::runtime_error("kind");
 write_json(c.at("output"),result);std::cout<<result.dump()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
