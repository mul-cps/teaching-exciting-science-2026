// Actual parent module calls; reuse the author's synthetic Vulkan ABI verbatim.
// No loader/driver/GPU, shader arithmetic, camera, or production callback.
// The author's original main is renamed and never executed; its implicit
// return is legal only under the original name. Suppress that unused warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main parent_author_contract_main
#include "parent_preparation_contract.cpp"
#undef main
#pragma GCC diagnostic pop
#include <time.h>

namespace {
constexpr unsigned perf_trials=5,perf_iterations=2000;
struct PerfSample {uint64_t cpu_ns=0,wall_ns=0,heap=0,buffers=0,waits=0,dispatch=0;};
struct PerfScenario {const char* name;unsigned count,all_red;std::array<PerfSample,perf_trials> samples{};};
uint64_t perf_clock(clockid_t id){timespec t{};require(clock_gettime(id,&t)==0);return uint64_t(t.tv_sec)*1000000000ull+uint64_t(t.tv_nsec);}
volatile uint64_t parent_perf_sink=0;
}

int main(int argc,char** argv){try{
 if(argc==2&&std::string(argv[1])=="--model")return walk_proof();
 require(argc==2);
 ParentConfig config;config.width=640;config.height=480;config.min_parent_pixels=30;
 config.generation=0x100000002ull;config.calibration_generation=0x300000004ull;
 auto context=std::make_shared<Context>();auto pool=std::make_shared<FramePool>(config.width,config.height,context,false);
 auto token=pool->source_token();auto source=pool->acquire();auto& slot=source.slot();
 slot.submitted=true;slot.metadata.calibration_generation=config.calibration_generation;
 slot.calibration.generation=config.calibration_generation;slot.metadata.frame_number=917;
 slot.metadata.depth_frame_number=916;slot.metadata.serial="synthetic-parent-host-perf";
 complete[uintptr_t(slot.fence)]=true;const auto initial_refs=slot.references.load();
 std::array<ParentRecord,kMaxComponents> expected{};
 std::array<PerfScenario,3> cases{{{"empty",0,0,{}},{"filtered_label_two",1,2,{}},{"capacity_4096",4096,4096,{}}}};
 auto pools_before=live_command_pools;uint64_t checksum=0;
 {
  ParentPreparation parents(context,token,config,argv[1]);
  for(auto& scenario:cases){
   // Generate valid bounded compact records once, outside measured work.
   // 4096 separated 5x6 red rectangles fit in 640x480 at min-area30.
   for(unsigned j=0;j<scenario.count;j++){
    unsigned x=6*(j%106),y=7*(j/106),key=(y/2)*320+x/2;
    expected[j]={0,j+1,key,30,x,y,5,6};
   }
   if(scenario.count==1)expected[0]={0,2,2,40,4,0,8,5};
   auto operation=[&]{
    {
     auto lease=parents.submit(source);auto copy=lease;auto moved=std::move(copy);lease={};
     require(!copy&&moved&&slot.references.load()==initial_refs+1);
     // Fake GPU completion payload, injected before the first real decode.
     // Injection/copy cost is INCLUDED in these synthetic host measurements.
     auto* words=static_cast<uint32_t*>(fake_memory[uintptr_t(last_readback)].data);
     words[7]=scenario.count;words[9]=scenario.all_red;
     if(scenario.count)std::memcpy(words+16,expected.data(),size_t(scenario.count)*sizeof(ParentRecord));
     require(parents.ready(moved));auto counts=parents.finish(moved);
     require(counts.record_count==scenario.count&&counts.all_parents[0]==scenario.all_red);
     require(counts.calibration_generation==config.calibration_generation&&counts.configuration_generation==config.generation);
     auto* records=moved.records();require(records&&moved.records()==records);
     if(scenario.count)require(records[0].label_id==expected[0].label_id&&records[scenario.count-1].parent_pixels==expected[scenario.count-1].parent_pixels);
     require(moved.resident_buffer()&&moved.resident_records_buffer());
     require(moved.original_mask_buffer()==slot.scratch.buffer&&moved.label_offset_bytes()==12288000);
     auto downstream=moved;moved={};require(slot.references.load()==initial_refs+1);
     require(&downstream.metadata()==&slot.metadata&&downstream.metadata().frame_number==917);
     checksum=(checksum^((uint64_t(counts.record_count)<<32)|counts.all_parents[0]))*1099511628211ull;
    }
    parents.collect();require(slot.references.load()==initial_refs);
   };
   for(unsigned warm=0;warm<8;warm++)operation();
   for(auto& sample:scenario.samples){
    auto heap_before=heap_allocations,buffers_before=buffer_allocations,waits_before=wait_calls;auto dispatch_before=dispatches;
    auto cpu_start=perf_clock(CLOCK_PROCESS_CPUTIME_ID),wall_start=perf_clock(CLOCK_MONOTONIC);
    for(unsigned iteration=0;iteration<perf_iterations;iteration++)operation();
    sample.wall_ns=perf_clock(CLOCK_MONOTONIC)-wall_start;sample.cpu_ns=perf_clock(CLOCK_PROCESS_CPUTIME_ID)-cpu_start;
    sample.heap=heap_allocations-heap_before;sample.buffers=buffer_allocations-buffers_before;
    sample.waits=wait_calls-waits_before;sample.dispatch=dispatches-dispatch_before;
    require(!sample.heap&&!sample.buffers&&!sample.waits&&sample.dispatch==73*perf_iterations);
   }
  }
 }
 require(live_command_pools==pools_before&&slot.references.load()==initial_refs);
 parent_perf_sink=checksum;
 std::cout<<"{\"scope\":\"actual parent module; synthetic Vulkan and compact payload injection; excludes real driver/GPU\",\"gpu_executed\":false,\"qualified\":false,\"iterations_per_trial\":"<<perf_iterations<<",\"trials\":"<<perf_trials<<",\"dispatches_per_iteration\":73,\"capacity\":4096,\"compact_copy_bytes_per_iteration\":"<<parent_compact_bytes(4096)<<",\"checksum\":"<<checksum<<",\"scenarios\":[";
 for(unsigned c=0;c<cases.size();c++){
  if(c)std::cout<<',';auto& scenario=cases[c];std::cout<<"{\"name\":\""<<scenario.name<<"\",\"record_count\":"<<scenario.count<<",\"samples\":[";
  for(unsigned t=0;t<perf_trials;t++){if(t)std::cout<<',';auto s=scenario.samples[t];std::cout<<"{\"cpu_ns\":"<<s.cpu_ns<<",\"wall_ns\":"<<s.wall_ns<<",\"heap_allocations\":"<<s.heap<<",\"buffer_allocations\":"<<s.buffers<<",\"wait_calls\":"<<s.waits<<",\"dispatches\":"<<s.dispatch<<'}';}
  std::cout<<"]}";
 }
 std::cout<<"]}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
