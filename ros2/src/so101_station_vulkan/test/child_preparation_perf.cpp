// Real child/parent implementation calls using the author's existing fake ABI.
// No Vulkan loader, shader arithmetic, camera, or production callback.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main child_author_contract_main
#include "child_preparation_contract.cpp"
#undef main
#pragma GCC diagnostic pop
#include <time.h>
namespace {
constexpr unsigned child_trials=5,child_iterations=2000;
struct ChildPerfSample{uint64_t cpu=0,wall=0,heap=0,buffers=0,waits=0,dispatch=0,indirect=0;};
struct ChildPerfScenario{const char* name;unsigned records;std::array<ChildPerfSample,child_trials> samples{};};
uint64_t child_clock(clockid_t id){timespec t{};require(clock_gettime(id,&t)==0);return uint64_t(t.tv_sec)*1000000000ull+t.tv_nsec;}
volatile uint64_t child_perf_sink=0;
}
int main(int argc,char** argv){try{
 require(argc==3);ParentConfig pc;pc.width=640;pc.height=480;pc.min_parent_pixels=1;
 pc.calibration_generation=0x100000002ull;pc.generation=0x200000003ull;
 ChildConfig cc;cc.generation=0x300000004ull;cc.min_child_pixels=8;
 auto context=std::make_shared<Context>();auto pool=std::make_shared<FramePool>(pc.width,pc.height,context,false);
 auto source=pool->acquire();source_metadata(source,pc);source.slot().metadata.serial="synthetic-child-host-perf";
 ParentPreparation parents(context,pool->source_token(),pc,argv[1]);auto parent=parents.submit(source);parent_result(1,pc.width,pc.height);
 // The resident parent is completed/decoded outside the child timing boundary.
 require(parent.configuration().generation==pc.generation);auto baseline_refs=source.slot().references.load();
 std::array<ChildPerfScenario,3> scenes{{{"empty",0,{}},{"ordinary",1,{}},{"capacity4096",4096,{}}}};
 auto pools=live_command_pools;uint64_t checksum=0;
 {
  ChildPreparation child(context,pool->source_token(),pc,cc,argv[2]);
  for(auto& scene:scenes){
   clear_result();fake_children=scene.records;
   for(unsigned j=0;j<scene.records;j++){
    unsigned x=scene.records==1?4:8*(j%80),y=scene.records==1?4:6*(j/80);
    unsigned w=scene.records==1?5:4,h=scene.records==1?6:2;
    unsigned left=x>2?x-2:0,top=y>2?y-2:0,hw=std::min(pc.width,x+w+2)-left,hh=std::min(pc.height,y+h+2)-top,area=hw*hh;
    fake_records[j]={y*pc.width+x+1,0,(y/2)*((pc.width+1)/2)+x/2,w*h,x,y,w,h,left,top,hw,hh,fake_packed,0,area,0};
    fake_packed+=area;fake_jobs+=(area+255)/256;
   }
   unsigned candidate=scene.records*(scene.records==1?30:8);fake_status[0]={candidate,candidate,scene.records,scene.records};
   auto operation=[&]{
    {
     auto initial=child.submit(parent);auto copy=initial;auto lease=std::move(copy);initial={};
     require(!copy&&lease&&source.slot().references.load()==baseline_refs+1);
     require(child.ready(lease));auto n=child.finish(lease);
     require(n.parent_count==1&&n.record_count==scene.records&&n.packed_pixels==fake_packed&&n.jobs==fake_jobs);
     require(n.calibration_generation==pc.calibration_generation&&n.parent_generation==pc.generation&&n.configuration_generation==cc.generation);
     auto* records=lease.records();auto* status=lease.parent_status();require(records&&status&&lease.records()==records&&lease.parent_status()==status);
     require(status[0].kept_children==scene.records&&status[0].candidate_pixels==candidate);
     if(scene.records)require(records[scene.records-1].key==fake_records[scene.records-1].key&&records[scene.records-1].support_pixels==fake_records[scene.records-1].support_pixels);
     require(lease.source_root_buffer()&&lease.packed_flags_buffer()&&lease.resident_records_buffer()&&lease.source_flags_offset_bytes()==4*pc.width*pc.height);
     {
      auto bridge=lease.parent();auto frame=bridge.source_frame();require(frame.belongs_to(pool->source_token())&&bridge.index()==parent.index());
      require(source.slot().references.load()==baseline_refs+2);
     }
     require(source.slot().references.load()==baseline_refs+1);
     auto consumer=lease;lease={};require(consumer.metadata().frame_number==911&&source.slot().references.load()==baseline_refs+1);
     checksum=(checksum^((uint64_t(n.record_count)<<32)|n.packed_pixels))*1099511628211ull;
    }
    child.collect();require(source.slot().references.load()==baseline_refs);
   };
   for(unsigned warm=0;warm<8;warm++)operation();
   for(auto& s:scene.samples){
    auto h=heap_allocations,b=buffer_allocations,w=wait_calls;auto d=dispatches,ind=indirects;
    auto cpu=child_clock(CLOCK_PROCESS_CPUTIME_ID),wall=child_clock(CLOCK_MONOTONIC);
    for(unsigned i=0;i<child_iterations;i++)operation();
    s.wall=child_clock(CLOCK_MONOTONIC)-wall;s.cpu=child_clock(CLOCK_PROCESS_CPUTIME_ID)-cpu;
    s.heap=heap_allocations-h;s.buffers=buffer_allocations-b;s.waits=wait_calls-w;s.dispatch=dispatches-d;s.indirect=indirects-ind;
    require(!s.heap&&!s.buffers&&!s.waits&&s.dispatch==166*child_iterations&&s.indirect==68*child_iterations);
   }
  }
 }
 require(live_command_pools==pools&&source.slot().references.load()==baseline_refs);child_perf_sink=checksum;
 ChildGraphLayout g(640,480);
 std::cout<<"{\"scope\":\"actual child module with reused synthetic Vulkan ABI; includes fake CPU fills/copies/payload injection, excludes real driver/GPU\",\"gpu_executed\":false,\"qualified\":false,\"iterations_per_trial\":"<<child_iterations<<",\"trials\":"<<child_trials<<",\"warmups_per_scenario\":8,\"dispatches_per_iteration\":166,\"indirect_per_iteration\":68,\"compact_copy_bytes_per_iteration\":"<<g.compact_bytes<<",\"fake_CPU_fill_bytes_per_iteration\":"<<(g.lookup_bytes+128+16*g.parent_capacity+64)<<",\"checksum\":"<<checksum<<",\"scenarios\":[";
 for(unsigned i=0;i<scenes.size();i++){
  if(i)std::cout<<',';const auto& c=scenes[i];std::cout<<"{\"name\":\""<<c.name<<"\",\"records\":"<<c.records<<",\"samples\":[";
  for(unsigned j=0;j<c.samples.size();j++){if(j)std::cout<<',';const auto& s=c.samples[j];std::cout<<"{\"cpu_ns\":"<<s.cpu<<",\"wall_ns\":"<<s.wall<<",\"heap_allocations\":"<<s.heap<<",\"buffer_allocations\":"<<s.buffers<<",\"wait_calls\":"<<s.waits<<",\"dispatches\":"<<s.dispatch<<",\"indirects\":"<<s.indirect<<'}';}
  std::cout<<"]}";
 }
 std::cout<<"]}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
