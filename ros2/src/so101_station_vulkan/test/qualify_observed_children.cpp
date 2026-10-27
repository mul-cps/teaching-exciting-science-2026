// Explicit real-Vulkan procedural diagnostic. --offline never creates Context.
// Reuse the bounded CHILDQv1 parser, source transfer and shader/source sealing.
#ifdef OBSERVED_QUALIFIER_CPU_ONLY
#define CHILD_QUALIFIER_CPU_ONLY
#endif
#ifndef CHILD_SHADER_SOURCE_SHA256
#define CHILD_SHADER_SOURCE_SHA256 "unrecorded"
#endif
#define main inherited_child_qualifier_main
#include "../../../../tools/performance/qualify_children.cpp"
#undef main
#ifndef OBSERVED_QUALIFIER_SOURCE_SHA256
#define OBSERVED_QUALIFIER_SOURCE_SHA256 "unrecorded"
#endif
#ifndef OBSERVED_QUALIFIER_HELPER_SHA256
#define OBSERVED_QUALIFIER_HELPER_SHA256 "unrecorded"
#endif
#ifndef OBSERVED_NATIVE_SHADER_HASH
#define OBSERVED_NATIVE_SHADER_HASH ""
#define OBSERVED_SOFTWARE_SHADER_HASH ""
#define OBSERVED_GEOMETRY_NATIVE_HASH ""
#define OBSERVED_GEOMETRY_SOFTWARE_HASH ""
#endif
#ifndef OBSERVED_QUALIFIER_CPU_ONLY
#include "so101_station_vulkan/observed_children.hpp"
#include <ctime>
using namespace so101_station_vulkan;
namespace {
struct ObservedGPUState {
 std::shared_ptr<Context> context;std::shared_ptr<FramePool> pool;
 std::unique_ptr<ParentPreparation> parent;std::unique_ptr<ChildPreparation> child;
 std::unique_ptr<GeometryPreparation> geometry;std::unique_ptr<ObservedPreparation> observed;
 std::unique_ptr<Transfer> transfer;
};
// Separate from inherited Child's intentionally Shader-only seal.
// Native byte-address double loads additionally use Int64 in qualified Slang.
// Accept Shader plus the precision capabilities; require the
// arithmetic entry to contain it, while dead-code-only entries may omit it.
std::string precision_shader_identity(const std::string& directory,const std::vector<std::string>& names,const std::string& precision,const char* required_entry,const std::string& expected_source){
 require(precision=="native"||precision=="software","invalid shader precision");
 auto source=small_file(directory+"/source.sha256");require(source.size()==65&&source.back()=='\n'&&source.find_first_not_of("0123456789abcdef")==64,"precision source digest malformed");source.pop_back();require(expected_source.empty()||source==expected_source,"precision source/build tag mismatch");
 const u32 requested=precision=="native"?10:11;std::ostringstream out;out<<"{\"source_sha256\":"<<quote(source)<<",\"precision\":"<<quote(precision)<<",\"spirv\":{";
 for(unsigned k=0;k<names.size();k++){
  auto file=directory+"/"+names[k]+".spv";std::ifstream input(file,std::ios::binary|std::ios::ate);require(bool(input)&&input.tellg()>=20&&input.tellg()<=1048576&&size_t(input.tellg())%4==0,"precision SPIR-V length");std::vector<u32> words(size_t(input.tellg())/4);input.seekg(0);input.read(reinterpret_cast<char*>(words.data()),words.size()*4);require(bool(input)&&words[0]==0x07230203&&words[1]<=0x00010300&&words[4]==0,"precision SPIR-V header");
  bool shader=false,scalar=false;for(size_t at=5;at<words.size();){u32 length=words[at]>>16,op=words[at]&65535;require(length&&length<=words.size()-at,"precision SPIR-V instruction bounds");if(op==17){require(length==2,"capability instruction shape");u32 cap=words[at+1];require(cap==1||cap==requested||(precision=="native"&&cap==11),"wrong/unsupported precision capability");shader|=cap==1;scalar|=cap==requested;}at+=length;}
  require(shader&&(names[k]!=required_entry||scalar),"required precision capability missing");auto digest=hash_file(file);require(small_file(file+".sha256")==digest+"  "+names[k]+".spv\n","precision SPIR-V digest mismatch");if(k)out<<',';out<<quote(names[k])<<":"<<quote(digest);
 }out<<"},\"provenance\":";auto provenance_path=directory+"/provenance.txt";if(std::filesystem::exists(provenance_path))out<<quote(small_file(provenance_path));else out<<"null";out<<'}';return out.str();
}
void observed_gpu(const Fixture& q,int argc,char** argv){
 require(q.profile==1&&q.width<=640&&q.height<=480,"Observed typed chain requires existing typed parent fixture within640x480");
 const std::string parent_dir=argv[3],child_dir=argv[4],geometry_dir=argv[5],observed_dir=argv[6],prefix=argv[7],precision=argv[8];
 require(precision=="native"||precision=="software","precision must be native or software");auto mutation=std::stoul(argv[9]);require(mutation<=15,"malformation diagnostic0..15");
 require(std::strlen(OBSERVED_QUALIFIER_SOURCE_SHA256)==64&&std::strlen(OBSERVED_QUALIFIER_HELPER_SHA256)==64,"missing qualifier source identity");
 std::vector<std::string> names;for(auto name:observed_detail::kernel_names)names.emplace_back(name);auto observed_identity=precision_shader_identity(observed_dir,names,precision,"observed_observe",precision=="native"?OBSERVED_NATIVE_SHADER_HASH:OBSERVED_SOFTWARE_SHADER_HASH);require(small_file(observed_dir+"/provenance.txt").find("mode="+precision+"\n")!=std::string::npos,"Observed shader precision identity mismatch");
 names.clear();for(auto name:child_detail::kernel_names)names.emplace_back(name);auto child_identity=shader_identity(child_dir,names);auto parent_identity=shader_identity(parent_dir,{"parents_init","parents_hook","parents_flatten","parents_validate","parents_scan","parents_offsets","parents_pixels","parents_labels","parents_records"});auto geom_identity=precision_shader_identity(geometry_dir,{"geometry_coordinates","geometry_support_scan","geometry_support_offsets","geometry_support_scatter","geometry_valid_count"},precision,"geometry_coordinates",precision=="native"?OBSERVED_GEOMETRY_NATIVE_HASH:OBSERVED_GEOMETRY_SOFTWARE_HASH);
 for(const char* suffix:{".running",".json",".observed.bin",".records.bin",".counts.bin",".child_roots.bin",".child_flags.bin",".world.bin"})require(!std::filesystem::exists(prefix+suffix),"output prefix already exists");
 auto output_dir=std::filesystem::path(prefix).parent_path();require(std::filesystem::is_directory(output_dir.empty()?".":output_dir),"output directory missing");
 ObservedConfig oc;oc.generation=31;if(argc==17){std::array<double,6> roi;for(unsigned i=0;i<6;i++){size_t end=0;roi[i]=std::stod(argv[11+i],&end);require(end==std::strlen(argv[11+i]),"ROI trailing data");}oc.observation_roi=roi;}oc.validate(unsigned(q.n()));
 fresh_text(prefix+".running","Observed typed fixture diagnostic; real external process-group watchdog REQUIRED; no camera/objects/world qualification\n");
 // Deliberately retained on unexpected exceptions. Never unwind an in-flight
 // driver resource after a deadline; the external watchdog owns process death.
 auto* state=new ObservedGPUState;auto& s=*state;s.context=std::make_shared<Context>();auto& c=*s.context;
 ParentConfig pc;pc.width=q.width;pc.height=q.height;pc.min_parent_pixels=q.parent_minimum;pc.record_capacity=q.P;pc.calibration_generation=17;pc.generation=23;
 ChildConfig cc;cc.generation=29;cc.max_observation_depth_m=q.maximum;cc.max_neighbor_jump_m=q.jump;cc.min_child_pixels=std::max(24u,q.parent_minimum);cc.record_capacity=q.C;cc.packed_pixel_capacity=q.S;
 GeometryCalibration gc;gc.width=q.width;gc.height=q.height;gc.generation=17;gc.depth_scale_m=double(.001f);gc.max_observation_depth_m=q.maximum;gc.board_from_camera={1,0,0,0,0,1,0,0,0,0,1,0};gc.camera_rays.resize(q.n());for(unsigned i=0;i<q.n();i++)gc.camera_rays[i]={double(i%q.width),double(i/q.width)};
 s.pool=std::make_shared<FramePool>(q.width,q.height,s.context,false);s.parent=std::make_unique<ParentPreparation>(s.context,s.pool->source_token(),pc,parent_dir);s.child=std::make_unique<ChildPreparation>(s.context,s.pool->source_token(),pc,cc,child_dir);s.geometry=std::make_unique<GeometryPreparation>(s.context,gc,geometry_dir,precision=="native"?GeometryPrecision::NativeFloat64:GeometryPrecision::SoftwareBinary64);s.observed=std::make_unique<ObservedPreparation>(s.context,s.pool->source_token(),s.geometry->producer_token(),pc,cc,oc,observed_dir);
 ObservedLayout layout(q.width,q.height,q.C,q.P);s.transfer=std::make_unique<Transfer>(s.context,std::max(layout.arena_bytes,size_t(8)*q.n()));auto& t=*s.transfer;
 auto source=s.pool->acquire();require(bool(source),"source unavailable");metadata(source,q,91);source.slot().metadata.publication_session_token=37;
 auto* uploaded=static_cast<u32*>(t.upload.mapped);std::memcpy(uploaded,q.mask.data(),4*q.n());std::memcpy(uploaded+q.n(),q.depth.data(),4*q.n());t.source(source,q.n());
 auto parent=s.parent->submit(source);complete(*s.parent,parent);s.parent->finish(parent);
 auto child=s.child->submit(parent);auto geometry=s.geometry->submit(source);complete(*s.child,child);complete(*s.geometry,geometry);auto child_counts=s.child->finish(child);s.geometry->finish(geometry);
 require(child.metadata().publication_session_token==37&&&geometry.source_frame().slot()==&source.slot(),"cohort metadata/source changed");
 if(mutation){
  // Explicit diagnostic corruption through PUBLIC resident outputs, never a
  // forged lease/private storage adapter. The production consumer must reject.
  auto view=child.resident_view();if(mutation<=3){require(child_counts.parent_count>0,"status diagnostic needs nonempty parent result");if(mutation==2)require(child.parent_status()[0].candidate_pixels>0,"candidate diagnostic needs positive original candidate count");if(mutation==3)require(child.parent_status()[0].kept_children>0,"all-children diagnostic needs positive kept count");}size_t offset=0;u32 value=0;const auto* record=child.records();
  switch(mutation){case 1:offset=128+12;value=child.parent_status()[0].kept_children+1;break;case 2:offset=128+4;break;case 3:offset=128+8;break;case 4:offset=24;value=child_counts.jobs+1;break;case 5:offset=20;value=child_counts.packed_pixels+1;break;case 6:offset=128+16*q.P;value=u32(q.n()+1);break;case 7:offset=128+16*q.P+4;value=q.P;break;case 8:require(child_counts.record_count>1,"duplicate-root diagnostic needs2 records");offset=128+16*q.P+64;value=record[0].root;break;case 9:offset=128+16*q.P+8;value=u32(q.n());break;case 10:offset=128+16*q.P+48;value=1;break;case 11:offset=128+16*q.P+56;value=u32(q.n()+1);break;case 12:offset=128+16*q.P+60;value=1;break;case 13:offset=28;value=1;break;case 14:offset=128+16*q.P;value=u32(q.n()+1);break;case 15:offset=128+16*q.P+12;value=child_counts.record_count?record[0].region_pixels+1:1;break;}
  require(mutation<=5||mutation==13||child_counts.record_count,"record diagnostic requires nonempty child result");require(offset+4<=128+16*size_t(q.P)+64*size_t(q.C),"diagnostic patch bound");*static_cast<u32*>(t.upload.mapped)=value;t.begin();t.barrier(VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);t.copy(t.upload.buffer,view.records,4,0,offset);t.barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT);t.run(t.fence);
 }
 const auto compact_wall_start=std::chrono::steady_clock::now();const auto compact_cpu_start=std::clock();
 auto observed=s.observed->submit(child,geometry);auto retained=observed;observed={};child={};geometry={};parent={};source={};
 auto spare_a=s.pool->acquire(),spare_b=s.pool->acquire();require(spare_a&&spare_b&&!s.pool->acquire(),"Observed failed to retain transitive source slot");
 observed=std::move(retained);complete(*s.observed,observed);ObservedCounts counts;bool usable=true;std::string reason;
 try{counts=s.observed->finish(observed);}catch(const std::runtime_error& e){usable=false;reason=e.what();}
 const double compact_wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-compact_wall_start).count();const double compact_cpu_ms=1000.0*double(std::clock()-compact_cpu_start)/CLOCKS_PER_SEC;const auto diagnostic_wall_start=std::chrono::steady_clock::now();const auto diagnostic_cpu_start=std::clock();
 if(mutation)require(!usable,"malformed child became usable Observed output");
 if(usable){
  child=observed.children();geometry=observed.geometry();
  auto snapshot=[&](VkBuffer buffer,size_t bytes,const char* suffix){require(bytes<=t.download.size,"diagnostic snapshot bound");t.begin();t.barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);t.copy(buffer,t.download.buffer,bytes);t.barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);t.run(t.fence);fresh_file(prefix+suffix,t.download.mapped,bytes);};
  // BOTH input leases and result survive every diagnostic transfer's OWN fence.
  snapshot(observed.resident_buffer(),layout.arena_bytes,".observed.bin");snapshot(geometry.resident_view().coordinates,24*q.n(),".world.bin");auto child_view=child.resident_view();snapshot(child_view.roots,8*q.n(),".child_roots.bin");
  fresh_file(prefix+".records.bin",observed.records(),80*size_t(counts.record_count));u32 compact_counts[]={counts.record_count,counts.eligible_count,counts.original_pixels,counts.closed_memberships};fresh_file(prefix+".counts.bin",compact_counts,sizeof(compact_counts));
 }
 const double diagnostic_wall_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-diagnostic_wall_start).count();const double diagnostic_cpu_ms=1000.0*double(std::clock()-diagnostic_cpu_start)/CLOCKS_PER_SEC;
 std::ostringstream report;report<<"{\"scope\":\"observed_preparation_only\",\"gpu_executed\":true,\"physical_camera\":false,\"objects\":false,\"world_qualified\":false,\"production_views_used\":true,\"status\":"<<quote(usable?"valid":"unusable")<<",\"reason\":"<<quote(reason)<<",\"unique_inputs\":1,\"completed_outputs\":"<<(usable?1:0)<<",\"compact_submit_finish_wall_ms\":"<<compact_wall_ms<<",\"compact_submit_finish_process_cpu_ms\":"<<compact_cpu_ms<<",\"dense_diagnostic_wall_ms\":"<<diagnostic_wall_ms<<",\"dense_diagnostic_process_cpu_ms\":"<<diagnostic_cpu_ms<<",\"timing_scope\":\"one unique procedural input; compact includes fence polling and lifetime checks; dense diagnostic excludes JSON/hashing\",\"width\":"<<q.width<<",\"height\":"<<q.height<<",\"min_child_pixels\":"<<cc.min_child_pixels<<",\"precision\":"<<quote(precision)<<",\"malformation\":"<<mutation<<",\"source_frame\":91,\"depth_source_frame\":90,\"publication_session_token\":37,\"external_watchdog_acknowledged\":true,\"device\":"<<quote(c.properties.deviceName)<<",\"float64_enabled\":"<<(c.enabled_features.features.shaderFloat64?"true":"false")<<",\"int64_enabled\":"<<(c.enabled_features.features.shaderInt64?"true":"false")<<",\"fixture_sha256\":"<<quote(q.hash)<<",\"executable_sha256\":"<<quote(hash_file("/proc/self/exe"))<<",\"qualifier_source_sha256\":"<<quote(OBSERVED_QUALIFIER_SOURCE_SHA256)<<",\"qualifier_helper_source_sha256\":"<<quote(OBSERVED_QUALIFIER_HELPER_SHA256)<<",\"parent_shader_identity\":"<<parent_identity<<",\"child_shader_identity\":"<<child_identity<<",\"geometry_shader_identity\":"<<geom_identity<<",\"observed_shader_identity\":"<<observed_identity<<",\"loaded_libraries\":["<<library(reinterpret_cast<const void*>(&observed_detail::record_graph))<<','<<library(reinterpret_cast<const void*>(&geometry_arena_bytes))<<','<<library(reinterpret_cast<const void*>(&child_detail::record_graph))<<','<<library(reinterpret_cast<const void*>(&parent_arena_bytes))<<','<<library(reinterpret_cast<const void*>(&cpu_reference))<<"]}\n";fresh_text(prefix+".json",report.str());
 observed={};child={};geometry={};parent={};source={};s.observed->collect();s.child->collect();s.geometry->collect();s.parent->collect();require(bool(s.pool->acquire()),"Observed source slot not reclaimed after own fence");spare_a={};spare_b={};delete state;
}
}
#endif
#ifndef OBSERVED_QUALIFIER_ENTRY
#define OBSERVED_QUALIFIER_ENTRY main
#endif
int OBSERVED_QUALIFIER_ENTRY(int argc,char** argv){try{
 require(argc>=3,"usage: --offline CHILDQ | --vulkan CHILDQ PARENT CHILD GEOMETRY OBSERVED NEWPREFIX native|software MUTATION --external-watchdog-ack [ROI6]");auto fixture=load(argv[2]);
 if(std::string(argv[1])=="--offline"){require(argc==3,"offline argument count");std::cout<<"{\"gpu_executed\":false,\"fixture_sha256\":"<<quote(fixture.hash)<<",\"width\":"<<fixture.width<<",\"height\":"<<fixture.height<<",\"min_child_pixels\":"<<std::max(24u,fixture.parent_minimum)<<"}\n";return 0;}
#ifndef OBSERVED_QUALIFIER_CPU_ONLY
 require(std::string(argv[1])=="--vulkan"&&(argc==11||argc==17)&&std::string(argv[10])=="--external-watchdog-ack","real external watchdog acknowledgment required BEFORE Context");observed_gpu(fixture,argc,argv);return 0;
#else
 throw std::runtime_error("CPU-only parser cannot execute Vulkan");
#endif
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';
#ifndef OBSERVED_QUALIFIER_CPU_ONLY
 fatal("Observed diagnostic failed; exit before possibly in-flight driver teardown",1);
#else
 return 1;
#endif
}}
