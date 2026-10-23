// Diagnostic only: real production API, explicit dense copy after completion.
#include "so101_station_vulkan/geometry.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace so101_station_vulkan;
namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
void check(VkResult r,const char* message){require(r==VK_SUCCESS,message);}
template<class T> void read(std::ifstream& f,T* p,size_t n=1){f.read(reinterpret_cast<char*>(p),n*sizeof(T));require(bool(f),"short fixture");}
struct Transfer {
 std::shared_ptr<Context> c;VkCommandPool pool=VK_NULL_HANDLE;VkCommandBuffer cmd=VK_NULL_HANDLE;VkFence fence=VK_NULL_HANDLE;VkFence pending=VK_NULL_HANDLE;
 Buffer upload,readback;
 Transfer(std::shared_ptr<Context> context,size_t source,size_t arena):c(context){
 try{
  auto host=VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  upload.create(*c,source,VK_BUFFER_USAGE_TRANSFER_SRC_BIT,host);readback.create(*c,arena,VK_BUFFER_USAGE_TRANSFER_DST_BIT,host);
  VkCommandPoolCreateInfo p{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};p.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;p.queueFamilyIndex=c->family;check(vkCreateCommandPool(c->device,&p,nullptr,&pool),"diagnostic pool");
  VkCommandBufferAllocateInfo a{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};a.commandPool=pool;a.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;a.commandBufferCount=1;check(vkAllocateCommandBuffers(c->device,&a,&cmd),"diagnostic command");
  VkFenceCreateInfo f{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};check(vkCreateFence(c->device,&f,nullptr,&fence),"diagnostic fence");
 }catch(...){if(fence)vkDestroyFence(c->device,fence,nullptr);if(pool)vkDestroyCommandPool(c->device,pool,nullptr);throw;}
 }
 ~Transfer(){if(pending)vkWaitForFences(c->device,1,&pending,VK_TRUE,UINT64_MAX);if(fence)vkDestroyFence(c->device,fence,nullptr);if(pool)vkDestroyCommandPool(c->device,pool,nullptr);}
 void begin(){check(vkResetCommandBuffer(cmd,0),"diagnostic reset");VkCommandBufferBeginInfo b{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};b.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;check(vkBeginCommandBuffer(cmd,&b),"diagnostic begin");}
 void barrier(VkPipelineStageFlags src,VkPipelineStageFlags dst,VkAccessFlags access,VkAccessFlags next){VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=access;b.dstAccessMask=next;vkCmdPipelineBarrier(cmd,src,dst,0,1,&b,0,nullptr,0,nullptr);}
 void run(VkFence target){check(vkEndCommandBuffer(cmd),"diagnostic end");check(vkResetFences(c->device,1,&target),"diagnostic fence reset");VkSubmitInfo s{VK_STRUCTURE_TYPE_SUBMIT_INFO};s.commandBufferCount=1;s.pCommandBuffers=&cmd;{std::lock_guard<std::mutex> lock(c->queue_mutex);check(vkQueueSubmit(c->queue,1,&s,target),"diagnostic submit");pending=target;}check(vkWaitForFences(c->device,1,&target,VK_TRUE,10000000000ULL),"diagnostic completion timeout");pending=VK_NULL_HANDLE;}
 void source(FrameLease& frame){auto& s=frame.slot();begin();barrier(VK_PIPELINE_STAGE_HOST_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);VkBufferCopy copy{0,0,upload.size};vkCmdCopyBuffer(cmd,upload.buffer,s.scratch.buffer,1,&copy);vkCmdFillBuffer(cmd,s.compact.buffer,0,16,0);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT|VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT|VK_ACCESS_SHADER_READ_BIT);copy.size=16;vkCmdCopyBuffer(cmd,s.compact.buffer,s.readback.buffer,1,&copy);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(s.fence);s.submitted=true;}
 void snapshot(const GeometryLease& frame){begin();barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT);VkBufferCopy copy{0,0,readback.size};vkCmdCopyBuffer(cmd,frame.resident_buffer(),readback.buffer,1,&copy);barrier(VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_HOST_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT);run(fence);}
};
template<class F> void rejected(F fn){bool caught=false;try{fn();}catch(const std::exception&){caught=true;}require(caught,"invalid source accepted");}
void complete(GeometryPreparation& geometry,const GeometryLease& lease){auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(!geometry.ready(lease)){require(std::chrono::steady_clock::now()<end,"geometry timeout");std::this_thread::yield();}}
}
int main(int argc,char** argv){try{
 require(argc==5&&(std::string(argv[1])=="--vulkan"||std::string(argv[1])=="--expect-fp64-rejection"||std::string(argv[1])=="--software-binary64"),"usage: qualify_geometry --vulkan|--expect-fp64-rejection|--software-binary64 fixture.bin shader_directory output_prefix");
 std::ifstream input(argv[2],std::ios::binary);char magic[8];read(input,magic,8);require(std::memcmp(magic,"GEOQv1\0\0",8)==0,"bad fixture magic");
 GeometryCalibration cal;read(input,&cal.width);read(input,&cal.height);read(input,&cal.generation);read(input,&cal.depth_scale_m);read(input,&cal.max_observation_depth_m);read(input,cal.board_from_camera.data(),12);read(input,cal.support_roi.data(),6);
 require(cal.width&&cal.width<=640&&cal.height&&cal.height<=480,"fixture dimensions");size_t n=size_t(cal.width)*cal.height;
 cal.camera_rays.resize(n);read(input,cal.camera_rays.data(),n);cal.validate();
 auto c=Context::shared();VkPhysicalDeviceFeatures supported{};vkGetPhysicalDeviceFeatures(c->physical,&supported);
 std::string prefix=argv[4];
 if(std::string(argv[1])=="--expect-fp64-rejection"){
  require(!supported.shaderFloat64&&!c->enabled_features.features.shaderFloat64,"rejection proof requires physically unsupported FP64");
  bool exact=false;try{GeometryPreparation geometry(c,cal,argv[3]);}catch(const std::runtime_error& e){exact=std::string(e.what()).find("shaderFloat64 was not enabled")!=std::string::npos;}
  require(exact,"missing explicit FP64 rejection");std::ofstream(prefix+".json")<<"{\"physical\":false,\"fp64_rejection\":true,\"vendor_id\":"<<c->properties.vendorID<<",\"device_id\":"<<c->properties.deviceID<<",\"driver_version\":"<<c->properties.driverVersion<<"}\n";return 0;
 }
 bool software=std::string(argv[1])=="--software-binary64";
 if(software)require((c->properties.vendorID==0x1002||c->properties.vendorID==0x8086)&&supported.shaderInt64&&c->enabled_features.features.shaderInt64,"software geometry qualification requires AMD/Intel and actually enabled Int64");
 else require(c->properties.vendorID==0x1002&&supported.shaderFloat64&&c->enabled_features.features.shaderFloat64,"full geometry qualification requires AMD and actually enabled FP64");
 auto pool=std::make_shared<FramePool>(cal.width,cal.height,c,false);GeometryPreparation geometry(c,cal,argv[3],software?GeometryPrecision::SoftwareBinary64:GeometryPrecision::NativeFloat64);
 Transfer transfer(c,n*8,geometry_arena_bytes(cal.width,cal.height));read(input,static_cast<char*>(transfer.upload.mapped),n*8);require(input.peek()==std::char_traits<char>::eof(),"extra fixture bytes");
 auto source=pool->acquire();auto& slot=source.slot();slot.calibration.generation=cal.generation;slot.calibration.depth_scale=float(cal.depth_scale_m);
 slot.metadata.calibration_generation=cal.generation;slot.metadata.frame_number=91;slot.metadata.depth_frame_number=90;slot.metadata.timestamp_ns=12345678;slot.metadata.serial="procedural-qualification-only";slot.metadata.hardware_timestamp_ms=123.25;slot.metadata.depth_hardware_timestamp_ms=122.5;slot.metadata.timestamp_domain=1;slot.metadata.depth_timestamp_domain=2;
 transfer.source(source);
 slot.fault=1;rejected([&]{geometry.submit(source);});slot.fault=0;
 static_cast<uint32_t*>(slot.readback.mapped)[1]=1;rejected([&]{geometry.submit(source);});static_cast<uint32_t*>(slot.readback.mapped)[1]=0;
 ++slot.metadata.calibration_generation;rejected([&]{geometry.submit(source);});--slot.metadata.calibration_generation;
 ++slot.calibration.generation;rejected([&]{geometry.submit(source);});--slot.calibration.generation;
 check(vkResetFences(c->device,1,&slot.fence),"pending-source reset");rejected([&]{geometry.submit(source);});transfer.source(source);
 auto result=geometry.submit(source);rejected([&]{geometry.submit(source);});auto copy=result;result={};
 unsigned source_index=source.index();source={};require(slot.references.load()==1,"geometry must retain its source");complete(geometry,copy);auto counts=geometry.finish(copy);
 require(counts.calibration_generation==cal.generation&&slot.metadata.frame_number==91&&slot.metadata.depth_frame_number==90&&slot.metadata.timestamp_ns==12345678&&slot.metadata.serial=="procedural-qualification-only"&&slot.metadata.width==cal.width&&slot.metadata.height==cal.height&&slot.metadata.calibration_generation==cal.generation&&slot.metadata.hardware_timestamp_ms==123.25&&slot.metadata.depth_hardware_timestamp_ms==122.5&&slot.metadata.timestamp_domain==1&&slot.metadata.depth_timestamp_domain==2,"provenance changed");
 transfer.snapshot(copy);std::ofstream arena(prefix+".arena",std::ios::binary);arena.write(static_cast<char*>(transfer.readback.mapped),transfer.readback.size);require(bool(arena),"arena write failed");arena.close();
 auto b=pool->acquire(),d=pool->acquire();require(bool(b)&&bool(d)&&!pool->acquire(),"copied geometry/source saturation");
 // Fill all three geometry arenas while the first result remains leased.
 b.slot().calibration=slot.calibration;b.slot().metadata=slot.metadata;transfer.source(b);auto gb=geometry.submit(b);
 d.slot().calibration=slot.calibration;d.slot().metadata=slot.metadata;transfer.source(d);auto gd=geometry.submit(d);
 complete(geometry,gb);complete(geometry,gd);require(geometry.finish(gb).support_points==counts.support_points&&geometry.finish(gd).valid_pixels==counts.valid_pixels,"three-arena saturation changed counts");
 gb={};gd={};geometry.collect();copy={};geometry.collect();auto recycled=pool->acquire();require(bool(recycled)&&recycled.index()==source_index,"last-copy release failed");b={};d={};recycled={};
 // Drop a submitted lease immediately; collect must eventually reap its source.
 auto dropped_source=pool->acquire();dropped_source.slot().calibration=slot.calibration;dropped_source.slot().metadata=slot.metadata;transfer.source(dropped_source);auto dropped=geometry.submit(dropped_source);dropped_source={};bool drop_was_pending=!geometry.ready(dropped);dropped={};
 auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(pool->slots[source_index].references.load()){geometry.collect();require(std::chrono::steady_clock::now()<deadline,"dropped source not reaped");std::this_thread::yield();}
 std::ofstream output(prefix+".json");output<<"{\"physical\":false,\"physical_camera\":false,\"objects\":false,\"precision\":\""<<(software?"software_binary64":"native_float64")<<"\",\"geometry_preparation_only\":true,\"valid_count\":"<<counts.valid_pixels<<",\"support_count\":"<<counts.support_points<<",\"generation\":"<<counts.calibration_generation<<",\"metadata_preserved\":true,\"copied_lease_saturation\":true,\"source_fault_rejections\":2,\"generation_rejections\":2,\"pending_source_rejected\":true,\"dropped_source_reaped\":true,\"drop_observed_pending\":"<<(drop_was_pending?"true":"false")<<",\"vendor_id\":"<<c->properties.vendorID<<",\"device_id\":"<<c->properties.deviceID<<",\"driver_version\":"<<c->properties.driverVersion<<",\"api_version\":"<<c->properties.apiVersion<<",\"shader_float64_enabled\":"<<(c->enabled_features.features.shaderFloat64?"true":"false")<<",\"shader_int64_enabled\":"<<(c->enabled_features.features.shaderInt64?"true":"false")<<",\"arena_bytes\":"<<transfer.readback.size<<"}\n";require(bool(output),"metadata write failed");
 std::cout<<"PASS geometry preparation physical=false\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
