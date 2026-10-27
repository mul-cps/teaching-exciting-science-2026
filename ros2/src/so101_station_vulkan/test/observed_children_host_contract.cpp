// Actual-module OWNERSHIP fixture only: fake Vulkan never claims numerical work.
// Serial AOT Slang witness independently verifies actual kernel arithmetic.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define vkUpdateDescriptorSets inherited_vkUpdateDescriptorSets
#define vkCmdPushConstants inherited_vkCmdPushConstants
#define vkCmdDispatch inherited_vkCmdDispatch
#define vkCmdCopyBuffer inherited_vkCmdCopyBuffer
#define main inherited_child_contract_main
#include "child_preparation_contract.cpp"
#undef main
#undef vkUpdateDescriptorSets
#undef vkCmdPushConstants
#undef vkCmdDispatch
#undef vkCmdCopyBuffer
#pragma GCC diagnostic pop
#include "so101_station_vulkan/observed_children.hpp"
namespace {
unsigned mode=0;uint32_t observed_parameters[16]{};VkBuffer observed_compact=VK_NULL_HANDLE,latest_geometry_counts=VK_NULL_HANDLE;uint32_t observed_fault=0;
}
extern "C" {
void vkUpdateDescriptorSets(VkDevice d,uint32_t n,const VkWriteDescriptorSet* w,uint32_t c,const VkCopyDescriptorSet* cp){
 if(n==7){mode=3;for(unsigned i=0;i<n;i++)if(w[i].dstBinding==6)observed_compact=w[i].pBufferInfo->buffer;}
 else if(n==4){mode=2;for(unsigned i=0;i<n;i++)if(w[i].dstBinding==3)latest_geometry_counts=w[i].pBufferInfo->buffer;}
 else {mode=n==11?1:0;inherited_vkUpdateDescriptorSets(d,n,w,c,cp);}
}
void vkCmdPushConstants(VkCommandBuffer c,VkPipelineLayout l,VkShaderStageFlags s,uint32_t o,uint32_t n,const void* data){
 if(n==64){mode=3;std::memcpy(observed_parameters,data,n);}else if(n==24)mode=2;else{mode=n==128?1:0;inherited_vkCmdPushConstants(c,l,s,o,n,data);}
}
void vkCmdDispatch(VkCommandBuffer c,uint32_t x,uint32_t y,uint32_t z){if(mode<2)inherited_vkCmdDispatch(c,x,y,z);else ++dispatches;}
void vkCmdCopyBuffer(VkCommandBuffer c,VkBuffer from,VkBuffer to,uint32_t n,const VkBufferCopy* copies){
 if(mode!=3){inherited_vkCmdCopyBuffer(c,from,to,n,copies);return;}
 auto* h=static_cast<uint32_t*>(fake_memory[uintptr_t(observed_compact)].data);auto* p=observed_parameters;
 h[0]=1;h[1]=p[0];h[2]=p[1];h[3]=0;h[7]=observed_fault;h[8]=p[7];h[9]=p[8];h[10]=p[11];h[11]=p[12];h[12]=p[13];h[13]=p[14];h[14]=p[5];h[15]=p[3];
 for(unsigned i=0;i<n;i++)std::memcpy(static_cast<char*>(fake_memory[uintptr_t(to)].data)+copies[i].dstOffset,static_cast<char*>(fake_memory[uintptr_t(from)].data)+copies[i].srcOffset,copies[i].size);last_readback=to;
}
}
int main(int argc,char** argv){try{
 require(argc==5);ParentConfig pc;pc.width=8;pc.height=4;pc.min_parent_pixels=1;pc.record_capacity=2;pc.calibration_generation=7;pc.generation=11;ChildConfig cc;cc.generation=13;cc.min_child_pixels=8;cc.record_capacity=2;ObservedConfig oc;oc.generation=17;
 auto context=std::make_shared<Context>();context->enabled_features.features.shaderFloat64=VK_TRUE;context->enabled_features.features.shaderInt64=VK_TRUE;
 auto pool=std::make_shared<FramePool>(8,4,context,false);auto source=pool->acquire();source_metadata(source,pc);source.slot().metadata.publication_session_token=29;
 ParentPreparation parents(context,pool->source_token(),pc,argv[1]);auto parent=parents.submit(source);parent_result(1,8,4);ChildPreparation children(context,pool->source_token(),pc,cc,argv[2]);clear_result();auto child=children.submit(parent);const auto child_readback=last_readback;
 GeometryCalibration gc;gc.width=8;gc.height=4;gc.generation=7;gc.camera_rays.resize(32);gc.depth_scale_m=double(source.slot().calibration.depth_scale);gc.board_from_camera={1,0,0,0,0,1,0,0,0,0,1,0};GeometryPreparation geometry(context,gc,argv[3]);auto geo=geometry.submit(source);
 auto token=geometry.producer_token();require(geo.belongs_to(token));auto gv=geo.resident_view();require(gv.configuration.width==8&&gv.configuration.precision==GeometryPrecision::NativeFloat64&&gv.configuration.layout_version==1&&gv.configuration.depth_scale_m==gc.depth_scale_m);require(&geo.source_frame().slot()==&source.slot());
 // Native double ByteAddressBuffer loads require BOTH actually enabled scalar64 features.
 {context->enabled_features.features.shaderInt64=VK_FALSE;const auto buffers_before=buffer_allocations,handles_before=next_handle;
 rejects<std::runtime_error>([&]{GeometryPreparation unavailable(context,gc,argv[3]);});require(buffer_allocations==buffers_before&&next_handle==handles_before);
 rejects<std::runtime_error>([&]{ObservedPreparation unavailable(context,pool->source_token(),token,pc,cc,oc,argv[4]);});require(buffer_allocations==buffers_before&&next_handle==handles_before);
 context->enabled_features.features.shaderInt64=VK_TRUE;}
 GeometryPreparation different(context,gc,argv[3]);auto foreign_geo=different.submit(source);const auto foreign_geometry_counts=latest_geometry_counts;require(!foreign_geo.belongs_to(token));
 ObservedPreparation observed(context,pool->source_token(),token,pc,cc,oc,argv[4]);rejects<std::invalid_argument>([&]{observed.submit(child,foreign_geo);});
 {auto narrow=gc;narrow.max_observation_depth_m=1.1;GeometryPreparation wrong(context,narrow,argv[3]);rejects<std::invalid_argument>([&]{ObservedPreparation bad(context,pool->source_token(),wrong.producer_token(),pc,cc,oc,argv[4]);});}
 // Actual admission failures on identity/configuration; no shader success invented.
 {auto other_pool=std::make_shared<FramePool>(8,4,context,false);ObservedPreparation bad(context,other_pool->source_token(),token,pc,cc,oc,argv[4]);rejects<std::invalid_argument>([&]{bad.submit(child,geo);});}
 {auto other=pool->acquire();source_metadata(other,pc);auto other_geometry=geometry.submit(other);rejects<std::invalid_argument>([&]{observed.submit(child,other_geometry);});}
 {auto wrong_parent=pc;wrong_parent.min_parent_pixels++;ObservedPreparation bad(context,pool->source_token(),token,wrong_parent,cc,oc,argv[4]);rejects<std::invalid_argument>([&]{bad.submit(child,geo);});}
 {auto wrong_child=cc;wrong_child.generation++;ObservedPreparation bad(context,pool->source_token(),token,pc,wrong_child,oc,argv[4]);rejects<std::invalid_argument>([&]{bad.submit(child,geo);});}
 ++source.slot().metadata.calibration_generation;rejects<std::invalid_argument>([&]{observed.submit(child,geo);});--source.slot().metadata.calibration_generation;
 ++source.slot().calibration.generation;rejects<std::invalid_argument>([&]{observed.submit(child,geo);});--source.slot().calibration.generation;
 auto scale=source.slot().calibration.depth_scale;source.slot().calibration.depth_scale=scale*2;rejects<std::invalid_argument>([&]{observed.submit(child,geo);});source.slot().calibration.depth_scale=scale;
 {GeometryProducerToken empty;GeometryLease missing;require(!empty&&!missing.belongs_to(token));rejects<std::logic_error>([&]{empty.configuration();});rejects<std::logic_error>([&]{missing.resident_view();});rejects<std::logic_error>([&]{missing.source_frame();});}
 auto view=child.resident_view();require(view.counts.record_count==0&&view.source.belongs_to(pool->source_token()));
 // Header-only borrow must not decode malformed compact child records; old finish MUST reject.
 {auto* data=static_cast<uint32_t*>(fake_memory[uintptr_t(child_readback)].data);data[4]=1;
  require(child.resident_view().counts.record_count==1);rejects<std::runtime_error>([&]{children.finish(child);});data[4]=0;
 }
 // Corrupt Geometry compact result: validated new view/source must reject.
 require(foreign_geometry_counts!=VK_NULL_HANDLE);auto* counts=static_cast<uint32_t*>(fake_memory[uintptr_t(foreign_geometry_counts)].data);counts[2]=1;rejects<std::runtime_error>([&]{foreign_geo.resident_view();});rejects<std::runtime_error>([&]{foreign_geo.source_frame();});counts[2]=0;
 auto baseline=source.slot().references.load();hold=true;{auto a=observed.submit(child,geo);auto copy=a;a={};child={};geo={};rejects<std::logic_error>([&]{children.submit(parent);});rejects<std::logic_error>([&]{geometry.submit(source);});require(!observed.ready(copy));rejects<std::logic_error>([&]{observed.finish(copy);});rejects<std::logic_error>([&]{observed.submit(child,geo);});}
 observed.collect();require(source.slot().references==baseline);rejects<std::logic_error>([&]{children.submit(parent);});rejects<std::logic_error>([&]{geometry.submit(source);}); // inputs retain via producer arenas, not extra FrameLease copies.
 hold=false;std::fill(std::begin(complete),std::end(complete),true);observed.collect();child=children.submit(parent);geo=geometry.submit(source);
 {auto a=observed.submit(child,geo);require(observed.finish(a).record_count==0&&a.resident_buffer()&&a.records());require(a.children().metadata().publication_session_token==29&&a.geometry().belongs_to(token));}
 for(unsigned fault:{1u,2u,4u,8u}){observed_fault=fault;auto a=observed.submit(child,geo);rejects<std::runtime_error>([&]{observed.finish(a);});rejects<std::runtime_error>([&]{a.records();});}observed_fault=0;
 auto heaps=heap_allocations,buffers=buffer_allocations,waits=wait_calls;auto before=dispatches;for(unsigned i=0;i<1000;i++){auto a=observed.submit(child,geo);require(observed.ready(a)&&observed.finish(a).record_count==0);}require(heap_allocations==heaps&&buffer_allocations==buffers&&wait_calls==waits&&dispatches-before==6000);
 std::cout<<"observed actual-module host identity/lifetime; zero hot heap/Buffer/waits;6dispatches;synthetic_empty_not_numeric_PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
