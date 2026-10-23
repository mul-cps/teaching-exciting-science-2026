#include "so101_station_vulkan/raw_capture_bridge.hpp"
#include <cmath>
#include <stdexcept>
namespace so101_station_vulkan {
RawCaptureBridge::RawCaptureBridge(std::unique_ptr<Pipeline> core,std::unique_ptr<Alignment> depth)
 :depth_(std::move(depth)),core_(std::move(core)){
 if(!core_)throw std::invalid_argument("raw capture requires a core pipeline");
 if(depth_){
  const auto& k=depth_->calibration();
  if(core_->backend()!="vulkan"||!core_->context()||core_->context()!=depth_->context()||
     core_->width()!=unsigned(k.color.width)||core_->height()!=unsigned(k.color.height))
   throw std::invalid_argument("raw capture requires the same Vulkan Context and color extent");
 }
}
// Drain consuming core commands while every queue-retained source still exists.
// Member order supplies the same guarantee during constructor unwinding.
RawCaptureBridge::~RawCaptureBridge(){core_.reset();}
void RawCaptureBridge::require_healthy()const{
 if(failed_)throw std::logic_error("raw capture failed; recreate generation before capture");
}
Calibration RawCaptureBridge::expected_calibration()const{
 if(!depth_)throw std::logic_error("vendor alignment supplies its own calibration");
 const auto& k=depth_->calibration();
 return {k.color.fx,k.color.fy,k.color.ppx,k.color.ppy,k.depth_units,k.generation};
}
RawFrame RawCaptureBridge::acquire(){
 require_healthy();RawFrame f;f.core=core_->acquire();if(!f.core)return {};
 if(depth_){f.depth=depth_->acquire();if(!f.depth)return {};}
 auto& s=f.core.slot();auto& m=s.metadata;
 // Preserve string capacity when clearing stale source provenance on slot reuse.
 m.width=core_->width();m.height=core_->height();m.frame_number=0;m.timestamp_ns=0;
 m.calibration_generation=0;m.depth_frame_number=0;m.hardware_timestamp_ms=0;
 m.depth_hardware_timestamp_ms=0;m.timestamp_domain=0;m.depth_timestamp_domain=0;m.serial.clear();
 s.calibration=Calibration{};
 return f;
}
void RawCaptureBridge::validate(const RawFrame& f)const{
 if(!f.core||!core_->owns(f.core))
  throw std::invalid_argument("foreign raw capture core lease");
 const auto& s=f.core.slot();const auto& m=s.metadata;
 if(s.submitted||m.width!=core_->width()||m.height!=core_->height())
  throw std::invalid_argument("raw capture core extent/submission mismatch");
 if(!depth_){if(f.depth)throw std::invalid_argument("unexpected raw depth lease");return;}
 if(!f.depth||!depth_->owns(f.depth))
  throw std::invalid_argument("foreign raw capture depth lease");
 const auto& d=f.depth.metadata();const auto k=expected_calibration();const auto& c=s.calibration;
 if(m.width!=d.width||m.height!=d.height||m.frame_number!=d.frame_number||
    m.depth_frame_number!=d.depth_frame_number||m.timestamp_ns!=d.timestamp_ns||
    m.calibration_generation!=d.calibration_generation||m.calibration_generation!=k.generation||
    !std::isfinite(m.hardware_timestamp_ms)||!std::isfinite(m.depth_hardware_timestamp_ms)||
    m.hardware_timestamp_ms!=d.hardware_timestamp_ms||m.depth_hardware_timestamp_ms!=d.depth_hardware_timestamp_ms||
    m.timestamp_domain!=d.timestamp_domain||m.depth_timestamp_domain!=d.depth_timestamp_domain||m.serial!=d.serial||
    c.fx!=k.fx||c.fy!=k.fy||c.cx!=k.cx||c.cy!=k.cy||c.depth_scale!=k.depth_scale||c.generation!=k.generation)
  throw std::invalid_argument("raw capture source cohort/calibration mismatch");
}
void RawCaptureBridge::offer(RawFrame f){require_healthy();validate(f);queue_.offer(std::move(f));}
void RawCaptureBridge::dispatch_pending(){
 require_healthy();
 queue_.dispatch_pending([this](const RawFrame& f){
  try{
   validate(f);
   if(depth_){depth_->submit(f.depth);core_->submit_device_depth(f.core,f.depth.aligned());}
   else core_->submit(f.core);
  }catch(...){failed_=true;throw;}
 });
}
bool RawCaptureBridge::collect(RawFrame& completed){
 require_healthy();const auto& f=queue_.active();if(!f)return false;
 try{
  if(!core_->ready(f.core))return false;
  if(depth_&&!depth_->ready(f.depth))throw std::runtime_error("raw capture producer incomplete after consumer fence");
  core_->finish(f.core);
 }catch(...){failed_=true;throw;}
 completed=queue_.take_completed();return true;
}
}
