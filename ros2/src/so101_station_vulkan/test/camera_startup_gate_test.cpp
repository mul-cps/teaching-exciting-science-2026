#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "so101_station_vulkan/camera_startup_gate.hpp"

using so101_station_vulkan::CameraSourceObservation;
using so101_station_vulkan::CameraStartupGate;
namespace {
void require(bool value,const char* message){if(!value){std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}}
template<class F> void refuses(F function,const char* message){try{function();}catch(const std::runtime_error&){return;}require(false,message);}
CameraSourceObservation frame(uint64_t color,uint64_t depth,double ct,double dt,int cd=2,int dd=2){return {color,depth,ct,dt,cd,dd};}
void minimum_observation_period(){
 CameraStartupGate gate(0);
 require(!gate.observe(frame(1,1,1,1),0),"immediate source admission");
 require(!gate.observe(frame(2,2,2,2),100000000),"early admission 100ms");
 require(!gate.observe(frame(3,3,3,3),200000000),"early admission 200ms");
 require(!gate.observe(frame(4,4,4,4),300000000),"early admission 300ms");
 require(!gate.observe(frame(5,5,5,5),400000000),"early admission 400ms");
 require(!gate.observe(frame(6,6,6,6),500000000),"stable source admitted before minimum period");
 require(gate.observe(frame(7,7,7,7),2000000000),"stable run beginning at time zero lost");
}
void stable_duration_and_advancement_count(){
 CameraStartupGate duration(0);
 require(!duration.observe(frame(1,1,1,1),2000000000),"new run admitted immediately");
 require(!duration.observe(frame(2,2,2,2),2100000000),"new run admitted at 100ms");
 require(!duration.observe(frame(3,3,3,3),2200000000),"new run admitted at 200ms");
 require(!duration.observe(frame(4,4,4,4),2300000000),"new run admitted at 300ms");
 require(!duration.observe(frame(5,5,5,5),2499999999),"stable duration boundary rounded early");
 require(duration.observe(frame(6,6,6,6),2500000000),"500ms stable advancing source not admitted");
 CameraStartupGate count(0);
 require(!count.observe(frame(1,1,1,1),2000000000),"count first");
 require(!count.observe(frame(2,2,2,2),2500000000),"two cohorts satisfy five-cohort requirement");
 require(!count.observe(frame(3,3,3,3),2600000000),"three cohorts satisfy five-cohort requirement");
 require(!count.observe(frame(4,4,4,4),2700000000),"four cohorts satisfy five-cohort requirement");
 require(count.observe(frame(5,5,5,5),2800000000),"fifth advancing cohort not admitted");
}
void captured_style_counter_resets(){
 CameraStartupGate gate(0);
 require(!gate.observe(frame(1,6,1,1),0),"initial source admitted");
 require(!gate.observe(frame(2,6,2,1),100000000),"repeated native depth admitted");
 require(!gate.observe(frame(40,0,40,2),1579000000),"first native depth zero admitted");
 require(!gate.observe(frame(41,1,41,3),1600000000),"reset source admitted immediately");
 require(!gate.observe(frame(45,0,45,4),1780000000),"second native depth zero admitted");
 require(!gate.observe(frame(49,2,49,5),1900000000),"post-reset baseline admitted");
 require(!gate.observe(frame(52,3,52,6),2000000000),"minimum period bypassed stability");
 require(!gate.observe(frame(55,4,55,7),2100000000),"short stable run admitted");
 require(!gate.observe(frame(58,5,58,8),2200000000),"short stable run admitted");
 require(!gate.observe(frame(64,6,64,9),2399999999),"reset did not restart stability duration");
 require(gate.observe(frame(65,7,65,10),2400000000),"clean advancing run after native resets not admitted");
}
void every_positive_anomaly_restarts_run(){
 const CameraSourceObservation anomalies[]={frame(11,12,12,12),frame(12,11,12,12),frame(9,12,12,12),frame(12,9,12,12),frame(12,12,10,12),frame(12,12,12,10),frame(12,12,12,12,3,2),frame(12,12,12,12,2,3)};
 for(const auto& bad:anomalies){
  CameraStartupGate gate(0);
  require(!gate.observe(frame(10,10,10,10),1800000000),"anomaly setup first");
  require(!gate.observe(frame(11,11,11,11),1900000000),"anomaly setup second");
  require(!gate.observe(bad,2000000000),"source anomaly admitted");
  for(unsigned i=1;i<=4;i++)require(!gate.observe(frame(bad.color_frame+i,bad.depth_frame+i,bad.color_time_ms+i,bad.depth_time_ms+i,bad.color_domain,bad.depth_domain),i==4?2499999999:2000000000+i*100000000),"anomaly retained old stable run");
  require(gate.observe(frame(bad.color_frame+5,bad.depth_frame+5,bad.color_time_ms+5,bad.depth_time_ms+5,bad.color_domain,bad.depth_domain),2500000000),"positive anomaly did not establish a fresh baseline");
 }
}
void zero_clears_baseline(){
 for(unsigned which=0;which<2;which++){
  CameraStartupGate gate(0);
  require(!gate.observe(frame(1,1,1,1),1800000000),"zero setup");
  require(!gate.observe(frame(which?2:0,which?0:2,2,2),2000000000),"zero source admitted");
  for(unsigned i=0;i<5;i++)require(!gate.observe(frame(3+i,3+i,3+i,3+i),2100000000+i*100000000),"zero preserved previous baseline");
  require(gate.observe(frame(8,8,8,8),2600000000),"positive run after zero never admits");
 }
}
void repeated_source_and_absence_timeout(){
 CameraStartupGate repeated(0);
 for(unsigned i=0;i<20;i++)require(!repeated.observe(frame(1,1,1,1),uint64_t(i)*500000000),"repeated source became ready");
 refuses([&]{repeated.observe(frame(2,2,2,2),10000000000);},"source admitted at deadline");
 CameraStartupGate absent(0);absent.check_deadline(9999999999);
 refuses([&]{absent.check_deadline(10000000000);},"no-input deadline ignored");
 CameraStartupGate ready_at_deadline(0);
 for(unsigned i=0;i<4;i++)require(!ready_at_deadline.observe(frame(i+1,i+1,i+1,i+1),9000000000+i*100000000),"four-cohort setup admitted");
 refuses([&]{ready_at_deadline.observe(frame(5,5,5,5),10000000000);},"ready result beat deadline");
}
void invalid_timestamps_and_clock(){
 for(unsigned lane=0;lane<2;lane++)for(double value:{std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
  CameraStartupGate gate(0);auto bad=frame(1,1,1,1);if(lane)bad.depth_time_ms=value;else bad.color_time_ms=value;
  refuses([&]{gate.observe(bad,0);},"nonfinite source timestamp accepted");
 }
 CameraStartupGate before_start(100);
 refuses([&]{before_start.check_deadline(99);},"clock before start accepted");
 CameraStartupGate regressed(0);regressed.observe(frame(1,1,1,1),100);
 refuses([&]{regressed.observe(frame(2,2,2,2),99);},"observation clock regression accepted");
 CameraStartupGate absent_regression(0);absent_regression.check_deadline(100);
 refuses([&]{absent_regression.check_deadline(99);},"no-input clock regression accepted");
}
void elapsed_bounds_near_uint64_max(){
 const uint64_t start=std::numeric_limits<uint64_t>::max()-10000000000ULL;
 CameraStartupGate gate(start);
 for(unsigned i=0;i<4;i++)require(!gate.observe(frame(i+1,i+1,i+1,i+1),start+2000000000ULL+i*100000000ULL),"overflow setup admitted");
 require(gate.observe(frame(5,5,5,5),start+2500000000ULL),"elapsed-time arithmetic overflowed");
 refuses([&]{gate.check_deadline(std::numeric_limits<uint64_t>::max());},"overflow deadline ignored");
}
}
int main(){minimum_observation_period();stable_duration_and_advancement_count();captured_style_counter_resets();every_positive_anomaly_restarts_run();zero_clears_baseline();repeated_source_and_absence_timeout();invalid_timestamps_and_clock();elapsed_bounds_near_uint64_max();std::cout<<"camera startup gate: 8 behavior groups PASS\n";}
