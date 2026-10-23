#include "so101_station_vulkan/video.hpp"
extern "C" {
#include <libswscale/swscale.h>
}
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
using namespace so101_station_vulkan;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Decoded {std::vector<uint64_t> hash;std::vector<double> luma;bool correct=true;};
void collect(VideoDecoder& decoder,Decoded& result,bool hardware){
 AVFrame* frame=av_frame_alloc();AVFrame* host=av_frame_alloc();require(frame&&host,"decode frame allocation");
 try{int status;while((status=decoder.receive(frame))>=0){
  require(!(frame->flags&AV_FRAME_FLAG_CORRUPT),"corrupt decoded frame");AVFrame* input=frame;
  if(hardware){require(frame->format==AV_PIX_FMT_VULKAN,"hardware decoder silently fell back");require(av_hwframe_transfer_data(host,frame,0)>=0,"qualification diagnostic readback failed");input=host;}
  require(input->width==640&&input->height==480,"wrong decoded dimensions");
  AVFrame* normalized=av_frame_alloc();require(normalized,"normalization frame allocation");normalized->format=AV_PIX_FMT_NV12;normalized->width=640;normalized->height=480;require(av_frame_get_buffer(normalized,32)>=0,"normalization image allocation");
  SwsContext* scale=sws_getContext(640,480,AVPixelFormat(input->format),640,480,AV_PIX_FMT_NV12,SWS_POINT|SWS_BITEXACT,nullptr,nullptr,nullptr);require(scale,"normalization context");auto rows=sws_scale(scale,input->data,input->linesize,0,480,normalized->data,normalized->linesize);sws_freeContext(scale);require(rows==480,"normalization incomplete");
  uint64_t hash=1469598103934665603ULL;uint64_t luma=0;
  for(unsigned plane=0;plane<2;plane++)for(unsigned y=0;y<(plane?240u:480u);y++)for(unsigned x=0;x<640;x++){uint8_t b=normalized->data[plane][y*normalized->linesize[plane]+x];hash=(hash^b)*1099511628211ULL;if(!plane)luma+=b;}
  unsigned r=30+(result.hash.size()%10)*10;int expected=((66*int(r)+129*90+25*150+128)>>8)+16;double actual=double(luma)/(640*480);result.correct&=std::abs(actual-expected)<=8;result.luma.push_back(actual);result.hash.push_back(hash);av_frame_free(&normalized);av_frame_unref(frame);av_frame_unref(host);
 }require(status==AVERROR(EAGAIN)||status==AVERROR_EOF,"strict decoder error");
 }catch(...){av_frame_free(&frame);av_frame_free(&host);throw;}av_frame_free(&frame);av_frame_free(&host);
}
}
int main(int argc,char** argv){
 if(argc!=4){std::cerr<<"usage: qualify_video cpu|vulkan RESIDENT_SHADER_DIR OUTPUT.json\n";return 2;}
 nlohmann::json report={{"schema","so101-video-qualification-v1"},{"backend",argv[1]},{"encode_frames",0},{"cpu_decode_frames",0},{"hardware_decode_frames",0},{"same_decoded_pixels",false},{"rgb_correct",false}};
 int exit_status=1;Decoded cpu_result,hardware_result;unsigned encoded=0;
 try{std::string mode=argv[1];require(mode=="cpu"||mode=="vulkan","invalid qualification mode");Pipeline pipeline(640,480,mode,argv[2]);std::shared_ptr<SharedVideoDevice> device;if(mode=="vulkan")device=std::make_shared<SharedVideoDevice>(pipeline.context());
  report["profile_key"]=video_profile_key(pipeline.context(),640,480,30);report["probe_only"]=true;
  // This explicit probe intentionally bypasses runtime admission. Its output is
  // admitted later only by the exact-identity record loader after ALL checks.
  VideoEncoder encoder(640,480,30,{mode,"explicit qualification probe"},device);VideoDecoder cpu({"cpu","independent strict decoder"});std::unique_ptr<VideoDecoder> hardware;if(device)hardware=std::make_unique<VideoDecoder>(VideoSelection{"vulkan","qualification probe"},device);
  AVPacket* packet=av_packet_alloc();require(packet,"packet allocation");
  auto drain=[&](){int status;while((status=encoder.receive(packet))>=0){encoded++;require(cpu.send(packet)>=0,"CPU packet admission failed");collect(cpu,cpu_result,false);if(hardware){require(hardware->send(packet)>=0,"hardware packet admission failed");collect(*hardware,hardware_result,true);}av_packet_unref(packet);}require(status==AVERROR(EAGAIN)||status==AVERROR_EOF,"encoder output error");};
  for(unsigned i=0;i<60;i++){
   auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);FrameLease lease;
   while(!(lease=pipeline.acquire())){drain();require(std::chrono::steady_clock::now()<deadline,"source lease timeout");std::this_thread::sleep_for(std::chrono::milliseconds(1));}
   auto* bytes=static_cast<uint8_t*>(lease.slot().upload.mapped);for(unsigned j=0;j<640*480;j++){bytes[j*3]=30+(i%10)*10;bytes[j*3+1]=90;bytes[j*3+2]=150;bytes[640*480*3+j*2]=0;bytes[640*480*3+j*2+1]=0;}pipeline.submit(lease);
   while(!pipeline.ready(lease)){require(std::chrono::steady_clock::now()<deadline,"compute timeout");std::this_thread::sleep_for(std::chrono::milliseconds(1));}pipeline.finish(lease);
   int status;while((status=encoder.submit_rgb(lease,i))==AVERROR(EAGAIN)){drain();require(std::chrono::steady_clock::now()<deadline,"video backpressure timeout");std::this_thread::sleep_for(std::chrono::milliseconds(1));}require(status>=0,"encode frame failed");drain();
  }
  while(encoder.flush()==AVERROR(EAGAIN))drain();drain();require(cpu.send(nullptr)>=0,"CPU flush failed");collect(cpu,cpu_result,false);if(hardware){require(hardware->send(nullptr)>=0,"hardware flush failed");collect(*hardware,hardware_result,true);}av_packet_free(&packet);
  report["cpu_frame_hashes"]=cpu_result.hash;report["hardware_frame_hashes"]=hardware_result.hash;report["cpu_luma_means"]=cpu_result.luma;report["encode_frames"]=encoded;report["cpu_decode_frames"]=cpu_result.hash.size();report["hardware_decode_frames"]=hardware_result.hash.size();report["rgb_correct"]=cpu_result.correct&&hardware_result.correct;report["same_decoded_pixels"]=hardware&&cpu_result.hash==hardware_result.hash;
  require(encoded==60&&cpu_result.hash.size()==60&&cpu_result.correct,"incomplete or incorrect independent CPU decode");if(hardware)require(hardware_result.hash.size()==60&&cpu_result.hash==hardware_result.hash&&hardware_result.correct,"hardware/CPU decoded image mismatch");exit_status=0;
 }catch(const std::exception& error){report["error"]=error.what();std::cerr<<error.what()<<"\n";}
 report["cpu_frame_hashes"]=cpu_result.hash;report["hardware_frame_hashes"]=hardware_result.hash;report["cpu_luma_means"]=cpu_result.luma;report["encode_frames"]=encoded;report["cpu_decode_frames"]=cpu_result.hash.size();report["hardware_decode_frames"]=hardware_result.hash.size();
 report["passed"]=exit_status==0;std::ofstream output(argv[3]);output<<report.dump(2)<<"\n";if(!output){std::cerr<<"cannot write qualification report\n";return 2;}std::cout<<report.dump()<<"\n";return exit_status;
}
