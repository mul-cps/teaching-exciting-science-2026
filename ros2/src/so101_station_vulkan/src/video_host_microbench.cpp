// CPU-only warmed production video APIs; codec/libav allocations are separate.
#include "so101_station_vulkan/video.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
using namespace so101_station_vulkan;
static std::atomic<uint64_t> allocations{0};
void* operator new(std::size_t n){if(void* p=std::malloc(n)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p) noexcept{std::free(p);}void operator delete[](void* p) noexcept{std::free(p);}
void operator delete(void* p,std::size_t) noexcept{std::free(p);}void operator delete[](void* p,std::size_t) noexcept{std::free(p);}
int main(){try{
 constexpr unsigned warmup=10,measured=60,total=warmup+measured;
 auto pool=std::make_shared<FramePool>(640,480,nullptr,false);auto input=pool->acquire();auto* rgb=static_cast<uint8_t*>(input.slot().upload.mapped);
 for(unsigned i=0;i<640*480;i++){rgb[3*i]=60;rgb[3*i+1]=90;rgb[3*i+2]=150;}
 VideoEncoder encoder(640,480,30,{"cpu","host-only warmed microbenchmark"});VideoDecoder decoder({"cpu","host-only warmed microbenchmark"});
 AVPacket* packet=av_packet_alloc();AVFrame* frame=av_frame_alloc();if(!packet||!frame)throw std::runtime_error("allocation");
 std::array<double,measured> ms{};unsigned decoded=0,packets=0;
 auto drain=[&](){int r;while((r=encoder.receive(packet))>=0){++packets;if(decoder.send(packet)<0)throw std::runtime_error("decode send");int d;while((d=decoder.receive(frame))>=0){++decoded;av_frame_unref(frame);}if(d!=AVERROR(EAGAIN)&&d!=AVERROR_EOF)throw std::runtime_error("decode receive");av_packet_unref(packet);}if(r!=AVERROR(EAGAIN)&&r!=AVERROR_EOF)throw std::runtime_error("encode receive");};
 uint64_t before=0;
 for(unsigned i=0;i<total;i++){
  if(i==warmup)before=allocations.load();auto start=std::chrono::steady_clock::now();
  int r=encoder.submit_rgb(input,i);if(r==AVERROR(EAGAIN)){drain();r=encoder.submit_rgb(input,i);}if(r<0)throw std::runtime_error("encode submit");drain();
  if(i>=warmup)ms[i-warmup]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 }
 auto hot=allocations.load()-before;if(encoder.flush()<0)throw std::runtime_error("flush");drain();if(decoder.send(nullptr)<0)throw std::runtime_error("decode flush");while(decoder.receive(frame)>=0){++decoded;av_frame_unref(frame);}
 if(decoded!=total||packets!=total)throw std::runtime_error("incomplete frames");std::sort(ms.begin(),ms.end());
 std::cout<<"{\"scope\":\"CPU-only warmed real RGB-NV12/H264 encode+decode APIs; excludes input generation/perception and libav malloc accounting\",\"frames\":"<<measured<<",\"warmup\":"<<warmup<<",\"width\":640,\"height\":480,\"p50_ms\":"<<ms[(measured-1)/2]<<",\"p95_ms\":"<<ms[(measured-1)*95/100]<<",\"p99_ms\":"<<ms[(measured-1)*99/100]<<",\"hot_cxx_new_calls\":"<<hot<<",\"decoded_frames\":"<<decoded<<"}\n";
 av_packet_free(&packet);av_frame_free(&frame);return hot?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
