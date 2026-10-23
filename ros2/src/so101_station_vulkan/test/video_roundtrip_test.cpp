#include "so101_station_vulkan/video.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace so101_station_vulkan;
int main(){
 constexpr unsigned width=64,height=48,total=12;
 auto require=[](bool ok,const char* message){if(!ok)throw std::runtime_error(message);};
 auto pool=std::make_shared<FramePool>(width,height,nullptr,false);
 VideoEncoder encoder(width,height,30,{"cpu","test"});
 VideoDecoder decoder({"cpu","test"});
 auto* packet=av_packet_alloc();auto* decoded=av_frame_alloc();require(packet&&decoded,"frame/packet allocation");
 unsigned packets=0,frames=0;std::vector<AVPacket*> first_packets;
 auto drain_decoded=[&](){int r;while((r=decoder.receive(decoded))>=0){require(decoded->width==int(width)&&decoded->height==int(height),"decoded dimensions");require(decoded->format!=AV_PIX_FMT_VULKAN,"CPU comparison must remain CPU");frames++;av_frame_unref(decoded);}require(r==AVERROR(EAGAIN)||r==AVERROR_EOF,"decode failed");};
 auto drain=[&](){int r;while((r=encoder.receive(packet))>=0){packets++;if(first_packets.size()<4)first_packets.push_back(av_packet_clone(packet));require(decoder.send(packet)>=0,"decoder send");drain_decoded();av_packet_unref(packet);}require(r==AVERROR(EAGAIN)||r==AVERROR_EOF,"encode failed");};
 require(encoder.submit_rgb({},0)==AVERROR(EINVAL),"empty lease rejection");
 for(unsigned i=0;i<total;i++){
   auto lease=pool->acquire();require(bool(lease),"frame lease unavailable");auto* bytes=static_cast<uint8_t*>(lease.slot().upload.mapped);
   for(unsigned j=0;j<width*height;j++){bytes[j*3]=uint8_t(30+i*10);bytes[j*3+1]=90;bytes[j*3+2]=150;}
   std::memset(bytes+width*height*3,0xa5,width*height*2);
   require(encoder.submit_rgb(lease,i)>=0,"encode submit");drain();
   require(std::all_of(bytes+width*height*3,bytes+width*height*5,[](uint8_t b){return b==0xa5;}),"metric depth changed");
 }
 require(encoder.flush()>=0,"flush");drain();require(decoder.send(nullptr)>=0,"decoder flush");drain_decoded();
 require(packets==total&&frames==total,"incomplete encoded/decoded delivery");require(encoder.submit_rgb(pool->acquire(),total)==AVERROR_EOF,"submit after flush");
 VideoDecoder bounded({"cpu","bounded output test"});std::array<AVFrame*,4> held{};for(auto*& f:held)f=av_frame_alloc();
 for(unsigned i=0;i<3;i++){require(bounded.send(first_packets[i])>=0,"bounded packet send");require(bounded.receive(held[i])>=0,"bounded output receive");}
 require(bounded.send(first_packets[3])>=0,"fourth packet send");require(bounded.receive(held[3])==AVERROR(EAGAIN),"fourth held output must backpressure");av_frame_unref(held[0]);require(bounded.receive(held[3])>=0,"released ticket must permit next output");
 AVPacket oversized{};oversized.size=2*1024*1024+1;require(bounded.send(&oversized)==AVERROR(EINVAL),"oversized input rejection");
 for(auto*& f:held)av_frame_free(&f);for(auto*& pkt:first_packets)av_packet_free(&pkt);
 av_packet_free(&packet);av_frame_free(&decoded);
 std::cout<<"CPU H264 roundtrip "<<frames<<" frames; depth unchanged; flush and invalid input PASS\n";
}
