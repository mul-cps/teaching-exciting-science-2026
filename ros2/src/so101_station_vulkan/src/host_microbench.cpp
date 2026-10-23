// Host-only: never constructs a Vulkan Context or opens a physical device.
#include "so101_station_vulkan/pipeline.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <new>
#include <cstring>
using namespace so101_station_vulkan;
static std::atomic<uint64_t> allocations{0};
void* operator new(std::size_t size){if(void* p=std::malloc(size)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](std::size_t size){return ::operator new(size);}
void operator delete(void* p) noexcept{std::free(p);}
void operator delete[](void* p) noexcept{std::free(p);}
void operator delete(void* p,std::size_t) noexcept{std::free(p);}
void operator delete[](void* p,std::size_t) noexcept{std::free(p);}
template<class F> double measure(unsigned iterations,F f){auto start=std::chrono::steady_clock::now();for(unsigned i=0;i<iterations;++i)f();return std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/iterations;}
// Independent direct closing definition, used as a CPU oracle before timing.
static void check_morphology(Slot& s,unsigned w,unsigned h){
 cpu_morphology(s,w,h);
 for(unsigned i=0;i<w*h;++i){unsigned expected_count=0;
  for(unsigned c=0;c<s.component_count;++c){auto& a=s.cpu_components[c];int x=int(i%w),y=int(i/w);bool selected=x>=int(a.x0)&&x<=int(a.x1)&&y>=int(a.y0)&&y<=int(a.y1);
   if(selected)for(int ey=-1;ey<=1;++ey)for(int ex=-1;ex<=1;++ex){int qx=x+ex,qy=y+ey;if(qx<int(a.x0)||qx>int(a.x1)||qy<int(a.y0)||qy>int(a.y1))continue;bool dilated=false;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int nx=qx+dx,ny=qy+dy;if(nx>=int(a.x0)&&nx<=int(a.x1)&&ny>=int(a.y0)&&ny<=int(a.y1))dilated|=s.cpu_parents[ny*int(w)+nx]==a.root;}selected&=dilated;
   }
   bool found=false;for(unsigned j=0;j<s.cpu_morph_counts[i];++j)found|=s.cpu_morph_roots[9*i+j]==a.root;
   if(found!=selected)std::abort();expected_count+=selected;
  }if(s.cpu_morph_counts[i]!=expected_count)std::abort();
 }
}
int main(){
 auto pool=std::make_shared<FramePool>(640,480,nullptr,false);
 Slot compact;std::vector<uint32_t> bytes(compact_bytes()/4);compact.readback.mapped=bytes.data();
 bytes[0]=kMaxComponents;auto* rows=reinterpret_cast<Component*>(bytes.data()+4);
 for(unsigned i=0;i<kMaxComponents;++i)rows[i]={kMaxComponents-i,1,24,0,0,1,1,0,0};
 // Warm up lazy/runtime state before counting hot allocations.
 {auto f=pool->acquire();auto consumer=f;pool->ready(f);}decode_compact(compact);
 auto oracle_pool=std::make_shared<FramePool>(16,16,nullptr,false);auto oracle=oracle_pool->acquire();auto& os=oracle.slot();os.component_count=4;
 for(unsigned c=0;c<4;++c)os.cpu_components[c]={c+1,1,32,0,0,15,15,0,0,0};
 for(unsigned i=0;i<256;++i)os.cpu_parents[i]=(i*17+i/16)%7<2?0:(i%4+1);
 check_morphology(os,16,16);
 auto morph_pool=std::make_shared<FramePool>(640,480,nullptr,false);auto morph_frame=morph_pool->acquire();auto& ms=morph_frame.slot();ms.component_count=1;ms.cpu_components[0]={1,1,640*480,0,0,639,479,0,0,0};std::fill(ms.cpu_parents.begin(),ms.cpu_parents.end(),1);
 cpu_morphology(ms,640,480);
 const auto before=allocations.load();
 auto schedule=measure(1000000,[&](){auto f=pool->acquire();auto consumer=f;if(!f||!pool->ready(f))std::abort();});
 auto saturated=measure(100000,[&](){auto a=pool->acquire(),b=pool->acquire(),c=pool->acquire();if(pool->acquire())std::abort();});
 auto full=measure(10000,[&](){decode_compact(compact);});
 bytes[0]=0;auto empty=measure(1000000,[&](){decode_compact(compact);});
 auto dense_morph=measure(3,[&](){cpu_morphology(ms,640,480);});
 uint64_t bbox_pixels=0;ms.component_count=120;
 for(unsigned c=0;c<120;++c){unsigned a=2*c;ms.cpu_components[c]={a*640+a+1,1,2*(640-2*a)+2*(480-2*a-2),a,a,639-a,479-a,0,0,0};bbox_pixels+=uint64_t(640-2*a)*(480-2*a);}
 for(unsigned y=0;y<480;++y)for(unsigned x=0;x<640;++x){unsigned d=std::min({x,y,639-x,479-y});ms.cpu_parents[y*640+x]=(d%2)?0:d*640+d+1;}
 auto rings_morph=measure(1,[&](){cpu_morphology(ms,640,480);});
 const auto hot=allocations.load()-before;
 std::cout<<"{\"scope\":\"host-only actual pool/compact decoder, no Vulkan submission\",\"iterations\":1000000,\"acquire_copy_ready_release_ns\":"<<schedule<<",\"three_slots_backpressure_ns\":"<<saturated<<",\"compact_decode_4096_ns\":"<<full<<",\"compact_decode_empty_ns\":"<<empty<<",\"hot_heap_allocations\":"<<hot<<",\"compact_transfer_bytes\":"<<compact_bytes()<<"}\n";
 std::cout<<"{\"scope\":\"actual CPU per-piece morphology, 640x480\",\"dense_one_piece_ms\":"<<dense_morph/1e6<<",\"nested_120_piece_ms\":"<<rings_morph/1e6<<",\"nested_bbox_pixels\":"<<bbox_pixels<<",\"oracle_exact\":true,\"hot_heap_allocations\":"<<hot<<"}\n";
 return hot?1:0;
}
