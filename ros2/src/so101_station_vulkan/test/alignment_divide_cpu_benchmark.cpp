// CPU-only timing of the extracted production helper. Never initializes Vulkan.
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cfenv>
using uint=uint32_t;
static uint asuint(float x){uint u;std::memcpy(&u,&x,4);return u;}
static float asfloat(uint u){float x;std::memcpy(&x,&u,4);return x;}
#include "alignment_divide_shared.h"
struct Pair{float a,b;uint na,nb,seed;};
static volatile uint checksum=0;
__attribute__((noinline)) static uint hardware(const Pair& p){volatile float a=p.a,b=p.b;return asuint(a/b);}
__attribute__((noinline)) static uint exact(const Pair& p){return asuint(alignment_divide(p.a,p.b));}
__attribute__((noinline)) static uint quotient(const Pair& p){auto q=alignment_exact_quotient(p.na,p.nb,p.seed);return q.whole^q.remainder;}
using Operation=uint(*)(const Pair&);
static double measure(const std::vector<Pair>& pairs,Operation op){
    std::vector<double> ns;
    for(int repeat=0;repeat<7;++repeat){
        uint sum=0;auto start=std::chrono::steady_clock::now();
        for(const auto& p:pairs)sum+=op(p);
        auto end=std::chrono::steady_clock::now();checksum=sum;
        ns.push_back(std::chrono::duration<double,std::nano>(end-start).count()/pairs.size());
    }
    std::sort(ns.begin(),ns.end());return ns[3];
}
int main(){
    if(std::fesetround(FE_TONEAREST))return 2;
    constexpr uint count=640*480;
    std::printf("{\"cpu_only\":true,\"samples_per_case\":%u,\"repetitions\":7,\"statistic\":\"median\",\"cases\":{",count);
    const char* names[]={"projection","deprojection","quotient_native_seed","quotient_four_corrections","quotient_fallback_out_of_range","quotient_fallback_distant_seed"};
    for(uint scenario=0;scenario<6;++scenario){
        std::vector<Pair> pairs;pairs.reserve(count);
        for(uint i=0;i<count;++i){
            float z=float(1+i%65000)*.001f;
            Pair p{};p.a=(float(i%640)-319.25f)/615.5f*z+.025f;p.b=z+.0125f;
            if(scenario==1){p.a=float((i&1)?i%640:i/640)-((i&1)?319.25f:239.75f);p.b=(i&1)?615.5f:611.75f;}
            p.nb=0x800000u|((i*719u)&0x7fffffu);p.na=0x800000u|((i*337u)&0x7fffffu);if(p.na<p.nb)p.na<<=1;
            if(scenario>=2){p.a=float(p.na);p.b=float(p.nb);}
            uint true_q=uint((uint64_t(p.na)<<23)/p.nb);
            p.seed=scenario==3?true_q-4:scenario==4?0:scenario==5?true_q+16:uint((float(p.na)/float(p.nb))*8388608.0f);
            pairs.push_back(p);
        }
        double native=measure(pairs,hardware),helper=measure(pairs,scenario<2?exact:quotient);
        std::printf("%s\"%s\":{\"helper_ns_per_op\":%.3f,\"hardware_ns_per_op\":%.3f}",scenario?",":"",names[scenario],helper,native);
    }
    std::printf("},\"checksum\":%u}\n",checksum);
}
