// CPU-only fixture: compile the extracted, unchanged shader helper as C++.
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cfenv>
#include <chrono>
#include <initializer_list>
using uint = uint32_t;
static uint asuint(float x) { uint u; std::memcpy(&u, &x, 4); return u; }
static float asfloat(uint u) { float x; std::memcpy(&x, &u, 4); return x; }
#include "alignment_divide_shared.h"
static uint random_bits() {
    static uint state = 0x5a17b9d3u;
    state ^= state << 13; state ^= state >> 17; state ^= state << 5;
    return state;
}
static uint64_t divisions = 0, quotients = 0;
static void check_divide(uint a, uint b) {
    volatile float left = asfloat(a), right = asfloat(b);
    float expected = left / right;
    uint want = asuint(expected), got = alignment_divide_bits(a, b);
    if ((want & 0x7fffffffu) > 0x7f800000u) want = 0x7fc00000u;
    if (got != want || asuint(alignment_divide(asfloat(a), asfloat(b))) != want) {
        std::fprintf(stderr, "division %08x / %08x expected %08x got %08x\n", a,b,want,got);
        std::exit(1);
    }
    ++divisions;
}
static void check_quotient(uint a, uint b, uint seed) {
    uint64_t numerator = uint64_t(a) << 23;
    auto got = alignment_exact_quotient(a,b,seed);
    if (got.whole != numerator/b || got.remainder != numerator%b) {
        std::fprintf(stderr,"quotient a=%u b=%u seed=%u expected=%llu/%llu got=%u/%u\n",
            a,b,seed,static_cast<unsigned long long>(numerator/b),
            static_cast<unsigned long long>(numerator%b),got.whole,got.remainder);
        std::exit(1);
    }
    ++quotients;
}
int main() {
    if (std::fesetround(FE_TONEAREST)) return 2;
    auto start = std::chrono::steady_clock::now();
    // A normal-domain regression: rounded reciprocal times numerator differs
    // from hardware RNE division by one ULP, reproducing the causal operation.
    volatile float regression_a=asfloat(0x3f330a0cu),regression_b=asfloat(0x3f73a9e7u);
    volatile float reciprocal=1.0f/regression_b;
    volatile float reciprocal_product=regression_a*reciprocal;
    if(asuint(reciprocal_product)!=0x3f3c1a8du) return 3;
    check_divide(0x3f330a0cu,0x3f73a9e7u);
    if(alignment_divide_bits(0x3f330a0cu,0x3f73a9e7u)!=0x3f3c1a8eu) return 4;
    // Explicit exact halfway underflow rounds both down to even zero and up
    // to even two; negative signs and the min-normal boundary are also covered.
    check_divide(1u,0x40000000u);check_divide(3u,0x40000000u);
    check_divide(0x80000001u,0x40000000u);check_divide(0x00ffffffu,0x40000000u);
    const uint boundaries[] = {0,1,2,3,0x003fffff,0x00400000,0x007ffffe,0x007fffff,
        0x00800000,0x00800001,0x00ffffff,0x01000000,0x3effffff,0x3f000000,
        0x3f000001,0x3f7fffff,0x3f800000,0x3f800001,0x40000000,0x4b000000,
        0x7effffff,0x7f000000,0x7f7ffffe,0x7f7fffff,0x7f800000,0x7f800001,0x7fc00000,0x7fffffff};
    for (uint a:boundaries) for(uint b:boundaries) for(uint sa:{0u,0x80000000u})
        for(uint sb:{0u,0x80000000u}) check_divide(a|sa,b|sb);
    // Every exponent, signs, near-boundary significands; includes subnormal ties,
    // the max-finite overflow boundary and subnormal-to-normal carry.
    for (uint exponent=0;exponent<255;++exponent) for (uint mantissa:{0u,1u,2u,0x3fffffu,0x7ffffeu,0x7fffffu}) {
        uint a = (exponent<<23)|mantissa;
        for(uint b:boundaries) {check_divide(a,b);check_divide(a|0x80000000u,b);}
    }
    for(uint i=0;i<3000000;++i) check_divide(random_bits(),random_bits());
    for(uint i=0;i<100000;++i) {
        uint b=0x800000u|(random_bits()&0x7fffffu);
        uint a=0x800000u|(random_bits()&0x7fffffu);
        if(a<b) a<<=1;
        uint exact=uint((uint64_t(a)<<23)/b);
        for(int delta:{-16,-4,-3,-1,0,1,3,4,16}) check_quotient(a,b,uint(int(exact)+delta));
        check_quotient(a,b,0);check_quotient(a,b,0xffffffffu);
    }
    double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::printf("PASS %llu hardware divisions, %llu independently checked quotients in %.3fs\n",
        static_cast<unsigned long long>(divisions),static_cast<unsigned long long>(quotients),elapsed);
}
