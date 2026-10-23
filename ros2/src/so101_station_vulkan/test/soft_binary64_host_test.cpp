// CPU oracle deliberately uses native arithmetic; the included production module does not.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cfenv>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif
#include "../shaders/soft_binary64.slang"
static uint64_t checks=0;
static uint64_t canonical(uint64_t a) { return ((a&0x7fffffffffffffffULL)>0x7ff0000000000000ULL)?0xfff8000000000000ULL:a; }
static double value(uint64_t a) { double d; std::memcpy(&d,&a,8); return d; }
static uint64_t bits(double a) { uint64_t u; std::memcpy(&u,&a,8); return u; }
static void equal(const char* op,uint64_t a,uint64_t b,uint64_t got,uint64_t want) {
 ++checks; if(got!=want) { std::fprintf(stderr,"%s a=%016llx b=%016llx got=%016llx want=%016llx\n",op,(unsigned long long)a,(unsigned long long)b,(unsigned long long)got,(unsigned long long)want); std::exit(1); }
}
static void pair(uint64_t a,uint64_t b) {
 volatile double x=value(a),y=value(b); volatile double sum=x+y,product=x*y;
 equal("add",a,b,sf64_add_bits(a,b),canonical(bits(sum)));
 equal("mul",a,b,sf64_mul_bits(a,b),canonical(bits(product)));
 equal("less",a,b,sf64_less(a,b),x<y);
 equal("less_equal",a,b,sf64_less_equal(a,b),x<=y);
 equal("finite",a,b,sf64_is_finite(a),std::isfinite(x));
}
static void conversion(uint32_t a) {
 float f; std::memcpy(&f,&a,4); volatile float x=f; volatile double y=x;
 equal("from_f32",a,0,sf64_from_f32_bits(a),canonical(bits(y)));
}
static uint64_t random_bits(uint64_t& state) { state^=state<<13; state^=state>>7; state^=state<<17; return state; }
struct Environment {
 fenv_t saved;
#if defined(__SSE__)
 unsigned csr;
#endif
 Environment() { std::fegetenv(&saved);
#if defined(__SSE__)
 csr=_mm_getcsr();
#endif
 std::fesetround(FE_TONEAREST);
#if defined(__SSE__)
 _mm_setcsr(_mm_getcsr()&~((1u<<15)|(1u<<6)));
#endif
 }
 ~Environment() { std::fesetenv(&saved);
#if defined(__SSE__)
 _mm_setcsr(csr);
#endif
 }
};
int main(int argc,char** argv) {
 // Verify the test oracle itself leaves caller rounding and exception state intact.
 fenv_t original; std::fegetenv(&original);
#if defined(__SSE__)
 unsigned originalCsr = _mm_getcsr();
#endif
 std::fesetround(FE_DOWNWARD); std::feraiseexcept(FE_INEXACT);
 { Environment nested; equal("oracle-rne",0,0,std::fegetround(),FE_TONEAREST); }
 equal("restore-rounding",0,0,std::fegetround(),FE_DOWNWARD);
 equal("restore-flags",0,0,std::fetestexcept(FE_INEXACT),FE_INEXACT);
 std::fesetenv(&original);
#if defined(__SSE__)
 _mm_setcsr(originalCsr);
#endif
 Environment environment;
 for(uint64_t a: {0ULL,1ULL,2ULL,0x8000000000000000ULL,0xffffffffffffffffULL,0x123456789abcdef0ULL}) {
   int expected=64; for(int i=63;i>=0;--i) if((a>>i)&1) { expected=63-i; break; }
   equal("clz",a,0,sf64_clz64(a),expected);
   for(uint32_t d: {0u,1u,31u,32u,62u,63u,64u,65u,127u,0xffffffffu}) {
     uint64_t want=a; if(d>=64) want=a!=0; else if(d) want=(a>>d)|((a&((uint64_t(1)<<d)-1))!=0);
     equal("jam",a,d,sf64_jam(a,d),want);
   }
 }
 auto product=sf64_product(0xffffffffffffffffULL,0xffffffffffffffffULL);
 equal("limb-hi",0,0,product.hi,0xfffffffffffffffeULL); equal("limb-lo",0,0,product.lo,1);
 product=sf64_product(0x100000001ULL,0x100000001ULL);
 equal("limb-hi",0,0,product.hi,1); equal("limb-lo",0,0,product.lo,0x200000001ULL);
 equal("round-even",0,0,sf64_round(false,0x3fe,0x4000000000000200ULL),0x3ff0000000000000ULL);
 equal("round-odd",0,0,sf64_round(false,0x3fe,0x4000000000000600ULL),0x3ff0000000000002ULL);
 equal("pack-carry",0,0,sf64_pack(false,0x3ff,0x10000000000000ULL),0x4000000000000000ULL);
 // Every encoding class, both signs, cancellation, carry, halfway and boundaries.
 std::vector<uint64_t> edge={0,1,2,3,0x000ffffffffffffeULL,0x000fffffffffffffULL,0x0010000000000000ULL,0x0010000000000001ULL,0x3c90000000000000ULL,0x3ca0000000000000ULL,0x3cb0000000000000ULL,0x3fdfffffffffffffULL,0x3fe0000000000000ULL,0x3fefffffffffffffULL,0x3ff0000000000000ULL,0x3ff0000000000001ULL,0x3ff0000000000002ULL,0x4000000000000000ULL,0x7fdfffffffffffffULL,0x7feffffffffffffeULL,0x7fefffffffffffffULL,0x7ff0000000000000ULL,0x7ff0000000000001ULL,0x7ff8000000000000ULL,0x7fffffffffffffffULL};
 auto n=edge.size(); for(size_t i=0;i<n;++i) edge.push_back(edge[i]|0x8000000000000000ULL);
 for(auto a:edge) for(auto b:edge) pair(a,b);
 // Deterministic ties across every normal exponent, including both rounding parities.
 for(uint64_t e=1;e<2047;++e) for(uint64_t f=0;f<4;++f) {
 uint64_t a=(e<<52)|f; pair(a,(e>53)?((e-53)<<52):1); pair(a,a^0x8000000000000000ULL);
 }
 uint64_t state=0x93e162a70d45bc89ULL;
 for(unsigned i=0;i<1000000;++i) { auto a=random_bits(state); auto b=random_bits(state); pair(a,b); }
 for(uint32_t a: {0u,1u,2u,0x007fffffu,0x00800000u,0x3f800000u,0x7f7fffffu,0x7f800000u,0x7f800001u,0x7fc00000u,0x80000000u,0x80000001u,0xff800000u,0xffffffffu}) conversion(a);
 for(unsigned i=0;i<200000;++i) conversion(uint32_t(random_bits(state)));
 // Optional unmodified TestFloat generated result vectors; exception column intentionally ignored.
 uint64_t vectors=0;
 if(argc==3) { std::ifstream in(argv[2]); if(!in) return 2; std::string op=argv[1]; uint64_t a,b,z,flags;
 while(in>>std::hex>>a) { if(op=="f32_to_f64") { in>>z>>flags; equal("vector-convert",a,0,sf64_from_f32_bits(uint32_t(a)),canonical(z)); }
 else { in>>b>>z>>flags; equal("vector",a,b,op=="f64_add"?sf64_add_bits(a,b):sf64_mul_bits(a,b),canonical(z)); } if(!in) return 3; ++vectors; }
 }
 std::printf("PASS checks=%llu random_pairs=1000000 random_conversions=200000 vectors=%llu\n",(unsigned long long)checks,(unsigned long long)vectors);
}
