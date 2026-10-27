// Independent native volatile oracle; production helper remains integer-only.
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cfenv>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif
#include "../shaders/soft_binary64.slang"
uint64_t canonical(uint64_t a){return (a&0x7fffffffffffffffULL)>0x7ff0000000000000ULL?0xfff8000000000000ULL:a;}
double value(uint64_t a){double v;std::memcpy(&v,&a,8);return v;}
uint64_t bits(double v){uint64_t a;std::memcpy(&a,&v,8);return a;}
void equal(const char* op,uint64_t a,uint64_t b,uint64_t got,uint64_t want){if(got!=want){std::fprintf(stderr,"%s a=%016llx b=%016llx got=%016llx want=%016llx\n",op,(unsigned long long)a,(unsigned long long)b,(unsigned long long)got,(unsigned long long)want);std::exit(1);}}
uint64_t native_div(uint64_t a,uint64_t b){volatile double x=value(a),y=value(b);volatile double z=x/y;return canonical(bits(z));}
uint64_t native_sqrt(uint64_t a){volatile double x=value(a);volatile double z=std::sqrt(x);return canonical(bits(z));}
void pair(uint64_t a,uint64_t b){equal("div",a,b,sf64_div_bits(a,b),native_div(a,b));equal("sqrt",a,0,sf64_sqrt_bits(a),native_sqrt(a));}
uint64_t next(uint64_t& state){state^=state<<13;state^=state>>7;state^=state<<17;return state;}
int main(int argc,char** argv){
 fenv_t saved;std::feholdexcept(&saved);std::fesetround(FE_TONEAREST);
#if defined(__SSE__)
 unsigned csr=_mm_getcsr();_mm_setcsr((_mm_getcsr()|0x1f80u)&~((1u<<15)|(1u<<6)));
#endif
 if(argc==3){std::string op=argv[1];std::ifstream in(argv[2]);if(!in||(op!="f64_div"&&op!="f64_sqrt"))return 2;std::string line;uint64_t count=0;
  while(std::getline(in,line)){std::istringstream row(line);uint64_t a,b=0,want,flags;if(!(row>>std::hex>>a))return 2;if(op=="f64_div"&&!(row>>b))return 2;if(!(row>>want>>flags))return 2;std::string extra;if(row>>extra)return 2;
   equal("native-TestFloat",a,b,op=="f64_div"?native_div(a,b):native_sqrt(a),canonical(want));equal(op.c_str(),a,b,op=="f64_div"?sf64_div_bits(a,b):sf64_sqrt_bits(a),canonical(want));++count;
  }std::printf("%s TestFloat vectors=%llu\n",op.c_str(),(unsigned long long)count);
 }else{
  const uint64_t edge[]={0,1,2,3,0x000ffffffffffffeULL,0x000fffffffffffffULL,0x0010000000000000ULL,0x0010000000000001ULL,0x3fe0000000000000ULL,0x3fefffffffffffffULL,0x3ff0000000000000ULL,0x3ff0000000000001ULL,0x4000000000000000ULL,0x400fffffffffffffULL,0x4010000000000000ULL,0x7fefffffffffffffULL,0x7ff0000000000000ULL,0x7ff0000000000001ULL,0x7ff8000000000000ULL};
  for(auto a:edge)for(auto b:edge)for(unsigned sign=0;sign<4;sign++)pair(a|(uint64_t(sign&1)<<63),b|(uint64_t(sign>>1)<<63));
  for(uint64_t exp=0;exp<2047;exp++)for(uint64_t tail:{0ULL,1ULL,2ULL,0xffffffffffffeULL,0xfffffffffffffULL}){uint64_t a=(exp<<52)|tail;pair(a,0x3ff0000000000001ULL);pair(a,0x4008000000000000ULL);pair(a,0x3fe0000000000000ULL);}
  // Exact squares and immediately adjacent encodings, including odd exponents.
  for(unsigned i=1;i<4096;i++){double x=double(i)/32;uint64_t square=bits(x*x);pair(square-1,bits(x));pair(square,bits(x));pair(square+1,bits(x));}
  uint64_t state=0x9182fced79a63145ULL;for(unsigned i=0;i<1000000;i++){auto a=next(state),b=next(state);pair(a,b);if(i%8==0)pair(a&0x7fffffffffffffffULL,b&0x000fffffffffffffULL);}
  std::puts("exact div/sqrt random_pairs=1000000 plus edge/exponent/square-neighbor cases PASS");
 }
#if defined(__SSE__)
 _mm_setcsr(csr);
#endif
 std::fesetenv(&saved);return 0;
}
