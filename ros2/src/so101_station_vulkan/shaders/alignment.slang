// RealSense zero-distortion scalar/SSE conventions. Offline precise FP32 only.
// BEGIN EXACT ALIGNMENT DIVISION
// Portable binary32 RNE division. Native FDiv supplies only a bounded seed;
// every accepted quotient and remainder is verified with exact uint arithmetic.
struct AlignmentWide { uint lo; uint hi; };
struct AlignmentQuotient { uint whole; uint remainder; };
AlignmentWide alignment_wide_multiply(uint a,uint b) {
    uint a0=a&65535u,a1=a>>16,b0=b&65535u,b1=b>>16;
    uint w0=a0*b0,t=a1*b0+(w0>>16);
    uint w1=(t&65535u)+a0*b1;
    AlignmentWide result;
    result.lo=(w1<<16)|(w0&65535u);
    result.hi=a1*b1+(t>>16)+(w1>>16);
    return result;
}
bool alignment_wide_greater(AlignmentWide a,AlignmentWide b) {
    return a.hi>b.hi || (a.hi==b.hi && a.lo>b.lo);
}
AlignmentWide alignment_wide_subtract(AlignmentWide a,AlignmentWide b) {
    AlignmentWide result;
    result.lo=a.lo-b.lo;result.hi=a.hi-b.hi-uint(a.lo<b.lo);
    return result;
}
AlignmentQuotient alignment_exact_quotient(uint normalized_a,uint normalized_b,uint seed) {
    // Preconditions: 2^23 <= B < 2^24 and B <= A < 2*B.
    // N=A*2^23 and q*B fit in 48 bits, represented as two uint limbs.
    AlignmentWide numerator;
    numerator.lo=normalized_a<<23;numerator.hi=normalized_a>>9;
    AlignmentWide divisor;divisor.lo=normalized_b;divisor.hi=0;
    AlignmentQuotient result;
    if(seed>=8388608u && seed<=16777216u) {
        uint q=seed;
        AlignmentWide product=alignment_wide_multiply(q,normalized_b);
        for(uint correction=0;correction<4;++correction) {
            if(alignment_wide_greater(product,numerator)) {
                --q;product=alignment_wide_subtract(product,divisor);
            } else {
                AlignmentWide remainder=alignment_wide_subtract(numerator,product);
                if(remainder.hi==0 && remainder.lo<normalized_b) {
                    result.whole=q;result.remainder=remainder.lo;return result;
                }
                ++q;
                uint previous=product.lo;product.lo+=normalized_b;
                product.hi+=uint(product.lo<previous);
            }
        }
        if(!alignment_wide_greater(product,numerator)) {
            AlignmentWide remainder=alignment_wide_subtract(numerator,product);
            if(remainder.hi==0 && remainder.lo<normalized_b) {
                result.whole=q;result.remainder=remainder.lo;return result;
            }
        }
    }
    // A bad native seed cannot affect correctness: exact restoring fallback.
    uint q=8388608u,remainder=normalized_a-normalized_b;
    for(int bit=22;bit>=0;--bit) {
        remainder<<=1;
        if(remainder>=normalized_b) {remainder-=normalized_b;q|=1u<<uint(bit);}
    }
    result.whole=q;result.remainder=remainder;return result;
}
uint alignment_divide_bits(uint a,uint b) {
    uint sign=(a^b)&0x80000000u;
    a&=0x7fffffffu;b&=0x7fffffffu;
    if(a>0x7f800000u || b>0x7f800000u ||
       (a==0x7f800000u && b==0x7f800000u) || (a==0 && b==0)) return 0x7fc00000u;
    if(a==0x7f800000u || b==0) return sign|0x7f800000u;
    if(a==0 || b==0x7f800000u) return sign;
    int ea=int(a>>23),eb=int(b>>23);
    a&=0x7fffffu;b&=0x7fffffu;
    if(ea!=0) a|=8388608u;
    else {
        ea=1;
        for(uint shift=0;shift<23 && a<8388608u;++shift) {a<<=1;--ea;}
    }
    if(eb!=0) b|=8388608u;
    else {
        eb=1;
        for(uint shift=0;shift<23 && b<8388608u;++shift) {b<<=1;--eb;}
    }
    int exponent=ea-eb+127;
    if(a<b) {a<<=1;--exponent;}
    // All seed operands/results are normal and exactly representable before
    // division; doubled A is even, so its float conversion is exact as well.
    uint seed=uint((float(a)/float(b))*8388608.0f);
    AlignmentQuotient quotient=alignment_exact_quotient(a,b,seed);
    uint q=quotient.whole,remainder=quotient.remainder;
    if(exponent<=0) {
        uint shift=uint(1-exponent);
        if(shift>24) return sign;
        uint base=q>>shift,lost=q&((1u<<shift)-1u),half=1u<<(shift-1u);
        if(lost>half || (lost==half && (remainder!=0 || (base&1u)!=0))) ++base;
        return sign|base;
    }
    uint twice=remainder<<1;
    if(twice>b || (twice==b && (q&1u)!=0)) ++q;
    if(q==16777216u) {q>>=1;++exponent;}
    if(exponent>=255) return sign|0x7f800000u;
    return sign|(uint(exponent)<<23)|(q&0x7fffffu);
}
float alignment_divide(float a,float b) {
    return asfloat(alignment_divide_bits(asuint(a),asuint(b)));
}
// END EXACT ALIGNMENT DIVISION
struct Parameters { float r0,r1,r2,r3,r4,r5,r6,r7,r8,t0,t1,t2,fx,fy,cx,cy,scale; uint corners,convention; };
[[vk::push_constant]] ConstantBuffer<Parameters> p;
[[vk::binding(0)]] ByteAddressBuffer raw;
[[vk::binding(1)]] StructuredBuffer<float4> rays;
[[vk::binding(2)]] RWStructuredBuffer<uint> scratch;
[[vk::binding(3)]] RWByteAddressBuffer packed;
[[vk::binding(4)]] RWStructuredBuffer<float4> xyz;
// Explicit ties-to-even; HLSL round() is not our portability contract.
int nearest_even(float v) { float lo=floor(v);float d=v-lo;int i=int(lo);return i+int(d>0.5 || (d==0.5 && (i&1)!=0)); }
int2 project(float2 ray,float z){
 precise float x=ray.x*z,y=ray.y*z;
 precise float tx,ty,tz;
 if(p.convention==1){tx=p.r0*x+(p.r3*y+(p.r6*z+p.t0));ty=p.r1*x+(p.r4*y+(p.r7*z+p.t1));tz=p.r2*x+(p.r5*y+(p.r8*z+p.t2));}
 else {tx=((p.r0*x+p.r3*y)+p.r6*z)+p.t0;ty=((p.r1*x+p.r4*y)+p.r7*z)+p.t1;tz=((p.r2*x+p.r5*y)+p.r8*z)+p.t2;}
 precise float u=((alignment_divide(tx,tz))*p.fx+p.cx)+0.5,v=((alignment_divide(ty,tz))*p.fy+p.cy)+0.5;
 if(!isfinite(u)||!isfinite(v)||abs(u)>2147483000.0||abs(v)>2147483000.0)return int2(-2147483647,-2147483647);
 return p.convention==1?int2(nearest_even(u),nearest_even(v)):int2(u,v);
}
[shader("compute")][numthreads(256,1,1)]
void splat(uint3 id:SV_DispatchThreadID){
 uint i=id.x;if(i>=307200)return;uint word=raw.Load((i/2)*4);uint d=(word>>((i&1)*16))&65535;if(d==0)return;
 precise float z=float(d)*p.scale;float4 ray=rays[i];int2 lo=project(ray.xy,z),hi=p.corners!=0?project(ray.zw,z):lo;
 if(p.convention==0&&(lo.x<0||lo.y<0||hi.x>=640||hi.y>=480))return;
 lo=max(lo,int2(0,0));hi=min(hi,int2(639,479));
 if(hi.x>=lo.x&&hi.y>=lo.y&&(hi.x-lo.x+1)*(hi.y-lo.y+1)>4096){InterlockedOr(scratch[307200],1);return;}
 for(int y=lo.y;y<=hi.y;y++)for(int x=lo.x;x<=hi.x;x++)InterlockedMin(scratch[y*640+x],d);
}
float4 point(uint i,uint d){precise float z=float(d)*p.scale;precise float x=z*(alignment_divide((float(i%640)-p.cx),p.fx)),y=z*(alignment_divide((float(i/640)-p.cy),p.fy));return float4(x,y,z,0);}
[shader("compute")][numthreads(256,1,1)]
void finalize(uint3 id:SV_DispatchThreadID){uint i=id.x*2;if(i>=307200)return;uint a=scratch[i],b=scratch[i+1];a=a==0xffffffff?0:a;b=b==0xffffffff?0:b;packed.Store(i*2,a|(b<<16));xyz[i]=point(i,a);xyz[i+1]=point(i+1,b);}
