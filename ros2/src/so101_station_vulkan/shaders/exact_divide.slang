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
