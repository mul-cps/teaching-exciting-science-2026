/*

License for Berkeley SoftFloat Release 3e

John R. Hauser
2018 January 20

The following applies to the whole of SoftFloat Release 3e as well as to
each source file individually.

Copyright 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018 The Regents of the
University of California.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions, and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions, and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the University nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS", AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE
DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
// Derived from Berkeley SoftFloat Release 3e by John R. Hauser.
// Full BSD-3-Clause notice: ../third_party/softfloat/NOTICE.
// Source hashes, line ranges and changes: ../third_party/softfloat/provenance.json.
// Fixed nearest-even with gradual underflow; all NaNs canonicalize to
// 0xFFF8000000000000. No exception flags or configurable rounding state.
// Same integer implementation compiled by C++ host tests and Slang shaders.
// Signed int quantities are exponents/distances only (within -4096..4096).
// Internal normalizers take nonzero significands: shift ranges are 0..52 for
// binary64, 1..23 for binary32; norm_round sees bit 63 clear, so dist is 0..62.
// No output references: value-returning helpers have no aliasing side effects.
#ifndef SO101_SOFT_BINARY64_INCLUDED
#define SO101_SOFT_BINARY64_INCLUDED
uint64_t sf64_pack(bool sign, int exp, uint64_t sig) {
    // Addition is essential: the hidden significand bit carries into exp.
    return (uint64_t(sign) << 63) + (uint64_t(exp) << 52) + sig;
}
uint64_t sf64_fraction(uint64_t a) { return a & 0x000FFFFFFFFFFFFFULL; }
int sf64_exponent(uint64_t a) { return int((a >> 52) & 0x7FF); }
bool sf64_nan(uint64_t a) { return (a & 0x7FFFFFFFFFFFFFFFULL) > 0x7FF0000000000000ULL; }
bool sf64_is_finite(uint64_t a) { return sf64_exponent(a) != 0x7FF; }
// Total for all unsigned distances, including zero and distances >= width.
uint64_t sf64_jam(uint64_t a, uint32_t dist) {
    if (dist == 0) return a;
    if (dist < 63) return (a >> dist) | uint64_t((a << (64 - dist)) != 0);
    return uint64_t(a != 0);
}
int sf64_clz64(uint64_t a) {
    if (a == 0) return 64;
    int n = 0;
    // Bounded binary search replaces upstream's immutable byte lookup table.
    if ((a >> 32) == 0) { n += 32; a <<= 32; }
    if ((a >> 48) == 0) { n += 16; a <<= 16; }
    if ((a >> 56) == 0) { n += 8; a <<= 8; }
    if ((a >> 60) == 0) { n += 4; a <<= 4; }
    if ((a >> 62) == 0) { n += 2; a <<= 2; }
    if ((a >> 63) == 0) n += 1;
    return n;
}
struct Sf64Product { uint64_t hi; uint64_t lo; };
Sf64Product sf64_product(uint64_t a, uint64_t b) {
    uint64_t aLo = uint64_t(uint32_t(a)), aHi = a >> 32;
    uint64_t bLo = uint64_t(uint32_t(b)), bHi = b >> 32;
    uint64_t low = aLo * bLo;
    uint64_t mid1 = aHi * bLo, mid = mid1 + aLo * bHi;
    uint64_t high = aHi * bHi + (uint64_t(mid < mid1) << 32) + (mid >> 32);
    mid <<= 32;
    low += mid;
    high += uint64_t(low < mid);
    Sf64Product result; result.hi = high; result.lo = low; return result;
}
uint64_t sf64_round(bool sign, int exp, uint64_t sig) {
    uint32_t roundBits = uint32_t(sig & 0x3FF);
    if (exp < 0) {
        sig = sf64_jam(sig, uint32_t(-exp)); exp = 0;
        roundBits = uint32_t(sig & 0x3FF);
    } else if (exp > 0x7FD || (exp == 0x7FD && sig + 0x200ULL >= 0x8000000000000000ULL)) {
        return sf64_pack(sign, 0x7FF, 0);
    }
    sig = (sig + 0x200ULL) >> 10;
    if (roundBits == 0x200) sig &= ~uint64_t(1);
    if (sig == 0) exp = 0;
    return sf64_pack(sign, exp, sig);
}
uint64_t sf64_norm_round(bool sign, int exp, uint64_t sig) {
    if (sig == 0) return sf64_pack(sign, 0, 0);
    int dist = sf64_clz64(sig) - 1;
    exp -= dist;
    if (dist >= 10 && exp >= 0 && exp < 0x7FD) return sf64_pack(sign, exp, sig << (dist - 10));
    return sf64_round(sign, exp, sig << dist);
}
uint64_t sf64_add_magnitudes(uint64_t a, uint64_t b, bool sign) {
    int ea = sf64_exponent(a), eb = sf64_exponent(b), diff = ea - eb, ez;
    uint64_t sa = sf64_fraction(a), sb = sf64_fraction(b), sz;
    if (diff == 0) {
        if (ea == 0) return a + sb;
        if (ea == 0x7FF) return a;
        ez = ea; sz = (0x0020000000000000ULL + sa + sb) << 9;
    } else {
        sa <<= 9; sb <<= 9;
        if (diff < 0) {
            if (eb == 0x7FF) return sf64_pack(sign, 0x7FF, 0);
            ez = eb;
            if (ea != 0) sa += 0x2000000000000000ULL; else sa <<= 1;
            sa = sf64_jam(sa, uint32_t(-diff));
        } else {
            if (ea == 0x7FF) return a;
            ez = ea;
            if (eb != 0) sb += 0x2000000000000000ULL; else sb <<= 1;
            sb = sf64_jam(sb, uint32_t(diff));
        }
        sz = 0x2000000000000000ULL + sa + sb;
        if (sz < 0x4000000000000000ULL) { --ez; sz <<= 1; }
    }
    return sf64_round(sign, ez, sz);
}
uint64_t sf64_sub_magnitudes(uint64_t a, uint64_t b, bool sign) {
    int ea = sf64_exponent(a), eb = sf64_exponent(b), diff = ea - eb;
    uint64_t sa = sf64_fraction(a), sb = sf64_fraction(b), sz;
    if (diff == 0) {
        if (ea == 0x7FF) return 0xFFF8000000000000ULL;
        if (sa == sb) return 0;
        // Magnitudes <= 52 bits: unsigned comparison avoids signed conversion.
        if (sa < sb) { sign = !sign; sz = sb - sa; } else sz = sa - sb;
        if (ea != 0) --ea;
        int dist = sf64_clz64(sz) - 11, ez = ea - dist;
        if (ez < 0) { dist = ea; ez = 0; }
        return sf64_pack(sign, ez, sz << dist);
    }
    sa <<= 10; sb <<= 10;
    int ez;
    if (diff < 0) {
        sign = !sign;
        if (eb == 0x7FF) return sf64_pack(sign, 0x7FF, 0);
        sa += ea != 0 ? 0x4000000000000000ULL : sa;
        sa = sf64_jam(sa, uint32_t(-diff));
        sb |= 0x4000000000000000ULL; ez = eb; sz = sb - sa;
    } else {
        if (ea == 0x7FF) return a;
        sb += eb != 0 ? 0x4000000000000000ULL : sb;
        sb = sf64_jam(sb, uint32_t(diff));
        sa |= 0x4000000000000000ULL; ez = ea; sz = sa - sb;
    }
    return sf64_norm_round(sign, ez - 1, sz);
}
uint64_t sf64_add_bits(uint64_t a, uint64_t b) {
    if (sf64_nan(a) || sf64_nan(b)) return 0xFFF8000000000000ULL;
    bool sign = (a >> 63) != 0;
    if (((a ^ b) >> 63) != 0) return sf64_sub_magnitudes(a, b, sign);
    return sf64_add_magnitudes(a, b, sign);
}
uint64_t sf64_mul_bits(uint64_t a, uint64_t b) {
    if (sf64_nan(a) || sf64_nan(b)) return 0xFFF8000000000000ULL;
    bool sign = ((a ^ b) >> 63) != 0;
    int ea = sf64_exponent(a), eb = sf64_exponent(b);
    uint64_t sa = sf64_fraction(a), sb = sf64_fraction(b);
    if (ea == 0x7FF || eb == 0x7FF) {
        if ((a << 1) == 0 || (b << 1) == 0) return 0xFFF8000000000000ULL;
        return sf64_pack(sign, 0x7FF, 0);
    }
    // Multiplication by finite +/-1 is exact, including subnormals and signed zeros.
    // NaN canonicalization and infinity handling above retain precedence.
    if ((a & 0x7FFFFFFFFFFFFFFFULL) == 0x3FF0000000000000ULL) return b ^ (a & 0x8000000000000000ULL);
    if ((b & 0x7FFFFFFFFFFFFFFFULL) == 0x3FF0000000000000ULL) return a ^ (b & 0x8000000000000000ULL);
    if (ea == 0) {
        if (sa == 0) return sf64_pack(sign, 0, 0);
        int dist = sf64_clz64(sa) - 11; ea = 1 - dist; sa <<= dist;
    }
    if (eb == 0) {
        if (sb == 0) return sf64_pack(sign, 0, 0);
        int dist = sf64_clz64(sb) - 11; eb = 1 - dist; sb <<= dist;
    }
    int ez = ea + eb - 0x3FF;
    sa = (sa | 0x0010000000000000ULL) << 10;
    sb = (sb | 0x0010000000000000ULL) << 11;
    Sf64Product product = sf64_product(sa, sb);
    uint64_t sz = product.hi | uint64_t(product.lo != 0);
    if (sz < 0x4000000000000000ULL) { --ez; sz <<= 1; }
    return sf64_round(sign, ez, sz);
}
uint64_t sf64_from_f32_bits(uint32_t a) {
    bool sign = (a >> 31) != 0;
    int exp = int((a >> 23) & 0xFF);
    uint32_t frac = a & 0x007FFFFF;
    if (exp == 0xFF) {
        if (frac != 0) return 0xFFF8000000000000ULL;
        return sf64_pack(sign, 0x7FF, 0);
    }
    if (exp == 0) {
        if (frac == 0) return sf64_pack(sign, 0, 0);
        int dist = sf64_clz64(uint64_t(frac)) - 40;
        exp = -dist; frac <<= dist;
    }
    return sf64_pack(sign, exp + 0x380, uint64_t(frac) << 29);
}
bool sf64_less(uint64_t a, uint64_t b) {
    if (sf64_nan(a) || sf64_nan(b)) return false;
    bool signA = (a >> 63) != 0, signB = (b >> 63) != 0;
    if (signA != signB) return signA && ((a | b) << 1) != 0;
    return a != b && (signA != (a < b));
}
bool sf64_less_equal(uint64_t a, uint64_t b) {
    if (sf64_nan(a) || sf64_nan(b)) return false;
    bool signA = (a >> 63) != 0, signB = (b >> 63) != 0;
    if (signA != signB) return signA || ((a | b) << 1) == 0;
    return a == b || (signA != (a < b));
}
#endif
