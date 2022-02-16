#pragma once

#include <stdint.h>
#include <type_traits>
#include <cassert>
#include <stdexcept>
#include <limits>
#include <x86intrin.h>

// uintn<13> : uint type with 13 bits                --> uint16_t
// uintv<13> : uint type which can hold values 0..13 --> uint8_t
//
// sintn     : signed version of uintn
// sintv     : signed version of uintv

using max_uint_t = uint64_t;
using max_sint_t = int64_t;
    
namespace detail
{
    template<uint8_t NBITS> struct t_uint {using t = typename t_uint<NBITS-1>::t; };

    template<> struct t_uint< 0> { };
    template<> struct t_uint< 1> { using t = uint8_t; };
    template<> struct t_uint< 8> { using t = uint8_t; };

    template<> struct t_uint< 9> { using t = uint16_t; };
    template<> struct t_uint<16> { using t = uint16_t; };

    template<> struct t_uint<17> { using t = uint32_t; };
    template<> struct t_uint<32> { using t = uint32_t; };

    template<> struct t_uint<33> { using t = uint64_t; };
    template<> struct t_uint<64> { using t = uint64_t; };

    template<> struct t_uint<65> { };
    
    constexpr uint8_t Pow2BitsNeeded( max_uint_t max_val, bool signd ){
    return max_val < (              0x100LLU >> signd) ? 8 
         : max_val < (            0x10000LLU >> signd) ? 16
         : max_val < (        0x100000000LLU >> signd) ? 32
         :                                                 64;
    }    
}

// Int from number of bits to support
template<uint8_t NBITS>
using uintn = typename detail::t_uint<NBITS>::t;

template<uint8_t NBITS>
using sintn = typename std::make_signed<typename detail::t_uint<NBITS>::t>::type;

// Int from MAX VAL it can hold
template<max_uint_t MAX_VAL>
using uintv = typename detail::t_uint<detail::Pow2BitsNeeded(MAX_VAL,false)>::t;

template<max_uint_t MAX_VAL>
using sintv = typename std::make_signed<
              typename detail::t_uint<detail::Pow2BitsNeeded(MAX_VAL,true)>::t
              >::type;


static_assert( sizeof(uintn<1>)  == 1, "Size error.");
static_assert( sizeof(uintn<3>)  == 1, "Size error.");
static_assert( sizeof(uintn<5>)  == 1, "Size error.");
static_assert( sizeof(uintn<8>)  == 1, "Size error.");

static_assert( sizeof(uintn<9>)  == 2, "Size error.");
static_assert( sizeof(uintn<11>) == 2, "Size error.");
static_assert( sizeof(uintn<13>) == 2, "Size error.");
static_assert( sizeof(uintn<16>) == 2, "Size error.");

static_assert( sizeof(uintn<17>) == 4, "Size error.");
static_assert( sizeof(uintn<21>) == 4, "Size error.");
static_assert( sizeof(uintn<25>) == 4, "Size error.");
static_assert( sizeof(uintn<32>) == 4, "Size error.");

static_assert( sizeof(uintn<33>) == 8, "Size error.");
static_assert( sizeof(uintn<36>) == 8, "Size error.");
static_assert( sizeof(uintn<41>) == 8, "Size error.");
static_assert( sizeof(uintn<51>) == 8, "Size error.");
static_assert( sizeof(uintn<64>) == 8, "Size error.");

static_assert( sizeof(sintn<5>)  == 1, "Size error.");
static_assert( sizeof(sintn<13>) == 2, "Size error.");
static_assert( sizeof(sintn<25>) == 4, "Size error.");
static_assert( sizeof(sintn<41>) == 8, "Size error.");


static_assert( sizeof(uintv<  1>) == 1, "Size error.");
static_assert( sizeof(uintv< 33>) == 1, "Size error.");
static_assert( sizeof(uintv<255>) == 1, "Size error.");

static_assert( sizeof(uintv<  256>) == 2, "Size error.");
static_assert( sizeof(uintv<  499>) == 2, "Size error.");
static_assert( sizeof(uintv<65535>) == 2, "Size error.");

static_assert( sizeof(uintv<65536>)      == 4, "Size error.");
static_assert( sizeof(uintv<0xffffffff>) == 4, "Size error.");

static_assert( sizeof(uintv<0x100000000LLu>) == 8, "Size error.");
static_assert( sizeof(uintv<0xffffffffffffffffLLu>) == 8, "Size error.");

constexpr size_t log2(size_t n) 
{
    return n<2 ? 1 : 1+log2(n/2);
}

constexpr size_t ceil_pow2(size_t n)
{
    return 1 << log2(n);
}

constexpr uint64_t pow10(size_t n)
{
    return n<1 ? 1 : 10*pow10(n-1);
}

template<class T>
constexpr inline size_t max_val_for( T )
{ return std::numeric_limits<T>::max();}

template<class D, class S>
bool assign_val_checked( D* dst, S const src )
{
    const D newval = src;
    if(newval == src){
        *dst = newval;
        return true;
    } else {
        ASSERT_EQ( newval, src );
        return false;
    }
}

template<class TT>
inline TT safe_cast(size_t vv, bool runtime_check = true)
{
    TT const ret = static_cast<TT>(vv);
    assert(ret == vv);
    if(runtime_check && ret != vv)
        throw std::logic_error("safe_cast failed");
}
