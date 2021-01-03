// Copyright 2020 Mobilinkd LLC.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <cassert>
#include <array>
#include <bitset>
#include <tuple>


namespace mobilinkd
{

// The make_bitset stuff only works as expected in GCC10 and later.

namespace detail {

template<std::size_t...Is, class Tuple>
constexpr std::bitset<sizeof...(Is)> make_bitset(std::index_sequence<Is...>, Tuple&& tuple)
{
    constexpr auto size = sizeof...(Is);
    std::bitset<size> result;
    using expand = int[];
    for (size_t i = 0; i != size; ++i)
    {
        void(expand {0, result[Is] = std::get<Is>(tuple)...});
    }
    return result;
}

template<typename FloatType, size_t LLR>
constexpr std::array<std::tuple<FloatType, std::tuple<int8_t, int8_t>>, (((1 << (LLR - 1)) - 1) * 6 ) + 1> make_llr_map()
{
    constexpr size_t size = (((1 << (LLR - 1)) - 1) * 6 ) + 1;
    std::array<std::tuple<FloatType, std::tuple<int8_t, int8_t>>, size> result;

    constexpr int8_t limit = (1 << (LLR - 1)) - 1;
    constexpr FloatType inc = 1.0 / FloatType(limit);
    int8_t i = limit;
    int8_t j = limit;

    // Output must be ordered by k, ascending.
    FloatType k = -3.0;
    for (size_t index = 0; index != size; ++index)
    {
        auto& a = result[index];
        std::get<0>(a) = k;
        std::get<0>(std::get<1>(a)) = i;
        std::get<1>(std::get<1>(a)) = j;

        if (k + 1.0 < inc / -2.0)
        {
            j--;
            if (j < -limit) j = -limit;
        }
        else if (k - 1.0 < inc / -2.0)
        {
            i--;
            if (i < -limit) i = -limit;
        }
        else
        {
            j++;
            if (j > limit) j = limit;
        }
        k += inc;
    }
    return result;
}

}

template<class...Bools>
constexpr auto make_bitset(Bools&&...bools)
{
    return detail::make_bitset(std::make_index_sequence<sizeof...(Bools)>(),
        std::make_tuple(bool(bools)...));
}

inline int from_4fsk(int symbol)
{
    // Convert a 4-FSK symbol to a pair of bits.
    switch (symbol)
    {
        case 1: return 0;
        case 3: return 1;
        case -1: return 2;
        case -3: return 3;
        default: abort();
    }
}

template <typename FloatType, size_t LLR>
auto llr(FloatType sample)
{
    static constexpr auto symbol_map = detail::make_llr_map<FloatType, LLR>();

    FloatType s = std::min(3.0, std::max(-3.0, sample));

    auto it = std::lower_bound(symbol_map.begin(), symbol_map.end(), s,
        [](std::tuple<FloatType, std::tuple<int8_t, int8_t>> const& e, FloatType s){
            return std::get<0>(e) < s;
        });
    
    if (it == symbol_map.end()) return std::get<1>(*symbol_map.rbegin());

    return std::get<1>(*it);
}

template <size_t M, typename T, size_t N, typename U, size_t IN>
auto depunctured(std::array<T, N> puncture_matrix, std::array<U, IN> in)
{
    static_assert(M % N == 0);
    std::array<U, M> result;
    size_t index = 0;
    size_t pindex = 0;
    for (size_t i = 0; i != M; ++i)
    {
        if (!puncture_matrix[pindex++])
        {
            result[i] = 0;
        }
        else
        {
            result[i] = in[index++];
        }
        if (pindex == N) pindex = 0;
    }
    return result;
}

template <size_t IN, size_t OUT, size_t P>
size_t depuncture(const std::array<int8_t, IN>& in,
    std::array<int8_t, OUT>& out, const std::array<int8_t, P>& p)
{
    size_t index = 0;
    size_t pindex = 0;
    size_t bit_count = 0;
    for (size_t i = 0; i != OUT && index < IN; ++i)
    {
        if (!p[pindex++])
        {
            out[i] = 0;
            bit_count++;
        }
        else
        {
            out[i] = in[index++];
        }
        if (pindex == P) pindex = 0;
    }
    return bit_count;
}


template <size_t IN, size_t OUT, size_t P>
size_t puncture(const std::array<uint8_t, IN>& in,
    std::array<int8_t, OUT>& out, const std::array<int8_t, P>& p)
{
    size_t index = 0;
    size_t pindex = 0;
    size_t bit_count = 0;
    for (size_t i = 0; i != IN && index != OUT; ++i)
    {
        if (p[pindex++])
        {
            out[index++] = in[i];
            bit_count++;
        }

        if (pindex == P) pindex = 0;
    }
    return bit_count;
}

template <size_t N>
constexpr bool get_bit_index(const std::array<uint8_t, N>& input, size_t index)
{
    auto byte_index = index >> 3;
    auto bit_index = 7 - (index & 7);

    return (input[byte_index] & (1 << bit_index)) >> bit_index;
}

template <size_t N>
void set_bit_index(std::array<uint8_t, N>& input, size_t index)
{
    auto byte_index = index >> 3;
    auto bit_index = 7 - (index & 7);
    input[byte_index] |= (1 << bit_index);
}

template <size_t N>
void reset_bit_index(std::array<uint8_t, N>& input, size_t index)
{
    auto byte_index = index >> 3;
    auto bit_index = 7 - (index & 7);
    input[byte_index] &= ~(1 << bit_index);
}

template <size_t N>
void assign_bit_index(std::array<uint8_t, N>& input, size_t index, bool value)
{
    if (value) set_bit_index(input, index);
    else reset_bit_index(input, index);
}


template <size_t IN, size_t OUT, size_t P>
size_t puncture_bytes(const std::array<uint8_t, IN>& in,
    std::array<uint8_t, OUT>& out, const std::array<int8_t, P>& p)
{
    size_t index = 0;
    size_t pindex = 0;
    size_t bit_count = 0;
    for (size_t i = 0; i != IN * 8 && index != OUT * 8; ++i)
    {
        if (p[pindex++])
        {
            assign_bit_index(out, index++, get_bit_index(in, i));
            bit_count++;
        }

        if (pindex == P) pindex = 0;
    }
    return bit_count;
}

/**
 * Sign-extend an n-bit value to a specific signed integer type.
 */
template <typename T, size_t n>
constexpr T to_int(uint8_t v)
{
    constexpr auto MAX_INPUT = (1 << (n - 1));
    constexpr auto NEGATIVE_OFFSET = std::numeric_limits<typename std::make_unsigned<T>::type>::max() - (MAX_INPUT - 1);
    T r = v & (1 << (n - 1)) ? NEGATIVE_OFFSET : 0;
    return r + (v & (MAX_INPUT - 1));
}

template <typename T, size_t N>
constexpr auto to_byte_array(std::array<T, N> in)
{
    std::array<uint8_t, (N + 7) / 8> out{};
    out.fill(0);
    size_t i = 0;
    size_t b = 0;
    for (auto c : in)
    {
        out[i] |= (c << (7 - b));
        if (++b == 8)
        {
            ++i;
            b = 0;
        }
    }
    return out;
}

} // mobilinkd
