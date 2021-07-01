// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "Util.h"
#include "Convolution.h"

#include <array>
#include <experimental/array>
#include <cstdlib>
#include <cstdint>

namespace mobilinkd
{

/// Puncture matrix for linx setup frame.
inline constexpr auto P1 = std::experimental::make_array<int8_t>(
    1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1);

/// Puncture matrix for audio frames.
inline constexpr auto P2 = std::experimental::make_array<int8_t>(
    1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 0);

/// Puncture matrix for packet frames (7/8).
inline constexpr auto P3 = std::experimental::make_array<int8_t>(
    1, 1, 1, 1, 1, 1, 1, 0);

/**
 * Convert an integer value to an array of bits, with the
 * high-bit at index 0.
 * 
 * At anything beyond -O0, the array is constructed at compile time.
 */
template <size_t N>
constexpr std::array<uint8_t, N> toBitArray(int8_t value)
{
    std::array<uint8_t, N> result{};
    for (size_t i = 0; i != N; ++i)
    {
        result[N - (i + 1)] = (value & 1);
        value >>= 1;
    }
    return result;
}

template <size_t N>
struct NextStateTable
{
    using nextStateTable_t = std::array<std::array<int8_t, N>, N>;
    
    nextStateTable_t nextStateTable = makeNextStateTable();

    static constexpr nextStateTable_t makeNextStateTable()
    {
        return nextStateTable_t();
    }
};

template <size_t N>
struct OutputTable
{

};

/**
 * Compute a cost table for a Trellis of size K, for input n of N,
 * and LLR size of LLR bits + 1. (i.e. LLR = 1 allows 2 bits to
 * represent -1, 0, +1).
 */
template <size_t K, size_t N, size_t LLR = 1>
struct CostTable
{
    static constexpr int8_t Price = 1 << LLR;
    static constexpr size_t InputValues = 1 << N;
    using cost_table_t = std::array<std::array<uint8_t, InputValues>, K>;

    template <typename Trellis_>
    static constexpr cost_table_t makeCostTable(const Trellis_& trellis)
    {
        cost_table_t result;
        for (size_t i = 0; i != K; ++i)
        {
            for (size_t j = 0; j != InputValues; ++j)
            {
                
            }
        }
    }
};


/**
 * Only valid for a k=1 (1:n) convolutional coder.
 */
template <size_t K_, size_t n_>
struct Trellis
{
    static constexpr size_t K = K_; // Memory depth of convolution.
    static constexpr size_t k = 1;  // Number of bits per input symbol.
    static constexpr size_t n = n_; // Number of coefficients / output bits.
    static constexpr size_t NumStates = (1 << K);   // Number of states in the convolutional coder.

    using polynomials_t = std::array<uint32_t, n_>;

    polynomials_t polynomials;

    Trellis(polynomials_t polys)
    : polynomials(polys)
    {}
};

template <size_t K, size_t n>
constexpr Trellis<K, n> makeTrellis(std::array<uint32_t, n> polys)
{
    return Trellis<K, n>(polys);
}

} // mobilinkd
