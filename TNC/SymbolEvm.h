// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "IirFilter.hpp"

#include <array>
#include <algorithm>
#include <numeric>

namespace mobilinkd
{

template <typename FloatType, size_t N>
struct SymbolEvm
{
    using filter_type = tnc::IirFilter<N>;
    using symbol_t = int;
    using result_type = std::tuple<symbol_t, FloatType>;

    filter_type filter_;
    FloatType evm_ = 0.0;

    SymbolEvm(const std::array<FloatType, N>& b, const std::array<FloatType, N>& a)
    : filter_(b, a)
    {}
    
    FloatType evm() const { return evm_; }
    
    /**
     * Decode a normalized sample into a symbol.  Symbols
     * are decoded into +3, +1, -1, -3.  If an erasure limit
     * is set, symbols outside this limit are 'erased' and
     * returned as 0.
     */
    result_type operator()(FloatType sample)
    {
        symbol_t symbol;
        FloatType evm;

        sample = std::min(3.0f, std::max(-3.0f, sample));

        if (sample > 2)
        {
            symbol = 3;
            evm = (sample - 3) * 0.333333f;
        }
        else if (sample > 0)
        {
            symbol = 1;
            evm = sample - 1;
        }
        else if (sample >= -2)
        {
            symbol = -1;
            evm = sample + 1;
        }
        else
        {
            symbol = -3;
            evm = (sample + 3) * 0.333333f;
        }
        
        evm_ = filter_(evm);
        
        return std::make_tuple(symbol, evm);
    }
};

} // mobilinkd
