// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "IirFilter.hpp"

#include <array>
#include <algorithm>
#include <numeric>

namespace mobilinkd
{

template <typename FloatType, size_t N = 32>
struct FrequencyError
{
    using float_type = FloatType;
    using array_t = std::array<FloatType, N>;
    using filter_type = tnc::IirFilter<3>;

    array_t samples_{0};
    size_t index_ = 0;
    float_type accum_ = 0.0;
    filter_type filter_;

    const float_type ZERO = 0.0;

    FrequencyError(const std::array<float, 3>& b, const std::array<float, 3>& a)
    : filter_(b, a)
    {
        samples_.fill(0.0);
    }
    
    auto operator()(float_type sample)
    {
        FloatType evm = 0;
        bool use = true;

        if (sample > 2)
        {
            evm = sample - 3;
        }
        else if (sample >= -2)
        {
            use = false;
        }
        else
        {
            evm = sample + 3;
        }

        if (use)
        {
            accum_ = accum_ - samples_[index_] + evm;
            samples_[index_++] = evm;
            if (index_ == N) index_ = 0;
        }

        return filter_(accum_ / N);
    }
};

} // mobilinkd
