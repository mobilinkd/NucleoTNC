// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "IirFilter.hpp"

#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace mobilinkd
{

template <typename FloatType>
struct CarrierDetect
{
    using result_t = std::tuple<bool, FloatType>;

    tnc::IirFilter<3> filter_;
    FloatType lock_;
    FloatType unlock_;
    bool locked_ = false;

    CarrierDetect(std::array<FloatType, 3> const& b, std::array<FloatType, 3> const& a, FloatType lock_level, FloatType unlock_level)
    : filter_(b, a), lock_(lock_level), unlock_(unlock_level)
    {
    }
    
    result_t operator()(FloatType value)
    {
        auto filtered = filter_(std::abs(value));
        if (locked_ && (filtered > unlock_)) locked_ = false;
        else if (!locked_ && (filtered < lock_)) locked_ = true;

        return std::make_tuple(locked_, filtered);
    }
};

} // mobilinkd
