// Copyright 2021 Mobilinkd LLC.

#pragma once

#include "KalmanFilter.h"
#include "Log.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <numeric>

namespace mobilinkd { namespace m17 {


template <typename FloatType = float, size_t SamplesPerSymbol = 10>
struct ClockRecovery
{
    static constexpr FloatType MAX_CLOCK_OFFSET = 0.0005; // 500ppm

    KalmanFilter<FloatType, SamplesPerSymbol> kf_;
    size_t count_ = 0;
    int8_t sample_index_ = 0;
    FloatType clock_estimate_ = 0.;
    FloatType sample_estimate_ = 0.;

    void reset(FloatType z)
    {
        INFO("CR Reset");
        kf_.reset(z);
        count_ = 0;
        sample_index_ = z;
        clock_estimate_ = 0.;
    }

    void operator()(FloatType)
    {
        ++count_;
    }

    static FloatType clock_limiter(FloatType offset)
    {
        return std::min(MAX_CLOCK_OFFSET, std::max(-MAX_CLOCK_OFFSET, offset));
    }

    bool update(uint8_t sw)
    {
        INFO("CR Update %d", int(sw));
        if (count_ < 480)
        {
            sample_index_ = sw;
            return false;
        }

        auto f = kf_.update(sw, count_);

        // Constrain sample index to [0..SamplesPerSymbol), wrapping if needed.
        sample_estimate_ = f[0];
        sample_index_ = int8_t(round(sample_estimate_));
        sample_index_ = sample_index_ < 0 ? sample_index_ + SamplesPerSymbol : sample_index_;
        sample_index_ = sample_index_ >= int8_t(SamplesPerSymbol) ? sample_index_ - SamplesPerSymbol : sample_index_;
        clock_estimate_ = f[1];
        count_ = 0;

        return true;
    }

    /**
     * This is used when no sync word is found. The sample index is updated
     * based on the current clock estimate, the last known good sample
     * estimate, and the number of samples processed.
     *
     * The sample and clock estimates from the filter remain unchanged.
     */
    bool update()
    {
        INFO("CR Update");
        auto csw = std::fmod((sample_estimate_ + (1.0 + clock_estimate_) * count_), SamplesPerSymbol);
        if (csw < 0.) csw += SamplesPerSymbol;
        else if (csw >= SamplesPerSymbol) csw -= SamplesPerSymbol;

        // Constrain sample index to [0..SamplesPerSymbol), wrapping if needed.
        sample_index_ = int8_t(round(csw));
        sample_index_ = sample_index_ < 0 ? sample_index_ + SamplesPerSymbol : sample_index_;
        sample_index_ = sample_index_ >= int8_t(SamplesPerSymbol) ? sample_index_ - SamplesPerSymbol : sample_index_;

        return true;
    }

    /**
     * Return the estimated sample clock increment based on the last update.
     *
     * The value is only valid after samples have been collected and update()
     * has been called.
     */
    FloatType clock_estimate() const
    {
        return clock_estimate_;
    }

    /**
     * Return the estimated "best sample index" based on the last update.
     *
     * The value is only valid after samples have been collected and update()
     * has been called.
     */
    uint8_t sample_index() const
    {
        return sample_index_;
    }
};

}} // mobilinkd::m17
