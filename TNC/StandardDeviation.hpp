// Copyright 2020-2022 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "main.h"

#include <arm_math.h>

#include <cstdint>

namespace mobilinkd {

/**
 * Compute a running standard deviation.
 *
 * Based on https://dsp.stackexchange.com/a/1187/36581
 */
struct StandardDeviation
{
    float mean{0.0};
    float S{0.0};
    uint32_t samples{0};

    void reset()
    {
        mean = 0.0;
        S = 0.0;
        samples = 0;
    }

    void capture(float sample)
    {
        auto prev = mean;
        samples += 1;
        mean = mean + (sample - mean) / samples;
        S = S + (sample - mean) * (sample - prev);
    }

    float variance() const
    {
        return samples == 0 ? -1.0 :  S / samples;
    }

    float stdev() const
    {
        float result = -1.0;
        arm_sqrt_f32(variance(), &result);
        return result;
    }

    // SNR in dB
    float SNR() const
    {
        return 10.0 * log10(mean / stdev());
    }
};

template <typename FloatType, size_t N>
struct RunningStandardDeviation
{
    FloatType S{1.0};
    FloatType alpha{1.0 / N};

    void reset()
    {
        S = 0.0;
    }

    void capture(float sample)
    {
        S -= S * alpha;
        S += (sample * sample) * alpha;
    }

    FloatType variance() const
    {
        return S;
    }

    FloatType stdev() const
    {
        return std::sqrt(variance());
    }
};

} // mobilinkd
