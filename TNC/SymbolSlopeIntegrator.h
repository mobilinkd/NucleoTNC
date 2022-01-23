// Copyright 2021 Mobilinkd LLC.

#pragma once

#include "Log.h"

#include <arm_math.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <numeric>

namespace mobilinkd { namespace m17 {

/**
 * Calculate the phase estimates for each sample position.
 *
 * This performs a running calculation of the phase of each bit position.
 * It is very noisy for individual samples, but quite accurate when
 * averaged over an entire M17 frame.
 *
 * It is designed to be used to calculate the best bit position for each
 * frame of data.  Samples are collected and averaged.  When update() is
 * called, the best sample index and clock are estimated, and the counters
 * reset for the next frame.
 *
 * It starts counting bit 0 as the first bit received after a reset.
 *
 * This is very efficient as it only uses addition and subtraction for
 * each bit sample.  And uses one multiply and divide per update (per
 * frame).
 *
 * This will permit a clock error of up to 500ppm.  This allows up to
 * 250ppm error for both transmitter and receiver clocks.  This is
 * less than one sample per frame when the sample rate is 48000 SPS.
 *
 * @inv current_index_ is in the interval [0, SAMPLES_PER_SYMBOL).
 * @inv sample_index_ is in the interval [0, SAMPLES_PER_SYMBOL).
 * @inv clock_ is in the interval [0.9995, 1.0005]
 */

template <typename FloatType = float, size_t SamplesPerSymbol = 10>
struct SymbolSlopeIntegrator
{
    static constexpr std::array<FloatType, 5> IMPULSE = {
        -1.0, -0.70710678, 0.0, 0.70710678, 1.0
    };

    static constexpr size_t BEGIN = SamplesPerSymbol / 2 + IMPULSE.size() / 2;
    static constexpr size_t END = BEGIN + SamplesPerSymbol;

    std::array<FloatType, SamplesPerSymbol> integrator_;
    std::array<FloatType, SamplesPerSymbol * 2> signal;
    std::array<FloatType, SamplesPerSymbol * 2 + IMPULSE.size() - 1> output;

    FloatType prev_ = 0.0;
    size_t index_ = 0;

    void reset()
    {
        integrator_.fill(0.0);
        prev_ = 0.0;
        index_ = 0;
    }

    void operator()(FloatType value)
    {
        auto dy = value - prev_;

        // Invert the phase estimate when sample midpoint is less than 0.
        integrator_[index_] += value < 0 ? -dy : dy;
        index_ = (index_ == (SamplesPerSymbol - 1)) ? 0 : index_ + 1;
        prev_ = value;
    }

    [[gnu::noinline]]
    int8_t update()
    {
        // Circular convolution, so we concatenate the signal back-to-back.
        // This is effectively "wrap" convolution, just offset by
        // (SamplesPerSignal + IMPULSE.size()) / 2.
        auto it = std::copy(integrator_.begin(), integrator_.end(), signal.begin());
        std::copy(integrator_.begin(), integrator_.end(), it);

        arm_conv_f32(signal.data(), signal.size(),
            (float*)IMPULSE.data(), IMPULSE.size(),
            output.data());

        auto argmax = std::max_element(&output[BEGIN], &output[END]);
        int8_t index = std::distance(&output[BEGIN], argmax);

        // Normalize index from wrapped position.
        index -= SamplesPerSymbol / 2;
        if (index < 0) index += SamplesPerSymbol;

#if 1
        INFO("output: %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d",
            int(output[7]*1000), int(output[8]*1000), int(output[9]*1000), int(output[10]*1000), int(output[11]*1000),
            int(output[12]*1000), int(output[13]*1000), int(output[14]*1000), int(output[15]*1000), int(output[16]*1000));

        INFO("Index = %d", index);
#endif

        // Reset integrator for next frame.
        integrator_.fill(0.0);

        return index;
    }

    /**
     * Return the current sample index.  This will always be in the range of
     * [0..SAMPLES_PER_SYMBOL).
     */
    uint8_t current_index() const
    {
        return index_;
    }
};

}} // mobilinkd::m17
