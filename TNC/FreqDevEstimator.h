// Copyright 2021-2022 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "KalmanFilter.h"
#include "StandardDeviation.hpp"

#include <cmath>
#include <cstddef>

namespace mobilinkd { namespace m17 {
template <typename FloatType, size_t SYNC_WORD_LEN = 8>
class FreqDevEstimator
{
    static constexpr FloatType DEVIATION = 1.;

    SymbolKalmanFilter<FloatType> minFilter_;
    SymbolKalmanFilter<FloatType> maxFilter_;
    RunningStandardDeviation<FloatType, 184> stddev_;
    FloatType idev_ = 1.;
    FloatType offset_ = 0.;
    uint8_t count_ = 0;
    FloatType min_ = 0.;
    FloatType max_ = 0.;
    uint8_t minCount_ = 0;
    uint8_t maxCount_ = 0;
    size_t updateCount_ = 0;
    bool reset_ = true;

public:

    void reset()
    {
        idev_ = 1.;
        offset_ = 0.;
        count_ = 0;
        min_ = 0.;
        max_ = 0.;
        minCount_ = 0;
        maxCount_ = 0;
        updateCount_ = 0;
        stddev_.reset();
        reset_ = true;
    }

    /**
     * This function takes the index samples from the correlator to build
     * the outer symbol samples for the frequency offset (signal DC offset)
     * and the deviation (signal magnitude). It expects bursts of 8 samples,
     * one for each symbol in a sync word.
     *
     * @param sample
     */
    void sample(FloatType sample)
    {
        count_ += 1;

        if (sample < 0)
        {
            minCount_ += 1;
            min_ += sample;
        }
        else
        {
            maxCount_ += 1;
            max_ += sample;
        }

        if (count_ == SYNC_WORD_LEN)
        {
            auto minAvg = min_ / minCount_;
            auto maxAvg = max_ / maxCount_;
            if (reset_)
            {
                minFilter_.reset(minAvg);
                maxFilter_.reset(maxAvg);
                idev_ = 6.0 / (maxAvg - minAvg);
                offset_ = maxAvg + minAvg;
                reset_ = false;
            }
            else
            {
                auto minFiltered = minFilter_.update(minAvg, count_ + updateCount_);
                auto maxFiltered = maxFilter_.update(maxAvg, count_ + updateCount_);
                idev_ = 6.0 / (maxFiltered[0] - minFiltered[0]);
                offset_ = maxFiltered[0] + minFiltered[0];
            }

            count_ = 0;
            updateCount_ = 0;
            min_ = 0.;
            max_ = 0.;
            minCount_ = 0;
            maxCount_ = 0;
        }
    }

    FloatType normalize(FloatType sample) const
    {
        return (sample - offset_) * idev_;
    }

    FloatType evm() const { return stddev_.stdev(); }

    void update() const {}

    /**
     * Capture EVM of a symbol.
     *
     * @param sample is a normalized sample captured at the best sample point.
     */
    void update(FloatType sample)
    {
        if (sample > 2)
        {
            stddev_.capture(sample - 3);
        }
        else if (sample > 0)
        {
            stddev_.capture(sample - 1);
        }
        else if (sample > -2)
        {
            stddev_.capture(sample + 1);
        }
        else
        {
            stddev_.capture(sample + 3);
        }

        updateCount_ += 1;
    }

    FloatType idev() const { return idev_; }
    FloatType offset() const { return offset_; }
    FloatType deviation() const { return DEVIATION / idev_; }
    FloatType error() const { return evm(); }
};


}} // mobilinkd::m17
