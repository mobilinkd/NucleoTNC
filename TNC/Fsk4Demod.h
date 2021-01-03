// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "FirFilter.h"
#include "PhaseEstimator.h"
#include "DeviationError.h"
#include "FrequencyError.h"
#include "SymbolEvm.h"

#include <array>
#include <experimental/array>
#include <optional>
#include <tuple>

namespace mobilinkd
{

namespace detail
{
inline const auto rrc_taps = std::experimental::make_array<int16_t>(
        -151, -100, -18, 80, 175, 246, 275, 249, 170, 49, -90, -219,
        -304, -318, -245, -88, 131, 373, 581, 695, 659, 437, 22, -556,
        -1229, -1890, -2409, -2641, -2452, -1738, -441, 1434, 3816, 6563,
        9480, 12334, 14880, 16891, 18179, 18622, 18179, 16891, 14880, 12334,
        9480, 6563, 3816, 1434, -441, -1738, -2452, -2641, -2409, -1890,
        -1229, -556, 22, 437, 659, 695, 581, 373, 131, -88, -245, -318, -304,
        -219, -90, 49, 170, 249, 275, 246, 175, 80, -18, -100, -151);

inline const auto evm_b = std::experimental::make_array<float>(0.02008337, 0.04016673, 0.02008337);
inline const auto evm_a = std::experimental::make_array<float>(1.0, -1.56101808, 0.64135154);
} // detail

struct Fsk4Demod
{
    using demod_result_t = std::tuple<float, float, int, float>;
    using result_t = std::optional<std::tuple<float, float, int, float, float, float, float>>;

    tnc::Q15FirFilter<320, std::tuple_size<decltype(detail::rrc_taps)>::value> rrc;
    PhaseEstimator<float> phase = PhaseEstimator<double>(48000, 4800);
    DeviationError<float> deviation;
    FrequencyError<float, 32> frequency;
    SymbolEvm<float,  std::tuple_size<decltype(detail::evm_b)>::value> symbol_evm = makeSymbolEvm(makeIirFilter(detail::evm_b, detail::evm_a));

    double sample_rate = 48000;
    double symbol_rate = 4800;
    double gain = 0.04;
    std::array<double, 3> samples{0};
    double t = 0;
    double dt = symbol_rate / sample_rate;
    double ideal_dt = dt;
    bool sample_now = false;
    double estimated_deviation = 1.0;
    double estimated_frequency_offset = 0.0;
    double evm_average = 0.0;

    Fsk4Demod(double sample_rate, double symbol_rate, double gain = 0.04)
    : sample_rate(sample_rate)
    , symbol_rate(symbol_rate)
    , gain(gain * symbol_rate / sample_rate)
    , dt(symbol_rate / sample_rate)
    , ideal_dt(dt)
    {
        samples.fill(0.0);
    }
    
    demod_result_t demod()
    {
        estimated_deviation = deviation(samples[1]);
        for (auto& sample : samples) sample *= estimated_deviation;
    
        estimated_frequency_offset = frequency(samples[1]);
        for (auto& sample : samples) sample -= estimated_frequency_offset;
    
        auto phase_estimate = phase(samples);
        if (samples[1] < 0) phase_estimate *= -1;
        
        dt = ideal_dt - (phase_estimate * gain);
        t += dt;
        
        auto [symbol, evm] = symbol_evm(samples[1]);
        evm_average = symbol_evm.evm();
        samples[0] = samples[2];

        return std::make_tuple(samples[1], phase_estimate, symbol, evm);
    }

    /**
     * Process the sample.  If a symbol is ready, return a tuple
     * containing the sample used, the estimated phase, the decoded
     * symbol, the EVM, the deviation error and the frequency error
     * (sample, phase, symbol, evm, ed, ef), otherwise None.
     */
    result_t operator()(double sample)
    {
        auto filtered_sample = rrc(sample);

        if (sample_now)
        {
            samples[2] = filtered_sample;
            sample_now = false;
            auto [prev_sample, phase_estimate, symbol, evm] = demod();
            return std::make_tuple(prev_sample, phase_estimate, symbol, evm, estimated_deviation, estimated_frequency_offset, evm_average);
        }
            
        t += dt;
        if (t < 1.0)
        {
            samples[0] = filtered_sample;
        }
        else
        {
            t -= 1.0;
            samples[1] = filtered_sample;
            sample_now = true;
        }

        return std::nullopt;
    }
};

} // mobilinkd
