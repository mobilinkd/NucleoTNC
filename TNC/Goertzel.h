// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "AudioLevel.hpp"
#include <arm_math.h>
#include <complex>
#include <cmath>

namespace mobilinkd {
namespace tnc {

extern const float WINDOW[];

template<uint32_t SAMPLES, uint32_t SAMPLE_RATE>
class GoertzelFilter
{
    float filterFreq_;
    int bin_;
    float coeff_;
    float d1 { 0.0f };
    float d2 { 0.0f };
    uint32_t count { 0 };
    const float* window_;

public:
    GoertzelFilter(float filter_freq, const float* window = WINDOW)
        : filterFreq_(filter_freq), bin_(
            0.5f + ((filter_freq * SAMPLES) / SAMPLE_RATE)), coeff_(
            2.0f * cos((2.0f * PI * bin_) / float(SAMPLES))), window_(window)
    {
    }

    void operator()(float* samples, uint32_t n)
    {

        for (size_t i = 0; i != n; ++i)
        {
            float w = window_ ? window_[count] : 1.0;
            float y = w * samples[i] + coeff_ * d1 - d2;
            d2 = d1;
            d1 = y;
            ++count;
        }
    }

    void operator()(uint16_t* samples, uint32_t n)
    {

        for (uint32_t i = 0; i != n; ++i)
        {
            float w = window_ ? window_[count] : 1.0;
            float sample = (float(samples[i]) - audio::virtual_ground)
                * audio::i_vgnd;
            float y = w * sample + coeff_ * d1 - d2;
            d2 = d1;
            d1 = y;
            ++count;
        }
    }

    operator float() const
    {
        return d2 * d2 + d1 * d1 - coeff_ * d1 * d2;
    }

    void reset()
    {
        d1 = 0.0f;
        d2 = 0.0f;
        count = 0;
    }
};

#if 0
template <uint32_t SAMPLES, uint32_t SAMPLE_RATE>
class GoertzelFactory
{
    float window_[SAMPLES];
    static void make_window(float* window)
    {
        for (size_t i = 0; i != SAMPLES; ++i)
        {
            window[i] = 0.54f - 0.46f * cos(2.0f * M_PI * (float(i) / float(SAMPLE_RATE)));
        }
    }

public:

    GoertzelFactory()
    : window_()
    {
        make_window(window_);
    }

    GoertzelFilter<SAMPLES, SAMPLE_RATE> make(float freq)
    {
        return GoertzelFilter<SAMPLES, SAMPLE_RATE>(freq, window_);
    }
};
#endif

// Complex Goertzel
// Based on https://dsp.stackexchange.com/a/23946/36581
template<typename Float>
struct Goertzel
{
    typedef Goertzel<Float> type;
    typedef std::complex<Float> complex_type;
    typedef Float float_type;

    float_type sin_;
    float_type cos_;
    float_type coeff_;
    float_type q0 { 0.0 }, q1 { 0.0 }, q2 { 0.0 };
    explicit Goertzel(float_type omega)
        : sin_(sin(omega)), cos_(cos(omega)), coeff_(2.0 * cos_)
    {
    }

    static type from_frequency(float_type frequency, float_type sample_rate)
    {
        return Goertzel(2.0 * PI * frequency / sample_rate);
    }

    template<typename Container>
    complex_type operator()(const Container& data)
    {
        for (auto& v : data)
        {
            q0 = coeff_ * q1 - q2 + v;
            q2 = q1;
            q1 = q0;
        }

        auto real = (q1 - q2 * cos_);
        auto imag = (q2 * sin_);

        q0 = q1 = q2 = 0.0;

        return complex_type(real, imag);
    }
};

typedef Goertzel<float> FloatGoertzel;

}
} // mobilinkd::tnc

