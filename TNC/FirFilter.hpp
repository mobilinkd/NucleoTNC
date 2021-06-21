// Copyright 2015-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <AudioLevel.hpp>

#include "arm_math.h"

#include <vector>
#include <cstdlib>

namespace mobilinkd { namespace tnc {

template <size_t N>
struct TFirCoefficients {
    float taps[N];
};

struct FirCoefficients {
    size_t size;
    const float* taps;

    template <size_t N>
    FirCoefficients(const TFirCoefficients<N>& c)
    : size(N), taps(c.taps)
    {}
};

constexpr size_t MAX_BLOCK_SIZE = 192;

namespace fir_detail {
extern float filter_input[MAX_BLOCK_SIZE];
extern float filter_output[MAX_BLOCK_SIZE];
} // fir_detail

template <size_t BLOCK_SIZE, size_t FILTER_SIZE>
struct FirFilter {
    static_assert(BLOCK_SIZE <= MAX_BLOCK_SIZE, "You must increase MAX_BLOCK_SIZE" );
    const float* filter_taps;
    float filter_state[BLOCK_SIZE + FILTER_SIZE - 1];
    float vgnd_;
    size_t filter_size;
    arm_fir_instance_f32 instance;

    FirFilter()
    : filter_taps(0), filter_state()
    , vgnd_(0.0f)
    , filter_size(FILTER_SIZE), instance()
    {}

    FirFilter(const float* taps, size_t len = FILTER_SIZE)
    : filter_taps(taps), filter_state()
      , filter_size(len), instance()
    {
        init(taps, len);
    }

    void init(const float* taps, size_t len = FILTER_SIZE)
    {
        vgnd_ = float(audio::virtual_ground);
        filter_size = len;
        filter_taps = taps;
        arm_fir_init_f32(&instance, filter_size, (float32_t*)filter_taps,
            filter_state, BLOCK_SIZE);
    }

    void init(const FirCoefficients& filt)
    {
        vgnd_ = float(audio::virtual_ground);
        filter_size = filt.size;
        filter_taps = filt.taps;
        arm_fir_init_f32(&instance, filter_size, (float32_t*)filter_taps,
            filter_state, BLOCK_SIZE);
    }

    // ADC input
    float* operator()(const int16_t* input, float scale = 1.f) // __attribute__((section(".bss2")))
    {
        for (size_t i = 0; i != BLOCK_SIZE; i++) {
            fir_detail::filter_input[i] = float(input[i]) * scale;
        }
        arm_fir_f32(&instance, fir_detail::filter_input, fir_detail::filter_output, BLOCK_SIZE);
        return fir_detail::filter_output;
    }

    float* operator()(float* input) // __attribute__((section(".bss2")))
    {
        arm_fir_f32(&instance, input, fir_detail::filter_output, BLOCK_SIZE);
        return fir_detail::filter_output;
    }

    float operator()(float input) // __attribute__((section(".bss2")))
    {
        arm_fir_f32(&instance, &input, fir_detail::filter_output, 1);
        return *fir_detail::filter_output;
    }
};

template <size_t BLOCK_SIZE, size_t FILTER_SIZE>
struct Q15FirFilter {
    const q15_t* filter_taps = nullptr;
    q15_t filter_state[BLOCK_SIZE + FILTER_SIZE - 1];
    q15_t filter_output[BLOCK_SIZE];
    q15_t vgnd_ = 0;
    q15_t i_vgnd_ = 0;
    arm_fir_instance_q15 instance;

    Q15FirFilter()
    {}

    Q15FirFilter(const q15_t* taps)
    : filter_taps(taps)
    {
        init(taps);
    }

    void init(const q15_t* taps)
    {
        vgnd_ = audio::virtual_ground;
        filter_taps = taps;
        arm_fir_init_q15(&instance, FILTER_SIZE,
            const_cast<q15_t*>(filter_taps),
            filter_state, BLOCK_SIZE);
    }

    q15_t* filter(const q15_t* input)
    {
        arm_fir_fast_q15(&instance, const_cast<q15_t*>(input), filter_output, BLOCK_SIZE);
        return filter_output;
    }
};

}} // mobilinkd::tnc
