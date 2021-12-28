// Copyright 2015-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <AudioLevel.hpp>

#include "arm_math.h"

#include <array>
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

template <size_t BLOCK_SIZE, size_t FILTER_SIZE>
struct FirFilter {
    float filter_state[BLOCK_SIZE + FILTER_SIZE - 1];
    float filter_output[BLOCK_SIZE];
    arm_fir_instance_f32 instance;

    FirFilter()
    {}

    FirFilter(std::array<float, FILTER_SIZE> const& taps)
    {
        init(taps);
    }

    FirFilter(const float* taps)
    {
        init(taps);
    }

    void init(std::array<float, FILTER_SIZE> const& taps)
    {
        arm_fir_init_f32(&instance, FILTER_SIZE, (float*)taps.data(),
            filter_state, BLOCK_SIZE);
    }

    void init(const float* taps)
    {
        arm_fir_init_f32(&instance, FILTER_SIZE, (float*)taps,
            filter_state, BLOCK_SIZE);
    }

    float* operator()(float* input)
    {
        arm_fir_f32(&instance, input, filter_output, BLOCK_SIZE);
        return filter_output;
    }

    float operator()(float input) // __attribute__((section(".bss2")))
    {
        arm_fir_f32(&instance, &input, filter_output, 1);
        return filter_output[0];
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
