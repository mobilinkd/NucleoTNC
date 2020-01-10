// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "HdlcFrame.hpp"
#include "FirFilter.hpp"
#include "AudioInput.hpp"

#include <functional>

#include <arm_math.h>

namespace mobilinkd { namespace tnc {

constexpr size_t FILTER_TAP_NUM = 132;

using demod_filter_t = std::function<q15_t*(q15_t*, size_t)>;
using demodulator_t = std::function<hdlc::IoFrame*(q15_t*)>;
using audio_filter_t = Q15FirFilter<audio::ADC_BUFFER_SIZE, FILTER_TAP_NUM>;

extern audio_filter_t demod_filter;

struct IDemodulator
{
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual hdlc::IoFrame* operator()(const q15_t* samples) = 0;
    virtual bool locked() const = 0;
    virtual size_t size() const = 0;

    virtual ~IDemodulator() {}

    static void startADC(uint32_t period) {
        ADC_ChannelConfTypeDef sConfig;

        sConfig.Channel = AUDIO_IN;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            CxxErrorHandler();

        htim6.Init.Period = period;
        htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
        {
            CxxErrorHandler();
        }

        if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
        {
            CxxErrorHandler();
        }
        if (HAL_ADC_Start_DMA(&hadc1, audio::adc_buffer,
            audio::ADC_BUFFER_SIZE * 2) != HAL_OK)
        {
            CxxErrorHandler();
        }
    }

    static void stopADC() {
        if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
            CxxErrorHandler();
        if (HAL_TIM_Base_Stop(&htim6) != HAL_OK)
            CxxErrorHandler();
    }
};

}} // mobilinkd::tnc
