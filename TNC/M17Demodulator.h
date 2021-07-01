// Copyright 2020-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "AudioInput.hpp"
#include "AudioLevel.hpp"
#include "ClockRecovery.h"
#include "Correlator.h"
#include "DataCarrierDetect.h"
#include "Demodulator.hpp"
#include "FreqDevEstimator.h"
#include "GPIO.hpp"
#include "KissHardware.hpp"
#include "Log.h"
#include "M17.h"
#include "M17FrameDecoder.h"
#include "M17Framer.h"
#include "Modulator.hpp"
#include "ModulatorTask.hpp"
#include "Util.h"

#include <arm_math.h>

#include <algorithm>
#include <array>
#include <optional>
#include <tuple>

namespace mobilinkd { namespace tnc {

struct M17Demodulator : IDemodulator
{
    static constexpr uint32_t ADC_BLOCK_SIZE = 192;
    static_assert(audio::ADC_BUFFER_SIZE >= ADC_BLOCK_SIZE);

    static constexpr uint32_t SAMPLE_RATE = 48000;
    static constexpr uint32_t SYMBOL_RATE = 4800;
    static constexpr uint32_t SAMPLES_PER_SYMBOL = SAMPLE_RATE / SYMBOL_RATE;
    static constexpr uint16_t VREF = 16383;
    static constexpr int16_t MAX_SAMPLE_ADJUST = 140;
    static constexpr int16_t MIN_SAMPLE_ADJUST = -140;

    static constexpr float sample_rate = SAMPLE_RATE;
    static constexpr float symbol_rate = SYMBOL_RATE;

    static constexpr uint8_t MAX_MISSING_SYNC = 5;

    using audio_filter_t = FirFilter<ADC_BLOCK_SIZE, m17::FILTER_TAP_NUM_15>;
    using sync_word_t = m17::SyncWord<m17::Correlator>;

    enum class DemodState { UNLOCKED, LSF_SYNC, STREAM_SYNC, PACKET_SYNC, FRAME };

    audio_filter_t demod_filter;
    std::array<float, ADC_BLOCK_SIZE> demod_buffer;
    m17::DataCarrierDetect<float, SAMPLE_RATE, 500> dcd{2500, 4000, 1.0, 10.0};
    m17::ClockRecovery<float, SAMPLE_RATE, SYMBOL_RATE> clock_recovery;

    m17::Correlator correlator;
    sync_word_t preamble_sync{{+3,-3,+3,-3,+3,-3,+3,-3}, 29.f};
    sync_word_t lsf_sync{{+3,+3,+3,+3,-3,-3,+3,-3}, 31.f, -31.f};
    sync_word_t packet_sync{{3,-3,3,3,-3,-3,-3,-3}, 32.f};

    m17::FreqDevEstimator<float> dev;

    std::array<int8_t, 368> buffer;
    int8_t polarity = 1;
    M17Framer<368> framer;
    M17FrameDecoder decoder;
    DemodState demodState = DemodState::UNLOCKED;
    M17FrameDecoder::SyncWordType sync_word_type = M17FrameDecoder::SyncWordType::LSF;
    uint8_t sample_index = 0;
    float idev;

    bool dcd_ = false;
	bool need_clock_reset_ = false;
	bool need_clock_update_ = false;

    bool passall_ = false;
    int ber = -1;
    int16_t sync_count = 0;
    uint16_t missing_sync_count = 0;
    uint8_t sync_sample_index = 0;
    int16_t adc_timing_adjust = 0;
    float prev_clock_estimate = 1.;


    virtual ~M17Demodulator() {}

    void start() override;

    void dcd_on();
    void dcd_off();
    void initialize(const q15_t* input);
    void update_dcd(const q15_t* input);
    void do_unlocked();
    void do_lsf_sync();
    void do_packet_sync();
    void do_stream_sync();
    void do_frame(float filtered_sample, hdlc::IoFrame*& frame_result);

    void stop() override
    {
//        getModulator().stop_loopback();
        stopADC();
        dcd_off();
    }

    bool locked() const override
    {
        return dcd_;
    }

    size_t size() const override
    {
        return ADC_BLOCK_SIZE;
    }

    void passall(bool enabled) override
    {
        passall_ = enabled;
        decoder.passall(enabled);
    }

    void update_values(uint8_t index);

    hdlc::IoFrame* operator()(const q15_t* input) override;

    /*
     * Return twist as a the difference in dB between mark and space.  The
     * expected values are about 0dB for discriminator output and about 5.5dB
     * for de-emphasized audio.
     */
    float readTwist() override
    {
        return 0;
    }

    uint32_t readBatteryLevel() override
    {
#ifndef NUCLEOTNC
        DEBUG("enter M17Demodulator::readBatteryLevel");

        ADC_ChannelConfTypeDef sConfig;

        sConfig.Channel = ADC_CHANNEL_VREFINT;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            CxxErrorHandler();

        htim6.Init.Period = 48000;
        if (HAL_TIM_Base_Init(&htim6) != HAL_OK) CxxErrorHandler();

        if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
            CxxErrorHandler();

        if (HAL_ADC_Start(&hadc1) != HAL_OK) CxxErrorHandler();
        if (HAL_ADC_PollForConversion(&hadc1, 3) != HAL_OK) CxxErrorHandler();
        auto vrefint = HAL_ADC_GetValue(&hadc1);
        if (HAL_ADC_Stop(&hadc1) != HAL_OK) CxxErrorHandler();

        // Disable battery charging while measuring battery voltage.
        auto usb_ce = gpio::USB_CE::get();
        gpio::USB_CE::on();

        gpio::BAT_DIVIDER::off();
        HAL_Delay(1);

        sConfig.Channel = ADC_CHANNEL_15;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            CxxErrorHandler();

        uint32_t vbat = 0;
        if (HAL_ADC_Start(&hadc1) != HAL_OK) CxxErrorHandler();
        for (size_t i = 0; i != 8; ++i)
        {
            if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK) CxxErrorHandler();
            vbat += HAL_ADC_GetValue(&hadc1);
        }

        vbat /= 8;

        if (HAL_ADC_Stop(&hadc1) != HAL_OK) CxxErrorHandler();
        if (HAL_TIM_Base_Stop(&htim6) != HAL_OK)
            CxxErrorHandler();

        gpio::BAT_DIVIDER::on();

        // Restore battery charging state.
        if (!usb_ce) gpio::USB_CE::off();

        INFO("Vref = %lu", vrefint);
        INFO("Vbat = %lu (raw)", vbat);

        // Order of operations is important to avoid underflow.
        vbat *= 6600;
        vbat /= (VREF + 1);

        uint32_t vref = ((vrefint * 3300) + (VREF / 2)) / VREF;

        INFO("Vref = %lumV", vref)
        INFO("Vbat = %lumV", vbat);

        DEBUG("exit M17Demodulator::readBatteryLevel");
        return vbat;
#else
        return 0;
#endif
    }
};

}} // mobilinkd::tnc
