// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Demodulator.hpp"
#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "GPIO.hpp"
#include "Log.h"
#include "PhaseEstimator.h"
#include "DeviationError.h"
#include "FrequencyError.h"
#include "SymbolEvm.h"
#include "Util.h"
#include "CarrierDetect.h"
#include "M17Synchronizer.h"
#include "M17Framer.h"
#include "M17FrameDecoder.h"
#include "KissHardware.hpp"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "M17.h"

#include <arm_math.h>

#include <algorithm>
#include <array>
#include <experimental/array>
#include <optional>
#include <tuple>

namespace mobilinkd { namespace tnc {

struct M17Demodulator : IDemodulator
{
    static constexpr uint32_t ADC_BLOCK_SIZE = 192;
    static_assert(audio::ADC_BUFFER_SIZE >= ADC_BLOCK_SIZE);

    static constexpr auto evm_b = std::experimental::make_array<float>(0.02008337, 0.04016673, 0.02008337);
    static constexpr auto evm_a = std::experimental::make_array<float>(1.0, -1.56101808, 0.64135154);

    static constexpr uint32_t SAMPLE_RATE = 48000;
    static constexpr uint16_t VREF = 4095;

    using audio_filter_t = Q15FirFilter<ADC_BLOCK_SIZE, m17::FILTER_TAP_NUM_9>;
    using demod_result_t = std::tuple<float, float, int, float>;

    enum class DemodState { UNLOCKED, LSF_SYNC, FRAME_SYNC, FRAME };

    audio_filter_t demod_filter{m17::rrc_taps_9.data()};
    const float sample_rate = 48000.0;
    const float symbol_rate = 4800.0;
    float gain = 0.01;
    std::array<q15_t, 3> samples;
    std::array<float, 3> f_samples;
    std::array<int8_t, 368> buffer;
    float t = 0.0f;
    float dt = 0.1f;    // symbol_rate / sample_rate.
    const float ideal_dt = dt;
    bool sample_now = false;
    float estimated_deviation = 1.0;
    float estimated_frequency_offset = 0.0;
    float evm_average = 0.0;
    bool decoding = false;
    PhaseEstimator<float> phase = PhaseEstimator<float>(sample_rate, symbol_rate);
    DeviationError<float> deviation;
    FrequencyError<float, 32> frequency{evm_b, evm_a};
    SymbolEvm<float, std::tuple_size<decltype(evm_b)>::value> symbol_evm{evm_b, evm_a};
    CarrierDetect<float> dcd{evm_b, evm_a, 0.01, 0.75};
    M17Synchronizer lsf_sync_1{m17::sync_word(m17::LSF_SYNC), 1};
    M17Synchronizer stream_sync_1{m17::sync_word(m17::STREAM_SYNC), 1};
    M17Synchronizer stream_sync_3{m17::sync_word(m17::STREAM_SYNC), 3};
    M17Synchronizer packet_sync_3{m17::sync_word(m17::PACKET_SYNC), 3};
    M17Framer<368> framer;
    M17FrameDecoder decoder;
    DemodState demodState = DemodState::UNLOCKED;
    M17FrameDecoder::SyncWordType sync_word_type = M17FrameDecoder::SyncWordType::LSF;

    bool locked_ = false;
    bool passall_ = false;
    int ber = -1;
    int sync_count = 0;

    virtual ~M17Demodulator() {}

    void start() override;

    void stop() override
    {
//        getModulator().stop_loopback();
        stopADC();
        locked_ = false;
    }

    bool locked() const override
    {
        return locked_;
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

    [[gnu::noinline]]
    void frame(demod_result_t demod_result, hdlc::IoFrame*& result)
    {
        auto [sample, phase, symbol, evm] = demod_result;
        auto [locked, evma] = dcd(evm);
        static size_t count = 0;
        gain = locked ? 0.0025f : 0.01f;

        switch (demodState)
        {
        case DemodState::UNLOCKED:
            if (!locked) {
                locked_ = false;
                break;
            }
            demodState = DemodState::LSF_SYNC;
            framer.reset();
            decoder.reset();
            sync_count = 0;
            [[fallthrough]];
        case DemodState::LSF_SYNC:
            if (!locked)
            {
                demodState = DemodState::UNLOCKED;
            }
            else if (lsf_sync_1(from_4fsk(symbol)))  // LSF SYNC?
            {
                INFO("LSF_SYNC/LSF");
                demodState = DemodState::FRAME;
                sync_word_type = M17FrameDecoder::SyncWordType::LSF;
            }
            else if (stream_sync_1(from_4fsk(symbol)))  // STREAM SYNC for LICH?
            {
                INFO("LSF_SYNC/STREAM");
                demodState = DemodState::FRAME;
                sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
            }
            break;
        case DemodState::FRAME_SYNC:
            if (!locked)
            {
                INFO("state: %d, dt: %5d, evm: %5d, evma: %5d, dev: %5d, freq: %5d, locked: %d, ber: %d",
                    int(demodState), int(dt * 10000), int(evm * 1000),
                    int(evma * 1000), int((1.0 / estimated_deviation) * 1000),
                    int(estimated_frequency_offset * 1000),
                    locked_, ber);
                demodState = DemodState::UNLOCKED;
            }
            else if (stream_sync_3(from_4fsk(symbol)) && sync_count == 7)
            {
                INFO("STREAM_SYNC");
                demodState = DemodState::FRAME;
                sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
            }
            else if (packet_sync_3(from_4fsk(symbol)) && sync_count == 7)
            {
                INFO("PACKET_SYNC");
                demodState = DemodState::FRAME;
                sync_word_type = M17FrameDecoder::SyncWordType::PACKET;
            }
            else if (++sync_count == 8)
            {
                demodState = DemodState::UNLOCKED;
                locked_ = false;
            }
            break;
        case DemodState::FRAME:
            {
                locked_ = true;
                auto n = llr<float, 4>(sample);
                int8_t* tmp;
                auto len = framer(n, &tmp);
                if (len != 0)
                {
                    demodState = DemodState::FRAME_SYNC;
                    sync_count = 0;
                    std::copy(tmp, tmp + len, buffer.begin());
                    auto valid = decoder(sync_word_type, buffer, result, ber);
                    switch (valid)
                    {
                    case M17FrameDecoder::DecodeResult::FAIL:
                        WARN("decode invalid");
                        if (result && !passall_)
                        {

                            if (result) hdlc::release(result);
                            result = 0;
                        }
                        break;
                    case M17FrameDecoder::DecodeResult::EOS:
                        demodState = DemodState::LSF_SYNC;
                        break;
                    case M17FrameDecoder::DecodeResult::OK:
                        INFO("valid frame for sync word type %d", int(sync_word_type));
                        break;
                    }
                }
            }
            break;
        }
        if ((count++ % 192) == 0)
        {
            INFO("state: %d, dt: %5d, evm: %5d, evma: %5d, dev: %5d, freq: %5d, locked: %d, ber: %d",
                int(demodState), int(dt * 10000), int(evm * 1000),
                int(evma * 1000), int((1.0 / estimated_deviation) * 1000),
                int(estimated_frequency_offset * 1000),
                locked, ber);
        }
    }

    [[gnu::noinline]]
    demod_result_t demod()
    {
        int16_t polarity = kiss::settings().rx_rev_polarity() ? -1 : 1;

        f_samples[0] = float(samples[0]) / 8192.0;
        f_samples[1] = float(samples[1]) / 8192.0;
        f_samples[2] = float(samples[2]) / 8192.0;

        estimated_deviation = deviation(f_samples[1]);
        for (auto& sample : f_samples) sample *= estimated_deviation;

        estimated_frequency_offset = frequency(f_samples[1]);
        for (auto& sample : f_samples) sample -= estimated_frequency_offset;

        auto phase_estimate = phase(f_samples);
        if (f_samples[1] < 0.0) phase_estimate *= -1.0;

        dt = ideal_dt - (phase_estimate * gain);
        // dt = std::min(std::max(0.095f, dt), 0.105f);
        t += dt;

        auto [symbol, evm] = symbol_evm(f_samples[1]);
        evm_average = symbol_evm.evm();
        samples[0] = samples[2];

        return std::make_tuple(f_samples[1], phase_estimate, symbol * polarity, evm);
    }

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
