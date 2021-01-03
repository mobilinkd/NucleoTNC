// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Demodulator.hpp"
#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "DigitalPLL.hpp"
#include "NRZI.hpp"
#include "HdlcDecoder.hpp"
#include "KissHardware.hpp"
#include "StandardDeviation.hpp"

namespace mobilinkd { namespace tnc {

struct Descrambler
{
    uint32_t state{0};

    bool operator()(bool bit)
    {
        bool result = (bit ^ (state >> 16) ^ (state >> 11)) & 1;
        state = ((state << 1) | (bit & 1)) & 0x1FFFF;
        return result;
    }
};

struct Fsk9600Demodulator : IDemodulator
{
    static constexpr size_t FILTER_TAP_NUM = 132;
    static constexpr uint32_t ADC_BLOCK_SIZE = 384;
    static_assert(audio::ADC_BUFFER_SIZE >= ADC_BLOCK_SIZE);

    static constexpr uint32_t SAMPLE_RATE = 192000;
    static constexpr uint16_t VREF = 4095;

    using bpf_coeffs_type = std::array<int16_t, FILTER_TAP_NUM>;
    using bpf_bank_type = std::array<bpf_coeffs_type, 13>;
    using audio_filter_t = Q15FirFilter<ADC_BLOCK_SIZE, FILTER_TAP_NUM>;

    static const bpf_bank_type bpf_bank;

    audio_filter_t demod_filter;
    BaseDigitalPLL<float> pll_{192000,9600};
    bool locked_{false};
    Descrambler lfsr_;
    libafsk::NRZI nrzi_;
    hdlc::NewDecoder hdlc_decoder_;
    StandardDeviation snr_;
    bool decoding_{false};

    virtual ~Fsk9600Demodulator() {}

    void start() override
    {
        SysClock80();

        auto const& bpf_coeffs = bpf_bank[kiss::settings().rx_twist + 3];
        const q15_t* bpf = bpf_coeffs.data();
        demod_filter.init(bpf);
        passall(kiss::settings().options & KISS_OPTION_PASSALL);

        hadc1.Init.OversamplingMode = DISABLE;
        if (HAL_ADC_Init(&hadc1) != HAL_OK)
        {
            CxxErrorHandler();
        }

        ADC_ChannelConfTypeDef sConfig;

        sConfig.Channel = AUDIO_IN;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            CxxErrorHandler();

        startADC(416, ADC_BLOCK_SIZE);
    }

    void stop() override
    {
        stopADC();
        locked_ = false;
    }

    float readTwist() override;

    uint32_t readBatteryLevel() override;

    hdlc::IoFrame* operator()(const q15_t* samples) override;

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
        hdlc_decoder_.setPassall(enabled);
    }
};

}} // mobilinkd::tnc
