// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Demodulator.hpp"
#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "DigitalPLL.hpp"
#include "NRZI.hpp"
#include "HdlcDecoder.hpp"

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
    /*
     * Generated with Scipy Filter, 152 coefficients, 55-4900Hz bandpass,
     * Hann window, starting and ending minimal value coefficients removed.
     *
     * np.array(
     *  firwin2(152,
     *      [
     *          0.0,
     *          40.0/(sample_rate/2),
     *          55.0/(sample_rate/2),
     *          4900.0/(sample_rate/2),
     *          7200.0/(sample_rate/2),
     *          1.0
     *      ],
     *      [0,0,1,1,0,0],
     *      antisymmetric = False,
     *      window='hann') * 32768,
     *  dtype=int)[10:-10]
     */
    static constexpr size_t FILTER_TAP_NUM = 132;
    static const q15_t bpf_coeffs[FILTER_TAP_NUM];

    BaseDigitalPLL<float> pll_{192000,9600};
    bool locked_{false};
    Descrambler lfsr_;
    libafsk::NRZI nrzi_;
    hdlc::NewDecoder hdlc_decoder_;

    virtual ~Fsk9600Demodulator() {}

    void start() override
    {
        demod_filter.init(bpf_coeffs);
        hadc1.Init.OversamplingMode = DISABLE;
        if (HAL_ADC_Init(&hadc1) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
        startADC(416);
    }

    void stop() override
    {
        stopADC();
        locked_ = false;
    }

    hdlc::IoFrame* operator()(const q15_t* samples) override
    {
        hdlc::IoFrame* result = nullptr;

        auto filtered = demod_filter(const_cast<q15_t* >(samples));

        for (size_t i = 0; i != audio::ADC_BUFFER_SIZE; ++i)
        {
            auto sample = filtered[i];

            bool bit = sample >= 0;
            auto pll = pll_(bit);

            if (pll.sample)
            {
                locked_ = pll.locked;

                // We will only ever get one frame because there are
                // not enough bits in a block for more than one.
                if (result) {
                    auto tmp = hdlc_decoder_(nrzi_.decode(lfsr_(bit)), true);
                    if (tmp) hdlc::release(tmp);
                } else {
                    result = hdlc_decoder_(nrzi_.decode(lfsr_(bit)), true);
                }
            }
        }
        return result;
    }

    bool locked() const override
    {
        return locked_;
    }

    constexpr size_t size() const override
    {
        return audio::ADC_BUFFER_SIZE;
    }
};

}} // mobilinkd::tnc
