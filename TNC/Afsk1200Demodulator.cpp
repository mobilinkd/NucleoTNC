// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Afsk1200Demodulator.hpp"
#include "Goertzel.h"
#include "AudioInput.hpp"
#include "GPIO.hpp"

namespace mobilinkd { namespace tnc {

afsk1200::emphasis_filter_type Afsk1200Demodulator::filter_1;
afsk1200::emphasis_filter_type Afsk1200Demodulator::filter_2;
afsk1200::emphasis_filter_type Afsk1200Demodulator::filter_3;

afsk1200::Demodulator Afsk1200Demodulator::demod1(26400, Afsk1200Demodulator::filter_1);
afsk1200::Demodulator Afsk1200Demodulator::demod2(26400, Afsk1200Demodulator::filter_2);
afsk1200::Demodulator Afsk1200Demodulator::demod3(26400, Afsk1200Demodulator::filter_3);

hdlc::IoFrame* Afsk1200Demodulator::operator()(const q15_t* samples)
{
    hdlc::IoFrame* result = nullptr;

    q15_t* filtered = demod_filter.filter(const_cast<q15_t* >(samples));

    ++counter;

#if 1
    auto frame1 = demod1(filtered, ADC_BLOCK_SIZE);
    if (frame1)
    {
        if (frame1->fcs() != last_fcs or counter > last_counter + 2)
        {
            last_fcs = frame1->fcs();
            last_counter = counter;
            result = frame1;
        }
        else
        {
            hdlc::release (frame1);
        }
    }
#endif

#if 1
    auto frame2 = demod2(filtered, ADC_BLOCK_SIZE);
    if (frame2)
    {
        if (frame2->fcs() != last_fcs or counter > last_counter + 2)
        {
            last_fcs = frame2->fcs();
            last_counter = counter;
            result = frame2;
        }
        else
        {
            hdlc::release(frame2);
        }
    }
#endif

#if 1
    auto frame3 = demod3(filtered, ADC_BLOCK_SIZE);
    if (frame3)
    {
        if (frame3->fcs() != last_fcs or counter > last_counter + 2)
        {
            last_fcs = frame3->fcs();
            last_counter = counter;
            result = frame3;
        }
        else
        {
            hdlc::release(frame3);
        }
    }
#endif
    locked_ = demod1.locked() or demod2.locked() or demod3.locked();
    return result;
}

/*
 * Return twist as a the difference in dB between mark and space.  The
 * expected values are about 0dB for discriminator output and about 5.5dB
 * for de-emphasized audio.
 */
float Afsk1200Demodulator::readTwist()
{
    TNC_DEBUG("enter Afsk1200Demodulator::readTwist");

    float g1200 = 0.0f;
    float g2200 = 0.0f;

    GoertzelFilter<ADC_BLOCK_SIZE, SAMPLE_RATE> gf1200(1200.0, 0);
    GoertzelFilter<ADC_BLOCK_SIZE, SAMPLE_RATE> gf2200(2200.0, 0);

    const uint32_t AVG_SAMPLES = 20;

    startADC(1817, ADC_BLOCK_SIZE);

    for (uint32_t i = 0; i != AVG_SAMPLES; ++i)
    {
        uint32_t count = 0;
        while (count < ADC_BLOCK_SIZE)
        {
            osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
            if (evt.status != osEventMessage)
                continue;

            auto block = (audio::adc_pool_type::chunk_type*) evt.value.p;
            uint16_t* data = (uint16_t*) block->buffer;
            gf1200(data, ADC_BLOCK_SIZE);
            gf2200(data, ADC_BLOCK_SIZE);

            audio::adcPool.deallocate(block);

            count += ADC_BLOCK_SIZE;
        }

        g1200 += (gf1200 / count);
        g2200 += (gf2200 / count);

        gf1200.reset();
        gf2200.reset();
    }

    IDemodulator::stopADC();

    g1200 = 10.0f * log10f(g1200 / AVG_SAMPLES);
    g2200 = 10.0f * log10f(g2200 / AVG_SAMPLES);

    auto result = g1200 - g2200;

    INFO("Twist = %d / 100 (%d - %d)", int(result * 100), int(g1200),
        int(g2200));

    TNC_DEBUG("exit readTwist");
    return result;
}

uint32_t Afsk1200Demodulator::readBatteryLevel()
{
#ifndef NUCLEOTNC
    TNC_DEBUG("enter Afsk1200Demodulator::readBatteryLevel");

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
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

    sConfig.Channel = BATTERY_ADC_CHANNEL;
    if (HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig) != HAL_OK)
        CxxErrorHandler();

    uint32_t vbat = 0;
    if (HAL_ADC_Start(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
    for (size_t i = 0; i != 8; ++i)
    {
        if (HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 1) != HAL_OK) CxxErrorHandler();
        vbat += HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    }

    vbat /= 8;

    if (HAL_ADC_Stop(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
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

    INFO("Vref = %lumV", vref);
    INFO("Vbat = %lumV", vbat);

    TNC_DEBUG("exit Afsk1200Demodulator::readBatteryLevel");
    return vbat;
#else
    return 0;
#endif
}

const q15_t Afsk1200Demodulator::bpf_coeffs[FILTER_TAP_NUM] = {
    4,     0,    -5,   -10,   -13,   -12,    -9,    -4,    -2,    -4,   -12,   -26,
  -41,   -52,   -51,   -35,    -3,    39,    83,   117,   131,   118,    83,    36,
   -6,   -32,   -30,    -3,    36,    67,    66,    19,   -74,  -199,  -323,  -408,
 -421,  -344,  -187,    17,   218,   364,   417,   369,   247,   106,    14,    26,
  166,   407,   676,   865,   866,   605,    68,  -675, -1484, -2171, -2547, -2471,
-1895,  -882,   394,  1692,  2747,  3337,  3337,  2747,  1692,   394,  -882, -1895,
-2471, -2547, -2171, -1484,  -675,    68,   605,   866,   865,   676,   407,   166,
   26,    14,   106,   247,   369,   417,   364,   218,    17,  -187,  -344,  -421,
 -408,  -323,  -199,   -74,    19,    66,    67,    36,    -3,   -30,   -32,    -6,
   36,    83,   118,   131,   117,    83,    39,    -3,   -35,   -51,   -52,   -41,
  -26,   -12,    -4,    -2,    -4,    -9,   -12,   -13,   -10,    -5,     0,     4,
};

}} // mobilinkd::tnc

