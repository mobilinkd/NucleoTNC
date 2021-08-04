// Copyright 2015-2020 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <stddef.h>

#include "PTT.hpp"
#include "Log.h"
#include "Modulator.hpp"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

#include <algorithm>

extern osMessageQId hdlcOutputQueueHandle;
extern osMessageQId dacOutputQueueHandle;
extern TIM_HandleTypeDef htim7;
extern DAC_HandleTypeDef hdac1;
extern IWDG_HandleTypeDef hiwdg;


namespace mobilinkd { namespace tnc {

const size_t SIN_TABLE_LEN = 264;

const int16_t sin_table[SIN_TABLE_LEN] = {
    2048, 2096, 2145, 2194, 2242, 2291, 2339, 2387,
    2435, 2483, 2530, 2577, 2624, 2671, 2717, 2763,
    2808, 2853, 2898, 2942, 2985, 3029, 3071, 3113,
    3154, 3195, 3235, 3274, 3313, 3351, 3388, 3424,
    3460, 3495, 3529, 3562, 3595, 3626, 3657, 3686,
    3715, 3743, 3770, 3795, 3820, 3844, 3867, 3889,
    3910, 3929, 3948, 3965, 3982, 3997, 4012, 4025,
    4037, 4048, 4058, 4066, 4074, 4080, 4085, 4089,
    4092, 4094, 4095, 4094, 4092, 4089, 4085, 4080,
    4074, 4066, 4058, 4048, 4037, 4025, 4012, 3997,
    3982, 3965, 3948, 3929, 3910, 3889, 3867, 3844,
    3820, 3795, 3770, 3743, 3715, 3686, 3657, 3626,
    3595, 3562, 3529, 3495, 3460, 3424, 3388, 3351,
    3313, 3274, 3235, 3195, 3154, 3113, 3071, 3029,
    2985, 2942, 2898, 2853, 2808, 2763, 2717, 2671,
    2624, 2577, 2530, 2483, 2435, 2387, 2339, 2291,
    2242, 2194, 2145, 2096, 2048, 1999, 1950, 1901,
    1853, 1804, 1756, 1708, 1660, 1612, 1565, 1518,
    1471, 1424, 1378, 1332, 1287, 1242, 1197, 1153,
    1110, 1066, 1024,  982,  941,  900,  860,  821,
     782,  744,  707,  671,  635,  600,  566,  533,
     500,  469,  438,  409,  380,  352,  325,  300,
     275,  251,  228,  206,  185,  166,  147,  130,
     113,   98,   83,   70,   58,   47,   37,   29,
      21,   15,   10,    6,    3,    1,    1,    1,
       3,    6,   10,   15,   21,   29,   37,   47,
      58,   70,   83,   98,  113,  130,  147,  166,
     185,  206,  228,  251,  275,  300,  325,  352,
     380,  409,  438,  469,  500,  533,  566,  600,
     635,  671,  707,  744,  782,  821,  860,  900,
     941,  982, 1024, 1066, 1110, 1153, 1197, 1242,
    1287, 1332, 1378, 1424, 1471, 1518, 1565, 1612,
    1660, 1708, 1756, 1804, 1853, 1901, 1950, 1999,
};


struct AFSKModulator : Modulator
{
    static const size_t DAC_BUFFER_LEN = 44;
    static const size_t BIT_LEN = DAC_BUFFER_LEN / 2;
    static const size_t MARK_SKIP = 12;
    static const size_t SPACE_SKIP = 22;

    size_t pos_{0};
    int running_{-1};
    osMessageQId dacOutputQueueHandle_;
    PTT* ptt_;
    uint8_t twist_{50};
    uint16_t volume_{4096};
    std::array<uint16_t, DAC_BUFFER_LEN> buffer_;

    AFSKModulator(osMessageQId queue, PTT* ptt)
    : dacOutputQueueHandle_(queue), ptt_(ptt)
    {
        for (size_t i = 0; i != DAC_BUFFER_LEN; i++)
            buffer_[i] = 2048;
    }

   void init(const kiss::Hardware& hw);

   void deinit() override
   {
   }

   void set_gain(uint16_t v) override
    {
        v = std::max<uint16_t>(256, v);
        v = std::min<uint16_t>(4096, v);
        volume_ = v;
    }

    void set_ptt(PTT* ptt) {
        if (ptt == ptt_) return;  // No change.
        auto old = ptt_;
        ptt_ = ptt;
        old->off();
        if (running_ == 1) {
            ptt_->on();
        }
    }

    void set_twist(uint8_t twist) {twist_ = twist;}

    void send(uint8_t bit) override
    {
        switch (running_) {
        case -1:
            ptt_->on();
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_DisableLSCO();
#endif

            fill_first(bit);
            running_ = 0;
            break;
        case 0:
            fill_last(bit);
            running_ = 1;
            start_conversion();
            break;
        case 1:
            osMessagePut(dacOutputQueueHandle_, bit, osWaitForever);
            break;
        }
    }

    void tone(uint16_t freq) override {}

    void fill(uint16_t* buffer, bool bit)
    {
        HAL_IWDG_Refresh(&hiwdg);

        for (size_t i = 0; i != BIT_LEN; i++)
        {
            int s = sin_table[pos_];
            s -= 2048;
            s *= volume_;
            s >>= 12;
            s += 2048;

            if (bit) {
                if (twist_ > 50) {
                    s = (((s - 2048) * (100 - twist_)) / 50) + 2048;
                }
            } else {
                if (twist_ < 50) {
                    s = (((s - 2048) * twist_) / 50) + 2048;
                }
            }
            if (s < 0 or s > 4095) {
              TNC_DEBUG("DAC inversion (%d)", s);
            }
            *buffer = uint16_t(s);
            ++buffer;
            pos_ += (bit ? MARK_SKIP : SPACE_SKIP);
            if (pos_ >= SIN_TABLE_LEN) pos_ -= SIN_TABLE_LEN;
        }
    }

    void fill_first(uint8_t bit) override
    {
        fill(buffer_.data(), bit);
    }

    void fill_last(uint8_t bit) override
    {
        fill(buffer_.data() + BIT_LEN, bit);
    }

    void empty_first() override
    {
        empty();
    }

    void empty_last() override
    {
        empty();
    }

    void empty()
    {
        HAL_IWDG_Refresh(&hiwdg);

        switch (running_) {
        case 1:
            running_ = 0;
            break;
        case 0:
            running_ = -1;
            stop_conversion();
            ptt_->off();
            pos_ = 0;
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            break;
        case -1:
            break;
        }
    }

    void abort() override
    {
        running_ = -1;
        stop_conversion();
        ptt_->off();
        pos_ = 0;
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
            HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif

        // Drain the queue.
        while (osMessageGet(dacOutputQueueHandle_, 0).status == osEventMessage);
   }

   float bits_per_ms() const override
   {
       return 1.2f;
   }

private:
   /**
    * Configure the DAC for timer-based DMA conversion, start the timer,
    * and start DMA to DAC.
    */
   void start_conversion()
   {
       DAC_ChannelConfTypeDef sConfig;

       sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
       sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
       sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
       sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
       sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
       if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
       {
         CxxErrorHandler();
       }

       HAL_TIM_Base_Start(&htim7);
       HAL_DAC_Start_DMA(
           &hdac1, DAC_CHANNEL_1,
           reinterpret_cast<uint32_t*>(buffer_.data()), buffer_.size(),
           DAC_ALIGN_12B_R);
   }

};

}} // mobilinkd::tnc
