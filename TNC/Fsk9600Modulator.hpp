// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Modulator.hpp"

#include "stm32l4xx_hal.h"

#include <array>
#include <algorithm>
#include <cstdint>

extern IWDG_HandleTypeDef hiwdg;

namespace mobilinkd { namespace tnc {

struct Scrambler
{
    uint32_t state{0};

    bool operator()(bool bit)
    {
        bool result = (bit ^ (state >> 16) ^ (state >> 11)) & 1;
        state = ((state << 1) | result) & 0x1FFFF;
        return result;
    }
};

struct Fsk9600Modulator : Modulator
{
    static constexpr int8_t DAC_BUFFER_LEN = 40;
    static constexpr int8_t BIT_LEN = DAC_BUFFER_LEN / 2;
    static constexpr uint16_t VREF = 4095;

    using cos_table_type = std::array<int16_t, Fsk9600Modulator::BIT_LEN>;
    static const cos_table_type cos_table;

    enum class Level { ZERO, HIGH, LOW };
    enum class State { STOPPED, STARTING, RUNNING, STOPPING };

    osMessageQId dacOutputQueueHandle_{0};
    PTT* ptt_{nullptr};
    uint16_t volume_{4096};
    std::array<uint16_t, DAC_BUFFER_LEN> buffer_;
    Level level{Level::HIGH};
    State state{State::STOPPED};
    Scrambler lfsr;

    Fsk9600Modulator(osMessageQId queue, PTT* ptt)
    : dacOutputQueueHandle_(queue), ptt_(ptt)
    {}

    ~Fsk9600Modulator() override {}

    void init(const kiss::Hardware& hw) override;

    void deinit() override
    {
        state = State::STOPPED;
        HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim7);
        ptt_->off();
    }

    void set_gain(uint16_t level) override
    {
        auto v = std::max<uint16_t>(256, level);
        v = std::min<uint16_t>(4096, v);
        volume_ = v;
    }

    void set_ptt(PTT* ptt) override
    {
        if (state != State::STOPPED)
        {
            ERROR("PTT change while not stopped");
            CxxErrorHandler();
        }
        ptt_ = ptt;
        ptt_->off();
    }

    void send(uint8_t bit) override
    {
        auto scrambled = lfsr(bit);

        switch (state)
        {
        case State::STOPPING:
        case State::STOPPED:
            ptt_->on();
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_DisableLSCO();
#endif

            fill_first(scrambled);
            state = State::STARTING;
            break;
        case State::STARTING:
            fill_last(scrambled);
            state = State::RUNNING;
            start_conversion();
            break;
        case State::RUNNING:
            osMessagePut(dacOutputQueueHandle_, scrambled, osWaitForever);
            break;
        }
    }

    void tone(uint16_t freq) override {}

    // DAC DMA interrupt functions.

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

        switch (state)
        {
        case State::STARTING:
            // fall-through
        case State::RUNNING:
            state = State::STOPPING;
            break;
        case State::STOPPING:
            state = State::STOPPED;
            stop_conversion();
            ptt_->off();
            level = Level::HIGH;
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            break;
        case State::STOPPED:
            break;
        }
    }

    void abort() override
    {
        state = State::STOPPED;
        stop_conversion();
        ptt_->off();
        level = Level::HIGH;
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
            HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
        // Drain the queue.
        while (osMessageGet(dacOutputQueueHandle_, 0).status == osEventMessage);
    }

    float bits_per_ms() const override
    {
        return 9.6f;
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

    uint16_t adjust_level(int32_t sample) const
    {
        sample *= volume_;
        sample >>= 12;
        sample += 2048;
        return sample;
    }

    void fill(uint16_t* buffer, bool bit)
    {
        HAL_IWDG_Refresh(&hiwdg);
        switch (level)
        {
        case Level::HIGH:
            if (bit)
            {
                std::fill(buffer, buffer + BIT_LEN, adjust_level(2047));
            }
            else
            {
                std::transform(cos_table.begin(), cos_table.end(), buffer,
                    [this](auto x){return adjust_level(x);});
                level = Level::LOW;
            }
            break;
        case Level::LOW:
            if (bit)
            {
                std::transform(cos_table.begin(), cos_table.end(), buffer,
                    [this](auto x){return adjust_level(-1 - x);});
                level = Level::HIGH;
            }
            else
            {
                std::fill(buffer, buffer + BIT_LEN, adjust_level(-2048));
            }
            break;
        default:
            CxxErrorHandler();
        }
    }
};

}} // mobilinkd::tnc
