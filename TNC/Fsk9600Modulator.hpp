// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Modulator.hpp"

#include <array>
#include <algorithm>
#include <cstdint>

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

    static constexpr std::array<int16_t, BIT_LEN> cos_table{
        2047,  2020,  1937,  1801,  1616,  1387,  1120,   822,   502,   169,
        -169,  -502,  -822, -1120, -1387, -1616, -1801, -1937, -2020, -2048
    };

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

    void init(const kiss::Hardware& hw) override
    {
        for (auto& x : buffer_) x = 2048;

        state = State::STOPPED;
        level = Level::HIGH;

        // Configure 80MHz clock for 192ksps.
        htim7.Init.Period = 416;
        if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
        {
            ERROR("htim7 init failed");
            CxxErrorHandler();
        }
    }

    void deinit() override
    {
        state = State::STOPPED;
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
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

    void send(bool bit) override
    {
        auto scrambled = lfsr(bit);

        switch (state)
        {
        case State::STOPPING:
        case State::STOPPED:
            fill_first(scrambled);
            state = State::STARTING;
            break;
        case State::STARTING:
            fill_last(scrambled);
            state = State::RUNNING;
            ptt_->on();
            HAL_TIM_Base_Start(&htim7);
            HAL_DAC_Start_DMA(
                &hdac1, DAC_CHANNEL_1,
                reinterpret_cast<uint32_t*>(buffer_.data()), buffer_.size(),
                DAC_ALIGN_12B_R);
            break;
        case State::RUNNING:
            osMessagePut(dacOutputQueueHandle_, scrambled, osWaitForever);
            break;
        }
    }

    // DAC DMA interrupt functions.

    void fill_first(bool bit) override
    {
        fill(buffer_.data(), bit);
    }

    void fill_last(bool bit) override
    {
        fill(buffer_.data() + BIT_LEN, bit);
    }

    void empty() override
    {
        switch (state)
        {
        case State::STARTING:
            // fall-through
        case State::RUNNING:
            state = State::STOPPING;
            break;
        case State::STOPPING:
            state = State::STOPPED;
            HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
            HAL_TIM_Base_Stop(&htim7);
            ptt_->off();
            break;
        case State::STOPPED:
            break;
        }
        state = State::STOPPING;
        level = Level::HIGH;
    }

    void abort() override
    {
        state = State::STOPPED;
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim7);
        ptt_->off();

        // Drain the queue.
        while (osMessageGet(dacOutputQueueHandle_, 0).status == osEventMessage);
    }

    float bits_per_ms() const override
    {
        return 9.6f;
    }

private:

    uint16_t adjust_level(int32_t sample) const
    {
        sample *= volume_;
        sample >>= 12;
        sample += 2048;
        return sample;
    }

    void fill(uint16_t* buffer, bool bit)
    {
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
                std::transform(cos_table.rbegin(), cos_table.rend(), buffer,
                    [this](auto x){return adjust_level(x);});
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
