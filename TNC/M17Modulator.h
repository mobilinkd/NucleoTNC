// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "AudioInput.hpp"
#include "Modulator.hpp"
#include "HdlcFrame.hpp"
#include "M17.h"

#include <arm_math.h>

#include <array>
#include <algorithm>
#include <cstdint>

namespace mobilinkd { namespace tnc {

/**
 * M17 modulator. Collects 16 symbols of data, upsamples by 10x using
 * an interpolating FIR filter, which is sent out via the DAC using
 * DMA.
 */
struct M17Modulator : Modulator
{
    // Six buffers per M17 frame, or 12 half-buffer interrupts.
    static constexpr uint8_t UPSAMPLE = 10;
    static constexpr uint32_t BLOCKSIZE = 4;
    static constexpr uint32_t STATE_SIZE = (m17::FILTER_TAP_NUM / UPSAMPLE) + BLOCKSIZE - 1;
    static constexpr int16_t DAC_BUFFER_LEN = 80;               // 8 symbols, 16 bits, 2 bytes.
    static constexpr int16_t TRANSFER_LEN = DAC_BUFFER_LEN / 2; // 4 symbols, 8 bits, 1 byte.
    static constexpr uint16_t VREF = 4095;
    enum class State { STOPPED, STARTING, RUNNING, STOPPING };

    arm_fir_interpolate_instance_q15 fir_interpolator;
    std::array<q15_t, STATE_SIZE> fir_state;
    std::array<int16_t, DAC_BUFFER_LEN> buffer_;
    std::array<int16_t, 4> symbols;
    osMessageQId dacOutputQueueHandle_{0};
    PTT* ptt_{nullptr};
    uint16_t volume_{4096};
    volatile uint16_t delay_count = 0;      // TX Delay
    volatile uint16_t stop_count = 0;       // Flush the RRC matched filter.
    State state{State::STOPPED};

    M17Modulator(osMessageQId queue, PTT* ptt)
    : dacOutputQueueHandle_(queue), ptt_(ptt)
    {
        arm_fir_interpolate_init_q15(
            &fir_interpolator, UPSAMPLE, m17::FILTER_TAP_NUM,
            (q15_t*) m17::rrc_taps.data(), fir_state.data(), BLOCKSIZE);
    }

    ~M17Modulator() override {}

    void start_loopback() override
    {
    }

    void stop_loopback() override
    {
    }

    void loopback(const void* input) override
    {
    }

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

    void send(uint8_t bits) override
    {
        uint16_t txdelay = 0;
        switch (state)
        {
        case State::STOPPING:
        case State::STOPPED:
#if defined(KISS_LOGGING) && !defined(NUCLEOTNC)
            HAL_RCCEx_DisableLSCO();
#endif
            delay_count = 0;
            txdelay = kiss::settings().txdelay * 12 - 5;
            fill_empty(buffer_.data());
            fill_empty(buffer_.data() + TRANSFER_LEN);
            state = State::STARTING;
            [[fallthrough]];
        case State::STARTING:
            osMessagePut(audioInputQueueHandle, tnc::audio::IDLE,
              osWaitForever);
            start_conversion();
            ptt_->on();
            while (delay_count < txdelay) osThreadYield();
            stop_count = 2; // 8 symbols to flush the RRC filter.
            osMessagePut(dacOutputQueueHandle_, bits, osWaitForever);
            state = State::RUNNING;
            break;
        case State::RUNNING:
            osMessagePut(dacOutputQueueHandle_, bits, osWaitForever);
            break;
        }
    }

    // DAC DMA interrupt functions.

    void fill_first(uint8_t bits) override
    {
        fill(buffer_.data(), bits);
    }

    void fill_last(uint8_t bits) override
    {
        fill(buffer_.data() + TRANSFER_LEN, bits);
    }

    /*
     * DAC queue is empty when STARTING.  It is filled with '0' symbols
     * for TX delay duration (using delay_count).  It then transitions
     * to the running state in send() after filling the DAC queue.
     *
     * When no more symbols are available, the DAC queue is empty in the
     * running state.  The FIR filter is flushed of the remaining data
     * using '0' symbols.
     */
    void empty_first() override
    {
        switch (state)
        {
        case State::STARTING:
            fill_empty(buffer_.data());
            delay_count += 1;
            break;
        case State::RUNNING:
            fill_empty(buffer_.data());
            state = State::STOPPING;
            break;
        case State::STOPPING:
            fill_empty(buffer_.data());
            if (--stop_count == 0) state = State::STOPPED;
            break;
        case State::STOPPED:
            stop_conversion();
            ptt_->off();
#if defined(KISS_LOGGING) && !defined(NUCLEOTNC)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            osMessagePut(audioInputQueueHandle, tnc::audio::DEMODULATOR,
              osWaitForever);

            break;
        }
    }

    /*
     * DAC queue is empty when STARTING.  It is filled with '0' symbols
     * for TX delay duration (using delay_count).  It then transitions
     * to the running state in send() after filling the DAC queue.
     *
     * When no more symbols are available, the DAC queue is empty in the
     * running state.  The FIR filter is flushed of the remaining data
     * using '0' symbols.
     */
    void empty_last() override
    {
        switch (state)
        {
        case State::STARTING:
            fill_empty(buffer_.data() + TRANSFER_LEN);
            delay_count += 1;
            break;
        case State::RUNNING:
            fill_empty(buffer_.data() + TRANSFER_LEN);
            state = State::STOPPING;
            break;
        case State::STOPPING:
            fill_empty(buffer_.data() + TRANSFER_LEN);
            if (--stop_count == 0) state = State::STOPPED;
            break;
        case State::STOPPED:
            stop_conversion();
            ptt_->off();
#if defined(KISS_LOGGING) && !defined(NUCLEOTNC)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            osMessagePut(audioInputQueueHandle, tnc::audio::DEMODULATOR,
              osWaitForever);
            break;
        }
    }

    void abort() override
    {
        state = State::STOPPED;
        stop_conversion();
        ptt_->off();
#if defined(KISS_LOGGING) && !defined(NUCLEOTNC)
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

    constexpr int8_t bits_to_symbol(uint8_t bits)
    {
        switch (bits)
        {
        case 0: return 1;
        case 1: return 3;
        case 2: return -1;
        case 3: return -3;
        }
        return 0;
    }

    void fill(int16_t* buffer, uint8_t bits)
    {
        int16_t polarity = kiss::settings().tx_rev_polarity() ? -1 : 1;

        for (size_t i = 0; i != 4; ++i)
        {
            symbols[i] = (bits_to_symbol(bits >> 6) << 10) * polarity;
            bits <<= 2;
        }

        arm_fir_interpolate_q15(
            &fir_interpolator, symbols.data(), buffer, BLOCKSIZE);

        for (size_t i = 0; i != TRANSFER_LEN; ++i)
        {
            buffer[i] = adjust_level(buffer[i]);
        }
    }

    void fill_empty(int16_t* buffer)
    {
        symbols.fill(0);

        arm_fir_interpolate_q15(
            &fir_interpolator, symbols.data(), buffer, BLOCKSIZE);

        for (size_t i = 0; i != TRANSFER_LEN; ++i)
        {
            buffer[i] = adjust_level(buffer[i]);
        }
    }
};

}} // mobilinkd::tnc
