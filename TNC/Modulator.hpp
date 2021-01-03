// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "PTT.hpp"
#include "KissHardware.hpp"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include <cstdint>

extern osMessageQId hdlcOutputQueueHandle;
extern osMessageQId dacOutputQueueHandle;
extern TIM_HandleTypeDef htim7;
extern DAC_HandleTypeDef hdac1;

namespace mobilinkd { namespace tnc {

/**
 * The modulator has three distinct interfaces.  The configuration interface
 * which is used to initialize the modulator, the bit sending interface used
 * by the HDLC encoder to transmit the bits, and the modulation interface
 * used by the DAC DMA engine to get the analog values output by the modem.
 */
struct Modulator
{
    virtual ~Modulator() {}

    /**
     * Implement all functionality required to configure the hardware and
     * the modulator.  For example, configuring the timer used by the DAC
     * should be done here.
     */
    virtual void init(const kiss::Hardware& hw) = 0;

    /**
     * Implement all functionality required to deactivate the hardware and
     * the modulator.  For example, disabling the timer used by the DAC
     * should be done here.
     */
    virtual void deinit() = 0;

    virtual void set_gain(uint16_t level) = 0;

    /**
     * Set the PTT controller used by the modulator.  It is only legal
     * to call this when the modulator is stopped.
     *
     * @param ptt
     */
    virtual void set_ptt(PTT* ptt) = 0;

    /**
     * Send a single bit.
     *
     * @param bit
     */
    virtual void send(uint8_t symbol) = 0;

    /// The next three functions are called by the DAC DMA interrupt handler.

    /**
     * Fill the first half of the DAC DMA buffer.
     *
     * @warning This function is called in an interrupt context.
     *
     * @param bit
     */
    virtual void fill_first(uint8_t symbol) = 0;

    /**
     * Fill the second half of the DAC DMA buffer.
     *
     * @warning This function is called in an interrupt context.
     *
     * @param bit
     */
    virtual void fill_last(uint8_t symbol) = 0;

    /**
     * The DAC bit buffer is empty.  There are no more bits to process.
     *
     * @warning This function is called in an interrupt context.
     */
    virtual void empty_first() = 0;
    virtual void empty_last() = 0;

    virtual void abort() = 0;

    virtual float bits_per_ms() const = 0;

    virtual void start_loopback() {}
    virtual void stop_loopback() {}
    virtual void loopback(const void*) {}

protected:

    /**
     * Stop the DMA conversion and the timer.  Configure DAC for no
     * trigger and set the DAC level to exactly mid-level.
     *
     * @note The DAC is set to mid-level to ensure the audio coupling
     *  capacitor is kept biased to ensure that there is no DC level
     *  ramp when we start conversions.  This is bad for FSK.
     */
    void stop_conversion()
    {
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim7);

        DAC_ChannelConfTypeDef sConfig;

        sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
        sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
        sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
        sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
        if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
        {
          CxxErrorHandler();
        }

        if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048) != HAL_OK)
        {
            CxxErrorHandler();
        }
        if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
        {
            CxxErrorHandler();
        }
    }

};

}} // mobilinkd::tnc
