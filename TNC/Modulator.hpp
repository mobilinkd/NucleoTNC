// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "PTT.hpp"
#include "KissHardware.hpp"

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
    virtual void send(bool bit) = 0;

    /// The next three functions are called by the DAC DMA interrupt handler.

    /**
     * Fill the first half of the DAC DMA buffer.
     *
     * @warning This function is called in an interrupt context.
     *
     * @param bit
     */
    virtual void fill_first(bool bit) = 0;

    /**
     * Fill the second half of the DAC DMA buffer.
     *
     * @warning This function is called in an interrupt context.
     *
     * @param bit
     */
    virtual void fill_last(bool bit) = 0;

    /**
     * The DAC bit buffer is empty.  There are no more bits to process.
     *
     * @warning This function is called in an interrupt context.
     */
    virtual void empty() = 0;

    virtual void abort() = 0;

    virtual float bits_per_ms() const = 0;
};

}} // mobilinkd::tnc
