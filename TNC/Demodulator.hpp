// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "HdlcFrame.hpp"
#include "FirFilter.hpp"
#include "AudioInput.hpp"

#include <functional>

#include <arm_math.h>

namespace mobilinkd { namespace tnc {

constexpr size_t FILTER_TAP_NUM = 132;

using demod_filter_t = std::function<q15_t*(q15_t*, size_t)>;
using demodulator_t = std::function<hdlc::IoFrame*(q15_t*)>;

struct IDemodulator
{
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual hdlc::IoFrame* operator()(const q15_t* samples) = 0;
    virtual float readTwist() = 0;
    virtual uint32_t readBatteryLevel() = 0;

    virtual bool locked() const = 0;
    virtual size_t size() const = 0;

    /**
     * Tell the demodulator to return all "passable" HDLC frames.  These
     * are frames which consist of an even multiple of eight bits and are
     * up to 330 bytes, but which do not have a valid checksum.
     *
     * @param enabled is true when enabled and false when disabled.  The
     *  default state is disabled.
     */
    virtual void passall(bool enabled) = 0;

    virtual ~IDemodulator() {}

    static void startADC(uint32_t period, uint32_t block_size);

    static void stopADC();
};

}} // mobilinkd::tnc
