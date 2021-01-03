// Copyright 2015-2020 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "HdlcFrame.hpp"

#include <cstdint>

namespace mobilinkd { namespace tnc { namespace hdlc {

struct NewDecoder
{
    enum class State {IDLE, SYNC, RECEIVE};
    using frame_type = IoFrame;
    using result_type = std::tuple<frame_type*, uint8_t>;
    using optional_result_type = frame_type*;

    static constexpr uint8_t STATUS_OK{0x01};
    static constexpr uint8_t STATUS_USER_CANCEL{0x02};
    static constexpr uint8_t STATUS_FRAME_ABORT{0x04};
    static constexpr uint8_t STATUS_FRAME_ERROR{0x08};
    static constexpr uint8_t STATUS_NO_CARRIER{0x10};
    static constexpr uint8_t STATUS_CRC_ERROR{0x20};

    static constexpr uint16_t VALID_CRC = 0xf0b8;

    State state{State::IDLE};

    uint8_t buffer{0};

    uint8_t bits{0};
    uint8_t report_bits{0};
    uint8_t ones{0};
    bool flag{0};
    bool had_dcd{false};

    /**
     * Tell the demodulator to return all "passable" HDLC frames.  These
     * are frames which consist of an even multiple of eight bits and are
     * up to 330 bytes, but which do not have a valid checksum.
     */
    bool passall{false};

    enum class DCD { ON, PARTIAL, OFF };

    DCD dcd{DCD::PARTIAL};

    frame_type* packet{nullptr};

    NewDecoder(bool pass_all=false)
    : passall(pass_all)
    {}

    bool can_pass(uint8_t status) const
    {
        return status == STATUS_OK or (passall and status == STATUS_CRC_ERROR);
    }

    optional_result_type operator()(bool input, bool pll_lock);
    uint8_t process(bool input, bool pll_lock);
    void setPassall(bool enabled)
    {
        passall = enabled;
    }

    void setDCD(DCD config)
    {
        dcd = config;
    }

    bool active() const
    {
        return state != State::IDLE;
    }
};

}}} // mobilinkd::tnc::hdlc
