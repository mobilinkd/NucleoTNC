// Copyright 2015-2019 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD___HDLC_DECODER_HPP_
#define MOBILINKD___HDLC_DECODER_HPP_

#include "HdlcFrame.hpp"

#include <cstdint>

namespace mobilinkd { namespace tnc { namespace hdlc {

struct Decoder
{
    static const uint16_t FLAG = 0x7E00;
    static const uint16_t ABORT = 0x7F;
    static const uint16_t IDLE = 0xFF;

    enum state {SEARCH, HUNT, FRAMING};

    state state_;
    int ones_;
    uint16_t buffer_;
    IoFrame* frame_;
    int bits_;
    bool passall_;
    bool ready_;

    Decoder(bool pass_all = false);

    void reset() {
        state_ = SEARCH;
        ones_ = 0;
        buffer_ = 0;
        if (frame_) frame_->clear();
        ready_ = false;
    }

    bool process(bool bit)
    {
        switch (state_)
        {
        case SEARCH:
            search(bit);
            break;
        case HUNT:
            hunt(bit);
            break;
        case FRAMING:
            frame(bit);
            break;
        default:
            reset();
            break;
        }

        return ready_;
    }

    /**
     * Process a bit.  If the bit results in a frame, a result set containing
     * the frame, the FCS, and a flag indicating whether it is valid is
     * returned.  If HDLC was contructed with passall=false, the flag returned
     * is always true as only valid frames are returned.
     *
     * When PLL is passed, when true it indicates that the bit should be
     * processed, and when false indicates that any frame in progress should
     * be dropped and the state reset to SEARCH.
     */
    IoFrame* operator()(bool bit, bool pll);

    void add_bit(bool bit)
    {
        const uint16_t BIT = 0x8000;

        buffer_ >>= 1;
        buffer_ |= (BIT * int(bit));
        bits_ += 1;
    }

    char get_char()
    {
        char result = (buffer_ & 0xFF);

        return result;
    }

    void consume_byte()
    {
        const uint16_t MASK = 0xFF00;

        buffer_ &= MASK;
        bits_ -= 8;
    }

    void consume_bit()
    {
        const uint16_t MASK = 0xFF00;

        uint16_t tmp = (buffer_ & 0x7F);
        tmp <<= 1;
        buffer_ &= MASK;
        buffer_ |= tmp;
        bits_ -= 1;
    }

    void start_search()
    {
        state_ = SEARCH;
    }

    bool have_flag()
    {
        const uint16_t MASK = 0xFF00;

        return (buffer_ & MASK) == FLAG;
    }

    void start_hunt()
    {
        state_ = HUNT;
        bits_ = 0;
        buffer_ = 0;
    }

    void search(bool bit)
    {
        add_bit(bit);

        if (have_flag())
        {
            start_hunt();
        }
    }

    bool have_frame_error()
    {
        switch (buffer_ & 0xFF00)
        {
        case 0xFF00:
        case 0xFE00:
        case 0xFC00:
        case 0x7F00:
        case 0x7E00:
        case 0x3F00:
            return true;
        default:
            return false;
        }
    }

    bool have_bogon()
    {
        const uint16_t MASK = 0xFF00;

        if  (bits_ != 8) return false;

        const uint16_t test = (buffer_ & MASK);

        switch (test)
        {
        case 0xFF00:
        case 0xFE00:
        case 0x7F00:
            return true;
        default:
            return false;
        }
    }

    void start_frame()
    {
        state_ = FRAMING;
        frame_->clear();
        ones_ = 0;
        buffer_ &= 0xFF00;
    }

    void hunt(bool bit)
    {
        const uint16_t MASK = 0xFF00;

        add_bit(bit);
        buffer_ &= MASK;

        if (bits_ != 8) return;

        if (have_flag())  {
            start_hunt();
            return;
        }

        if (have_bogon()) {
            start_search();
            return;
        }

        if (not have_frame_error())  {
            start_frame();
            return;
        }

        start_search();
    }

    void frame(bool bit)
    {
        add_bit(bit);

        if (ones_ < 5) {
            if (buffer_ & 0x80) ones_ += 1;
            else ones_ = 0;

            if (bits_ == 16) {
                if (frame_->push_back(get_char())) {
                    consume_byte();
                } else {
                    // Allocation error.
                    frame_->clear();
                    start_search();
                }
            }

            if (have_flag()) {
                if (frame_->size() >= 15) {
                    ready_ = true;
                }
            }
        } else {
            // 5 ones in a row means the next one should be 0 and be skipped.

            if ((buffer_ & 0x80) == 0) {
                ones_ = 0;
                consume_bit();
                return;
            } else {

                // Framing error.  Drop the frame.  If there is a FLAG
                // in the buffer, go into HUNT otherwise SEARCH.
                if (frame_->size() > 15) {
                    ready_ = true;
                    return;
                }

                if ((buffer_ >> (16 - bits_) & 0xFF) == 0x7E) {
                    // Cannot call start_hunt() here because we need
                    // to preserve buffer state.
                    bits_ -= 8;
                    state_ = HUNT;
                    frame_->clear();
                } else {
                    start_search();
                }
            }
        }
    }

    bool frame_end()
    {
        uint16_t tmp = (buffer_ >> (16 - bits_));
        return (tmp & 0xFF) == FLAG;
    }

    bool frame_abort()
    {
        uint16_t tmp = (buffer_ >> (16 - bits_));
        return (tmp & 0x7FFF) == 0x7FFF;
    }

    void abort_frame()
    {
        bits_ = 8;
        buffer_ &= 0xFF00;
        frame_->clear();
    }

    bool ready() const
    {
        return ready_;
    }
};

#if 0

Decoder::Decoder(bool pass_all)
: state_(SEARCH), ones_(0), buffer_(0), frame_(acquire())
, bits_(0), passall_(pass_all), ready_(false)
{}

IoFrame* Decoder::operator()(bool bit, bool pll)
{
    IoFrame* result = nullptr;

    // It appears that the ioFramePool may not get initialized in the proper
    // order during some builds.
    if (nullptr == frame_) frame_ = acquire();

    if (not pll) {
        if ((state_ == FRAMING) and (frame_->size() > 15) and passall_) {
            frame_->parse_fcs();
            if (passall_ or frame_->ok()) {
                result = frame_;
                ready_ = false;
                frame_ = acquire();
                return result;
            }
        }
        reset();
    } else {
        if (process(bit)) {
            ready_ = false;
            frame_->parse_fcs();
            if (passall_ or frame_->ok()) {
                result = frame_;
                frame_ = acquire();
                return result;
            }
            frame_->clear();
        }
    }

    return result;
}
#endif


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

    const uint16_t VALID_CRC = 0xf0b8;

    State state{State::IDLE};

    uint8_t buffer{0};

    uint8_t bits{0};
    uint8_t ones{0};
    bool flag{0};
    bool passall{false};

    frame_type* packet{nullptr};

    NewDecoder(bool pass_all=false)
    : passall(pass_all)
    {}

    optional_result_type operator()(bool input, bool pll_lock);
    uint8_t process(bool input, bool pll_lock);
};

}}} // mobilinkd::tnc::hdlc

#endif // MOBILINKD___HDLC_DECODER_HPP_
