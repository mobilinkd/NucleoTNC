// Copyright 2016-2019 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "HdlcDecoder.hpp"
#include "GPIO.hpp"
#include "Log.h"

namespace mobilinkd { namespace tnc { namespace hdlc {

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
    if (nullptr == frame_) return result;

    if (not pll) {
        if ((state_ == FRAMING) and (frame_->size() >= 15) and passall_) {
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
                if (frame_) start_hunt();
                return result;
            }
            frame_->clear();
        }
    }

    return result;
}

NewDecoder::optional_result_type NewDecoder::operator()(bool input, bool pll_lock)
{
    optional_result_type result = nullptr;

    auto status = process(input, pll_lock);
    if (status)
    {
//        INFO("Frame Status = %02x, size = %d, CRC = %04x",
//            int(status), int(packet->size()), int(packet->crc()));
        if (((status & STATUS_OK) || passall) && packet->size() > 10)
        {
            result = packet;
            packet = nullptr;
        } else {
            packet->clear();
        }
    }

    return result;
}

uint8_t NewDecoder::process(bool input, bool pll_lock)
{
    uint8_t result_code = 0;

    while (packet == nullptr) {
        packet = ioFramePool().acquire();
        if (!packet) osThreadYield();
    }

    if (pll_lock) {
        if (ones == 5) {
            if (input) {
                // flag byte
                flag = 1;
            } else {
                // bit stuffing...
                flag = 0;
                ones = 0;
                return result_code;
            }
        }

        buffer >>= 1;
        buffer |= (input * 128);
        bits += 1;                      // Free-running until Sync byte.
        if (input) {
            ++ones;
        } else {
            ones = 0;
        }

        if (flag) {
            switch (buffer) {
            case 0x7E:
                if (packet->size()) {
                    packet->parse_fcs();
                    result_code = packet->ok() ? STATUS_OK : STATUS_CRC_ERROR;
                }
                state = State::SYNC;
                flag = 0;
                bits = 0;
                break;
            case 0xFE:
                if (packet->size()) {
                    result_code = STATUS_FRAME_ABORT;
                }
                state = State::IDLE;
                flag = 0;
                bits = 0;
                break;
            default:
                /* pass */
                break;
            }
            return result_code;
        }

        switch (state) {

        case State::IDLE:
            break;

        case State::SYNC:
            if (bits == 8) {    // 8th bit.
                // Start of frame data.
                state = State::RECEIVE;
                packet->push_back(buffer);
                bits = 0;
            }
            break;

        case State::RECEIVE:
            if (bits == 8) {    // 8th bit.
                packet->push_back(buffer);
                bits = 0;
            }
        }
    } else {
        // PLL unlocked.
        if (packet->size()) {
            packet->parse_fcs();
            result_code = packet->ok() ? STATUS_OK | STATUS_NO_CARRIER : STATUS_NO_CARRIER;
        }
        if (state != State::IDLE) {
            buffer = 0;
            flag = 0;
            bits = 0;
            state = State::IDLE;
        }
    }

    return result_code;
}

}}} // mobilinkd::tnc::hdlc
