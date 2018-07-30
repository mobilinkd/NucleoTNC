// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "HdlcDecoder.hpp"
#include "GPIO.hpp"

namespace mobilinkd { namespace tnc { namespace hdlc {

Decoder::Decoder(bool pass_all)
: state_(SEARCH), ones_(0), buffer_(0), frame_(ioFramePool().acquire())
, bits_(0), passall_(pass_all), ready_(false)
{}

IoFrame* Decoder::operator()(bool bit, bool pll)
{
    IoFrame* result = nullptr;

    // It appears that the ioFramePool may not get initialized in the proper
    // order during some builds.
    if (nullptr == frame_) frame_ = ioFramePool().acquire();
    if (nullptr == frame_) return result;

    if (not pll) {
        if ((state_ == FRAMING) and (frame_->size() > 17) and passall_) {
            frame_->parse_fcs();
            if (passall_ or frame_->ok()) {
                result = frame_;
                ready_ = false;
                frame_ = ioFramePool().acquire();
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
                frame_ = ioFramePool().acquire();
                return result;
            }
            frame_->clear();
        }
    }

    return result;
}


}}} // mobilinkd::tnc::hdlc


