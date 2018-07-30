// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include <Log.h>
#include "Kiss.hpp"
#include "KissHardware.hpp"

// extern osMessageQId hdlcOutputQueueHandle;

namespace mobilinkd { namespace tnc { namespace kiss {

void handle_frame(uint8_t frame_type, hdlc::IoFrame* frame) {

    if (frame->size() == 0) {
        hdlc::release(frame);
        return;
    }

    uint8_t value = *(frame->begin());
    switch (frame_type) {
    case kiss::FRAME_DATA:
        DEBUG("FRAME_DATA");
        // osMessagePut(hdlcOutputQueueHandle, (uint32_t) frame, 0);
        break;
    case kiss::FRAME_TX_DELAY:
        DEBUG("FRAME_TX_DELAY");
        kiss::settings().txdelay = value;
        hdlc::release(frame);
        break;
    case kiss::FRAME_P_PERSIST:
        DEBUG("FRAME_P_PERSIST");
        kiss::settings().ppersist = value;
        hdlc::release(frame);
        break;
    case kiss::FRAME_SLOT_TIME:
        DEBUG("FRAME_SLOT_TIME");
        kiss::settings().slot = value;
        hdlc::release(frame);
        break;
    case kiss::FRAME_TX_TAIL:
        DEBUG("FRAME_TX_TAIL");
        // Ignore.
        hdlc::release(frame);
        break;
    case kiss::FRAME_DUPLEX:
        DEBUG("FRAME_DUPLEX");
        kiss::settings().duplex = value;
        hdlc::release(frame);
        break;
    case kiss::FRAME_HARDWARE:
        DEBUG("FRAME_HARDWARE");
        kiss::settings().handle_request(frame);
        hdlc::release(frame);
        break;
    case kiss::FRAME_LOG:
        DEBUG("FRAME_LOG");
        hdlc::release(frame);
        // IGNORE
        break;
    case kiss::FRAME_RETURN:
        DEBUG("FRAME_RETURN");
        hdlc::release(frame);
        // Leave KISS mode
        break;
    default:
        DEBUG("Unknown KISS frame type");
        hdlc::release(frame);
    }
}

}}} // mobilinkd::tnc::kiss
