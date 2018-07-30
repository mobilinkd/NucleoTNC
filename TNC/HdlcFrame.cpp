// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "HdlcFrame.hpp"

namespace mobilinkd { namespace tnc { namespace hdlc {

FrameSegmentPool frameSegmentPool __attribute__((section(".bss2")));

IoFramePool& ioFramePool() {
    static IoFramePool pool;
    return pool;
}

void release(IoFrame* frame)
{
    ioFramePool().release(frame);
}

}}} // mobilinkd::tnc::hdlc
