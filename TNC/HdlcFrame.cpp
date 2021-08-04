// Copyright 2015-2021 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "HdlcFrame.hpp"
#include "Log.h"
#include "main.h"
#include "cmsis_os.h"

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

IoFrame* acquire()
{
    auto result = ioFramePool().acquire();
    if (result == nullptr) CxxErrorHandler();
    return result;
}


IoFrame* acquire_wait()
{
    IoFrame* result = nullptr;
    while ((result = ioFramePool().acquire()) == nullptr) {
        osThreadYield();
    }
    return result;
}

}}} // mobilinkd::tnc::hdlc
