// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__NULL_PORT_HPP_
#define MOBILINKD__TNC__NULL_PORT_HPP_

#include "PortInterface.hpp"

#include <atomic>

namespace mobilinkd { namespace tnc {

/**
 * This interface defines the semi-asynchronous interface used for reading
 * and writing
 */
struct NullPort : PortInterface
{
    virtual ~NullPort() {}
    virtual bool open()
    {
        if (open_) return open_;
        open_ = true;
        return open_;
    }

    virtual bool isOpen() const { return open_; }

    virtual void close() {
        open_ = false;
    }
    virtual osMessageQId queue() const { return 0; }
    virtual bool write(const uint8_t*, uint32_t, uint8_t, uint32_t)
    {
        return true;
    }
    virtual bool write(const uint8_t*, uint32_t, uint32_t)
    {
        return true;
    }
    virtual bool write(hdlc::IoFrame* frame, uint32_t = osWaitForever)
    {
        hdlc::release(frame);
        return true;
    }

    void init() {}

    std::atomic<bool> open_{false};
};

NullPort* getNullPort();

}} // mobilinkd::tnc

#endif // MOBILINKD__TNC__NULL_PORT_HPP_
