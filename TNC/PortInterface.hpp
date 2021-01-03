// Copyright 2016-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "cmsis_os.h"

#include "HdlcFrame.hpp"
#include "PortInterface.h"

namespace mobilinkd { namespace tnc {

const uint32_t TX_BUFFER_SIZE = 64;     // Cannot be larger that USB_FS_MAX_PACKET_SIZE.
extern uint8_t TxBuffer[TX_BUFFER_SIZE];

/**
 * This interface defines the semi-asynchronous interface used for reading
 * and writing.  The write interface is synchronous.  The read interface
 * is asynchronous.  The call to open() starts a task that reads from port
 * and puts that data on the read_queue, one byte at a time.
 */
struct PortInterface {
    virtual ~PortInterface() {}
    virtual bool open() = 0;
    virtual bool isOpen() const = 0;
    virtual void close() = 0;
    virtual osMessageQId queue() const = 0;
    virtual bool write(const uint8_t* data, uint32_t size, uint8_t type,
        uint32_t timeout) = 0;
    virtual bool write(const uint8_t* data, uint32_t size, uint32_t timeout) = 0;
    virtual bool write(hdlc::IoFrame* frame, uint32_t timeout = osWaitForever) = 0;
};

extern PortInterface* ioport;

int write(hdlc::IoFrame* frame, uint32_t timeout = osWaitForever);

}} // mobilinkd::tnc
