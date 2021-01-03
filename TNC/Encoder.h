// Copyright 2015-2020 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

namespace mobilinkd
{

/**
 * The encoder is responsible for the data link layer (OSI layer 2 of the
 * network stack (preamble, sync words, framing, etc), and is also responsible
 * for collision avoidance (CSMA) if needed.
 */
struct Encoder
{
    enum class EncoderType { HDLC, M17, M17STREAM, M17PACKET };
    virtual ~Encoder() {}

    /**
     * Pull off complete frames and encode them, feeding the modulator.
     */
    virtual void run() = 0;
    virtual void update_settings() = 0;
    virtual void updateModulator() = 0;
    virtual void stop() = 0;
    virtual EncoderType encoder_type() const = 0;
};

} // mobilinkd
