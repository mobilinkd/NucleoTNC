// Copyright 2015-2020 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Encoder.h"
#include "HdlcFrame.hpp"
#include "LinkSetupFrame.h"
#include "CRC16.h"
#include "Convolution.h"
#include "M17Randomizer.h"
#include "PolynomialInterleaver.h"

#include <cmsis_os.h>

#include <array>
#include <cstdint>

namespace mobilinkd
{

/**
 * M17 audio stream encoder.  This is the data link layer for M17
 * audio streams.  This is responsible for sending the preamble, sync
 * words, encoding, interleaving and randomizing the data.
 *
 * The sender is responsible for sending a link setup frame and for
 * constructing the payload portion of the audio stream (frame number,
 * Codec2-encoded audio, CRC).  The encoder is responsible for encoding
 * the LSF into LICH segments and assembling the LICH and payload into
 * an audio stream frame.
 *
 * @note: the stream encoder is actually agnostic about the payload
 *  content.  It could be V/V, V/D, or D/C mode.  It is also agnostic
 *  about the payload encoding.  It is assumed that, if encryption is
 *  used, the payload arrives encrypted.
 *
 * There is no collision avoidance used for audio streams.
 *
 * The M17 audio encoder will insert up to 2 "dead" frames (frames
 * with the payload set to all 0) before dropping the connection.
 * It will do dropped frame detection by looking at the frame number.
 * It will also do end of stream detection by looking at the frame
 * number.  If more than two frames time slots are missed, the
 * encoder assumes the transmission has ended and stops transmitting.
 *
 * It will not transmit until a new LSF is sent to establish a new
 * session.
 *
 * The uses the frame size to determine the type of frame it is
 * handling.  There are only two acceptable frame sizes:
 *
 *  - 30 byte frames are link setup frames.
 *  - 20 byte frames are data frames.
 *
 * The 20-byte data frame consists of a 16-bit frame number, 128-bit
 * payload, and 16-bit CRC, for a total of 160 bits or 20 bytes.
 *
 * If the modulator receives any other frame size, it will immediately
 * stop transmitting and signal an error condition.
 */
struct M17Encoder : public Encoder
{
    static const std::array<uint8_t, 2> LSF_SYNC;
    static const std::array<uint8_t, 2> STREAM_SYNC;
    static const std::array<uint8_t, 2> PACKET_SYNC;

    enum class State {INACTIVE, IDLE, ACTIVE};
    enum class FrameType {BASIC_PACKET, FULL_PACKET, VOICE_STREAM};

    M17Encoder(osMessageQId input);
    ~M17Encoder();
    void run() override;
    void update_settings() override;
    void updateModulator() override;
    void stop() override;
    EncoderType encoder_type() const override { return EncoderType::M17; }

private:

    using lsf_t = std::array<uint8_t, 30>;
    using lich_segment_t = std::array<uint8_t, 12>; // Golay-encoded LICH.
    using bitstream_t = std::array<uint8_t, 48>;    // M17 frame of bits (in bytes).
    using payload_t = std::array<uint8_t, 34>;      // Bytes in the payload of a data frame.
    using frame_t = std::array<uint8_t, 46>;        // M17 frame (without sync word).

    static void encoderTask(void const*);

    void process_packet(tnc::hdlc::IoFrame*, FrameType type);
    void process_stream(tnc::hdlc::IoFrame*, FrameType type);

    void send_preamble();
    void send_link_setup();
    void send_basic_packet(tnc::hdlc::IoFrame*);
    void send_full_packet(tnc::hdlc::IoFrame*);
    void send_packet_frame(const std::array<uint8_t, 26>& packet_frame);
    void send_stream(tnc::hdlc::IoFrame*, FrameType type);

    void create_link_setup(tnc::hdlc::IoFrame*, FrameType type);
    lich_segment_t make_lich_segment(std::array<uint8_t, 6> segment);

    /**
     * Do the p*persistent CSMA handling.  In order to prevent resource
     * starvation, we drop any packets delayed by more than 5 seconds.
     *
     * 0. CSMA called.
     * 1. If the channel is open
     *    1a. Pick a random number between 0-255.
     *    1b. If it less than or equal to p, transmit the packet.
     * 2. Otherwise wait slot_time * 10 ms.
     *    2.b Go to step 1.
     *
     * @pre The demodulator is running in order to detect the data carrier.
     *
     * @note For this to work, the demodulator must be left running
     *  while CSMA is taking place in order to do carrier detection.
     *
     * @return true if OK to send, otherwise CSMA has timed out and
     *  the packet should be dropped.
     */
    bool do_csma();

    /**
     * Convolutional encode N bytes of data, returning N*2 + 1 bytes
     * of encoded data (4 zero flush bits are added).  If total bits
     * are not an even byte boundary, the
     * @param data
     * @param total_bits
     * @return
     */
    template <typename T, size_t N>
    static std::array<T, N * 2 + 1> conv_encode(std::array<T, N> data,
        size_t total_bits = N * 8)
    {
        std::array<T, N * 2 + 1> result;

        uint8_t bit_index = 0;
        uint8_t byte_index = 0;
        uint8_t tmp = 0;

        uint32_t memory = 0;
        for (auto b : data)
        {
            auto bits = std::min(size_t(8), total_bits);
            for (size_t i = 0; i != bits; ++i)
            {
                uint32_t x = (b & 0x80) >> 7;
                b <<= 1;
                memory = update_memory<4>(memory, x);
                tmp = (tmp << 1) | convolve_bit(031, memory);
                tmp = (tmp << 1) | convolve_bit(027, memory);
                bit_index += 2;
                if (bit_index == 8)
                {
                    bit_index = 0;
                    result[byte_index++] = tmp;
                    tmp = 0;
                }
            }
            total_bits -= bits;
        }

        // Flush the encoder.
        for (size_t i = 0; i != 4; ++i)
        {
            memory = update_memory<4>(memory, 0);
            tmp = (tmp << 1) | convolve_bit(031, memory);
            tmp = (tmp << 1) | convolve_bit(027, memory);
            bit_index += 2;
            if (bit_index == 8)
            {
                bit_index = 0;
                result[byte_index++] = tmp;
                tmp = 0;
            }
        }

        // Frame may not end on a byte boundary.
        if (bit_index != 0)
        {
            tmp <<= (8 - bit_index);
            result[byte_index] = tmp;
        }

        return result;
    }

    osMessageQId input_queue;
    osThreadId encoderTaskHandle;
    State state = State::INACTIVE;

    frame_t punctured;
    lsf_t current_lsf;
    LinkSetupFrame::encoded_call_t src;
    LinkSetupFrame::encoded_call_t dest;
    M17ByteRandomizer<46> randomizer;
    PolynomialInterleaver<45, 92, 368> interleaver;
    CRC16<0x5935, 0xFFFF> crc;

    bool back2back = false;
};

} // mobilinkd
