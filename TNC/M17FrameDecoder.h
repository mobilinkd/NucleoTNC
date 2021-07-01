// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "M17Randomizer.h"
#include "PolynomialInterleaver.h"
#include "Trellis.h"
#include "Viterbi.h"
#include "CRC16.h"
#include "LinkSetupFrame.h"
#include "HdlcFrame.hpp"
#include "Golay24.h"

#include <algorithm>
#include <array>
#include <functional>

namespace mobilinkd
{

namespace detail
{

template <typename T, size_t N>
tnc::hdlc::IoFrame* to_frame(std::array<T, N> in)
{
    auto frame = tnc::hdlc::acquire_wait();

    uint8_t out = 0;
    size_t b = 0;

    for (auto c : in)
    {
        out = (out << 1) | c;
        if (++b == 8)
        {
            frame->push_back(out);
            out = 0;
            b = 0;
        }
    }

    return frame;
}

template <typename T, size_t N>
void to_frame(tnc::hdlc::IoFrame* frame, std::array<T, N> in)
{
    uint8_t out = 0;
    size_t b = 0;

    for (auto c : in)
    {
        out = (out << 1) | c;
        if (++b == 8)
        {
            frame->push_back(out);
            out = 0;
            b = 0;
        }
    }

    if (b) // Not a full byte boundary.
    {
        while (b++ != 8) out <<= 1;
        frame->push_back(out);
    }
}

template <typename T, size_t M, size_t N>
void to_bytes(const std::array<T, M>& in, std::array<uint8_t, N>& out)
{
    uint8_t output = 0;
    size_t b = 0;
    size_t index = 0;

    for (auto c : in)
    {
        output = (output << 1) | c;
        if (++b == 8)
        {
            out[index++] = output;
            output = 0;
            b = 0;
        }
    }

    if (b) // Not a full byte boundary.
    {
        while (b++ != 8) output <<= 1;
        out[index++] = output;
    }
}

} // detail

template <typename C, size_t N>
void dump(const std::array<C,N>& data, char header = 'D')
{
    ITM_SendChar(header);
    ITM_SendChar('=');
    for (auto c : data)
    {
        const char hex[] = "0123456789ABCDEF";
        ITM_SendChar(hex[uint8_t(c)>>4]);
        ITM_SendChar(hex[uint8_t(c)&0xf]);
    }
    ITM_SendChar('\r');
    ITM_SendChar('\n');
}

/**
 * Decode M17 frames.  The decoder uses the sync word to determine frame
 * type and to update its state machine.
 *
 * The decoder receives M17 frame type indicator (based on sync word) and
 * frames from the M17 demodulator.
 *
 * If the frame is
 *
 *  - LSF if the CRC is bad or a valid LSF is not available.
 *  - STREAM if the LSF type field indicates Stream.
 *  - BASIC_PACKET if the LSF type field indicates Packet and the packet
 *    type is RAW.
 *  - FULL_PACKET if the LSF type field indicates Packet and the packet
 *    type is ENCAPSULATED or RESERVED.
 *
 * When in LSF mode, if an LSF frame is received it is parsed as an LSF.
 * When a STREAM frame is received, it attempts to recover an LSF from
 * the LICH.  PACKET frame types are ignored.
 *
 * When in STREAM mode, the state machine can transition to either:
 *
 *  - STREAM when a any stream frame is received.
 *  - LSF when the EOS indicator is set, or when a packet frame is received.
 *
 * When in BASIC_PACKET mode, the state machine can transition to either:
 *
 *  - BASIC_PACKET when any packet frame is received.
 *  - LSF when the EOS indicator is set, or when a stream frame is received.
 *
 * When in FULL_PACKET mode, the state machine can transition to either:
 *
 *  - FULL_PACKET when any packet frame is received.
 *  - LSF when the EOS indicator is set, or when a stream frame is received.
 */
struct M17FrameDecoder
{
    M17Randomizer<368> derandomize_;
    PolynomialInterleaver<45, 92, 368> interleaver_;
    Trellis<4,2> trellis_{makeTrellis<4, 2>({031,027})};
    Viterbi<decltype(trellis_), 4> viterbi_{trellis_};
    CRC16<0x5935, 0xFFFF> crc_;


    enum class State {LSF, STREAM, BASIC_PACKET, FULL_PACKET};
    enum class SyncWordType { LSF, STREAM, PACKET, RESERVED };
    enum class DecodeResult { FAIL, OK, EOS, INCOMPLETE };

    State state_ = State::LSF;

    using buffer_t = std::array<int8_t, 368>;

    using lsf_conv_buffer_t = std::array<uint8_t, 46>;
    using lsf_buffer_t = std::array<uint8_t, 30>;

    using audio_conv_buffer_t = std::array<uint8_t, 34>;
    using audio_buffer_t = std::array<uint8_t, 18>;

    using link_setup_callback_t = std::function<void(audio_buffer_t)>;
    using audio_callback_t = std::function<void(audio_buffer_t)>;
    
    link_setup_callback_t link_setup_callback_;
    audio_callback_t audio_callback_;
    union
    {
        std::array<uint8_t, 30> lich;
        std::array<uint8_t, 240> lsf;
        std::array<uint8_t, 206> packet;
        std::array<uint8_t, 144> stream;
    } output;

    union {
        std::array<int8_t, 488> lsf;
        std::array<int8_t, 420> packet;
        std::array<int8_t, 272> stream;
        std::array<uint8_t, 6> lich;
    } tmp;

    lsf_buffer_t current_lsf;

    uint8_t lich_segments{0};       ///< one bit per received LICH fragment.
    tnc::hdlc::IoFrame* current_packet = nullptr;
    uint8_t packet_frame_counter = 0;
    bool passall_ = false;


    M17FrameDecoder(
        link_setup_callback_t link_setup_callback = link_setup_callback_t(),
        audio_callback_t audio_callback = audio_callback_t()
    )
    : link_setup_callback_(link_setup_callback)
    , audio_callback_(audio_callback)
    {}

    ~M17FrameDecoder()
    {}

    State state() const { return state_; }

    void reset() { state_ = State::LSF; }

    void passall(bool enabled) { passall_ = enabled; }

    void update_state()
    {
        if (output.lsf[111]) // LSF type bit 0
        {
            INFO("LSF for stream");
            state_ = State::STREAM;
        }
        else    // packet frame comes next.
        {
            uint8_t packet_type = (output.lsf[109] << 1) | output.lsf[110];

            if (current_packet)
            {
                WARN("Incomplete packet found");
                current_packet->clear();
            }
            else
            {
                current_packet = tnc::hdlc::acquire_wait();
            }
            packet_frame_counter = 0;

            switch (packet_type)
            {
            case 1: // RAW -- ignore LSF.
                INFO("LSF for raw packet");
                state_ = State::BASIC_PACKET;
                break;
            case 2: // ENCAPSULATED
                INFO("LSF for encapsulated packet");
                state_ = State::FULL_PACKET;
                packet_frame_counter = 0;
                detail::to_frame(current_packet, output.lsf);
                break;
            default:
                WARN("LSF for reserved packet type");
                state_ = State::FULL_PACKET;
                packet_frame_counter = 0;
                detail::to_frame(current_packet, output.lsf);
            }
        }
    }

    /**
     * Decode the LSF and, if it is valid, transition to the next state.
     *
     * The LSF is returned for STREAM mode, dropped for BASIC_PACKET mode,
     * and captured for FULL_PACKET mode.
     *
     * @param buffer
     * @param lsf
     * @param ber
     * @return
     */
    [[gnu::noinline]]
    DecodeResult decode_lsf(buffer_t& buffer, tnc::hdlc::IoFrame*& lsf, int& ber)
    {
        depuncture(buffer, tmp.lsf, P1);
        ber = viterbi_.decode(tmp.lsf, output.lsf);
        ber = ber > 60 ? ber - 60 : 0;
        detail::to_bytes(output.lsf, current_lsf);
        crc_.reset();
        for (auto c : current_lsf) crc_(c);
        auto checksum = crc_.get();
        INFO("LSF crc = %04x", checksum);
#ifdef KISS_LOGGING
        dump(current_lsf);
#endif

        if (checksum == 0)
        {
            update_state();
            if (state_ == State::STREAM)
            {
                lsf = tnc::hdlc::acquire_wait();
                for (auto c : current_lsf) lsf->push_back(c);
                lsf->push_back(0);
                lsf->push_back(0);
                lsf->source(0x20);
                return DecodeResult::OK;
            }
            lsf = nullptr;
            return DecodeResult::OK;
        }
        else
        {
            lich_segments = 0;
            output.lsf.fill(0);
            return DecodeResult::FAIL;
        }
    }

    // Unpack  & decode LICH fragments into tmp_buffer.
    bool unpack_lich(buffer_t& buffer)
    {
        size_t index = 0;
        // Read the 4 24-bit codewords from LICH
        for (size_t i = 0; i != 4; ++i) // for each codeword
        {
            uint32_t codeword = 0;
            for (size_t j = 0; j != 24; ++j) // for each bit in codeword
            {
                codeword <<= 1;
                codeword |= (buffer[i * 24 + j] > 0);
            }
            uint32_t decoded = 0;
            if (!Golay24::decode(codeword, decoded))
            {
                INFO("Golay decode failed for %08lx (%du)", codeword, i);
                return false;
            }
            decoded >>= 12; // Remove check bits and parity.
            TNC_DEBUG("Golay decode good for %08lx (%du)", decoded, i);
            // append codeword.
            if (i & 1)
            {
                tmp.lich[index++] |= (decoded >> 8);     // upper 4 bits
                tmp.lich[index++] = (decoded & 0xFF);    // lower 8 bits
            }
            else
            {
                tmp.lich[index++] |= (decoded >> 4);     // upper 8 bits
                tmp.lich[index] = (decoded & 0x0F) << 4; // lower 4 bits
            }
        }
        return true;
    }

    [[gnu::noinline]]
    DecodeResult decode_lich(buffer_t& buffer, tnc::hdlc::IoFrame*& lsf, int& ber)
    {
        tmp.lich.fill(0);
        // Read the 4 12-bit codewords from LICH into buffers.lich.
        if (!unpack_lich(buffer)) return DecodeResult::FAIL;

        uint8_t fragment_number = tmp.lich[5];   // Get fragment number.
        fragment_number = (fragment_number >> 5) & 7;

        // Copy decoded LICH to superframe buffer.
        std::copy(tmp.lich.begin(), tmp.lich.begin() + 5,
            output.lich.begin() + (fragment_number * 5));

        lich_segments |= (1 << fragment_number);        // Indicate segment received.
        INFO("got segment %d, have %02x", int(fragment_number), int(lich_segments));
        if (lich_segments != 0x3F) return DecodeResult::INCOMPLETE;        // More to go...

        crc_.reset();
        for (auto c : output.lich) crc_(c);
        auto checksum = crc_.get();
        INFO("LICH crc = %04x", checksum);
        if (checksum == 0)
        {
        	lich_segments = 0;
            state_ = State::STREAM;
            lsf = tnc::hdlc::acquire_wait();
            for (auto c : output.lich) lsf->push_back(c);
            lsf->push_back(0);
            lsf->push_back(0);
            lsf->source(0x20);
            ber = 0;
            dump(output.lich);
            return DecodeResult::OK;
        }
#ifdef KISS_LOGGING
        dump(output.lich);
#endif
        // Failed CRC... try again.
        lich_segments = 0;
        output.lich.fill(0);
        ber = 128;
        return DecodeResult::INCOMPLETE;
    }

    [[gnu::noinline]]
    DecodeResult decode_stream(buffer_t& buffer, tnc::hdlc::IoFrame*& stream, int& ber)
    {
        std::array<uint8_t, 18> stream_segment;
        DecodeResult result = DecodeResult::OK;

        unpack_lich(buffer);

        stream = tnc::hdlc::acquire_wait();
        for (auto c : tmp.lich) stream->push_back(c);

        std::copy(buffer.begin() + 96, buffer.end(), tmp.stream.begin());
        auto dp = depunctured<296>(P2, tmp.stream);
        ber = viterbi_.decode(dp, output.stream);
        detail::to_frame(stream, output.stream);
        detail::to_bytes(output.packet, stream_segment);

        if ((ber < 70) && (stream_segment[0] & 0x80))
        {
            INFO("EOS");
            state_ = State::LSF;
            result = DecodeResult::EOS;
        }
        stream->push_back(0);
        stream->push_back(0);
        stream->source(0x20);
        return result;
    }

    /**
     * Capture packet frames until an EOF bit is found.  The raw packet is
     * returned without the checksum.
     *
     * @pre current_packet is not null.

     * @param buffer the demodulated M17 symbols in LLR format.
     * @param packet a pointer to the decoded packet.
     * @param ber the estimated BER (really more SNR) of the packet.
     * @return true if a valid packet is returned, otherwise false.
     */
    [[gnu::noinline]]
    DecodeResult decode_basic_packet(buffer_t& buffer, tnc::hdlc::IoFrame*& packet, int& ber)
    {
        std::array<uint8_t, 26> packet_segment;

        depuncture(buffer, tmp.packet, P3);
        ber = viterbi_.decode(tmp.packet, output.packet);
        INFO("Raw BER = %u", ber);
        ber = ber > 26 ? ber - 26 : 0;
        detail::to_bytes(output.packet, packet_segment);

#ifdef KISS_LOGGING
        dump(packet_segment, 'P');
#endif

        if (packet_segment[25] & 0x80) // last frame of packet.
        {
            size_t packet_size = (packet_segment[25] & 0x7F) >> 2;
            packet_size = std::min(packet_size, size_t(25));
            for (size_t i = 0; i != packet_size; ++i)
            {
                current_packet->push_back(packet_segment[i]);
            }
            packet_frame_counter = 0;
            state_ = State::LSF;
            // Check CRC but drop it.
            current_packet->parse_fcs();
            if (current_packet->ok())
            {
                current_packet->source(0);
                packet = current_packet;
                current_packet = nullptr;
                return DecodeResult::OK;
            }
            WARN("packet bad fcs = %04x, crc = %04x", current_packet->fcs(), current_packet->crc());
            if (passall_)
            {
                packet = current_packet;
                current_packet = nullptr;
            }
            else
            {
                tnc::hdlc::release(current_packet);
                current_packet = nullptr;
            }
            return DecodeResult::FAIL;
        }

        size_t frame_number = (packet_segment[25] & 0x7F) >> 2;
        if (frame_number != packet_frame_counter++)
        {
            WARN("Packet frame sequence error");
        }

        for (size_t i = 0; i != 25; ++i)
        {
            current_packet->push_back(packet_segment[i]);
        }

        packet = nullptr;
        return DecodeResult::OK;
    }

    /**
     * Decode full packet types.  The packet is returned without checking
     * the CRC.
     *
     * @pre current_packet is not null and contains a link setup frame.
     *
     * @param buffer
     * @param packet
     * @param ber
     * @return
     */
    [[gnu::noinline]]
    DecodeResult decode_full_packet(buffer_t& buffer, tnc::hdlc::IoFrame*& packet, int& ber)
    {
        std::array<uint8_t, 26> packet_segment;

        depuncture(buffer, tmp.packet, P3);
        ber = viterbi_.decode(tmp.packet, output.packet);
        INFO("Raw BER = %u", ber);
        ber = ber > 26 ? ber - 26 : 0;
        detail::to_bytes(output.packet, packet_segment);

#ifdef KISS_LOGGING
        dump(packet_segment, 'P');
#endif

        if (packet_segment[25] & 0x80) // last packet;
        {
            size_t packet_size = (packet_segment[25] & 0x7F) >> 2;
            packet_size = std::min(packet_size, size_t(25));
            for (size_t i = 0; i != packet_size; ++i)
            {
                current_packet->push_back(packet_segment[i]);
            }
            current_packet->push_back(0);
            current_packet->push_back(0);
            current_packet->source(0x10);
            packet = current_packet;
            current_packet = nullptr;
            packet_frame_counter = 0;
            state_ = State::LSF;
            return DecodeResult::OK;
        }

        size_t frame_number = (packet_segment[25] & 0x7F) >> 2;
        if (frame_number != packet_frame_counter++)
        {
            WARN("Packet frame sequence error");
        }

        for (size_t i = 0; i != 25; ++i)
        {
            current_packet->push_back(packet_segment[i]);
        }

        packet = nullptr;
        return DecodeResult::OK;
    }

    /**
     * Decode M17 frames.  The decoder uses the sync word to determine frame
     * type and to update its state machine.
     *
     * The decoder receives M17 frame type indicator (based on sync word) and
     * frames from the M17 demodulator.
     *
     * If the frame is an LSF, the state immediately changes to LSF. When
     * in LSF mode, the state machine can transition to:
     *
     *  - LSF if the CRC is bad.
     *  - STREAM if the LSF type field indicates Stream.
     *  - BASIC_PACKET if the LSF type field indicates Packet and the packet
     *    type is RAW.
     *  - FULL_PACKET if the LSF type field indicates Packet and the packet
     *    type is ENCAPSULATED or RESERVED.
     *
     * When in LSF mode, if an LSF frame is received it is parsed as an LSF.
     * When a STREAM frame is received, it attempts to recover an LSF from
     * the LICH.  PACKET frame types are ignored when state is LSF.
     *
     * When in STREAM mode, the state machine can transition to either:
     *
     *  - STREAM when a any stream frame is received.
     *  - LSF when the EOS indicator is set, or when a packet frame is received.
     *
     * When in BASIC_PACKET mode, the state machine can transition to either:
     *
     *  - BASIC_PACKET when any packet frame is received.
     *  - LSF when the EOS indicator is set, or when a stream frame is received.
     *
     * When in FULL_PACKET mode, the state machine can transition to either:
     *
     *  - FULL_PACKET when any packet frame is received.
     *  - LSF when the EOS indicator is set, or when a stream frame is received.
     */
    [[gnu::noinline]]
    DecodeResult operator()(SyncWordType frame_type, buffer_t& buffer,
        tnc::hdlc::IoFrame*& result, int& ber)
    {
        derandomize_(buffer);
        interleaver_.deinterleave(buffer);

        // This is out state machined.
        switch(frame_type)
        {
        case SyncWordType::LSF:
            state_ = State::LSF;
            return decode_lsf(buffer, result, ber);
        case SyncWordType::STREAM:
            switch (state_)
            {
            case State::LSF:
                return decode_lich(buffer, result, ber);
            case State::STREAM:
                return decode_stream(buffer, result, ber);
            default:
                state_ = State::LSF;
            }
            break;
        case SyncWordType::PACKET:
            switch (state_)
            {
            case State::BASIC_PACKET:
                return decode_basic_packet(buffer, result, ber);
            case State::FULL_PACKET:
                return decode_full_packet(buffer, result, ber);
            default:
                state_ = State::LSF;
            }
            break;
        case SyncWordType::RESERVED:
            state_ = State::LSF;
            break;
        }

        return DecodeResult::FAIL;
    }
};

} // mobilinkd
