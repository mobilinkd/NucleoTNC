// Copyright 2015-2021 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "M17Encoder.h"
#include "HdlcFrame.hpp"
#include "Modulator.hpp"
#include "ModulatorTask.hpp"
#include "KissHardware.hpp"
#include "DCD.h"
#include "Golay24.h"
#include "Trellis.h"
#include "M17.h"
#include "AudioInput.hpp"

#include "stm32l4xx_hal.h"

#include <algorithm>
#include <array>
#include <cstdint>

static uint32_t encoderTaskBuffer[ 256 ];
static osStaticThreadDef_t encoderTaskControlBlock;
static uint8_t m17EncoderInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
static osStaticMessageQDef_t m17EncoderInputQueueControlBlock;
static uint8_t m17EncoderOutputQueueBuffer[ 3 * sizeof( uint32_t ) ];
static osStaticMessageQDef_t m17EncoderOutputQueueControlBlock;

static osMessageQId m17EncoderInputQueueHandle;
static osMessageQId m17EncoderOutputQueueHandle;

extern RNG_HandleTypeDef hrng;
extern IWDG_HandleTypeDef hiwdg;

namespace mobilinkd
{

constexpr int8_t bits_to_symbol(uint8_t bits)
{
    switch (bits)
    {
    case 0: return 1;
    case 1: return 3;
    case 2: return -1;
    case 3: return -3;
    }
    return 0;
}

M17Encoder::M17Encoder(osMessageQId input)
: input_queue(input)
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wwrite-strings"    // cmsis-os is not const-correct.

    osMessageQStaticDef(m17EncoderInputQueue, 3, uint32_t, m17EncoderInputQueueBuffer, &m17EncoderInputQueueControlBlock);
    m17EncoderInputQueueHandle = osMessageCreate(osMessageQ(m17EncoderInputQueue), NULL);
    osMessageQStaticDef(m17EncoderOutputQueue, 3, uint32_t, m17EncoderOutputQueueBuffer, &m17EncoderOutputQueueControlBlock);
    m17EncoderOutputQueueHandle = osMessageCreate(osMessageQ(m17EncoderOutputQueue), NULL);
    osThreadStaticDef(encoderTask, M17Encoder::encoderTask, osPriorityNormal,
        0, 256, encoderTaskBuffer, &encoderTaskControlBlock);

    #pragma GCC diagnostic pop

    encoderTaskHandle = osThreadCreate(osThread(encoderTask), this);
    osThreadSuspend(encoderTaskHandle);
    src.fill(0xff);
    dest.fill(0xff);
}

M17Encoder::~M17Encoder() {}

void M17Encoder::run()
{
    using tnc::hdlc::IoFrame;
    using tnc::hdlc::release;

    state = State::IDLE;

    osThreadResume(encoderTaskHandle);

    auto start = osKernelSysTick();
    int32_t delay_ms = 0;

    state = State::IDLE;

    while (state != State::INACTIVE)
    {
        osEvent evt = osMessageGet(input_queue, osWaitForever);
        if (evt.status == osEventMessage)
        {
            // Changing encoders when nullptr is received.
            if (evt.value.p == nullptr)
            {
                state = State::INACTIVE;
                break;
            }
            auto frame = static_cast<IoFrame*>(evt.value.p);
            switch (frame->source())
            {
            case 0x00: // Basic packet data
                delay_ms = ((frame->size() / 25) + 1) * 40;
                start = osKernelSysTick();
                process_packet(frame, FrameType::BASIC_PACKET);
                break;
            case 0x10: // Encapsulated packet data
                delay_ms = (frame->size() / 25) * 40;
                start = osKernelSysTick();
                process_packet(frame, FrameType::FULL_PACKET);
                break;
            case 0x20: // Voice stream
                if (back2back) delay_ms += 40;
                else delay_ms = 80 + tnc::kiss::settings().txdelay * 10;
                start = osKernelSysTick();
                process_stream(frame, FrameType::VOICE_STREAM);
                break;
            default:
                release(frame);
                // ignore it
            }

            evt = osMessagePeek(input_queue, delay_ms);
            // auto num_msgs = osMessageWaiting(input_queue);
            back2back = (evt.status == osEventMessage);
            if (!back2back)
            {
                if (state != State::IDLE)
                {
                    WARN("Timed out waiting for packet (%lums).", delay_ms);
                    size_t counter = 5;
                    do {
                        send_link_setup();
                        evt = osMessagePeek(input_queue, 40);
                    } while (evt.status != osEventMessage && --counter != 0);

                    if (evt.status != osEventMessage) {
                        state = State::IDLE;
                        WARN("Timed out waiting for packet.");
                    }
                }
                delay_ms = 0;
            }
            else
            {
                int duration = osKernelSysTick() - start;
                if (duration >= delay_ms) delay_ms = 0;
                else delay_ms -= duration;
                INFO("Slack time = %lums", delay_ms);
            }
        }
    }

    osThreadSuspend(encoderTaskHandle);
}

/**
 *
 * @param frame is the frame to transmit. Ownership has been transferred
 *  and this function is responsible for either transferring ownership
 *  downstream or releasing it.
 * @param type is whether this is a basic packet or encapsulated packet.
 */
void M17Encoder::process_packet(tnc::hdlc::IoFrame* frame, FrameType type)
{
    using namespace mobilinkd::tnc::kiss;
    using tnc::hdlc::release;

    switch (state)
    {
    case State::IDLE:
        if (!back2back) // Do CSMA and preamble?
        {
            if (!settings().duplex) // Only do CSMA in half-duplex mode.
            {
                if (!do_csma())     // Wait for channel to clear.
                {
                    release(frame);
                    WARN("Could not send frame; channel busy.");
                    return;
                }
            }

            send_preamble();
        }
        create_link_setup(frame, type);
        send_link_setup();
        if (type == FrameType::BASIC_PACKET)
            send_basic_packet(frame);
        else
            send_full_packet(frame);
        break;
    case State::ACTIVE:
        // Protocol violation.
        WARN("Protocol violation -- packet received while stream active.");
        state = State::IDLE;
        break;
    default:
        ERROR("M17 encoder bad state");
    }
    release(frame);
}

void M17Encoder::process_stream(tnc::hdlc::IoFrame* frame, FrameType type)
{
    using namespace mobilinkd::tnc::kiss;
    using tnc::hdlc::release;

    switch (state)
    {
    case State::IDLE:
        if (frame->size() == 30)    // Ignore anything but a 30-byte LSF.
        {
            // todo: check for stream frame type.
            if (!back2back) send_preamble();
            create_link_setup(frame, type);
            release(frame);
            send_link_setup();
            state = State::ACTIVE;
        } else {
            WARN("Unexpected LSF frame size = %u", frame->size());
            release(frame);
        }
        break;
    case State::ACTIVE:
        if (frame->size() == 24)
        {
            send_stream(frame, type);   // Consumes frame.
        }
        else if (frame->size() == 26)
        {
            // Old-style frame with trailing CRC.
            frame->resize(24);
            send_stream(frame, type);
        }
        else
        {
            WARN("Unexpected AUDIO frame size = %u", frame->size());
            release(frame);             // Ignore it.
        }
        break;
    case State::INACTIVE:
        release(frame);
    }
}

void M17Encoder::send_preamble()
{
    auto frame = tnc::hdlc::acquire_wait();
    for (size_t i = 0; i != 48; ++i) frame->push_back(0x77);
    auto status = osMessagePut(
        m17EncoderInputQueueHandle,
        reinterpret_cast<uint32_t>(frame),
        osWaitForever);
    if (status != osOK)
    {
        tnc::hdlc::release(frame);
        WARN("M17 failed to send preamble");
    }
}

void M17Encoder::send_link_setup()
{
    m17_frame.fill(0);
    auto frame = tnc::hdlc::acquire_wait();

    // Encoder, puncture, interleave & randomize.
    auto encoded = conv_encode(current_lsf);
    puncture_bytes(encoded, m17_frame, P1);
    interleaver.interleave(m17_frame);
    randomizer(m17_frame);
    for (auto c : m17::LSF_SYNC) frame->push_back(c);
    for (auto c : m17_frame) frame->push_back(c);

    auto status = osMessagePut(
        m17EncoderInputQueueHandle,
        reinterpret_cast<uint32_t>(frame),
        osWaitForever);
    if (status != osOK)
    {
        tnc::hdlc::release(frame);
        WARN("M17 failed to send LSF");
    }
}

void M17Encoder::send_basic_packet(tnc::hdlc::IoFrame* frame)
{
    std::array<uint8_t, 26> packet_frame;

    if (!frame->complete()) frame->add_fcs();

    uint8_t frame_number = 0;
    auto it = frame->begin();
    size_t i = 0;
    while (it != frame->end())
    {
        size_t len = std::min(frame->size() - i, size_t(25));
        auto start = it;
        std::advance(it, len);
        std::copy(start, it, packet_frame.begin());
        if (it == frame->end()) frame_number = 32 + len;    // last packet frame.
        packet_frame[25] = (frame_number << 2);             // 6 bits of last byte.
        send_packet_frame(packet_frame);
        frame_number += 1;
        i += len;
    }
}

void M17Encoder::send_full_packet(tnc::hdlc::IoFrame* frame)
{
    std::array<uint8_t, 26> packet_frame;

    if (!frame->complete()) frame->add_fcs();

    uint8_t frame_number = 0;
    auto it = frame->begin();
    std::advance(it, 30);   // First 30 bytes are link setup frame.
    size_t i = 0;
    while (it != frame->end())
    {
        size_t len = std::min(frame->size() - i, size_t(25));
        auto start = it;
        std::advance(it, len);
        std::copy(start, it, packet_frame.begin());
        if (it == frame->end()) frame_number = 32 + len;    // last packet frame.
        packet_frame[25] = (frame_number << 2);             // 6 bits of last byte.
        send_packet_frame(packet_frame);
        frame_number += 1;
        i += len;
    }
}

void M17Encoder::send_packet_frame(const std::array<uint8_t, 26>& packet_frame)
{
    // Encoder, puncture, interleave & randomize.
    auto encoded = conv_encode(packet_frame, 206);
    puncture_bytes(encoded, m17_frame, P3);
    interleaver.interleave(m17_frame);
    randomizer(m17_frame);

    auto frame = tnc::hdlc::acquire_wait();

    for (auto c : m17::PACKET_SYNC) frame->push_back(c);
    for (auto c : m17_frame) frame->push_back(c);

    auto status = osMessagePut(
        m17EncoderInputQueueHandle,
        reinterpret_cast<uint32_t>(frame),
        osWaitForever);
    if (status != osOK)
    {
        tnc::hdlc::release(frame);
        WARN("M17 failed to send packet frame");
    }
}

void M17Encoder::send_stream(tnc::hdlc::IoFrame* frame, FrameType)
{
    // Construct LICH.
    auto it = frame->begin();
    std::advance(it, 6);
    std::array<uint8_t, 6> segment; // LICH is 6 bytes, 48 bits.
    std::copy(frame->begin(), it, segment.begin());
    auto lich = make_lich_segment(segment);     // Golay encode.
    auto fit = std::copy(lich.begin(), lich.end(), m17_frame.begin());

    // Encode &  puncture
    std::array<uint8_t, 18> data;
    std::copy(it, frame->end(), data.begin());

    if (data[0] & 0x80) state = State::IDLE; // EOS

    auto encoded = conv_encode(data);
    puncture_bytes(encoded, stream_payload, P2);
    std::copy(stream_payload.begin(), stream_payload.end(), fit); // Copy after LICH.

    interleaver.interleave(m17_frame);  // Interleave entire frame.
    randomizer(m17_frame);              // Randomize entire frame.

    frame->clear();                     // Re-use existing frame.
    for (auto c : m17::STREAM_SYNC) frame->push_back(c);
    for (auto c : m17_frame) frame->push_back(c);

    auto status = osMessagePut(
        m17EncoderInputQueueHandle,
        reinterpret_cast<uint32_t>(frame),
        osWaitForever);
    if (status != osOK)
    {
        tnc::hdlc::release(frame);
        WARN("M17 failed to send stream frame");
    }
}

void M17Encoder::create_link_setup(tnc::hdlc::IoFrame* frame, FrameType type)
{
    using namespace mobilinkd::tnc::kiss;

    const LinkSetupFrame::call_t SRC = {'M','B','L','K','D','T','N','C','3',0};

    switch (type)
    {
    case FrameType::BASIC_PACKET:
        {
            current_lsf.fill(0);
            auto src = LinkSetupFrame::encode_callsign(SRC);
            std::copy(src.begin(), src.end(), current_lsf.begin());
            std::fill(current_lsf.begin() + 6, current_lsf.begin() + 12, 0xff);
            current_lsf[13] = 0x02;
            crc.reset();
            for (size_t i = 0; i != 28; ++i) crc(current_lsf[i]);
            auto checksum = crc.get_bytes();
            current_lsf[28] = checksum[0];
            current_lsf[29] = checksum[1];
        }
        break;
    case FrameType::FULL_PACKET:
        // First 30 bytes of frame are the LSF.
        {
            auto it = frame->begin();
            std::advance(it, 30);
            std::copy(frame->begin(), it, current_lsf.begin());
            break;
        }
    case FrameType::VOICE_STREAM:
        // The frame is the LSF.
        std::copy(frame->begin(), frame->end(), current_lsf.begin());
        break;
    }
}

/**
 * Encode each LSF segment into a Golay-encoded LICH segment bitstream.
 */
[[gnu::noinline]]
M17Encoder::lich_segment_t M17Encoder::make_lich_segment(
    std::array<uint8_t, 6> segment)
{
    lich_segment_t result;
    uint16_t tmp;
    uint32_t encoded;

    tmp = segment[0] << 4 | ((segment[1] >> 4) & 0x0F);
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 0; i != 24; ++i)
    {
        assign_bit_index(result, i, (encoded & (1 << 23)) != 0);
        encoded <<= 1;
    }

    tmp = ((segment[1] & 0x0F) << 8) | segment[2];
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 24; i != 48; ++i)
    {
        assign_bit_index(result, i, (encoded & (1 << 23)) != 0);
        encoded <<= 1;
    }

    tmp = segment[3] << 4 | ((segment[4] >> 4) & 0x0F);
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 48; i != 72; ++i)
    {
        assign_bit_index(result, i, (encoded & (1 << 23)) != 0);
        encoded <<= 1;
    }

    tmp = ((segment[4] & 0x0F) << 8) | segment[5];
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 72; i != 96; ++i)
    {
        assign_bit_index(result, i, (encoded & (1 << 23)) != 0);
        encoded <<= 1;
    }

    return result;
}


void M17Encoder::update_settings() {}
void M17Encoder::updateModulator() {}

void M17Encoder::stop()
{
    state = State::INACTIVE;
    if (osMessagePut(input_queue, 0, osWaitForever) != osOK)
    {
        CxxErrorHandler();
    }
}

bool M17Encoder::do_csma() {
    // Wait until we can transmit.  If we cannot transmit for 10s
    // drop the frame.

    if (!dcd()) {
        // Channel is clear... send now.
        return true;
    }

    uint16_t counter = 0;
    uint32_t random = 0;
    while (counter < 1000)
    {
        osDelay(40);    // Slot time for M17 is fixed at 40ms.

        if ((counter & 3) == 0)
        {
            auto status = HAL_RNG_GenerateRandomNumber(&hrng, &random);
            if (status != HAL_OK)
            {
                WARN("RNG failure code %d", status);
            }
        }

        if (((random & 0xFF) < 63) && !dcd()) return true;

        random >>= 8;   // Might as well use all 32 bits of randomness.

        counter += 1;
    }
    return false;
}

tnc::hdlc::IoFrame* M17Encoder::create_bert_frame() {
    static PRBS9 prbs;

    std::array<uint8_t, 25> bert_frame;

    uint8_t byte = 0;
    size_t j = 0;
    for (size_t i = 0; i != 197; ++i) {
        byte <<= 1;
        byte |= prbs.generate();
        if ((i & 7) == 7) {
            bert_frame[j] = byte;
            j += 1;
            byte = 0;
        }
    }
    byte <<= 3;
    bert_frame[j] = byte;

    // Encoder, puncture, interleave & randomize.
    auto encoded = conv_encode(bert_frame, 197);
    puncture_bytes(encoded, m17_frame, P2);
    interleaver.interleave(m17_frame);
    randomizer(m17_frame);

    auto frame = tnc::hdlc::acquire_wait();

    for (auto c : m17::BERT_SYNC) frame->push_back(c);
    for (auto c : m17_frame) frame->push_back(c);

    return frame;
}

/*
 * The encoder task is responsible for feeding symbols to the M17 modulator.
 */
void M17Encoder::encoderTask(void const*)
{
    using tnc::hdlc::IoFrame;
    using tnc::hdlc::release;

    auto& modulator = ::getModulator();

    while (true)
    {
        osEvent evt = osMessageGet(m17EncoderInputQueueHandle, osWaitForever);
        if (evt.status != osEventMessage) continue;

        HAL_IWDG_Refresh(&hiwdg);

        auto frame = static_cast<IoFrame*>(evt.value.p);
        if (frame->size() != 48)
        {
            WARN("Bad frame size %u", frame->size());
        }

        for (uint8_t c : *frame) modulator.send(c); // This takes ~40ms.

        release(frame);
    }
}

} // mobilinkd
