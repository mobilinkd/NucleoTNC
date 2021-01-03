// Copyright 2015-2020 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Encoder.h"
#include "Modulator.hpp"
#include "ModulatorTask.hpp"
#include "HdlcFrame.hpp"
#include "NRZI.hpp"
#include "PTT.hpp"
#include "GPIO.hpp"
#include "KissHardware.hpp"
#include "AudioInput.hpp"
#include "DCD.h"

#include "main.h"

#include <cmsis_os.h>

#include <cstdint>

namespace mobilinkd { namespace tnc { namespace hdlc {

using namespace mobilinkd::libafsk;

struct Encoder : public ::mobilinkd::Encoder
{

    // static constexpr uint8_t IDLE = 0x00;
    static constexpr uint8_t IDLE = 0x7E;
    static constexpr uint8_t FLAG = 0x7E;

    enum class state_type {
        STATE_IDLE,
        STATE_HEAD,
        STATE_FRAME,
        STATE_CRC_LOW,
        STATE_CRC_HIGH,
        STATE_TAIL,
    };

    uint8_t tx_delay_;
    uint8_t tx_tail_;
    uint8_t p_persist_;
    uint8_t slot_time_;
    bool duplex_;
    state_type state_;
    int ones_;
    NRZI nrzi_;
    uint16_t crc_;
    osMessageQId input_;
    Modulator* modulator_;
    volatile bool running_;
    bool send_delay_;   // Avoid sending the preamble for back-to-back frames.

    Encoder(osMessageQId input)
    : tx_delay_(kiss::settings().txdelay), tx_tail_(kiss::settings().txtail)
    , p_persist_(kiss::settings().ppersist), slot_time_(kiss::settings().slot)
    , duplex_(kiss::settings().duplex), state_(state_type::STATE_IDLE)
    , ones_(0), nrzi_(), crc_()
    , input_(input), modulator_(&getModulator())
    , running_(false), send_delay_(true)
    {}

    void run() override
    {
        running_ = true;
        send_delay_ = true;
        while (running_) {
            state_ = state_type::STATE_IDLE;
            osEvent evt = osMessageGet(input_, osWaitForever);
            if (evt.status == osEventMessage) {
                if (evt.value.p == nullptr) return;
                tx_delay_ = kiss::settings().txdelay;
                tx_tail_ = kiss::settings().txtail;
                p_persist_ = kiss::settings().ppersist;
                slot_time_ = kiss::settings().slot;
                duplex_ = kiss::settings().duplex;
                auto frame = (IoFrame*) evt.value.p;
                process(frame);
                // See if we have back-to-back frames.
                evt = osMessagePeek(input_, 0);
                if (evt.status != osEventMessage) {
                    send_raw(IDLE);
                    send_raw(IDLE);
                    send_delay_ = true;
                    if (!duplex_) {
                      osMessagePut(audioInputQueueHandle, audio::DEMODULATOR,
                        osWaitForever);
                    }
                }
            }
        }
    }

    void update_settings() override
    {
        using namespace mobilinkd::tnc::kiss;

        tx_delay(settings().txdelay);
        p_persist(settings().ppersist);
        slot_time(settings().slot);
        tx_tail(settings().txtail);
    }

    EncoderType encoder_type() const override { return EncoderType::HDLC; }

    int tx_delay() const { return tx_delay_; }
    void tx_delay(int ms) { tx_delay_ = ms; }

    int tx_tail() const { return tx_tail_; }
    void tx_tail(int ms) { tx_tail_ = ms; }

    int slot_time() const { return slot_time_; }
    void slot_time(int value) { slot_time_ = value; }

    int p_persist() const { return p_persist_; }
    void p_persist(int value) { p_persist_ = value; }

    void updateModulator() override
    {
        modulator_ = &(getModulator());
    }

    state_type status() const {return state_; }
    void stop() override { running_ = false; }

    int rng_() const {return osKernelSysTick() & 0xFF;}

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
     * In general, a p*persistent CSMA protocol should be adaptive.  The
     * p value should be dynamically computed based on network load.  In
     * practice, this is rather difficult to do with APRS because there
     * is no easy way to measure the collision rate.
     *
     * For APRS digipeaters, the slot_time and p values should be 0 and 255,
     * respectively.  This is equivalent to 1-persistent CSMA.
     *
     * @pre The demodulator is running in order to detect the data carrier.
     *
     * @note For this to work, the demodulator must be left running
     *  while CSMA is taking place in order to do carrier detection.
     *
     * @return true if OK to send, otherwise CSMA has timed out and
     *  the packet should be dropped.
     */
    bool do_csma() {
        // Wait until we can transmit.  If we cannot transmit for 10s
        // drop the frame.

        if (!dcd()) {
            // Channel is clear... send now.
            return true;
        }

        uint16_t counter = 0;
        while (counter < 1000) {
            osDelay(slot_time_ * 10);    // We count on minimum delay = 1.
            counter += slot_time_;

            if (rng_() < p_persist_) {
                if (!dcd()) {
                    // Channel is clear... send now.
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Send the frame.  If send_delay_ is set, we are sending the first
     * of potentially multiple frames.  We must do two things in this case:
     * wait for the channel to clear and send the TX delay preamble.
     * Otherwise, we are sending a follow-on frame and can skip the
     * CSMA and preamble.  The channel is ours and the begin/end frame
     * bytes are enough to delimit the frames.
     *
     * If CSMA fails (times out), the frame is dropped and the send_delay_
     * flag is not cleared.  This will cause CSMA and TX delay to be
     * attempted on the next frame.
     *
     * @pre either send_delay_ is false or the demodulator is running.  We
     *  expect that send_delay_ is false only when we have back-to-back
     *  packets.
     *
     * @param frame
     */
    void process(IoFrame* frame) {
        ones_ = 0;      // Reset the ones count for each frame.

        frame->add_fcs();

        if (send_delay_) {
            if (not do_csma()) {
                release(frame);
                return;
            }
            if (!duplex_) {
                osMessagePut(audioInputQueueHandle, audio::IDLE, osWaitForever);
            }
            send_delay();
            send_delay_ = false;
        } else {
            send_raw(FLAG);
        }

        for (auto c : *frame) send(c);
        release(frame);
        send_tail();
    }

    void send_delay() {
        const size_t tmp = tx_delay_ * 1.25 * modulator_->bits_per_ms();

        INFO("Sending %u IDLE bytes", tmp);
        for (size_t i = 0; i != tmp; i++) {
            send_raw(IDLE);
        }
        send_raw(FLAG);
    }

    void send_fcs(uint16_t fcs) {
        uint8_t low = fcs & 0xFF;
        uint8_t high = (fcs >> 8) & 0xFF;

        send(low);
        send(high);
    }

    void send_tail() {
        send_raw(FLAG);
    }

    // No bit stuffing for PREAMBLE and TAIL
    void send_raw(uint8_t byte) {
        for (size_t i = 0; i != 8; i++) {
            uint8_t bit = byte & 1;
            modulator_->send(nrzi_.encode(bit));
            byte >>= 1;
        }
    }

    void send(uint8_t byte) {
        for (size_t i = 0; i != 8; i++) {
            uint8_t bit = byte & 1;
            modulator_->send(nrzi_.encode(bit));
            if (bit) {
                ++ones_;
                if (ones_ == 5) {
                    modulator_->send(nrzi_.encode(0));
                    ones_ = 0;
                }
            } else {
                ones_ = 0;
            }
            byte >>= 1;
        }
    }
};

}}} // mobilinkd::tnc::hdlc
