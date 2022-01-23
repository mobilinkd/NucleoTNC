// Copyright 2020-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

#include "AudioLevel.hpp"
#include "M17Demodulator.h"
#include "Util.h"

#include "main.h"

#include "stm32l4xx_hal.h"
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <cstdint>

namespace mobilinkd { namespace tnc {

//m17::Indicator lsf_indicator{GPIOB, GPIO_PIN_0};
//m17::Indicator dcd_indicator{GPIOA, GPIO_PIN_2};
//m17::Indicator str_indicator{GPIOA, GPIO_PIN_7};

static float scale = 1.f / 2560.f;

static float dc_block(float x)
{
#if 1
    return x;
#else
    static float xm1 = 0.0;
    static float ym1 = 0.0;

    float y = x - xm1 + 0.9999 * ym1;
    xm1 = x;
    ym1 = y;

    return y;
#endif
}

void M17Demodulator::start()
{
    SysClock72();
#if defined(HAVE_LSCO)
    HAL_RCCEx_DisableLSCO();
#endif

    demod_filter.init(m17::rrc_taps_f);
    passall(kiss::settings().options & KISS_OPTION_PASSALL);
    polarity = kiss::settings().rx_rev_polarity() ? -1 : 1;
    scale = 1.f / 2560.f * polarity;
    // audio::virtual_ground = (VREF + 1) / 2;

    hadc1.Init.OversamplingMode = ENABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        CxxErrorHandler();
    }

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = AUDIO_IN;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        CxxErrorHandler();

    startADC(1499, ADC_BLOCK_SIZE);
//    getModulator().start_loopback();
    dcd_off();
}

void M17Demodulator::update_values(uint8_t index)
{
    correlator.apply([this,index](float t){dev.sample(t);}, index);
    dev.update();
    idev = dev.idev();
    sync_sample_index = index;
}

void M17Demodulator::dcd_on()
{
    // Data carrier newly detected.
    INFO("dcd = %d", int(dcd.level() * 1000));
    dcd_ = true;
    sync_count = 0;
    dev.reset();
    framer.reset();
    decoder.reset();
    missing_sync_count = 0;
}

void M17Demodulator::dcd_off()
{
    // Just lost data carrier.
    INFO("dcd = %d", int(dcd.level() * 1000));
    dcd_ = false;
    demodState = DemodState::UNLOCKED;
    adc_timing_adjust = 0;
}

void M17Demodulator::initialize(const q15_t* input)
{
    for (size_t i = 0; i != ADC_BLOCK_SIZE; i++) {
        demod_buffer[i] = float(input[i]) * scale;
    }

    auto filtered = demod_filter(demod_buffer.data());
    for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
    {
        auto filtered_sample = dc_block(filtered[i]);
        correlator.sample(filtered_sample);
    }
}

void M17Demodulator::update_dcd(const q15_t* input)
{
    static constexpr float inv = 1.0 / 16384.0;

    if (!dcd_ && dcd.dcd())
    {
        dcd_on();
        need_clock_reset_ = true;
    }
    else if (dcd_ && !dcd.dcd())
    {
        dcd_off();
    }

    for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
    {
        dcd(input[i] * inv);
    }
}

[[gnu::noinline]]
void M17Demodulator::do_unlocked()
{
    // We expect to find the preamble immediately after DCD.
    if (missing_sync_count < 1920)
    {
        missing_sync_count += 1;
        auto sync_index = preamble_sync(correlator);
        auto sync_updated = preamble_sync.updated();
        if (sync_updated)
        {
            sync_count = 0;
            missing_sync_count = 0;
            need_clock_reset_ = true;
            dev.reset();
            update_values(sync_index);
            sample_index = sync_index;
            demodState = DemodState::LSF_SYNC;
            INFO("P sync %d", sync_index);
        }
        return;
    }
    // Otherwise we start searching for a sync word.
    auto sync_index = lsf_sync(correlator);
    auto sync_updated = lsf_sync.updated();
    if (sync_updated)
    {
        sync_count = 0;
        missing_sync_count = 0;
        need_clock_reset_ = true;
        dev.reset();
        update_values(sync_index);
        sample_index = sync_index;
        demodState = DemodState::FRAME;
        if (sync_updated < 0)
        {
            sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
            INFO("S sync %d", int(sync_index));
        }
        else
        {
            sync_word_type = M17FrameDecoder::SyncWordType::LSF;
            INFO("L sync %d", int(sync_index));
        }
    }
    sync_index = packet_sync(correlator);
    sync_updated = packet_sync.updated();
    if (sync_updated < 0)
    {
        sync_count = 0;
        missing_sync_count = 0;
        need_clock_reset_ = true;
        dev.reset();
        update_values(sync_index);
        sample_index = sync_index;
        demodState = DemodState::FRAME;
        sync_word_type = M17FrameDecoder::SyncWordType::BERT;
        INFO("B sync %d", int(sync_index));
    }
}

/**
 * Check for LSF sync word.  We only enter the DemodState::LSF_SYNC state
 * if a preamble sync has been detected, which also means that sample_index
 * has been initialized to a sane value for the baseband.
 */
[[gnu::noinline]]
void M17Demodulator::do_lsf_sync()
{
    float sync_triggered = 0.;
    float bert_triggered = 0.;

    if (correlator.index() == sample_index)
    {
        sync_triggered = preamble_sync.triggered(correlator);
        // INFO("PSync = %d", int(sync_triggered));
        if (sync_triggered > 0.1)
        {
            sync_count += 1;
            return;
        }
        sync_triggered = lsf_sync.triggered(correlator);
        bert_triggered = packet_sync.triggered(correlator);
        if (bert_triggered < 0)
        {
            missing_sync_count = 0;
            need_clock_update_ = true;
            update_values(sample_index);
            demodState = DemodState::FRAME;
            sync_word_type = M17FrameDecoder::SyncWordType::BERT;
            INFO("+b sync");
        }
        else if (std::abs(sync_triggered) > 0.1)
        {
            missing_sync_count = 0;
            need_clock_update_ = true;
            update_values(sample_index);
            INFO("LSync = %d", int(sync_triggered));
            if (sync_triggered > 0)
            {
                demodState = DemodState::FRAME;
                sync_word_type = M17FrameDecoder::SyncWordType::LSF;
                INFO("+l sync %d", int(sample_index));
            }
            else
            {
                demodState = DemodState::FRAME;
                sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
                INFO("+s sync %d", int(sample_index));
            }
        }
        else if (++missing_sync_count > 192)
        {
            if (sync_count >= 10) {
                missing_sync_count = 0;
                need_clock_update_ = true;
                INFO("long preamble");
            } else {
                sync_count = 0;
                demodState = DemodState::UNLOCKED;
                dcd.unlock();
                INFO("l unlock %d", int(missing_sync_count));
                missing_sync_count = 0;
            }
        }
        else
        {
            update_values(sample_index);
        }
    }
}

/**
 * Check for a stream sync word (LSF sync word that is maximally negative).
 * We can enter DemodState::STREAM_SYNC from either a valid LSF decode for
 * an audio stream, or from a stream frame decode.
 *
 */
[[gnu::noinline]]
void M17Demodulator::do_stream_sync()
{
    static bool eot_flag = false;

    sync_count += 1;
    if (sync_count < MIN_SYNC_INDEX) {
        return;
    }

    if (eot_sync.triggered(correlator) > 0.1) {
        sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
        demodState = DemodState::FRAME;
        eot_flag = true;
        INFO("EOT");
        return;
    }

    uint8_t sync_index = lsf_sync(correlator);
    int8_t sync_updated = lsf_sync.updated();
    if (sync_updated < 0)
    {
        missing_sync_count = 0;
        update_values(sync_index);
        sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
        demodState = DemodState::FRAME;
        INFO("s sync %d", int(sync_count));
        eot_flag = false;
    }
    else if (sync_count > MAX_SYNC_INDEX)
    {
        // update_values(sync_index);
        if (ber >= 0 && ber < 80)
        {
            // Sync word missed but we are still decoding a stream reasonably well.
            sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
            demodState = DemodState::FRAME;
            if (!missing_sync_count) missing_sync_count = 1;
            INFO("s bunsync");
        }
        else if (eot_flag) {
            demodState = DemodState::UNLOCKED;
            dcd.unlock();
            INFO("s eot");
        }
        else if (missing_sync_count < MAX_MISSING_SYNC)
        {
            missing_sync_count += 1;
            sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
            demodState = DemodState::FRAME;
            INFO("s unsync %d", int(missing_sync_count));
        }
        else
        {
            demodState = DemodState::UNLOCKED;
            dcd.unlock();
            INFO("s unlock");
        }
        eot_flag = false;
    }
}

/**
 * Check for a packet sync word.  DemodState::PACKET_SYNC can only be
 * entered from a valid LSF frame decode with the data/packet type bit set.
 */
[[gnu::noinline]]
void M17Demodulator::do_packet_sync()
{
    sync_count += 1;
    if (sync_count < MIN_SYNC_INDEX) {
        return;
    }

    auto sync_index = packet_sync(correlator);
    auto sync_updated = packet_sync.updated();

    if (sync_updated)
    {
        missing_sync_count = 0;
        update_values(sync_index);
        sync_word_type = M17FrameDecoder::SyncWordType::PACKET;
        demodState = DemodState::FRAME;
        INFO("k sync");
        return;
    }
    else if (sync_count > MAX_SYNC_INDEX)
    {
        missing_sync_count += 1;
        if (missing_sync_count < MAX_MISSING_SYNC)
        {
            sync_word_type = M17FrameDecoder::SyncWordType::PACKET;
            demodState = DemodState::FRAME;
            INFO("k unsync");
        }
        else if (ber >= 0 && ber < 80)
        {
            sync_word_type = M17FrameDecoder::SyncWordType::PACKET;
            demodState = DemodState::FRAME;
            INFO("k bunsync");
        }
        else
        {
            demodState = DemodState::UNLOCKED;
            dcd.unlock();
            INFO("k unlock");
        }
    }
}

/**
 * Check for a bert sync word.
 */
[[gnu::noinline]]
void M17Demodulator::do_bert_sync()
{
    sync_count += 1;
    if (sync_count < MIN_SYNC_INDEX) {
        return;
    }

    auto sync_index = packet_sync(correlator);
    auto sync_updated = packet_sync.updated();

    if (sync_updated < 0)
    {
        missing_sync_count = 0;
        update_values(sync_index);
        sync_word_type = M17FrameDecoder::SyncWordType::BERT;
        INFO("b sync");
        demodState = DemodState::FRAME;
    }
    else if (sync_count > MAX_SYNC_INDEX)
    {
        missing_sync_count += 1;
        if (missing_sync_count < MAX_MISSING_SYNC)
        {
            sync_word_type = M17FrameDecoder::SyncWordType::BERT;
            demodState = DemodState::FRAME;
            INFO("b unsync");
        }
        else if (ber >= 0 && ber < 80)
        {
            sync_word_type = M17FrameDecoder::SyncWordType::BERT;
            demodState = DemodState::FRAME;
            INFO("b bunsync");
        }
        else
        {
            demodState = DemodState::UNLOCKED;
            dcd.unlock();
            INFO("b unlock");
        }
    }
}

[[gnu::noinline]]
void M17Demodulator::do_frame(float filtered_sample, hdlc::IoFrame*& frame_result)
{
    if (correlator.index() != sample_index) return;

    float sample = filtered_sample - dev.offset();
    sample *= idev;

    auto n = mobilinkd::llr<float, 4>(sample);
    int8_t* tmp;
    auto len = framer(n, &tmp);
    if (len != 0)
    {
        need_clock_update_ = true;

        std::copy(tmp, tmp + len, buffer.begin());
        auto valid = decoder(sync_word_type, buffer, frame_result, ber);
        INFO("demod: %d, dt: %7d, evma: %5d, dev: %5d, freq: %5d, dcd: %d, index: %d, %d ber: %d",
            int(decoder.state()), int(clock_recovery.clock_estimate() * 1000000),
            int(dev.error() * 1000), int(dev.deviation() * 1000),
            int(dev.offset() * 1000), int(dcd.level() * 1000),
            int(clock_recovery.sample_index()), int(sync_sample_index),
            ber);

        switch (decoder.state())
        {
        case M17FrameDecoder::State::STREAM:
            demodState = DemodState::STREAM_SYNC;
            break;
        case M17FrameDecoder::State::LSF:
            // If state == LSF, we need to recover LSF from LICH.
            demodState = DemodState::STREAM_SYNC;
            break;
        case M17FrameDecoder::State::BERT:
            demodState = DemodState::BERT_SYNC;
            break;
        default:
            demodState = DemodState::PACKET_SYNC;
            break;
        }

        sync_count = 0;

        switch (valid)
        {
        case M17FrameDecoder::DecodeResult::FAIL:
            WARN("decode invalid");
            if (frame_result && !passall_)
            {
               hdlc::release(frame_result);
               frame_result = nullptr;
            }
            break;
        case M17FrameDecoder::DecodeResult::EOS:
            // demodState = DemodState::LSF_SYNC;
            INFO("EOS");
            break;
        case M17FrameDecoder::DecodeResult::OK:
            break;
        case M17FrameDecoder::DecodeResult::INCOMPLETE:
            break;
        }
    }
}

hdlc::IoFrame* M17Demodulator::operator()(const q15_t* input)
{
    static int16_t initializing = 10;

    hdlc::IoFrame* frame_result = nullptr;

    // We need to pump a few frames through on startup to initialize
    // the demodulator.
    if (__builtin_expect((initializing), 0))
    {
        --initializing;
        initialize(input);
        return frame_result;
    }

//    str_indicator.on();
    update_dcd(input);

    if (!dcd_)
    {
        dcd.update();
//        str_indicator.off();
        return frame_result;
    }

    // Do adc_micro_adjustment() here?
    // adc_micro_adjustment();

    for (size_t i = 0; i != ADC_BLOCK_SIZE; i++) {
        demod_buffer[i] = float(input[i]) * scale;
    }

    auto filtered = demod_filter(demod_buffer.data());
//    getModulator().loopback(filtered);

    for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
    {
        auto filtered_sample = dc_block(filtered[i]);
        correlator.sample(filtered_sample);

        if (correlator.index() == 0)
        {
            if (need_clock_reset_)
            {
                clock_recovery.reset(sync_sample_index);
                need_clock_reset_ = false;
                sample_index = sync_sample_index;
                adc_timing_adjust = 0;
            }
            else if (need_clock_update_) // must avoid update immediately after reset.
            {
                if (!missing_sync_count) {
                    // Have a real sync word match.
                    clock_recovery.update(sync_sample_index);
                } else if (decoder.state() != M17FrameDecoder::State::LSF) {
                    // Not waiting on clock recovery.
                    clock_recovery.update();
                }
                sample_index = clock_recovery.sample_index();
                need_clock_update_ = false;
            }
        }

        clock_recovery(filtered_sample);

        if (demodState != DemodState::UNLOCKED && correlator.index() == sample_index)
        {
            dev.sample(filtered_sample);
        }

        switch (demodState)
        {
        case DemodState::UNLOCKED:
            // In this state, the sample_index is unknown.  We need to find
            // a sync word to find the proper sample_index.  We only leave
            // this state if we believe that we have a valid sample_index.
            do_unlocked();
            break;
        case DemodState::LSF_SYNC:
            do_lsf_sync();
            break;
        case DemodState::STREAM_SYNC:
            do_stream_sync();
            break;
        case DemodState::PACKET_SYNC:
            do_packet_sync();
            break;
        case DemodState::BERT_SYNC:
            do_bert_sync();
            break;
        case DemodState::FRAME:
            do_frame(filtered_sample,frame_result);
            break;
        }
    }
    dcd.update();
//    str_indicator.off();

    return frame_result;
}

}} // mobilinkd::tnc
