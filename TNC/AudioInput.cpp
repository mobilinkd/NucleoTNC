// Copyright 2015 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AudioInput.hpp"
#include "AfskDemodulator.hpp"
#include "AudioLevel.hpp"
#include "Log.h"
#include "KissHardware.hpp"
#include "GPIO.hpp"
#include "HdlcFrame.hpp"
#include "memory.hpp"
#include "IirFilter.hpp"
#include "FilterCoefficients.hpp"
#include "PortInterface.hpp"
#include "Goertzel.h"
#include "Led.h"

#include "arm_math.h"
#include "stm32l4xx_hal.h"

#include <algorithm>
#include <numeric>
#include <cstring>
#include <cstdint>
#include <atomic>

extern osMessageQId ioEventQueueHandle;

extern "C" void SystemClock_Config(void);

// 1kB
typedef mobilinkd::tnc::memory::Pool<
    8, mobilinkd::tnc::audio::ADC_BUFFER_SIZE * 2> adc_pool_type;
adc_pool_type adcPool;

// DMA Conversion first half complete.
extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef*) {
    using namespace mobilinkd::tnc::audio;

    auto block = adcPool.allocate();
    if (!block) return;
    memmove(block->buffer, adc_buffer, ADC_BUFFER_SIZE * 2);
    auto status = osMessagePut(adcInputQueueHandle, (uint32_t) block, 0);
    if (status != osOK) adcPool.deallocate(block);
}

// DMA Conversion second half complete.
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*) {
    using namespace mobilinkd::tnc::audio;

    auto block = adcPool.allocate();
    if (!block) return;
    memmove(block->buffer, adc_buffer + DMA_TRANSFER_SIZE, ADC_BUFFER_SIZE * 2);
    auto status = osMessagePut(adcInputQueueHandle, (uint32_t) block, 0);
    if (status != osOK) adcPool.deallocate(block);
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* /* hadc */) {
    using namespace mobilinkd::tnc::audio;

    // __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
    // HAL_DMA_Start(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)adc_buffer, ADC_BUFFER_SIZE * 2);
}

extern "C" void startAudioInputTask(void const*) {

    using namespace mobilinkd::tnc::audio;
    DEBUG("startAudioInputTask");

    adcPool.init();

    uint8_t adcState = mobilinkd::tnc::audio::IDLE;

    while (true) {
        osEvent event = osMessageGet(audioInputQueueHandle, osWaitForever);
        if (event.status != osEventMessage) continue;
        adcState = event.value.v;

        switch (adcState) {
        case STOPPED:
            DEBUG("STOPPED");
            // stop();
            break;
        case DEMODULATOR:
            DEBUG("DEMODULATOR");
            demodulatorTask();
            break;
        case STREAM_AMPLIFIED_INPUT_LEVEL:
            DEBUG("STREAM_AMPLIFIED_INPUT_LEVEL");
            streamAmplifiedInputLevels();
            break;
        case POLL_AMPLIFIED_INPUT_LEVEL:
            DEBUG("POLL_AMPLIFIED_INPUT_LEVEL");
            pollAmplifiedInputLevel();
            break;
        case POLL_TWIST_LEVEL:
            DEBUG("POLL_TWIST_LEVEL");
            pollInputTwist();
            break;
        case STREAM_AVERAGE_TWIST_LEVEL:
            DEBUG("STREAM_AVERAGE_TWIST_LEVEL");
            streamAverageInputTwist();
            break;
        case STREAM_INSTANT_TWIST_LEVEL:
            DEBUG("STREAM_INSTANT_TWIST_LEVEL");
            streamInstantInputTwist();
            break;
        case AUTO_ADJUST_INPUT_LEVEL:
            DEBUG("AUTO_ADJUST_INPUT_LEVEL");
            autoAudioInputLevel();
            break;
        case CONFIGURE_INPUT_LEVELS:
            DEBUG("CONFIGURE_INPUT_LEVELS");
            setAudioInputLevels();
            break;
        case UPDATE_SETTINGS:
            DEBUG("UPDATE_SETTINGS");
            setAudioInputLevels();
            break;
        case IDLE:
            DEBUG("IDLE");
            break;
        default:
            break;
        }
    }
}

namespace mobilinkd { namespace tnc { namespace audio {

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 26400 Hz

* 0 Hz - 800 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.713187739640446 dB

* 1100 Hz - 2300 Hz
  gain = 1
  desired ripple = 3 dB
  actual ripple = 1.9403554103597218 dB

* 2600 Hz - 13200 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.713187739640446 dB

*/

#define FILTER_TAP_NUM 121

const float taps_0dB_121[] = {
  0.00404434702588704,
  -0.0003678805989470367,
  -0.0011037474176397869,
  -0.0023718433790735397,
  -0.004074774206090812,
  -0.005873042355767296,
  -0.007185024927682025,
  -0.00733586918005499,
  -0.005849936137673611,
  -0.0026340821242355635,
  0.001829866887380395,
  0.006596559367932984,
  0.010513363703436297,
  0.012581963716759209,
  0.012278717176141912,
  0.009711068794837135,
  0.005595156551222992,
  0.0010151465722928203,
  -0.0028911100291917724,
  -0.00525632756952279,
  -0.005713225280738633,
  -0.004481173891886628,
  -0.0022979447479362165,
  -0.00020042687909196258,
  0.0007698191454281245,
  -0.00010521900913954391,
  -0.0029591195718788473,
  -0.00722329412770922,
  -0.01171525034180743,
  -0.014940096229612041,
  -0.015533747313789608,
  -0.012735965353880947,
  -0.006719733175529481,
  0.0013488096154956725,
  0.00958058819325866,
  0.015905894530456915,
  0.018729580500272548,
  0.01748781569748245,
  0.012897342590446394,
  0.006795493031300682,
  0.0015881037163025886,
  -0.0005374299552534915,
  0.0016047331197704602,
  0.007673729209166328,
  0.015724242346878387,
  0.022631203832237833,
  0.024935722640487233,
  0.01989507252801227,
  0.00645078432931977,
  -0.014172629267618218,
  -0.038442876378037664,
  -0.06113274585379875,
  -0.07647113797360244,
  -0.07957164384983365,
  -0.06778356575105521,
  -0.04159845430900078,
  -0.0048373906972505945,
  0.03598583922416374,
  0.0730150987796154,
  0.09880455186134979,
  0.10804223448811107,
  0.09880455186134979,
  0.0730150987796154,
  0.03598583922416374,
  -0.0048373906972505945,
  -0.04159845430900078,
  -0.06778356575105521,
  -0.07957164384983365,
  -0.07647113797360244,
  -0.06113274585379875,
  -0.038442876378037664,
  -0.014172629267618218,
  0.006450784329319771,
  0.01989507252801227,
  0.024935722640487233,
  0.022631203832237833,
  0.015724242346878387,
  0.007673729209166332,
  0.0016047331197704602,
  -0.0005374299552534915,
  0.0015881037163025886,
  0.0067954930313006805,
  0.012897342590446394,
  0.017487815697482458,
  0.01872958050027255,
  0.015905894530456915,
  0.00958058819325866,
  0.0013488096154956762,
  -0.006719733175529481,
  -0.012735965353880947,
  -0.015533747313789608,
  -0.014940096229612034,
  -0.01171525034180743,
  -0.00722329412770922,
  -0.0029591195718788473,
  -0.00010521900913954758,
  0.0007698191454281245,
  -0.00020042687909196258,
  -0.0022979447479362165,
  -0.004481173891886626,
  -0.005713225280738633,
  -0.005256327569522788,
  -0.0028911100291917724,
  0.0010151465722928203,
  0.005595156551222992,
  0.009711068794837135,
  0.012278717176141915,
  0.012581963716759209,
  0.010513363703436297,
  0.006596559367932984,
  0.001829866887380395,
  -0.0026340821242355635,
  -0.005849936137673611,
  -0.0073358691800549875,
  -0.007185024927682024,
  -0.005873042355767296,
  -0.004074774206090811,
  -0.0023718433790735397,
  -0.0011037474176397869,
  -0.00036788059894704404,
  0.00404434702588704
};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 26400 Hz

* 0 Hz - 600 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.59537969202882 dB

* 1100 Hz - 2300 Hz
  gain = 1
  desired ripple = 3 dB
  actual ripple = 1.9670775534013671 dB

* 2800 Hz - 13200 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.59537969202882 dB

*/

// #define FILTER_TAP_NUM 73

const float taps_0dB_73[] = {
  0.0010893641938196257,
  0.0029403198794405202,
  -0.0037231874681753637,
  -0.005078116094780293,
  -0.008797286521082463,
  -0.011317340935878852,
  -0.012200463385017889,
  -0.01036925371439487,
  -0.005637326405238566,
  0.0014334055832832988,
  0.009462055516437227,
  0.016585173167785613,
  0.020968649539195763,
  0.021402512805125434,
  0.017768177789191805,
  0.011189277365350641,
  0.003796773667470304,
  -0.0019035128640327481,
  -0.003853114272608765,
  -0.0012626334798333488,
  0.004945485136075468,
  0.012177421685799305,
  0.016832103112238102,
  0.01536679512873413,
  0.005573647945409955,
  -0.012436210925634471,
  -0.03581550676890827,
  -0.05935854007614169,
  -0.07666569528110105,
  -0.08180873487685453,
  -0.07107700533422828,
  -0.0442469908102239,
  -0.005044918510227134,
  0.03944927956419565,
  0.0803589200537307,
  0.10908069178917619,
  0.11940367705752576,
  0.1090806917891762,
  0.0803589200537307,
  0.03944927956419565,
  -0.005044918510227135,
  -0.0442469908102239,
  -0.07107700533422828,
  -0.08180873487685453,
  -0.07666569528110105,
  -0.05935854007614169,
  -0.03581550676890827,
  -0.012436210925634473,
  0.005573647945409955,
  0.015366795128734134,
  0.016832103112238102,
  0.012177421685799305,
  0.004945485136075468,
  -0.0012626334798333488,
  -0.003853114272608765,
  -0.001903512864032745,
  0.003796773667470304,
  0.011189277365350641,
  0.017768177789191805,
  0.021402512805125434,
  0.020968649539195763,
  0.016585173167785607,
  0.009462055516437227,
  0.0014334055832832988,
  -0.005637326405238568,
  -0.01036925371439487,
  -0.012200463385017889,
  -0.011317340935878852,
  -0.008797286521082463,
  -0.005078116094780293,
  -0.0037231874681753637,
  0.0029403198794405202,
  0.0010893641938196242
};

uint32_t adc_buffer[ADC_BUFFER_SIZE];       // Two samples per element.

typedef FirFilter<ADC_BUFFER_SIZE, FILTER_TAP_NUM> audio_filter_type;

audio_filter_type audio_filter;

mobilinkd::tnc::afsk1200::emphasis_filter_type filter_1;
mobilinkd::tnc::afsk1200::emphasis_filter_type filter_2;
mobilinkd::tnc::afsk1200::emphasis_filter_type filter_3;

mobilinkd::tnc::afsk1200::Demodulator& getDemod1(const TFirCoefficients<9>& f) __attribute__((noinline));
mobilinkd::tnc::afsk1200::Demodulator& getDemod2(const TFirCoefficients<9>& f) __attribute__((noinline));
mobilinkd::tnc::afsk1200::Demodulator& getDemod3(const TFirCoefficients<9>& f) __attribute__((noinline));

mobilinkd::tnc::afsk1200::Demodulator& getDemod1(const TFirCoefficients<9>& f) {
    filter_1.init(f);
    static mobilinkd::tnc::afsk1200::Demodulator instance(26400, filter_1);
    return instance;
}

mobilinkd::tnc::afsk1200::Demodulator& getDemod2(const TFirCoefficients<9>& f) {
    filter_2.init(f);
    static mobilinkd::tnc::afsk1200::Demodulator instance(26400, filter_2);
    return instance;
}

mobilinkd::tnc::afsk1200::Demodulator& getDemod3(const TFirCoefficients<9>& f) {
    filter_3.init(f);
    static mobilinkd::tnc::afsk1200::Demodulator instance(26400, filter_3);
    return instance;
}


void demodulatorTask() {

    DEBUG("enter demodulatorTask");

    audio_filter.init(taps_0dB_121);

    // rx_twist is 6dB for discriminator input and 0db for de-emphasized input.
    auto twist = kiss::settings().rx_twist;

    mobilinkd::tnc::afsk1200::Demodulator& demod1 = getDemod1(*filter::fir::AfskFilters[twist + 3]);
    mobilinkd::tnc::afsk1200::Demodulator& demod2 = getDemod2(*filter::fir::AfskFilters[twist + 6]);
    mobilinkd::tnc::afsk1200::Demodulator& demod3 = getDemod3(*filter::fir::AfskFilters[twist + 9]);

    startADC(AUDIO_IN);

    mobilinkd::tnc::hdlc::IoFrame* frame = 0;

    uint16_t last_fcs = 0;
    uint32_t last_counter = 0;
    uint32_t counter = 0;

    bool dcd_status{false};

    while (true) {
        osEvent peek = osMessagePeek(audioInputQueueHandle, 0);
        if (peek.status == osEventMessage) break;

        osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
        if (evt.status != osEventMessage) {
            continue;
        }
        ++counter;

        auto block = (adc_pool_type::chunk_type*) evt.value.p;
        auto samples = (int16_t*) block->buffer;

        float* audio = audio_filter(samples);
        adcPool.deallocate(block);

#if 1
        frame = demod1(audio, ADC_BUFFER_SIZE);
        if (frame) {
            if (frame->fcs() != last_fcs or counter > last_counter + 2) {
                auto save_fcs = frame->fcs();
                if (osMessagePut(ioEventQueueHandle, (uint32_t) frame, 1) == osOK) {
                  last_fcs = save_fcs;
                  last_counter = counter;
                } else {
                  hdlc::ioFramePool().release(frame);
                }
            }
            else {
                hdlc::ioFramePool().release(frame);
            }
        }
#endif

#if 1
        frame = demod2(audio, ADC_BUFFER_SIZE);
        if (frame) {
          if (frame->fcs() != last_fcs or counter > last_counter + 2) {
              auto save_fcs = frame->fcs();
              if (osMessagePut(ioEventQueueHandle, (uint32_t) frame, 1) == osOK) {
                last_fcs = save_fcs;
                last_counter = counter;
              } else {
                hdlc::ioFramePool().release(frame);
              }
          }
          else {
              hdlc::ioFramePool().release(frame);
          }
        }
#endif

#if 1
        frame = demod3(audio, ADC_BUFFER_SIZE);
        if (frame) {
          if (frame->fcs() != last_fcs or counter > last_counter + 2) {
              auto save_fcs = frame->fcs();
              if (osMessagePut(ioEventQueueHandle, (uint32_t) frame, 1) == osOK) {
                last_fcs = save_fcs;
                last_counter = counter;
              } else {
                hdlc::ioFramePool().release(frame);
              }
          }
          else {
              hdlc::ioFramePool().release(frame);
          }
        }
#endif
        bool new_dcd_status = demod1.locked() or demod2.locked() or demod3.locked();
        if (new_dcd_status xor dcd_status) {
            dcd_status = new_dcd_status;
            if (dcd_status) {
                led_dcd_on();
            } else {
                led_dcd_off();
            }
        }
    }

    stopADC();
    led_dcd_off();
    DEBUG("exit demodulatorTask");
}


void streamLevels(uint32_t channel, uint8_t cmd) {

    // Stream out Vpp, Vavg, Vmin, Vmax as four 16-bit values, left justified.

    uint8_t data[9];
    INFO("streamLevels: start");
    startADC(channel);

    while (true) {
        osEvent peek = osMessagePeek(audioInputQueueHandle, 0);
        if (peek.status == osEventMessage) break;

        uint16_t count = 0;
        uint32_t accum = 0;
        uint16_t min = 4096;
        uint16_t max = 0;

        while (count < 2640) {
            osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
            if (evt.status != osEventMessage) continue;

            count += ADC_BUFFER_SIZE;

            auto block = (adc_pool_type::chunk_type*) evt.value.p;
            uint16_t* start =  (uint16_t*) block->buffer;
            uint16_t* end = (uint16_t*) block->buffer + ADC_BUFFER_SIZE;

            min = std::min(min, *std::min_element(start, end));
            max = std::max(max, *std::max_element(start, end));
            accum = std::accumulate(start, end, accum);

            adcPool.deallocate(block);
        }

        uint16_t pp = (max - min) << 4;
        uint16_t avg = (accum / count) << 4;
        min <<= 4;
        max <<= 4;

        data[0] = cmd;
        data[1] = (pp >> 8) & 0xFF;   // Vpp
        data[2] = (pp & 0xFF);
        data[3] = (avg >> 8) & 0xFF;  // Vavg (DC level)
        data[4] = (avg & 0xFF);
        data[5] = (min >> 8) & 0xFF;  // Vmin
        data[6] = (min & 0xFF);
        data[7] = (max >> 8) & 0xFF;  // Vmax
        data[8] = (max & 0xFF);

        ioport->write(data, 9, 6, 10);
    }

    stopADC();
    DEBUG("exit streamLevels");
}

levels_type readLevels(uint32_t channel, uint32_t samples) {

    DEBUG("enter readLevels");

    // Return Vpp, Vavg, Vmin, Vmax as four 16-bit values, right justified.

    uint16_t count = 0;
    uint32_t accum = 0;
    uint16_t vmin = 4096;
    uint16_t vmax = 0;

    INFO("readLevels: start");
    startADC(channel);

    while (count < samples) {

        osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
        if (evt.status != osEventMessage) continue;

        count += ADC_BUFFER_SIZE;

        auto block = (adc_pool_type::chunk_type*) evt.value.v;
        uint16_t* start =  (uint16_t*) block->buffer;
        uint16_t* end = (uint16_t*) block->buffer + ADC_BUFFER_SIZE;

        vmin = std::min(vmin, *std::min_element(start, end));
        vmax = std::max(vmax, *std::max_element(start, end));
        accum = std::accumulate(start, end, accum);

        adcPool.deallocate(block);
    }

    stopADC();

    uint16_t pp = vmax - vmin;
    uint16_t avg = accum / count;
    DEBUG("exit readLevels");

    return levels_type(pp, avg, vmin, vmax);
}


constexpr uint32_t TWIST_SAMPLE_SIZE = 264 * 5;

/*
 * Return twist as a the difference in dB between mark and space.  The
 * expected values are about 0dB for discriminator output and about 5.5dB
 * for de-emphasized audio.
 */
float readTwist()
{

  DEBUG("enter readTwist");
  constexpr uint32_t channel = AUDIO_IN;

  float g1200 = 0.0f;
  float g2200 = 0.0f;

  GoertzelFilter<TWIST_SAMPLE_SIZE, SAMPLE_RATE> gf1200(1200.0);
  GoertzelFilter<TWIST_SAMPLE_SIZE, SAMPLE_RATE> gf2200(2200.0);

  const uint32_t AVG_SAMPLES = 100;

  startADC(channel);

  for (uint32_t i = 0; i != AVG_SAMPLES; ++i) {

    uint32_t count = 0;
    while (count < TWIST_SAMPLE_SIZE) {

        osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
        if (evt.status != osEventMessage) continue;

        count += ADC_BUFFER_SIZE;

        auto block = (adc_pool_type::chunk_type*) evt.value.v;
        uint16_t* data =  (uint16_t*) block->buffer;
        gf1200(data, ADC_BUFFER_SIZE);
        gf2200(data, ADC_BUFFER_SIZE);

        adcPool.deallocate(block);
    }

    g1200 += 10.0 * log10(gf1200);
    g2200 += 10.0 * log10(gf2200);

    gf1200.reset();
    gf2200.reset();
  }

  stopADC();
  DEBUG("exit readTwist");

  return   (g1200 / AVG_SAMPLES) - (g2200 / AVG_SAMPLES);
}

/*
 * Get the input twist level as a pair of numbers -- the relative dB
 * level of the Bell 202 mark and space tones.
 *
 * This is intended to measure noise levels on an empty channel.
 *
 * When de-emphasis is applied, the noise at 1200Hz will be about 5.5dB
 * higher than at 2200Hz.  When de-emphasis is not applied (discriminator
 * output), the levels should be about the same.
 *
 * This is used to adjust the demodulator filters so that the proper
 * input twist is applied to the signal.  In general, properly modulated
 * signals are expected to be pre-emphasized so that they are equal
 * when de-emphasis is applied.
 *
 * If no de-emphasis is detected, the de-emphasis has to be applied in
 * the demodulator.
 *
 * This takes about 5 seconds to complete as it averages 100 50ms samples
 * to get a reasonable sampling of the noise.
 */
void pollInputTwist()
{
    DEBUG("enter pollInputTwist");
    constexpr uint32_t channel = AUDIO_IN;

    float g1200 = 0.0f;
    float g2200 = 0.0f;

    GoertzelFilter<TWIST_SAMPLE_SIZE, SAMPLE_RATE> gf1200(1200.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, SAMPLE_RATE> gf2200(2200.0);

    const uint32_t AVG_SAMPLES = 100;

    startADC(channel);

    for (uint32_t i = 0; i != AVG_SAMPLES; ++i) {

      uint32_t count = 0;
      while (count < TWIST_SAMPLE_SIZE) {

          osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
          if (evt.status != osEventMessage) continue;

          count += ADC_BUFFER_SIZE;

          auto block = (adc_pool_type::chunk_type*) evt.value.v;
          uint16_t* data =  (uint16_t*) block->buffer;
          gf1200(data, ADC_BUFFER_SIZE);
          gf2200(data, ADC_BUFFER_SIZE);

          adcPool.deallocate(block);
      }

      g1200 += 10.0 * log10(gf1200);
      g2200 += 10.0 * log10(gf2200);

      gf1200.reset();
      gf2200.reset();
    }

    stopADC();

    DEBUG("pollInputTwist: MARK=%d, SPACE=%d (x100)",
      int(g1200 * 100.0 / AVG_SAMPLES), int(g2200 * 100.0 / AVG_SAMPLES));

    int16_t g1200i = int16_t(g1200 * 256 / AVG_SAMPLES);
    int16_t g2200i = int16_t(g2200 * 256 / AVG_SAMPLES);

    uint8_t buffer[5];
    buffer[0] = kiss::hardware::POLL_INPUT_TWIST;
    buffer[1] = (g1200i >> 8) & 0xFF;
    buffer[2] = g1200i & 0xFF;
    buffer[3] = (g2200i >> 8) & 0xFF;
    buffer[4] = g2200i & 0xFF;

    ioport->write(buffer, 5, 6, 10);

    DEBUG("exit pollInputTwist");
}

void streamAverageInputTwist()
{
    DEBUG("enter streamAverageInputTwist");

    constexpr uint32_t channel = AUDIO_IN;

    startADC(channel);

    uint32_t acount = 0;
    float g700 = 0.0f;
    float g1200 = 0.0f;
    float g1700 = 0.0f;
    float g2200 = 0.0f;
    float g2700 = 0.0f;

    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf700(700.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf1200(1200.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf1700(1700.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf2200(2200.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf2700(2700.0);

    while (true) {
      osEvent peek = osMessagePeek(audioInputQueueHandle, 0);
      if (peek.status == osEventMessage) break;

      acount++;
      uint32_t count = 0;
      while (count < TWIST_SAMPLE_SIZE) {

          osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
          if (evt.status != osEventMessage) continue;

          count += ADC_BUFFER_SIZE;

          auto block = (adc_pool_type::chunk_type*) evt.value.v;
          uint16_t* data =  (uint16_t*) block->buffer;
          gf700(data, ADC_BUFFER_SIZE);
          gf1200(data, ADC_BUFFER_SIZE);
          gf1700(data, ADC_BUFFER_SIZE);
          gf2200(data, ADC_BUFFER_SIZE);
          gf2700(data, ADC_BUFFER_SIZE);

          adcPool.deallocate(block);
      }

      g700 += 10.0 * log10(gf700);
      g1200 += 10.0 * log10(gf1200);
      g1700 += 10.0 * log10(gf1700);
      g2200 += 10.0 * log10(gf2200);
      g2700 += 10.0 * log10(gf2700);

      char* buffer = 0;
      int len = asiprintf(
        &buffer,
        "_%f, %f, %f, %f, %f\r\n",
        g700 / acount,
        g1200 / acount,
        g1700 / acount,
        g2200 / acount,
        g2700 / acount);

      if (len > 0) {
        buffer[0] = kiss::hardware::POLL_INPUT_TWIST;
        ioport->write((uint8_t*)buffer, len - 1, 6, 10);
        free(buffer);
      }

      gf700.reset();
      gf1200.reset();
      gf1700.reset();
      gf2200.reset();
      gf2700.reset();
    }

    stopADC();
    DEBUG("exit streamAverageInputTwist");
}

void streamInstantInputTwist()
{
    DEBUG("enter streamInstantInputTwist");

    constexpr uint32_t channel = AUDIO_IN;

    startADC(channel);

    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf700(700.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf1200(1200.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf1700(1700.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf2200(2200.0);
    GoertzelFilter<TWIST_SAMPLE_SIZE, 26400> gf2700(2700.0);

    while (true) {
      osEvent peek = osMessagePeek(audioInputQueueHandle, 0);
      if (peek.status == osEventMessage) break;

      uint32_t count = 0;
      while (count < TWIST_SAMPLE_SIZE) {

          osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
          if (evt.status != osEventMessage) continue;

          count += ADC_BUFFER_SIZE;

          auto block = (adc_pool_type::chunk_type*) evt.value.v;
          uint16_t* data =  (uint16_t*) block->buffer;
          gf700(data, ADC_BUFFER_SIZE);
          gf1200(data, ADC_BUFFER_SIZE);
          gf1700(data, ADC_BUFFER_SIZE);
          gf2200(data, ADC_BUFFER_SIZE);
          gf2700(data, ADC_BUFFER_SIZE);

          adcPool.deallocate(block);
      }

      char* buffer = 0;
      int len = asiprintf(
        &buffer,
        "_%f, %f, %f, %f, %f\r\n",
        10.0 * log10(gf700),
        10.0 * log10(gf1200),
        10.0 * log10(gf1700),
        10.0 * log10(gf2200),
        10.0 * log10(gf2700));

      if (len > 0) {
        buffer[0] = kiss::hardware::POLL_INPUT_TWIST;
        ioport->write((uint8_t*)buffer, len - 1, 6, 10);
        free(buffer);
      }

      gf700.reset();
      gf1200.reset();
      gf1700.reset();
      gf2200.reset();
      gf2700.reset();
    }

    stopADC();
    DEBUG("exit streamInstantInputTwist");
}

void streamAmplifiedInputLevels() {
    DEBUG("enter streamAmplifiedInputLevels");
    streamLevels(AUDIO_IN, kiss::hardware::POLL_INPUT_LEVEL);
    DEBUG("exit streamAmplifiedInputLevels");
}

void pollAmplifiedInputLevel() {
    DEBUG("enter pollAmplifiedInputLevel");

    uint16_t Vpp, Vavg, Vmin, Vmax;
    std::tie(Vpp, Vavg, Vmin, Vmax) = readLevels(AUDIO_IN);

    Vpp <<= 4;
    Vavg <<= 4;
    Vmin <<= 4;
    Vmax <<= 4;

    uint8_t data[9];
    data[0] = kiss::hardware::POLL_INPUT_LEVEL;
    data[1] = (Vpp >> 8) & 0xFF;   // Vpp
    data[2] = (Vpp & 0xFF);
    data[3] = (Vavg >> 8) & 0xFF;  // Vavg (DC level)
    data[4] = (Vavg & 0xFF);
    data[5] = (Vmin >> 8) & 0xFF;  // Vmin
    data[6] = (Vmin & 0xFF);
    data[7] = (Vmax >> 8) & 0xFF;  // Vmax
    data[8] = (Vmax & 0xFF);

    ioport->write(data, 9, 6, 10);
    DEBUG("exit pollAmplifiedInputLevel");
}

void stop() {
    osDelay(100);
#if 0
    auto restore = SysTick->CTRL;

    kiss::settings().input_offset += 6;
    setAudioInputLevels();
    kiss::settings().input_offset -= 6;
    DEBUG("Stop");
    // __disable_irq();
    vTaskSuspendAll();
    SysTick->CTRL = 0;
    HAL_COMP_Init(&hcomp1);
    HAL_COMP_Start_IT(&hcomp1);
    while (adcState == STOPPED) {
        // PWR_MAINREGULATOR_ON / PWR_LOWPOWERREGULATOR_ON
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    }
    SystemClock_Config();
    SysTick->CTRL = restore;
    // __enable_irq();
    HAL_COMP_Stop_IT(&hcomp1);
    HAL_COMP_DeInit(&hcomp1);
    xTaskResumeAll();
    setAudioInputLevels();
    // adcState = DEMODULATOR;
    DEBUG("Wake");
#endif
}

}}} // mobilinkd::tnc::audio
