// Copyright 2015-2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "memory.hpp"

#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include <tuple>
#include <atomic>

#ifdef __cplusplus
extern "C" {
#endif

extern osMessageQId hdlcInputQueueHandle;
extern osMessageQId audioInputQueueHandle;
extern osMessageQId adcInputQueueHandle;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef*);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);

/**
 * This is the long-running audio input task/thread.  The ADC can be
 * connected to one of 2 inputs:
 *
 * - AMPLIFIED_AUDIO_IN, ADC1_IN7 -- this is the input used by the demodulator.
 * - BATTERY_VOLTAGE -- this is 1/2 raw battery voltage (nominal 1.6-2.1V).
 *    This input should not be enabled unless BAT_DIVIDER is enabled and
 *    pulled low.
 *
 * These inputs can be measured in a couple of different ways:
 * - Vavg -- for the average DC level of the signal
 * - Vpp -- for the peak-to-peak level of the signal
 *
 * The outputs can be routed five ways:
 *
 * 1. KISS output via serial (data or hardware frames)
 * 2. AX.25 output via serial (data frames)
 * 3. Text output via serial (measurements)
 * 4. To another process
 * 5. /dev/null
 *
 * The usual case is that the AMPLIFIED_AUDIO_IN is routed to the
 * DEMODULATOR and that output is sent to the serial port as KISS
 * output.
 *
 * Each process monitors the adcState and loops while in that state
 * that matches its process.  Each combination of ADC source,
 * measurement/demodulation and output will have a different adcState.
 *
 * @param argument is ignored.
 */
void startAudioInputTask(void const * argument);

void TNC_Error_Handler(int dev, int err);

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc { namespace audio {

constexpr const uint32_t SAMPLE_RATE = 26400;

enum AdcState {
    STOPPED,                        // STOP MODE, wait for comparator
    DEMODULATOR,                    // Demod
    STREAM_RAW_INPUT_LEVEL,         // Unamplified input levels
    STREAM_AMPLIFIED_INPUT_LEVEL,   // Amplified input levels
    POLL_AMPLIFIED_INPUT_LEVEL,     // Amplified input levels
    POLL_BATTERY_LEVEL,             // Battery level
    STREAM_OUTPUT_LEVEL,            // Amplified & filtered output levels
    AUTO_ADJUST_INPUT_LEVEL,        // Automatically adjust input levels
    CONFIGURE_INPUT_LEVELS,         // Set configured input levels
    UPDATE_SETTINGS,                // Update the device settings
    IDLE,                           // No DMA; sleep for 10ms
    POLL_TWIST_LEVEL,
    STREAM_AVERAGE_TWIST_LEVEL,
    STREAM_INSTANT_TWIST_LEVEL
};

const size_t ADC_BUFFER_SIZE = 384;
extern uint32_t adc_buffer[];       // Two int16_t samples per element.
extern volatile uint32_t adc_block_size;
extern volatile uint32_t dma_transfer_size;
extern volatile uint32_t half_buffer_size;

// 3kB
typedef memory::Pool<8, ADC_BUFFER_SIZE * 2> adc_pool_type;
extern adc_pool_type adcPool;

void set_adc_block_size(uint32_t block_size);

/// Vpp, Vavg, Vmin, Vmax
typedef std::tuple<uint16_t, uint16_t, uint16_t, uint16_t> levels_type;
levels_type readLevels(uint32_t channel);
float readTwist();

void demodulatorTask();
void streamRawInputLevels();
void streamAmplifiedInputLevels();
void pollAmplifiedInputLevel();
void pollBatteryLevel();
void streamOutputLevels();
void stop();
void pollInputTwist();
void streamAverageInputTwist();
void streamInstantInputTwist();

}}} // mobilinkd::tnc::audio

#endif // __cplusplus
