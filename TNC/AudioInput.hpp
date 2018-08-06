// Copyright 2015 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__AUDIO__INPUT_HPP_
#define MOBILINKD__TNC__AUDIO__INPUT_HPP_

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
#endif

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

const size_t ADC_BUFFER_SIZE = 88;
const size_t DMA_TRANSFER_SIZE = ADC_BUFFER_SIZE / 2;
extern uint32_t adc_buffer[];       // Two int16_t samples per element.

inline void stopADC() {
    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_Base_Stop(&htim6) != HAL_OK)
        Error_Handler();
}

inline void startADC(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
        Error_Handler();
    if (HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUFFER_SIZE * 2) != HAL_OK)
        Error_Handler();
}

inline void restartADC() {
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
        Error_Handler();
    if (HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUFFER_SIZE * 2) != HAL_OK)
        Error_Handler();
}

/// Vpp, Vavg, Vmin, Vmax
typedef std::tuple<uint16_t, uint16_t, uint16_t, uint16_t> levels_type;
levels_type readLevels(uint32_t channel, uint32_t samples = 2640);
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

#endif // MOBILINKD__TNC__AUDIO__INPUT_HPP_
