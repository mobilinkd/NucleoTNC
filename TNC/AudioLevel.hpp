// Copyright 2015-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "arm_math.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osMessageQId audioInputQueueHandle;

#ifdef __cplusplus
}
#endif

constexpr const uint32_t AUDIO_IN = ADC_CHANNEL_8;

namespace mobilinkd { namespace tnc { namespace audio {

void init_log_volume();
void autoAudioInputLevel();
void setAudioInputLevels();
void setAudioOutputLevel();

extern bool streamInputDCOffset;
constexpr const uint16_t vref = 4095; // Must match ADC output (adjust when oversampling)
extern int16_t virtual_ground;
extern float i_vgnd;

}}} // mobilinkd::tnc::audio
