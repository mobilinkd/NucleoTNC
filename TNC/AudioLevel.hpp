// Copyright 2015 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__AUDIO__AUDIO_LEVEL_H_
#define MOBILINKD__TNC__AUDIO__AUDIO_LEVEL_H_

#include "arm_math.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osMessageQId audioInputQueueHandle;

#ifdef __cplusplus
}
#endif

#define AUDIO_IN ADC_CHANNEL_8

namespace mobilinkd { namespace tnc { namespace audio {

void init_log_volume();
void autoAudioInputLevel();
void setAudioInputLevels();
void setAudioOutputLevel();

extern bool streamInputDCOffset;
extern uint16_t virtual_ground;

}}} // mobilinkd::tnc::audio

#endif // MOBILINKD__TNC__AUDIO__AUDIO_LEVEL_H_
