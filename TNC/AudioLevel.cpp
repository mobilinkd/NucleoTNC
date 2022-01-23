// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "KissHardware.hpp"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "GPIO.hpp"
#include "LEDIndicator.h"
#include "ModulatorTask.hpp"

#include "main.h"
#include "stm32l4xx_hal.h"

#include <algorithm>
#include <tuple>
#include <array>
#include <cstdlib>
#include <cstdint>

#include <inttypes.h>
#include <Log.h>

extern OPAMP_HandleTypeDef hopamp1;
extern DAC_HandleTypeDef hdac1;

namespace mobilinkd { namespace tnc { namespace audio {

int16_t virtual_ground{0};
float i_vgnd{0.0f};

void set_input_gain(int level)
{
    uint32_t dc_offset{};

    // Stop and de-init the op amp before changing its state.
    if (HAL_OPAMP_Stop(&hopamp1) != HAL_OK)
        CxxErrorHandler();
    if (HAL_OPAMP_DeInit(&hopamp1) != HAL_OK)
        CxxErrorHandler();

    level = std::max(0, level);
    level = std::min(4, level);

    // Adjust configuration and, if PGA, gain.
    switch (level) {
    case 0: // 0dB
        hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
        dc_offset = 2048;
        break;
    case 1: // 6dB
        hopamp1.Init.Mode = OPAMP_PGA_MODE;
        hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2;
        dc_offset = 1024;
        break;
    case 2: // 12dB
        hopamp1.Init.Mode = OPAMP_PGA_MODE;
        hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_4;
        dc_offset = 512;
        break;
    case 3: // 18dB
        hopamp1.Init.Mode = OPAMP_PGA_MODE;
        hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8;
        dc_offset = 256;
        break;
    case 4: // 24dB
        hopamp1.Init.Mode = OPAMP_PGA_MODE;
        hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16;
        dc_offset = 128;
        break;
    default:
        CxxErrorHandler();
    }

    // Adjust DC offset.  It is different for each gain setting.
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dc_offset) != HAL_OK)
        CxxErrorHandler();

    // Init and start the op amp after the change.
    if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
        CxxErrorHandler();
    if (HAL_OPAMP_Start(&hopamp1)!= HAL_OK)
        CxxErrorHandler();
}

int adjust_input_gain() __attribute__((noinline));

int adjust_input_gain() {

    INFO("\nAdjusting input gain...\n");

    int gain{0};
    uint16_t vpp, vavg, vmin, vmax;

    set_input_gain(gain);
    osDelay(1000);   // Need time for DC offset to settle.

    std::tie(vpp, vavg, vmin, vmax) = readLevels(AUDIO_IN);
    INFO("\nVpp = %" PRIu16 ", Vavg = %" PRIu16 "\n", vpp, vavg);
    INFO("\nVmin = %" PRIu16 ", Vmax = %" PRIu16 ", setting = %d\n", vmin, vmax, gain);

    while (gain == 0 and (vmax == vref or vmin == 0))
    {
        std::tie(vpp, vavg, vmin, vmax) = readLevels(AUDIO_IN);

        INFO("\nVpp = %" PRIu16 ", Vavg = %" PRIu16 "\n", vpp, vavg);
        INFO("\nVmin = %" PRIu16 ", Vmax = %" PRIu16 ", setting = %d\n", vmin, vmax, gain);
    }

    auto desired_gain = vref / vpp;

    if (desired_gain >= 16) gain = 4;
    else if (desired_gain >= 8) gain = 3;
    else if (desired_gain >= 4) gain = 2;
    else if (desired_gain >= 2) gain = 1;
    else gain = 0;

    set_input_gain(gain);
    osDelay(1000);   // Need time for DC offset to settle.

    std::tie(vpp, vavg, vmin, vmax) = readLevels(AUDIO_IN);
    INFO("\nVpp = %" PRIu16 ", Vavg = %" PRIu16 "\n", vpp, vavg);
    INFO("\nVmin = %" PRIu16 ", Vmax = %" PRIu16 ", setting = %d\n", vmin, vmax, gain);

    virtual_ground = vavg;
    i_vgnd = 1.0 / virtual_ground;

    return gain;
}


void autoAudioInputLevel()
{
    INFO("autoInputLevel");

    mobilinkd::tnc::kiss::settings().input_gain = adjust_input_gain();

    int rx_twist = readTwist() + 0.5f;
    if (rx_twist < -3) rx_twist = -3;
    else if (rx_twist > 9) rx_twist = 9;
    INFO("TWIST = %ddB", rx_twist);
    mobilinkd::tnc::kiss::settings().rx_twist = rx_twist;
    mobilinkd::tnc::kiss::settings().announce_input_settings();
    mobilinkd::tnc::kiss::settings().update_crc();
    mobilinkd::tnc::kiss::settings().store();
}

/**
 * Set the audio input levels from the values stored in EEPROM.  Then
 * analyze the input levels to set the VGND level.
 */
void setAudioInputLevels()
{
    // setAudioPins();
    INFO("Setting input gain: %d", kiss::settings().input_gain);
    set_input_gain(kiss::settings().input_gain);

    osDelay(1000);   // Need time for DC offset to settle.

    uint16_t vpp, vavg, vmin, vmax;

    std::tie(vpp, vavg, vmin, vmax) = readLevels(AUDIO_IN);
    INFO("Vpp = %" PRIu16 ", Vavg = %" PRIu16, vpp, vavg);
    INFO("Vmin = %" PRIu16 ", Vmax = %" PRIu16, vmin, vmax);
    virtual_ground = vavg;
    i_vgnd = 1.0 / virtual_ground;
}

std::array<int16_t, 128> log_volume;

void init_log_volume()
{
    int16_t level = 256;
    float gain = 1.0f;
    float factor = 1.02207f;

    for (auto& i : log_volume) {
        i = int16_t(roundf(level * gain));
        gain *= factor;
    }
}

std::tuple<int16_t, int16_t> computeLogAudioLevel(int16_t level)
{
  int16_t l = level & 0x80 ? 1 : 0;
  int16_t r = log_volume[(level & 0x7F)];

  return std::make_tuple(l, r);
}

/**
 * Set the audio output level from the values stored in EEPROM.
 */
void setAudioOutputLevel()
{
  uint16_t l,r;
  std::tie(l, r) = computeLogAudioLevel(kiss::settings().output_gain);

  INFO("Setting output gain: %" PRIi16 " (log %" PRIi16 " + %" PRIi16 ")", kiss::settings().output_gain, l, r);

  if (l) {
      gpio::AUDIO_OUT_ATTEN::on();
  } else {
      gpio::AUDIO_OUT_ATTEN::off();
  }
  getModulator().set_gain(r);
}

}}} // mobilinkd::tnc::audio
