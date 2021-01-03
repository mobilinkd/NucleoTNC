// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Fsk9600Modulator.hpp"

namespace mobilinkd { namespace tnc {

/*
 * Cosine.
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
    2047,  2020,  1937,  1801,  1616,  1387,  1120,   822,   502,   169,
    -169,  -502,  -822, -1120, -1387, -1616, -1801, -1937, -2020, -2048
};
*/

/*
 * Square wave -- filtered in hardware at 7200Hz
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
     2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,
    -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048
};
*/

// Gaussian
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
    2042,  2027,  1995,  1931,  1815,  1626,  1345,   968,   507,     0,
    -507,  -968, -1345, -1626, -1815, -1931, -1995, -2027, -2042, -2048
};

void Fsk9600Modulator::init(const kiss::Hardware& hw)
{
    for (auto& x : buffer_) x = 2048;

    (void) hw; // unused

    state = State::STOPPED;
    level = Level::HIGH;

    SysClock80();

    // Configure 80MHz clock for 192ksps.
    htim7.Init.Period = 416;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        ERROR("htim7 init failed");
        CxxErrorHandler();
    }

    DAC_ChannelConfTypeDef sConfig;

    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
      CxxErrorHandler();
    }

    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048) != HAL_OK) CxxErrorHandler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) CxxErrorHandler();
}


}} // mobilinkd::tnc
