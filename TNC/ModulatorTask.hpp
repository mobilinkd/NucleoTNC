// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.


#ifndef MOBILINKD__MODULATOR_TASK_HPP_
#define MOBILINKD__MODULATOR_TASK_HPP_

#include "HDLCEncoder.hpp"
#include "AFSKModulator.hpp"
#include "PTT.hpp"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

extern mobilinkd::tnc::SimplexPTT simplexPtt;
extern mobilinkd::tnc::MultiplexPTT multiplexPtt;

extern mobilinkd::tnc::AFSKModulator* modulator;
extern mobilinkd::tnc::hdlc::Encoder* encoder;

mobilinkd::tnc::AFSKModulator& getModulator();
mobilinkd::tnc::hdlc::Encoder& getEncoder();

void startModulatorTask(void const * argument);

enum class PTT {SIMPLEX, MULTIPLEX};

void setPtt(PTT ptt);

void updatePtt(void);

#ifdef __cplusplus
}
#endif

#endif // MOBILINKD__MODULATOR_TASK_HPP_
