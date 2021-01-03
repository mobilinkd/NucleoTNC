// Copyright 2015-2020 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "PTT.hpp"
#include "Encoder.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace mobilinkd { namespace tnc {

class Modulator;

namespace hdlc {
class Encoder;
}

}}

extern mobilinkd::tnc::SimplexPTT simplexPtt;
extern mobilinkd::tnc::MultiplexPTT multiplexPtt;

mobilinkd::tnc::Modulator& getModulator();
mobilinkd::Encoder& getEncoder();

void startModulatorTask(void const * argument);

enum class PTT {SIMPLEX, MULTIPLEX};

void setPtt(PTT ptt);

void updatePtt(void);
void updateModulator(void);

#ifdef __cplusplus
}
#endif
