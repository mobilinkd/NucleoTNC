// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Digipeater.h"
#include "Digipeater.hpp"
#include "IOEventTask.h"

void startDigipeaterTask(void* arg)
{
  using mobilinkd::tnc::Digipeater;
  using mobilinkd::tnc::hdlc::IoFrame;

  auto digi = static_cast<Digipeater*>(arg);
  for(;;)
  {
    osEvent evt = osMessageGet(digipeaterQueueHandle, osWaitForever);
    if (evt.status != osEventMessage) continue;

    uint32_t cmd = evt.value.v;
    if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
    {
      // this is a command, not a packet.
      return;
    }

    digi->clean_history();

    auto frame = static_cast<IoFrame*>(evt.value.p);

    if (!digi->can_repeat(frame)) continue;

    auto digi_frame = digi->rewrite_frame(frame);

  }
}

void onBeaconTimer1(void const *)
{

}
void onBeaconTimer2(void const *)
{

}
void onBeaconTimer3(void const *)
{

}
void onBeaconTimer4(void const *)
{

}

namespace mobilinkd { namespace tnc {

}}  // mobilinkd::tnc
