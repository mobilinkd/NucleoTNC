// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "DCD.h"
#include "LEDIndicator.h"
#include "GPIO.hpp"

bool& dcd_status()
{
    static bool dcd_status{false};
    return dcd_status;
}
void dcd_on(void)
{
  rx_on();
  dcd_status() = true;
}

void dcd_off(void)
{
  rx_off();
  dcd_status() = false;
}

int dcd(void)
{
  return dcd_status();
}



