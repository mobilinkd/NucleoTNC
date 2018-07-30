// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Led.h"
#include "GPIO.hpp"

void led_tx_on()
{
  mobilinkd::tnc::getTxLed().on();
}

void led_tx_off()
{
  mobilinkd::tnc::getTxLed().off();
}

void led_tx_toggle()
{
    mobilinkd::tnc::gpio::LED_TX::toggle();
}

int led_tx_status()
{
  return mobilinkd::tnc::getTxLed().status();
}

void led_dcd_on()
{
  mobilinkd::tnc::getDcdLed().on();
}

void led_dcd_off()
{
  mobilinkd::tnc::getDcdLed().off();
}

int led_dcd_status()
{
  return mobilinkd::tnc::getDcdLed().status();
}

void led_other_on()
{
  mobilinkd::tnc::getOtherLed().on();
}

void led_other_off()
{
  mobilinkd::tnc::getOtherLed().off();
}

void led_other_toggle()
{
  mobilinkd::tnc::getOtherLed().toggle();
}

int led_other_status()
{
  return mobilinkd::tnc::getOtherLed().status();
}

namespace mobilinkd { namespace tnc {

Led& getTxLed()
{
  static Led led(LED_RED_GPIO_Port, LED_RED_Pin);
  return led;
}

Led& getDcdLed()
{
  static Led led(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  return led;
}

Led& getOtherLed()
{
  static Led led(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
  return led;
}

}} // mobilinkd::tnc



