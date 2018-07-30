// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__LED_H_
#define MOBILINKD__TNC__LED_H_

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void led_tx_on(void);
void led_tx_off(void);
void led_tx_toggle(void);
int led_tx_status(void);

void led_dcd_on(void);
void led_dcd_off(void);
int led_dcd_status(void);

void led_other_on(void);
void led_other_off(void);
void led_other_toggle(void);
int led_other_status(void);

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc {

struct Led
{
  GPIO_TypeDef* GPIOx_;
  uint16_t pin_;
  uint32_t count_;

  Led(GPIO_TypeDef* GPIOx, uint16_t pin)
  : GPIOx_(GPIOx), pin_(pin), count_(0)
  {}

  void on() {
    taskENTER_CRITICAL();
    ++count_;
    if (count_ == 1) {
      HAL_GPIO_WritePin(GPIOx_, pin_, GPIO_PIN_RESET);
    }
    taskEXIT_CRITICAL();
  }

  void off() {
    taskENTER_CRITICAL();
    if (count_ > 0) --count_;
    if (count_ == 0) {
      HAL_GPIO_WritePin(GPIOx_, pin_, GPIO_PIN_SET);
    }
    taskEXIT_CRITICAL();
  }

  void toggle() {
    taskENTER_CRITICAL();
    HAL_GPIO_TogglePin(GPIOx_, pin_);
    count_ = 0;
    taskEXIT_CRITICAL();
  }

  uint32_t status() const {return count_;}
};

Led& getTxLed();
Led& getDcdLed();
Led& getOtherLed();

}} // mobilinkd::tnc

#endif //__cplusplus

#endif /* MOBILINKD__TNC__LED_H_ */
