// Copyright 2017 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__GPIO_HPP_
#define MOBILINKD__TNC__GPIO_HPP_

#include "main.h"
#include "stm32l4xx_hal.h"
#include <cstdint>

namespace mobilinkd { namespace tnc {

template <uint32_t BANK, uint16_t PIN>
struct GPIO {
    static void on() {
        HAL_GPIO_WritePin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN, GPIO_PIN_SET);
    }
    static void off() {
        HAL_GPIO_WritePin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN, GPIO_PIN_RESET);
    }
    static void toggle() {
        HAL_GPIO_TogglePin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN);
    }
    static bool get() {
        return HAL_GPIO_ReadPin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN);
    }
#if 0
    static bool operator bool() {
        return HAL_GPIO_ReadPin(BANK, PIN);
    }
    static bool operator=(bool value) {
        return HAL_GPIO_WritePin(BANK, PIN, GPIO_PinState(value));
    }
#endif
};

namespace gpio {

typedef GPIO<(uint32_t)GPIOA_BASE,PTT_S_Pin> PTT_SIMPLEX;
typedef GPIO<(uint32_t)GPIOA_BASE,PTT_M_Pin> PTT_MULTIPLEX;
typedef GPIO<(uint32_t)GPIOA_BASE,LED_RED_Pin> LED_TX;
typedef GPIO<(uint32_t)GPIOA_BASE,LED_GREEN_Pin> LED_DCD;
typedef GPIO<(uint32_t)GPIOA_BASE,LED_YELLOW_Pin> LED_OTHER;
typedef GPIO<(uint32_t)GPIOB_BASE,AUDIO_OUT_ATTEN_Pin> AUDIO_OUT_ATTEN;
typedef GPIO<(uint32_t)GPIOB_BASE,LD3_Pin> LD3;

}}} // mobilinkd::tnc::gpio

#endif // MOBILINKD__TNC__GPIO_HPP_
