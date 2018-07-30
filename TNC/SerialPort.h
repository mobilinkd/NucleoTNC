// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include "stm32l4xx_hal.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void initSerial(void);
int openSerial(void);
void closeSerial(void);
void idleInterruptCallback(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* SERIALPORT_H_ */
