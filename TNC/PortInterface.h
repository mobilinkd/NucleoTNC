// Copyright 2016-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "cmsis_os.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_ioport(void);

#ifndef NUCLEOTNC
void initCDC(void);
int openCDC(void);
void closeCDC(void);
int writeCDC(const uint8_t* data, uint32_t size, uint32_t timeout);
#endif

void initSerial(void);
int openSerial(void);
void closeSerial(void);

void initNull(void);
int openNull(void);

int writeLog(const uint8_t* data, uint32_t size, uint32_t timeout);
int writeTNC(const uint8_t* data, uint32_t size, uint32_t timeout);
int printTNC(const char* zstring, uint32_t timeout);


#ifdef __cplusplus
}
#endif
