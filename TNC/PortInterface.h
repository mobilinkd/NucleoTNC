// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD_PORTINTERFACE_H_
#define MOBILINKD_PORTINTERFACE_H_

#include "cmsis_os.h"

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_ioport(void);

void initNull(void);

int openNull(void);

void closeCDC(void);

int writeCDC(const uint8_t* data, uint32_t size, uint32_t timeout);

int writeLog(const uint8_t* data, uint32_t size, uint32_t timeout);
int writeTNC(const uint8_t* data, uint32_t size, uint32_t timeout);
int printTNC(const char* zstring, uint32_t timeout);


#ifdef __cplusplus
}
#endif

#endif /* MOBILINKD_PORTINTERFACE_H_ */
