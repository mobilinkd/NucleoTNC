// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__IO_EVENT_TASK_H_
#define MOBILINKD__TNC__IO_EVENT_TASK_H_

#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void startIOEventTask(void const* argument);
void startCdcBlinker(void const* argument);

#ifdef __cplusplus
}

extern osMessageQId ioEventQueueHandle;

namespace mobilinkd { namespace tnc {

void print_startup_banner() __attribute__((noinline));
void start_cdc_blink();
void stop_cdc_blink();

}} // mobilinkd::tnc

#endif


#endif /* MOBILINKD__TNC__IO_EVENT_TASK_H_ */
