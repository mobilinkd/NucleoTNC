// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.


#ifndef MOBILINKD__TNC__DCD_H_
#define MOBILINKD__TNC__DCD_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

void dcd_on(void);  // DCD detected
void dcd_off(void); // DCD not detected
int dcd(void);      // Is DCD detected?

#ifdef __cplusplus
}
#endif

#endif // MOBILINKD__TNC__DCD_H_
