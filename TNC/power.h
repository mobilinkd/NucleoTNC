// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__POWER_H_
#define MOBILINKD__TNC__POWER_H_

#ifdef __cplusplus
extern "C" {
#endif

void shutdown_normal(void);
void shutdown_with_usb(void);
void shutdown_safe_mode(void);

void wakeup();

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc {

/**
 * The type of power on or off process to follow.
 *
 * - POWERON occurs when the RTC backup domain has been erased due to
 *   complete power loss.  This should only happen if the battery is
 *   completely drained or removed.
 * - NORMAL occurs when the TNC is powered off and the TNC is not on USB
 *   power.  Note that this is the state when the TNC was powered off,
 *   not the state when it is powered on.
 * - USB occurs when the TNC is powered off and the TNC is connected to
 *   USB power.
 */
enum PowerType {POWERON, NORMAL, USB, SAFE};

void shutdown(PowerType type);
void wakeup(PowerType type);

}} // mobilinkd::tnc

#endif //__cplusplus

#endif // MOBILINKD__TNC__POWER_H_
