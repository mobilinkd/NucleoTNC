// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.


#ifndef MOBILINKD__TNC__DIGIPEATER_HPP_
#define MOBILINKD__TNC__DIGIPEATER_HPP_

#include <cmsis_os.h>

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId digipeaterTaskHandle;
extern osMessageQId digipeaterQueueHandle;

void startDigipeaterTask(void* arg);
void beacon(void* arg);

#ifdef __cplusplus
} // extern "C"

#include "KissHardware.hpp"
#include "HdlcFrame.hpp"

namespace mobilinkd { namespace tnc {

/**
 * We only process ALL, BEACON, CQ, QST, GPSxxx and APxxxx TOCALLs,
 * and exact matches to our TOCALL and any of our aliases.
 *
 * The following constants are used:
 * - kiss::NUMBER_OF_ALIASES
 * - kiss::NUMBER_OF_BEACONS
 * - kiss::BEACON_PATH_LEN
 * - kiss::BEACON_TEXT_LEN
 * - kiss::CALLSIGN_LEN
 * - kiss::TOCALL
 */
struct Digipeater
{
  const kiss::Alias* aliases_;
  const kiss::Beacon* beacons_;

  Digipeater(const kiss::Alias* aliases, const kiss::Beacon* beacons)
  : aliases_(aliases), beacons_(beacons)
  {}

  /**
   * Scan the history table and remove outdated entries.
   */
  void clean_history()
  {

  }

  /**
   * Can the frame be digipeated?
   *
   *  - Does it match an active digi alias?
   *    - set = true
   *    - use = true
   *    - hops > 0
   *    - alias name matches first frame via OR preempt = true and alias name
   *      matches any frame via.
   *  - Does it not exist in history?
   * @param frame
   * @return
   */
  bool can_repeat(hdlc::IoFrame* frame)
  {
    return false;
  }

  hdlc::IoFrame* rewrite_frame(hdlc::IoFrame* frame)
  {
    frame->source(hdlc::IoFrame::DIGI_DATA);
    return frame;
  }
};


}} // mobilinkd::tnc

#endif // __cplusplus

#endif // MOBILINKD__TNC__DIGIPEATER_HPP_
