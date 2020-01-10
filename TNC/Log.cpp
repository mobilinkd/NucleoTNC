// Copyright 2015 Rob RIggs <rob@mobilinkd.com>
// All rights reserved.

#include <Log.h>
#include "PortInterface.hpp"

void log_(int level, const char* fmt, ...)
{
  if (level < mobilinkd::tnc::log().level_) return;
  if (!mobilinkd::tnc::ioport) return;

  va_list args;
  va_start(args, fmt);
  char* buffer = 0;
  int len = vasiprintf(&buffer, fmt, args);
  va_end(args);

  if (len >= 0) {
      mobilinkd::tnc::ioport->write((uint8_t*)buffer, len, 7, 10);
      free(buffer);
  } else {
      mobilinkd::tnc::ioport->write((uint8_t*) "Allocation Error\r\n", 18, 7, 10);
  }
}

namespace mobilinkd { namespace tnc {

#ifdef KISS_LOGGING

Log& log(void) {
    static Log log(Log::Level::info);
    return log;
}

#endif

#if 1
void Log::log(Level level, const char* fmt, ...) {

    if (level < level_) return;

    va_list args;
    va_start(args, fmt);
    char* buffer = 0;
    int len = vasiprintf(&buffer, fmt, args);
    va_end(args);

    if (len >= 0) {
        ioport->write((uint8_t*)buffer, len, 10);
        free(buffer);
    } else {
        ioport->write((uint8_t*) "Allocation Error\r\n", 18, 10);
    }
}
#else
void Log::log(Level level, const char* fmt, ...) {

    if (level < level_) return;
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\r\n");
}
#endif

}} // mobilinkd::tnc

