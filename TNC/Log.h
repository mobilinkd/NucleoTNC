// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC_LOG_HPP_
#define MOBILINKD__TNC_LOG_HPP_

#ifdef __cplusplus
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

extern "C" {
#else
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#endif

void log_(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#ifdef KISS_LOGGING
#define DEBUG(...)    log_(0, __VA_ARGS__)
#define INFO(...)     log_(1, __VA_ARGS__)
#define WARN(...)     log_(2, __VA_ARGS__)
#define ERROR(...)    log_(3, __VA_ARGS__)
#define SEVERE(...)   log_(4, __VA_ARGS__)
#else
#define DEBUG(...)
#define INFO(...)
#define WARN(...)
#define ERROR(...)
#define SEVERE(...)
#endif

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc {

#define APP_TX_DATA_SIZE 64

struct Log {
    enum Level {debug = 0, info, warn, error, severe};

    Level level_;

    Log()
    : level_(Level::debug)
    {}

    Log(Level level)
    : level_(level)
    {}

    void setLevel(Level level) {level_ = level;}

    void log(Level level, const char *fmt, ...);
};

Log& log(void);

}} // mobilinkd::tnc

#endif // __cplusplus

#endif // MOBILINKD__TNC_LOG_HPP_
