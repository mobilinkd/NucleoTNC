// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "PortInterface.hpp"

#include <algorithm>
#include <cstring>

namespace mobilinkd { namespace tnc {

uint8_t TxBuffer[TX_BUFFER_SIZE];
PortInterface* ioport{0};


int write(hdlc::IoFrame* frame, uint32_t timeout)
{
  if (mobilinkd::tnc::ioport == 0) return -1;

     return mobilinkd::tnc::ioport->write(frame, timeout);
}

}} // mobilinkd::tnc

int writeLog(const uint8_t* data, uint32_t size, uint32_t timeout)
{
  if (mobilinkd::tnc::ioport == 0) return -1;
  return mobilinkd::tnc::ioport->write(data, size, 7, timeout);
}

int writeTNC(const uint8_t* data, uint32_t size, uint32_t timeout)
{
  if (mobilinkd::tnc::ioport == 0) return -1;
  return mobilinkd::tnc::ioport->write(data, size, timeout);
}

int printTNC(const char* zstring, uint32_t timeout)
{
  if (mobilinkd::tnc::ioport == 0) return -1;
  return mobilinkd::tnc::ioport->write((uint8_t*) zstring, strlen(zstring), timeout);
}

