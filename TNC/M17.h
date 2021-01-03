// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <array>
#include <cstdint>

namespace mobilinkd { namespace m17 {

constexpr size_t FILTER_TAP_NUM = 80;

extern const std::array<int16_t, FILTER_TAP_NUM> rrc_taps;
constexpr std::array<uint8_t, 2> LSF_SYNC = { 0x55, 0xF7 };
constexpr std::array<uint8_t, 2> STREAM_SYNC = { 0xFF, 0x5D };
constexpr std::array<uint8_t, 2> PACKET_SYNC = { 0x75, 0xFF };

inline constexpr uint16_t sync_word(std::array<uint8_t, 2> sw)
{
    return sw[0] * 256 + sw[1];
}

}} // mobilinkd::m17
