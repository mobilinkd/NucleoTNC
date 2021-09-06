// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

namespace mobilinkd { namespace m17 {

constexpr size_t FILTER_TAP_NUM = 80;
constexpr size_t FILTER_TAP_NUM_9 = 90;
constexpr size_t FILTER_TAP_NUM_11 = 110;
constexpr size_t FILTER_TAP_NUM_15 = 150;
constexpr size_t FILTER_TAP_NUM_21 = 210;

extern const std::array<int16_t, FILTER_TAP_NUM> rrc_taps;
extern const std::array<int16_t, FILTER_TAP_NUM_9> rrc_taps_9;
extern const std::array<int16_t, FILTER_TAP_NUM_11> rrc_taps_11;
extern const std::array<int16_t, FILTER_TAP_NUM_15> rrc_taps_15;
extern const std::array<int16_t, FILTER_TAP_NUM_21> rrc_taps_21;

extern const std::array<float, FILTER_TAP_NUM_9> rrc_taps_f9;
extern const std::array<float, FILTER_TAP_NUM_15> rrc_taps_f15;

constexpr std::array<uint8_t, 2> LSF_SYNC = { 0x55, 0xF7 };
constexpr std::array<uint8_t, 2> STREAM_SYNC = { 0xFF, 0x5D };
constexpr std::array<uint8_t, 2> PACKET_SYNC = { 0x75, 0xFF };
constexpr std::array<uint8_t, 2> BERT_SYNC = { 0xDF, 0x55 };
constexpr std::array<uint8_t, 2> EOT_SYNC = { 0x55, 0x5D };

inline constexpr uint16_t sync_word(std::array<uint8_t, 2> sw)
{
    return sw[0] * 256 + sw[1];
}

}} // mobilinkd::m17
