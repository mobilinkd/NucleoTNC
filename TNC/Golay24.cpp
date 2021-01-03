// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Golay24.h"

namespace mobilinkd {
namespace Golay24
{

const std::array<SyndromeMapEntry, LUT_SIZE> LUT = make_lut();

bool decode(uint32_t input, uint32_t& output)
{
    auto syndrm = syndrome(input >> 1);
    auto it = std::lower_bound(LUT.begin(), LUT.end(), syndrm,
        [](const SyndromeMapEntry& sme, uint32_t val){
            return (sme.a >> 8) < val;
        });

    if ((it->a >> 8) == syndrm)
    {
        // Build the correction from the compressed entry.
        auto correction = ((((it->a & 0xFF) << 16) | it->b) << 1);
        // Apply the correction to the input.
        output = input ^ correction;
        // Only test parity for 3-bit errors.
        return __builtin_popcount(syndrm) < 3 || !parity(output);
    }

    return false;
}

} // Golay24
} // mobilinkd
