// Copyright 2020 Mobilinkd LLC.

#pragma once

#include <array>
#include <cstdint>
#include <string_view> // Don't have std::span in C++17.
#include <stdexcept>

namespace mobilinkd
{

struct LinkSetupFrame
{
    using call_t = std::array<char,10>;             // NUL-terminated C-string.
    using encoded_call_t = std::array<uint8_t, 6>;
    using frame_t = std::array<uint8_t, 30>;
    using nonce_t = std::string_view;               // std::span would be better here.

    enum TxType { PACKET, STREAM };
    enum DataType { DT_RESERVED, DATA, VOICE, MIXED };
    enum EncType { NONE, AES, LFSR, ET_RESERVED };

    call_t mycall_ = {0};
    call_t tocall_ = {0};
    TxType  tx_type_ = TxType::STREAM;
    DataType data_type_ = DataType::VOICE;
    EncType encryption_type_ = EncType::NONE;

    template <size_t N>
    static encoded_call_t encode_callsign(std::array<char, N> callsign)
    {
        // Encode the characters to base-40 digits.
        uint64_t encoded = 0;

        std::reverse(callsign.begin(), callsign.end());

        for (auto c : callsign)
        {
            if (c == 0) continue;
            encoded *= 40;
            if (c >= 'A' and c <= 'Z')
            {
                encoded += c - 'A' + 1;
            }
            else if (c >= '0' and c <= '9')
            {
                encoded += c - '0' + 27;
            }
            else if (c == '-')
            {
                encoded += 37;
            }
            else if (c == '/')
            {
                encoded += 38;
            }
            else if (c == '.')
            {
                encoded += 39;
            }
        }
        const auto p = reinterpret_cast<uint8_t*>(&encoded);

        encoded_call_t result;
        std::copy(p, p + 6, result.rbegin());
        
        return result;
    }

    static call_t decode_callsign(encoded_call_t callsign)
    {
        static const char callsign_map[] = "xABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";

        uint64_t encoded = 0; // This only works on little endian architectures.
        auto p = reinterpret_cast<uint8_t*>(&encoded);
        std::copy(callsign.rbegin(), callsign.rend(), p);

        // decode each base-40 digit and map them to the appriate character.
        call_t result;
        result.fill(0);
        size_t index = 0;
        while (encoded)
        {
            result[index++] = callsign_map[encoded % 40];
            encoded /= 40;
        }

        return result;
    }


    LinkSetupFrame()
    {}

    LinkSetupFrame& myCall(const char*)
    {
        return *this;
    }
};

} // mobilinkd
