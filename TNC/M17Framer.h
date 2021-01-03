// Copyright 2020 Mobilinkd LLC.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>

namespace mobilinkd
{

template <size_t N = 368>
struct M17Framer
{
    using buffer_t = std::array<int8_t, N>;

    alignas(4) buffer_t buffer_;
    size_t index_ = 0;

    M17Framer()
    {
        reset();
    }

    static constexpr size_t size() { return N; }

    // Expects data in lower two bits of dibit.
    size_t operator()(int dibit, int8_t** result)
    {
        buffer_[index_++] = (dibit >> 1) ? 1 : -1;
        buffer_[index_++] = (dibit & 1) ? 1 : -1;
        if (index_ == N)
        {
            index_ = 0;
            *result = buffer_.begin();
            return N;
        }
        return 0;
    }

    // LLR mode
    size_t operator()(std::tuple<int8_t, int8_t> symbol, int8_t** result)
    {
        buffer_[index_++] = std::get<0>(symbol);
        buffer_[index_++] = std::get<1>(symbol);
        if (index_ == N)
        {
            index_ = 0;
            *result = buffer_.begin();
            return N;
        }
        return 0;
    }
    
    void reset()
    { 
        buffer_.fill(0);
        index_ = 0;
    }
};

} // mobilinkd
