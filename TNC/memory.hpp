// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__MEMORY_HPP_
#define MOBILINKD__MEMORY_HPP_

#include "cmsis_os.h"

#include <boost/intrusive/list.hpp>

#include <cstdint>

namespace mobilinkd { namespace tnc { namespace memory {

using boost::intrusive::list_base_hook;
using boost::intrusive::list;
using boost::intrusive::constant_time_size;

template <uint16_t BLOCK_SIZE = 256>
struct chunk : public list_base_hook<>
{
    uint8_t buffer[BLOCK_SIZE];

    static uint16_t constexpr size() { return BLOCK_SIZE; }

    chunk()
    : list_base_hook<>(), buffer()
    {}
};


template <uint16_t SIZE, uint16_t CHUNK_SIZE=256>
struct Pool {
    typedef chunk<CHUNK_SIZE> chunk_type;
    typedef list<chunk_type, constant_time_size<false> > chunk_list;

    chunk_type segments[SIZE];
    chunk_list free_list;

    Pool() {
        for(uint16_t i = 0; i != SIZE; ++i) {
            free_list.push_back(segments[i]);
        }
    }

    void init() {
        free_list.clear();
        for(uint16_t i = 0; i != SIZE; ++i) {
            free_list.push_back(segments[i]);
        }
    }

    chunk_type* allocate() {
        auto x = taskENTER_CRITICAL_FROM_ISR();
        chunk_type* result = 0;
        if (not free_list.empty()) {
            result = &free_list.front();
            free_list.pop_front();
        }
        taskEXIT_CRITICAL_FROM_ISR(x);
        return result;
    }

    void deallocate(chunk_type* item) {
        auto x = taskENTER_CRITICAL_FROM_ISR();
        free_list.push_back(*item);
        taskEXIT_CRITICAL_FROM_ISR(x);
    }
};


}}} // mobilinkd::tnc::memory

#endif // MOBILINKD__MEMORY_HPP_
