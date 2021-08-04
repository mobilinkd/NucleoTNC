// Copyright 2015-2021 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "memory.hpp"

#include <boost/iterator/iterator_facade.hpp>

#include <cstdint>

namespace mobilinkd { namespace tnc { namespace buffer {


using boost::intrusive::list_base_hook;
using boost::intrusive::list;
using boost::intrusive::constant_time_size;

template <uint16_t SIZE, uint16_t CHUNK_SIZE = 256>
struct Pool {
    typedef memory::chunk<CHUNK_SIZE> chunk_type;
    typedef list<chunk_type, constant_time_size<false> > chunk_list;
    chunk_type segments[SIZE];
    chunk_list free_list;

    Pool() {
        for(uint16_t i = 0; i != SIZE; ++i) {
            free_list.push_back(segments[i]);
        }
    }

    bool allocate(chunk_list& list) {
        bool result = false;
        auto x = taskENTER_CRITICAL_FROM_ISR();
        if (!free_list.empty()) {
            list.splice(list.end(), free_list, free_list.begin());
            result = true;
        }
        taskEXIT_CRITICAL_FROM_ISR(x);
        return result;
    }

    void deallocate(chunk_list& list) {
        auto x = taskENTER_CRITICAL_FROM_ISR();
        free_list.splice(free_list.end(), list);
        taskEXIT_CRITICAL_FROM_ISR(x);
    }
};

template <typename POOL, POOL* allocator> struct SegmentedBufferIterator;

template <typename POOL, POOL* allocator>
struct SegmentedBuffer {
    typedef uint8_t value_type;
    typedef value_type* pointer;
    typedef value_type& reference;

    typedef SegmentedBufferIterator<POOL, allocator> iterator;
    typedef const SegmentedBufferIterator<POOL, allocator> const_iterator;

    typename POOL::chunk_list segments_;
    typename POOL::chunk_list::iterator current_;
    uint16_t size_;

    SegmentedBuffer()
    : segments_(), current_(segments_.end()), size_(0)
    {}

    ~SegmentedBuffer() {
        allocator->deallocate(segments_);
    }

    void clear() {
        if (size_) {
            allocator->deallocate(segments_);
            size_ = 0;
            current_ = segments_.end();
        }
    }

    uint16_t size() const {return size_;}

    bool resize(uint16_t size)
    {
        if (size <= size_)
        {
            size_ = size;
        }
        else
        {
            for (;size_ != size;)
            {
                if (!push_back(value_type()))
                    return false;
            }
        }
        return true;
    }

    bool push_back(value_type value) {
        uint16_t offset = size_ & 0xFF;
        if (offset == 0) { // Must allocate.
            if (not allocator->allocate(segments_))
                return false;
            current_ = segments_.end();
            --current_;
        }
        current_->buffer[offset] = value;
        ++size_;
        return true;
    }

    iterator begin() __attribute__((noinline)) {
        return iterator(segments_.begin(), 0);
    }
    iterator end()  __attribute__((noinline)) {
        return iterator(segments_.end(), size_);
    }
};

template <typename POOL, POOL* allocator>
struct SegmentedBufferIterator : public boost::iterator_facade<
    SegmentedBufferIterator<POOL, allocator>, uint8_t, boost::bidirectional_traversal_tag>
{
    typename POOL::chunk_list::iterator iter_;
    uint16_t index_;

    SegmentedBufferIterator()
    : iter_(), index_(0)
    {}

    SegmentedBufferIterator(typename POOL::chunk_list::iterator it, uint16_t index)
    : iter_(it), index_(index)
    {}

    friend class boost::iterator_core_access;

    void increment() {
        ++index_;
        if ((index_ & 0xFF) == 0) ++iter_;
    }

    void decrement() {
        if ((index_ & 0xFF) == 0) --iter_;
        --index_;
    }

    bool equal(SegmentedBufferIterator const& other) const {
        return (index_ == other.index_);
    }

    uint8_t& dereference() const {
        return iter_->buffer[index_ & 0xFF];
    }

};

}}} // mobilinkd::tnc::buffer
