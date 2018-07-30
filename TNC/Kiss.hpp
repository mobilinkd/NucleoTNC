// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__KISS_HPP_
#define MOBILINKD__KISS_HPP_

#include <iterator>
#include <algorithm>
#include <cstdint>

#include "HdlcFrame.hpp"

namespace mobilinkd { namespace tnc { namespace kiss {

const uint8_t FRAME_DATA = 0x00;
const uint8_t FRAME_TX_DELAY = 0x01;
const uint8_t FRAME_P_PERSIST = 0x02;
const uint8_t FRAME_SLOT_TIME = 0x03;
const uint8_t FRAME_TX_TAIL = 0x04;
const uint8_t FRAME_DUPLEX = 0x05;
const uint8_t FRAME_HARDWARE = 0x06;
const uint8_t FRAME_LOG = 0x07;
const uint8_t FRAME_RETURN = 0xFF;

void handle_frame(uint8_t frame_type, hdlc::IoFrame* frame) __attribute__((optimize("-Os")));
// void handle_frame(uint8_t frame_type, hdlc::IoFrame* frame);

struct slip_encoder
{
    typedef std::forward_iterator_tag iterator_category;
    typedef size_t difference_type;
    typedef char value_type;
    typedef char& reference;
    typedef char* pointer;

    static const char FEND = 0xC0;
    static const char FESC = 0xDB;
    static const char TFEND = 0xDC;
    static const char TFESC = 0xDD;

    const char* packet_;
    size_t size_;
    size_t pos_;
    char current_;
    bool shifting_;

    slip_encoder()
    : packet_(0), size_(0), pos_(0), current_(0), shifting_(false)
    {
        set_current();
    }

    slip_encoder(const char* packet, size_t len)
    : packet_(packet), size_(len), pos_(0), current_(0), shifting_(false)
    {
        set_current();
    }

    void set_current() {
        if ((packet_[pos_] == FEND) or (packet_[pos_] == FESC)) {
            current_ = FESC;
            shifting_ = true;
        } else {
            current_ = packet_[pos_];
        }
    }

    char operator*() const {
        return current_;
    }

    slip_encoder& operator++() {

        if (!size_) return *this;

        if (shifting_) {
            shifting_ = false;
            current_ = (packet_[pos_] == FEND ? TFEND : TFESC);
            return *this;
        }
        pos_ += 1;

        if (pos_ != size_) {
            set_current();
        } else {
            packet_ = 0;
            pos_ = 0;
            size_ = 0;
        }

        return *this;
    }

    slip_encoder operator++(int) {
        slip_encoder tmp(*this);
        ++(*this);
        return tmp;
    }

    bool operator==(const slip_encoder& other) const {
        return (packet_ == other.packet_) and
            (pos_ == other.pos_) and (size_ == other.size_);
    }

    bool operator!=(const slip_encoder& other) const {
        return not ((*this) == other);
    }
};

struct slip_encoder2
{
    typedef std::forward_iterator_tag iterator_category;
    typedef size_t difference_type;
    typedef char value_type;
    typedef char& reference;
    typedef char* pointer;

    static const uint8_t FEND = 0xC0;
    static const uint8_t FESC = 0xDB;
    static const uint8_t TFEND = 0xDC;
    static const uint8_t TFESC = 0xDD;

    hdlc::IoFrame::iterator iter_;
    mutable char current_;
    mutable bool shifting_;

    slip_encoder2()
    : iter_(), current_(0), shifting_(false)
    {}

    slip_encoder2(hdlc::IoFrame::iterator iter)
    : iter_(iter), current_(0), shifting_(false)
    {}

    slip_encoder2(const slip_encoder2& other)
    : iter_(other.iter_), current_(other.current_), shifting_(other.shifting_)
    {}

    void set_current() const {
        uint8_t c = *iter_;
        if ((c == FEND) or (c == FESC)) {
            current_ = FESC;
            shifting_ = true;
        } else {
            current_ = c;
        }
    }

    char operator*() const {
        set_current();
        return current_;
    }

    slip_encoder2& operator++() {

        if (shifting_) {
            shifting_ = false;
            current_ = (*iter_ == FEND ? TFEND : TFESC);
            return *this;
        }
        ++iter_;

        return *this;
    }

    slip_encoder2 operator++(int) {
        slip_encoder2 tmp(*this);
        ++(*this);
        return tmp;
    }

    bool operator==(const slip_encoder2& other) const {
        return iter_ == other.iter_;
    }

    bool operator!=(const slip_encoder2& other) const {
        return iter_ != other.iter_;
    }
};

struct slip_decoder
{
    typedef std::forward_iterator_tag iterator_category;
    typedef size_t difference_type;
    typedef char value_type;
    typedef char& reference;
    typedef char* pointer;

    static const char FEND = 0xC0;
    static const char FESC = 0xDB;
    static const char TFEND = 0xDC;
    static const char TFESC = 0xDD;

    const char* packet_;
    size_t size_;
    size_t pos_;
    char current_;

    slip_decoder()
    : packet_(0), size_(0), pos_(0), current_(0)
    {
        set_current();
    }

    slip_decoder(const char* packet, size_t len)
    : packet_(packet), size_(len), pos_(0), current_(0)
    {
        set_current();
    }

    void set_current() {
        if (packet_[pos_] == FESC) {
            pos_ += 1;
            current_ = (packet_[pos_] == TFEND ? FEND : FESC);
        } else {
            current_ = packet_[pos_];
        }
    }

    char operator*() const {
        return current_;
    }

    slip_decoder& operator++() {

        if (!size_) return *this;

        pos_ += 1;

        if (pos_ != size_) {
            set_current();
        } else {
            packet_ = 0;
            pos_ = 0;
            size_ = 0;
        }

        return *this;
    }

    slip_decoder operator++(int) {
        slip_decoder tmp(*this);
        ++(*this);
        return tmp;
    }

    bool operator==(const slip_decoder& other) const {
        return (packet_ == other.packet_) and
            (pos_ == other.pos_) and (size_ == other.size_);
    }

    bool operator!=(const slip_decoder& other) const {
        return not ((*this) == other);
    }
};

}}} // mobilinkd::tnc::kiss

#endif // MOBILINKD_KISS_HPP_
