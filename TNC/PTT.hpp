// Copyright 2015 Robert C. Riggs <rob@pangalactic.org>
// All rights reserved.

#ifndef INC_PTT_HPP_
#define INC_PTT_HPP_

#include "GPIO.hpp"
#include "Led.h"

namespace mobilinkd { namespace tnc {

struct PTT {
    virtual void on() = 0;
    virtual void off() = 0;
    virtual ~PTT() {}
};

struct SimplexPTT : PTT {
    void on() {
        led_tx_on();                // LED
        gpio::PTT_SIMPLEX::on();    // PTT
    }
    void off() {
        led_tx_off();               // LED
        gpio::PTT_SIMPLEX::off();   // PTT
    }
};


struct MultiplexPTT : PTT {
    void on() {
        led_tx_on();                // LED
        gpio::PTT_MULTIPLEX::on();  // PTT
    }
    void off() {
        led_tx_off();               // LED
        gpio::PTT_MULTIPLEX::off(); // PTT
    }
};

}} // mobilinkd::tnc


#endif // INC_PTT_HPP_
