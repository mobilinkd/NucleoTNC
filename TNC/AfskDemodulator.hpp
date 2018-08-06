// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__AFSK_DEMODULATOR_HPP_
#define MOBILINKD__AFSK_DEMODULATOR_HPP_

#include <arm_math.h>
#include "AGC.hpp"
#include "DelayLine.hpp"
#include "AudioInput.hpp"
#include "DigitalPLL.hpp"
// #include "Filter.h"
#include "HdlcDecoder.hpp"
#include "Hysteresis.hpp"
#include "FirFilter.hpp"
#include "IirFilter.hpp"
#include "NRZI.hpp"
#include "GPIO.hpp"

#include <algorithm>
#include <functional>

namespace mobilinkd { namespace tnc { namespace afsk1200 {

#if 0
const float b[] = {
    4.57519926037e-06,
    2.28759963018e-05,
    4.57519926037e-05,
    4.57519926037e-05,
    2.28759963018e-05,
    4.57519926037e-06,
};

const float a[] = {
    1.0,
    -4.41489189545,
    7.82710410154,
    -6.96306748269,
    3.10736843037,
    -0.556366747391,
};

// Bessel 760Hz
const float b[] = {
    4.36034607e-06,
    2.18017304e-05,
    4.36034607e-05,
    4.36034607e-05,
    2.18017304e-05,
    4.36034607e-06,
};

const float a[] = {
    1.0,
    -4.32673235,
    7.51393353,
    -6.54579279,
    2.86009139,
    -0.50136025,
};
#endif

// Bessel 1200Hz 7-pole low-pass
const float b[] = {
    6.10481382e-07,
    4.27336967e-06,
    1.28201090e-05,
    2.13668484e-05,
    2.13668484e-05,
    1.28201090e-05,
    4.27336967e-06,
    6.10481382e-07,
};

const float a[] = {
    1.0,
    -5.56209875,
    13.34528507,
    -17.89828744,
    14.48661262,
    -7.07391246,
    1.92904679,
    -0.22656769,
};

typedef FirFilter<audio::ADC_BUFFER_SIZE, 9> emphasis_filter_type;
// typedef IirFilter<audio::ADC_BUFFER_SIZE, 2> emphasis_filter_type;

struct Demodulator {

    typedef float float_type;

    typedef std::function<float_type(float_type)> filter_function_type;

    static const size_t SYMBOL_RATE = 1200;
    static const size_t BUFFER_SIZE = 330;

    typedef BaseDigitalPLL<float_type> DPLL;

    size_t sample_rate_;
    emphasis_filter_type& audio_filter_;
    libafsk::BaseAGC<float_type> agc_;
    libafsk::BlockHysteresis<bool, audio::ADC_BUFFER_SIZE> input_comparator_;
    libafsk::FixedDelayLine<40> delay_line_;
    IirFilter<8> correlator_filter_;
    libafsk::BaseHysteresis<float_type> output_comparator_;
    DPLL pll_;
    libafsk::NRZI nrzi_;
    hdlc::Decoder hdlc_decoder_;
    bool locked_;
    float_type buffer_a[audio::ADC_BUFFER_SIZE];

    Demodulator(size_t sample_rate, emphasis_filter_type& c)
    : sample_rate_(sample_rate)
    , audio_filter_(c)
    , agc_(.01, .001, 0.3, 1000.0)
    , input_comparator_(-0.0005, 0.0005)
    , delay_line_(sample_rate, 0.000448)
    , correlator_filter_(b, a)
    , output_comparator_(-0.05, 0.05)
    , pll_(sample_rate, SYMBOL_RATE)
    , nrzi_(), hdlc_decoder_(false), locked_(false)
    {}

    hdlc::IoFrame* operator()(float* samples, size_t len);

    bool locked() const {return locked_;}
};


}}} // mobilinkd::tnc::afsk1200


#endif // MOBILINKD__AFSK_DEMODULATOR_HPP_
