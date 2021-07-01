// Copyright 2015-2019 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__AFSK_DEMODULATOR_HPP_
#define MOBILINKD__AFSK_DEMODULATOR_HPP_

#include <arm_math.h>
#include "DelayLine.hpp"
#include "AudioInput.hpp"
#include "DigitalPLL.hpp"
#include "HdlcDecoder.hpp"
#include "Hysteresis.hpp"
#include "FirFilter.hpp"
#include "NRZI.hpp"

#include <algorithm>
#include <functional>

namespace mobilinkd { namespace tnc { namespace afsk1200 {

const size_t LPF_FILTER_LEN = 96;

const q15_t lpf_coeffs[] = {
    0,     1,     3,     5,     8,    11,    14,    17,    19,    20,    18,    14,
    7,    -2,   -16,   -33,   -53,   -76,  -101,  -126,  -151,  -174,  -194,  -208,
 -215,  -212,  -199,  -173,  -133,   -79,   -10,    74,   173,   287,   413,   549,
  693,   842,   993,  1142,  1287,  1423,  1547,  1656,  1747,  1817,  1865,  1889,
 1889,  1865,  1817,  1747,  1656,  1547,  1423,  1287,  1142,   993,   842,   693,
  549,   413,   287,   173,    74,   -10,   -79,  -133,  -173,  -199,  -212,  -215,
 -208,  -194,  -174,  -151,  -126,  -101,   -76,   -53,   -33,   -16,    -2,     7,
   14,    18,    20,    19,    17,    14,    11,     8,     5,     3,     1,     0,
};

static constexpr uint32_t ADC_BUFFER_SIZE = 88;
typedef FirFilter<ADC_BUFFER_SIZE, 9> emphasis_filter_type;

struct Demodulator {

    typedef float float_type;

    typedef std::function<float_type(float_type)> filter_function_type;

    static const size_t SYMBOL_RATE = 1200;

    typedef BaseDigitalPLL<float_type> DPLL;

    size_t sample_rate_;
    emphasis_filter_type& audio_filter_;
    float audio_filter_input_buffer[ADC_BUFFER_SIZE];
    libafsk::FixedDelayLine<40> delay_line_;
    DPLL pll_;
    Q15FirFilter<ADC_BUFFER_SIZE, LPF_FILTER_LEN> lpf_filter_;
    libafsk::NRZI nrzi_;
    hdlc::NewDecoder hdlc_decoder_;
    bool locked_;
    q15_t buffer_[ADC_BUFFER_SIZE];

    Demodulator(size_t sample_rate, emphasis_filter_type& c)
    : sample_rate_(sample_rate)
    , audio_filter_(c)
    , delay_line_(sample_rate, 0.000448)
    , pll_(sample_rate, SYMBOL_RATE)
    , nrzi_(), hdlc_decoder_(false), locked_(false)
    {
        lpf_filter_.init(lpf_coeffs);
    }

    hdlc::IoFrame* operator()(q15_t* samples, size_t len);

    bool locked() const {return locked_;}
};


}}} // mobilinkd::tnc::afsk1200


#endif // MOBILINKD__AFSK_DEMODULATOR_HPP_
