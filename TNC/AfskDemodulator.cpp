// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AfskDemodulator.hpp"

namespace mobilinkd { namespace tnc { namespace afsk1200 {


hdlc::IoFrame* Demodulator::operator()(float* samples, size_t len)
{
    using namespace mobilinkd::tnc::gpio;

    hdlc::IoFrame* result = 0;

    float* fa = audio_filter_(samples);
    std::transform(fa, fa + audio::ADC_BUFFER_SIZE, buffer_a, agc_);
    auto levels = input_comparator_(buffer_a);

    for (size_t i = 0; i != len; i++) {

        bool delayed = delay_line_(levels[i]);
        float_type x = float_type(levels[i] ^ delayed) - .5f;
        float_type fc = correlator_filter_(x);
        bool bit = output_comparator_(fc);
        auto pll = pll_(bit);

        if (pll.sample) {
            if (locked_ != pll.locked) {
                locked_ = pll.locked;
            }

            // We will only ever get one frame because there are
            // not enough bits in a block for more than one.
            if (result) {
                auto tmp = hdlc_decoder_(nrzi_.decode(bit), pll.locked);
                if (tmp) hdlc::ioFramePool().release(tmp);
            } else {
                result = hdlc_decoder_(nrzi_.decode(bit), pll.locked);
            }
        }
    }
    return result;
}

}}} // mobilinkd::tnc::afsk1200
