// Copyright 2017-2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AfskDemodulator.hpp"

namespace mobilinkd { namespace tnc { namespace afsk1200 {

hdlc::IoFrame* Demodulator::operator()(q15_t* samples, size_t len)
{
    hdlc::IoFrame* result = 0;

    for (size_t i = 0; i != len; i++) {
        audio_filter_input_buffer[i] = float(samples[i]);
    }

    float* fa = audio_filter_(audio_filter_input_buffer);

    for (size_t i = 0; i != len; i++) {
        buffer_[i] = int16_t(fa[i]);
    }

    for (size_t i = 0; i != len; i++) {
        bool level = (buffer_[i] >= 0);
        bool delayed = delay_line_(level);
        buffer_[i] = (int16_t(level ^ delayed) << 1) - 1;
    }

    auto* fc = lpf_filter_.filter(buffer_);

    for (size_t i = 0; i != len; i++) {
        bool bit = fc[i] >= 0;
        auto pll = pll_(bit);

        if (pll.sample) {
            locked_ = pll.locked;

            // We will only ever get one frame because there are
            // not enough bits in a block for more than one.
            if (result) {
                auto tmp = hdlc_decoder_(nrzi_.decode(bit), true);
                if (tmp) hdlc::release(tmp);
            } else {
                result = hdlc_decoder_(nrzi_.decode(bit), true);
            }
        }
    }
    return result;
}

}}} // mobilinkd::tnc::afsk1200
