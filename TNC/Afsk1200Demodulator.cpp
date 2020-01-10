// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Afsk1200Demodulator.hpp"

namespace mobilinkd { namespace tnc {

afsk1200::emphasis_filter_type Afsk1200Demodulator::filter_1;
afsk1200::emphasis_filter_type Afsk1200Demodulator::filter_2;
afsk1200::emphasis_filter_type Afsk1200Demodulator::filter_3;

afsk1200::Demodulator Afsk1200Demodulator::demod1(26400, Afsk1200Demodulator::filter_1);
afsk1200::Demodulator Afsk1200Demodulator::demod2(26400, Afsk1200Demodulator::filter_2);
afsk1200::Demodulator Afsk1200Demodulator::demod3(26400, Afsk1200Demodulator::filter_3);

const q15_t Afsk1200Demodulator::bpf_coeffs[FILTER_TAP_NUM] = {
    4,     0,    -5,   -10,   -13,   -12,    -9,    -4,    -2,    -4,   -12,   -26,
  -41,   -52,   -51,   -35,    -3,    39,    83,   117,   131,   118,    83,    36,
   -6,   -32,   -30,    -3,    36,    67,    66,    19,   -74,  -199,  -323,  -408,
 -421,  -344,  -187,    17,   218,   364,   417,   369,   247,   106,    14,    26,
  166,   407,   676,   865,   866,   605,    68,  -675, -1484, -2171, -2547, -2471,
-1895,  -882,   394,  1692,  2747,  3337,  3337,  2747,  1692,   394,  -882, -1895,
-2471, -2547, -2171, -1484,  -675,    68,   605,   866,   865,   676,   407,   166,
   26,    14,   106,   247,   369,   417,   364,   218,    17,  -187,  -344,  -421,
 -408,  -323,  -199,   -74,    19,    66,    67,    36,    -3,   -30,   -32,    -6,
   36,    83,   118,   131,   117,    83,    39,    -3,   -35,   -51,   -52,   -41,
  -26,   -12,    -4,    -2,    -4,    -9,   -12,   -13,   -10,    -5,     0,     4,
};

}} // mobilinkd::tnc

