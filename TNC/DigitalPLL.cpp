// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "DigitalPLL.hpp"

namespace mobilinkd { namespace tnc {

namespace pll {

// Loop low-pass filter taps (64Hz Bessel)
float loop_b[] = {
    0.144668495309,
    0.144668495309,
};
float loop_a[] = {
    1.0,
    -0.710663009381,
};

// Lock low-pass filter taps (40Hz Bessel)
float lock_b[] = {
    0.0951079834025,
    0.0951079834025,
};
float lock_a[] = {
    1.0,
    -0.809784033195,
};

} // pll

}} // mobilinkd::tnc
