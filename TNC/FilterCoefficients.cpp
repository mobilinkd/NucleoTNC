// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "FilterCoefficients.hpp"

namespace mobilinkd { namespace tnc { namespace filter {

namespace fir {

const TFirCoefficients<9>* AfskFilters[19] = {
  &dB_6,
  &dB_5,
  &dB_4,
  &dB_3,
  &dB_2,
  &dB_1,
  &dB0,
  &dB1,
  &dB2,
  &dB3,
  &dB4,
  &dB5,
  &dB6,
  &dB7,
  &dB8,
  &dB9,
  &dB10,
  &dB11,
  &dB12
};

} // fir


}}} // mobilinkd::tnc::filter
