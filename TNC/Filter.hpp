// Copyright 2015 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__FILTER_H_
#define MOBILINKD__TNC__FILTER_H_

namespace mobilinkd { namespace tnc {

struct Filter {
    virtual float operator()(float) = 0;
    virtual ~Filter() {}
};

}} // mobilinkd::tnc

#endif // MOBILINKD__TNC__FILTER_H_
