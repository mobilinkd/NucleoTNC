// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "NullPort.hpp"

namespace mobilinkd { namespace tnc {

NullPort* getNullPort()
{
    static NullPort instance;
    return &instance;
}

}} // mobilinkd::tnc

