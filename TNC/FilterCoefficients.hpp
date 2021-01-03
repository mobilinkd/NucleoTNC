// Copyright 2015-2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "IirFilter.hpp"
#include "FirFilter.hpp"

namespace mobilinkd { namespace tnc { namespace filter {

namespace fir {

// 1200Hz = -12dB, 2200Hz = 0dB; 3653Hz cutoff, 4.93 gain; cosine.
const TFirCoefficients<9> dB12 = {
    {
        0.0223997567081,
        -0.132588208904,
        -0.590869965255,
        -1.12325491747,
        3.55525416434,
        -1.12325491747,
        -0.590869965255,
        -0.132588208904,
        0.0223997567081,
    }
};

// 1200Hz = -11dB, 2200Hz = 0dB; 3537Hz cutoff, 4.51 gain; cosine.
const TFirCoefficients<9> dB11 = {
    {
        0.0138943225784,
        -0.137800909104,
        -0.544488104185,
        -1.00269495093,
        3.29019584315,
        -1.00269495093,
        -0.544488104185,
        -0.137800909104,
        0.0138943225784,
    }
};

// 1200Hz = -10dB, 2200Hz = 0dB; 3405Hz cutoff, 4.11 gain; cosine.
const TFirCoefficients<9> dB10 = {
    {
        0.00564557245371,
        -0.141643920093,
        -0.498513944796,
        -0.887264289827,
        3.03792032485,
        -0.887264289827,
        -0.498513944796,
        -0.141643920093,
        0.00564557245371,
    }
};

// 1200Hz = -9dB, 2200Hz = 0dB; 3252Hz cutoff, 3.7 gain; cosine.
const TFirCoefficients<9> dB9 = {
    {
        -0.00232554104135,
        -0.142858725752,
        -0.449053780255,
        -0.770264863826,
        2.77651146344,
        -0.770264863826,
        -0.449053780255,
        -0.142858725752,
        -0.00232554104135,
    }
};

// 1200Hz = -8dB, 2200Hz = 0dB; 3075Hz cutoff, 3.31 gain; cosine.
const TFirCoefficients<9> dB8 = {
    {
        -0.0096785005294,
        -0.141786249744,
        -0.399423790874,
        -0.658608816643,
        2.52741445003,
        -0.658608816643,
        -0.399423790874,
        -0.141786249744,
        -0.0096785005294,
    }
};

// 1200Hz = -7dB, 2200Hz = 0dB; 2874Hz cutoff, 2.949 gain; cosine.
const TFirCoefficients<9> dB7 = {
    {
        -0.0159546975608,
        -0.137623905223,
        -0.349491872081,
        -0.553149017309,
        2.28934729422,
        -0.553149017309,
        -0.349491872081,
        -0.137623905223,
        -0.0159546975608,
    }
};

// 1200Hz = -6dB, 2200Hz = 0dB; 2640Hz cutoff, 2.59 gain; cosine.
const TFirCoefficients<9> dB6 = {
    {
        -0.0209448226653,
        -0.130107651829,
        -0.299004731072,
        -0.45336946386,
        2.0629448761,
        -0.45336946386,
        -0.299004731072,
        -0.130107651829,
        -0.0209448226653,
    }
};

// 1200Hz = -5dB, 2200Hz = 0dB; 2372Hz cutoff, 2.26 gain; cosine.
const TFirCoefficients<9> dB5 = {
    {
        -0.0209448226653,
        -0.130107651829,
        -0.299004731072,
        -0.45336946386,
        2.0629448761,
        -0.45336946386,
        -0.299004731072,
        -0.130107651829,
        -0.0209448226653,
    }
};


// 1200Hz = -4dB, 2200Hz = 0dB; 2064Hz cutoff, 1.96 gain; cosine.
const TFirCoefficients<9> dB4 = {
    {
        -0.0209448226653,
        -0.130107651829,
        -0.299004731072,
        -0.45336946386,
        2.0629448761,
        -0.45336946386,
        -0.299004731072,
        -0.130107651829,
        -0.0209448226653,
    }
};

// 1200Hz = -3dB, 2200Hz = 0dB; 1700Hz cutoff, 1.68 gain; cosine.
const TFirCoefficients<9> dB3 = {
    {
        -0.0231416146776,
        -0.0833375337803,
        -0.147937602401,
        -0.197411259519,
        1.46066084756,
        -0.197411259519,
        -0.147937602401,
        -0.0833375337803,
        -0.0231416146776,
    }
};


// 1200Hz = -2dB, 2200Hz = 0dB; 1270Hz cutoff, 1.44 gain; cosine.
const TFirCoefficients<9> dB2 = {
    {
        -0.0185923370593,
        -0.0601029235689,
        -0.0996864670836,
        -0.128090353439,
        1.30017105427,
        -0.128090353439,
        -0.0996864670836,
        -0.0601029235689,
        -0.0185923370593,
    }
};

// 1200Hz = -1dB, 2200Hz = 0dB; 730Hz cutoff, 1.22 gain; cosine.
const TFirCoefficients<9> dB1 = {
    {
        -0.0107931468169,
        -0.0322211933056,
        -0.0506402474814,
        -0.0630689498437,
        1.1522865023,
        -0.0630689498437,
        -0.0506402474814,
        -0.0322211933056,
        -0.0107931468169,
    }
};

const TFirCoefficients<9> dB0 = {
    {
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
    }
};

// 1200Hz = 0dB, 2200Hz = -1dB; 4280Hz cutoff, 1.045 gain; cosine.
const TFirCoefficients<9> dB_1 = {
    {
        -0.0107931468169,
        -0.0322211933056,
        -0.0506402474814,
        -0.0630689498437,
        1.1522865023,
        -0.0630689498437,
        -0.0506402474814,
        -0.0322211933056,
        -0.0107931468169,
    }
};

// 1200Hz = 0dB, 2200Hz = -2dB; 3098Hz cutoff, 1.1 gain; cosine.
const TFirCoefficients<9> dB_2 = {
    {
        0.00299520319909,
        0.0482175156295,
        0.137632853632,
        0.228067265055,
        0.266174324969,
        0.228067265055,
        0.137632853632,
        0.0482175156295,
        0.00299520319909,
    }
};

// 1200Hz = 0dB, 2200Hz = -3dB; 1830Hz cutoff, 1.149 gain; cosine.
const TFirCoefficients<9> dB_3 = {
    {
        0.0221215152936,
        0.0832006412609,
        0.151534395598,
        0.205025020719,
        0.225236854257,
        0.205025020719,
        0.151534395598,
        0.0832006412609,
        0.0221215152936,
    }
};

// 1200Hz = 0dB, 2200Hz = -4dB; 2606Hz cutoff, 1.194 gain; boxcar.
const TFirCoefficients<9> dB_4 = {
    {
        0.0498539382844,
        0.103801174967,
        0.153695746099,
        0.188874162863,
        0.201549955573,
        0.188874162863,
        0.153695746099,
        0.103801174967,
        0.0498539382844,
    }
};

// 1200Hz = 0dB, 2200Hz = -5dB; 2174Hz cutoff, 1.237 gain; boxcar.
const TFirCoefficients<9> dB_5 = {
    {
        0.0782137588209,
        0.118736939542,
        0.153156300897,
        0.176223458893,
        0.184339083696,
        0.176223458893,
        0.153156300897,
        0.118736939542,
        0.0782137588209,
    }
};

// 1200Hz = 0dB, 2200Hz = -6dB; 1706Hz cutoff, 1.275 gain; boxcar.
const TFirCoefficients<9> dB_6 = {
    {
        0.104477241089,
        0.130913242609,
        0.151854419973,
        0.165293215366,
        0.169923761926,
        0.165293215366,
        0.151854419973,
        0.130913242609,
        0.104477241089,
    }
};

extern const TFirCoefficients<9>* AfskFilters[19];

} // fir


}}} // mobilinkd::tnc::filter
