// Copyright 2022 Mobilinkd LLC.

#pragma once

#include "Log.h"

#define BLAZE_THROW( EXCEPTION ) \
  ERROR(#EXCEPTION); \
  abort()

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <blaze/math/Matrix.h>
#include <blaze/math/Vector.h>

// STM32 headers define RNG, and this conflicts with Blaze.
#if defined(RNG)
#define RNG_SAVED_VALUE__ RNG
#undef RNG
#include <blaze/Math.h>
#define RNG RNG_SAVED_VALUE__
#undef RNG_SAVED_VALUE__
#else
#include <blaze/Math.h>
#endif
#pragma GCC diagnostic pop

#include <cmath>

namespace mobilinkd { namespace m17 {

template <typename FloatType, size_t SamplesPerSymbol>
struct KalmanFilter
{
    blaze::StaticVector<FloatType, 2> x;
    blaze::StaticMatrix<FloatType, 2, 2> P;
    blaze::StaticMatrix<FloatType, 2, 2> F;
    blaze::StaticMatrix<FloatType, 1, 2> H = {{1., 0.}};
    blaze::StaticMatrix<FloatType, 1, 1> R = {{0.5}};
    blaze::StaticMatrix<FloatType, 2, 2> Q = {{6.25e-13, 1.25e-12},{1.25e-12, 2.50e-12}};

    KalmanFilter()
    {
        reset(0.);
    }

    void reset(FloatType z)
    {
        x = {z, 0.};
        P = {{4., 0.}, {0., 0.00000025}};
        F = {{1., 1.}, {0., 1.}};
    }

    [[gnu::noinline]]
    auto update(FloatType z, size_t dt)
    {
        F(0,1) = FloatType(dt);

        x = F * x;
        P = F * P * blaze::trans(F) + Q;
        auto S = H * P * blaze::trans(H) + R;
        auto K = P * blaze::trans(H) * (1.0 / S(0, 0));

        // Normalize incoming index
        if (z - x[0] < (SamplesPerSymbol / -2.0)) // wrapped forwards 9 -> 0
            z += SamplesPerSymbol;
        else if (z - x[0] > (SamplesPerSymbol / 2.0)) // wrapped 0 -> 9
            z -= SamplesPerSymbol;

        auto y = z - H * x;

        x += K * y;

        // Normalize the filtered sample point
        while (x[0] >= SamplesPerSymbol) x[0] -= SamplesPerSymbol;
        while (x[0] < 0) x[0] += SamplesPerSymbol;
        P = P - K * H * P;
        return x;
    }
};

template <typename FloatType>
struct SymbolKalmanFilter
{
    blaze::StaticVector<FloatType, 2> x;
    blaze::StaticMatrix<FloatType, 2, 2> P;
    blaze::StaticMatrix<FloatType, 2, 2> F;
    blaze::StaticMatrix<FloatType, 1, 2> H = {{1., 0.}};
    blaze::StaticMatrix<FloatType, 1, 1> R = {{0.5}};
    blaze::StaticMatrix<FloatType, 2, 2> Q = {{6.25e-4, 1.25e-3},{1.25e-3, 2.50e-3}};

    SymbolKalmanFilter()
    {
        reset(0.);
    }

    void reset(FloatType z)
    {
        x = {z, 0.};
        P = {{4., 0.}, {0., 0.00000025}};
        F = {{1., 1.}, {0., 1.}};
    }

    [[gnu::noinline]]
    auto update(FloatType z, size_t dt)
    {
        F(0,1) = FloatType(dt);

        x = F * x;
        P = F * P * blaze::trans(F) + Q;
        auto S = H * P * blaze::trans(H) + R;
        auto K = P * blaze::trans(H) * (1.0 / S(0, 0));

        auto y = z - H * x;

        x += K * y;

        // Normalize the filtered sample point
        P = P - K * H * P;
        return x;
    }
};

}} // mobilinkd::m17
