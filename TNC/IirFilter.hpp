// Copyright 2015-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <array>
#include <cstring>
#include <cstddef>

namespace mobilinkd { namespace tnc {

template <size_t N>
struct IirFilter {

	const std::array<float, N>& numerator_;
	const std::array<float, N>& denominator_;
	std::array<float, N> history_;

    IirFilter(const std::array<float, N>& b,
        const std::array<float, N>& a)
    : numerator_(b), denominator_(a)
    , history_()
    {
        history_.fill(0.0f);
    }

    ~IirFilter() {}

    static constexpr size_t size() { return N; }

	float operator()(float input)
	{

		for (size_t i = N - 1; i != 0; i--) history_[i] = history_[i - 1];
		
		history_[0] = input;

		for (size_t i = 1; i != N; i++) {
			history_[0] -= denominator_[i] * history_[i];
		}
		
		float result = 0;
		for (size_t i = 0; i != N; i++) {
			result += numerator_[i] * history_[i];
		}
		
		return result;
	}
};

}} // mobilinkd::tnc
