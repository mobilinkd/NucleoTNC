// Copyright 2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "IirFilter.hpp"

#include "stm32l4xx_hal.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <tuple>

namespace mobilinkd { namespace m17 {

struct Correlator
{
	static constexpr size_t SYMBOLS = 8;
	static constexpr size_t SAMPLES_PER_SYMBOL = 10;

	using value_type = float;
    using buffer_t = std::array<float, SYMBOLS * SAMPLES_PER_SYMBOL>;
    using sync_t = std::array<int8_t, SYMBOLS>;
    using sample_filter_t = tnc::IirFilter<3>;

    buffer_t buffer_;

    float limit_ = 0.;
    uint8_t symbol_pos_ = 0;
    uint8_t buffer_pos_ = 0;
    uint8_t prev_buffer_pos_ = 0;
    int code = -1;

    // IIR with Nyquist of 1/240.  This is used to determine the baseline
    // signal level, which is then used to scale the correlation value.
    // This makes the detector self-calibrating.
    static constexpr std::array<float,3> b = {4.24433681e-05, 8.48867363e-05, 4.24433681e-05};
    static constexpr std::array<float,3> a = {1.0, -1.98148851,  0.98165828};
    sample_filter_t sample_filter{b, a};
    std::array<int, SYMBOLS> tmp;

    void sample(float value);

    float correlate(sync_t sync);

    float limit() const {return limit_;}
    uint8_t index() const {return prev_buffer_pos_ % SAMPLES_PER_SYMBOL;}

    /**
     * Get the average outer symbol levels at a given index.  This makes trhee
     * assumptions.
     *
     *  1. The max symbol value is above 0 and the min symbol value is below 0.
     *  2. The samples at the given index only contain outer symbols.
     *  3. The index is a peak correlation index.
     *
     *  The first should hold true except for extreme frequency errors.  The
     *  second holds true for the sync words used for M17.
     */
    std::tuple<float, float> outer_symbol_levels(uint8_t sample_index);

    template <typename F>
    void apply(F func, uint8_t index)
    {
    	for (size_t i = index; i < buffer_.size(); i += SAMPLES_PER_SYMBOL)
    	{
    		func(buffer_[i]);
    	}
    }
};

struct Indicator
{
	GPIO_TypeDef* gpio;
	uint16_t pin;

	void on()
	{
		HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
	}

	void off()
	{
		HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
	}
};

template <typename Correlator>
struct SyncWord
{
	static constexpr size_t SYMBOLS = Correlator::SYMBOLS;
	static constexpr size_t SAMPLES_PER_SYMBOL = Correlator::SAMPLES_PER_SYMBOL;
	using value_type = typename Correlator::value_type;

	using buffer_t = std::array<int8_t, SYMBOLS>;
	using sample_buffer_t = std::array<value_type, SAMPLES_PER_SYMBOL>;

	buffer_t sync_word_;
	sample_buffer_t samples_;
	uint8_t pos_ = 0;
	uint8_t timing_index_ = 0;
	bool triggered_ = false;
	int8_t updated_ = 0;
	float magnitude_1_ = 1.f;
	float magnitude_2_ = -1.f;

	SyncWord(buffer_t&& sync_word, float magnitude_1, float magnitude_2 = std::numeric_limits<float>::lowest())
	: sync_word_(std::move(sync_word)), magnitude_1_(magnitude_1), magnitude_2_(magnitude_2)
	{}

	float triggered(Correlator& correlator)
	{
		float limit_1 = correlator.limit() * magnitude_1_;
		float limit_2 = correlator.limit() * magnitude_2_;
		auto value = correlator.correlate(sync_word_);

		return (value > limit_1 || value < limit_2) ? value : 0.0;
	}

	uint8_t operator()(Correlator& correlator)
	{
		auto value = triggered(correlator);

		value_type peak_value = 0;

		if (std::abs(value) > 0.0)
		{
			if (!triggered_)
			{
				samples_.fill(0);
				triggered_ = true;
			}
			samples_[correlator.index()] = value;
		}
		else
		{
			if (triggered_)
			{
				// Calculate the timing index on the falling edge.
				triggered_ = false;
				timing_index_ = 0;
				peak_value = value;
				uint8_t index = 0;
				for (auto f : samples_)
				{
					if (abs(f) > abs(peak_value))
					{
						peak_value = f;
						timing_index_ = index;
					}
					index += 1;
				}
				updated_ = peak_value > 0 ? 1 : -1;
			}
		}
		return timing_index_;
	}

	int8_t updated()
	{
		auto result = updated_;
		updated_ = 0;
		return result;
	}
};

}} // mobilinkd::m17
