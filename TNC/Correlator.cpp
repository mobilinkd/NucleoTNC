// Copyright 2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Correlator.h"
#include "Log.h"

namespace mobilinkd { namespace m17 {

void Correlator::sample(float value)
{
	limit_ = sample_filter(std::abs(value));
	buffer_[buffer_pos_] = value;
	prev_buffer_pos_ = buffer_pos_;
	if (++buffer_pos_ == buffer_.size()) buffer_pos_ = 0;
}

float Correlator::correlate(sync_t sync)
{
	float result = 0.f;
	size_t pos = prev_buffer_pos_ + SAMPLES_PER_SYMBOL;

	for (size_t i = 0; i != sync.size(); ++i)
	{
		if (pos >= buffer_.size()) pos -= buffer_.size(); // wrapped
		result += sync[i] * buffer_[pos];
		pos += SAMPLES_PER_SYMBOL;
	}
	return result;
}


/**
 * Get the average outer symbol levels at a given index.  This makes three
 * assumptions.
 *
 *  1. The max symbol value is above 0 and the min symbol value is below 0.
 *  2. The samples at the given index only contain outer symbols.
 *  3. The index is a peak correlation index.
 *
 *  The first should hold true except for extreme frequency errors.  The
 *  second holds true for the sync words used for M17.
 */
std::tuple<float, float> Correlator::outer_symbol_levels(uint8_t sample_index)
{
	float min_sum = 0;
	float max_sum = 0;
	uint8_t min_count = 0;
	uint8_t max_count = 0;
	uint8_t index = 0;
	for (size_t i = sample_index; i < buffer_.size(); i += SAMPLES_PER_SYMBOL)
	{
		tmp[index++] = buffer_[i] * 1000.f;
		max_sum += buffer_[i] * (buffer_[i] > 0.f);
		min_sum += buffer_[i] * (buffer_[i] < 0.f);
		max_count += (buffer_[i] > 0.f);
		min_count += (buffer_[i] < 0.f);
	}
	INFO("osl: %d, %d, %d, %d,%d, %d, %d, %d",
		tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7]);
	return std::make_tuple(min_sum / min_count, max_sum / max_count);
}

}} // mobilinkd::m17
