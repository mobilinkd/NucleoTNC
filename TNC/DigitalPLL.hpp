#ifndef MOBILINKD__DIGITAL_PLL_H_
#define MOBILINKD__DIGITAL_PLL_H_

#include "Hysteresis.hpp"
#include "IirFilter.hpp"

#include "arm_math.h"

#include <algorithm>
#include <numeric>

#include <cstring>

namespace mobilinkd { namespace tnc {

namespace pll {

// Loop low-pass filter taps (64Hz Bessel)
extern float loop_b[2];
extern float loop_a[2];
// Lock low-pass filter taps (40Hz Bessel)
extern float lock_b[2];
extern float lock_a[2];

template <typename FloatType>
struct PLLResult {
    FloatType jitter;
    bool sample;
    bool locked;
};

} // pll

template <typename T>
struct BaseDigitalPLL {

	static const size_t N = 16;

	typedef T float_type;
	typedef pll::PLLResult<float_type> result_type;

	float_type sample_rate_;
	float_type symbol_rate_;
	float_type sps_; 			///< Samples per symbol
	float_type limit_;			///< Samples per symbol / 2
	libafsk::BaseHysteresis<float_type> lock_;
    IirFilter<2> loop_filter_;
    IirFilter<2> lock_filter_;
	
	bool last_;
	float_type count_;
	
	bool sample_;
	float_type jitter_;
	uint8_t bits_;

	BaseDigitalPLL(float_type sample_rate, float_type symbol_rate)
	: sample_rate_(sample_rate), symbol_rate_(symbol_rate)
	, sps_(sample_rate / symbol_rate)
	, limit_(sps_ / 2.0f)
    , lock_(sps_ * 0.025, sps_ * .15, 1, 0)
    , loop_filter_(pll::loop_b, pll::loop_a)
    , lock_filter_(pll::lock_b, pll::lock_a)
	, last_(false), count_(0), sample_(false)
	, jitter_(0.0), bits_(1)
	{}

	result_type operator()(bool input)
	{
		
		sample_ = false;
		
        if (input != last_ or bits_ > 127) {
			// Record transition.
			last_ = input;

            if (count_ > limit_) {
                count_ -= sps_;
            }

            float_type offset = count_ / bits_;
            float_type jitter = loop_filter_(offset);
			jitter_ = lock_filter_(abs(offset));

			count_ -= jitter * sps_ * 0.023f;

            bits_ = 1;
		} else {
			if (count_ > limit_) {
				sample_ = true;
				count_ -= sps_;
				++bits_;
			}
		}

		count_ += 1;
		result_type result = {jitter_, sample_, locked()};
		return result;
	}
	
	bool locked() {
		return lock_(jitter_);
	}
	
	bool sample() const {
		return sample_;
	}
};

typedef BaseDigitalPLL<double> DigitalPLL;
typedef BaseDigitalPLL<float> FastDigitalPLL;

}} // mobilinkd::tnc

#endif // MOBILINKD__DIGITAL_PLL_H_

