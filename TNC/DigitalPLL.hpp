#ifndef MOBILINKD__DIGITAL_PLL_H_
#define MOBILINKD__DIGITAL_PLL_H_

#include "Hysteresis.hpp"
#include "IirFilter.hpp"
#include "FirFilter.hpp"

#include "arm_math.h"

#include <algorithm>
#include <numeric>
#include <array>
#include <cmath>
#include <cstring>

namespace mobilinkd { namespace tnc {

namespace pll {


template <typename FloatType>
struct PLLResult {
    FloatType jitter;
    bool sample;
    bool locked;
};

// Lock low-pass filter taps (80Hz Bessel)
// scipy.signal:
//      b, a = bessel(4, [80.0/(1200/2)], 'lowpass')
//
const std::array<float, 5> lock_b = {
    1.077063e-03,4.308253e-03,6.462379e-03,4.308253e-03,1.077063e-03,
};
const std::array<float, 5> lock_a = {
    1.000000e+00,-2.774567e+00,2.962960e+00,-1.437990e+00,2.668296e-01,
};

// 64 Hz loop filter.
// scipy.signal:
//      loop_coeffs = firwin(9, [64.0/(1200/2)], width = None,
//          pass_zero = True, scale = True, window='hann')
//
const std::array<float, 7> loop_coeffs = {
//     0.08160962754214955, 0.25029850550446403, 0.3361837339067726, 0.2502985055044641, 0.08160962754214969
    3.196252e-02,1.204223e-01,2.176819e-01,2.598666e-01,2.176819e-01,1.204223e-01,3.196252e-02
};

} // pll

template <typename T>
struct BaseDigitalPLL
{
	typedef T float_type;
	typedef pll::PLLResult<float_type> result_type;

	float_type sample_rate_;
	float_type symbol_rate_;
	float_type sps_; 			///< Samples per symbol
	float_type limit_;			///< Samples per symbol / 2
	libafsk::BaseHysteresis<float_type> lock_;
    FirFilter<1, 7> loop_filter_{pll::loop_coeffs.begin()};
    IirFilter<5> lock_filter_{pll::lock_b, pll::lock_a};

	bool last_;
	float_type count_;
	
	bool sample_;
	float_type jitter_;
	uint8_t bits_;

	BaseDigitalPLL(float_type sample_rate, float_type symbol_rate)
	: sample_rate_(sample_rate), symbol_rate_(symbol_rate)
	, sps_(sample_rate / symbol_rate)
	, limit_(sps_ / float_type(2.0))
    , lock_(sps_ * float_type(0.03), sps_ * float_type(0.15), 1, 0)
	, last_(false), count_(0), sample_(false)
	, jitter_(0.0), bits_(1)
	{}

	result_type operator()(bool input)
	{
		sample_ = false;
		
        if (input != last_ or bits_ > 16) {
			// Record transition.
			last_ = input;

            if (count_ > limit_) {
                count_ -= sps_;
            }

            // Force lock off when no stimulus is present (squelch closed).
            const float_type adjust = bits_ > 16 ? 5.0 : 0.0;

            const float_type offset = count_ / bits_;
            const float_type jitter = loop_filter_(offset);
            const float_type abs_offset = std::abs(offset) + adjust;
			jitter_ = lock_filter_(abs_offset);

			count_ -= jitter / 2;

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

