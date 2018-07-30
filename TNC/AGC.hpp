#ifndef MOBILINKD__AGC_H_
#define MOBILINKD__AGC_H_

#include <cmath>

namespace mobilinkd { namespace libafsk {

template <typename T>
struct BaseAGC {

	typedef T float_type;
	float_type attack_;
	float_type decay_;
	float_type reference_;
	float_type max_gain_;
	bool has_max_gain_;
	
	float_type gain_;
	
	BaseAGC(float_type attack, float_type decay, float_type reference = 1.0)
	: attack_(attack), decay_(decay), reference_(reference)
	, max_gain_(0.0), has_max_gain_(false), gain_(1.0)
	{}

	BaseAGC(float_type decay, float_type attack, float_type reference, float_type max_gain)
	: attack_(attack), decay_(decay), reference_(reference)
	, max_gain_(max_gain), has_max_gain_(true), gain_(1.0)
	{}
	
	float_type operator()(float_type value) {
		
		float_type output = value * gain_;
		float_type tmp = fabs(output) - reference_;
		float_type rate = decay_;
		
		if (fabs(tmp) > gain_) {
			rate = attack_;
		}

		gain_ -= tmp * rate;

		if (gain_ < 0.0f) {
			gain_ = .000001f;
		}

		if (has_max_gain_ and (gain_ > max_gain_)) {
			gain_ = max_gain_;
		}

		return output;
	}
};

typedef BaseAGC<double> AGC;
typedef BaseAGC<float> FastAGC;

}} // mobilinkd::libafsk

#endif // MOBILINKD__AGC_H_


