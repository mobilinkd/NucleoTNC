#ifndef MOBILINKD__HYSTERESIS_H_
#define MOBILINKD__HYSTERESIS_H_

#include <cstddef>

namespace mobilinkd { namespace libafsk {

template <typename T>
struct BaseHysteresis {

	typedef T float_type;
	float_type min_;
	float_type max_;
	
	int low_;
	int high_;
	
	float_type last_;
	
	BaseHysteresis(float_type minimum, float_type maximum, int low = 0, int high = 1)
	: min_(minimum), max_(maximum), low_(low), high_(high), last_(0.0)
	{}
	
	int operator()(float_type value) {
		if (value <= min_) {
			last_ = low_;
		} else if (value >= max_) {
			last_ = high_;
		}
		
		return last_;
	}
};

typedef BaseHysteresis<double> Hysteresis;
typedef BaseHysteresis<float> FastHysteresis;

template <typename T, size_t N>
struct BlockHysteresis {

    typedef T* result_type;

    float min_;
    float max_;

    T low_;
    T high_;

    T last_;
    T buffer_[N];

    BlockHysteresis(float minimum, float maximum, T low = 0, T high = 1)
    : min_(minimum), max_(maximum), low_(low), high_(high), last_(low)
    , buffer_()
    {}

    result_type operator()(float* value) {

        for (size_t i = 0; i != N; ++i) {
            if (value[i] <= min_) {
                last_ = low_;
            } else if (value[i] >= max_) {
                last_ = high_;
            }
            buffer_[i] = last_;
        }

        return buffer_;
    }
};
/*
template <bool, size_t N>
struct BlockHysteresis {

    typedef uint8_t* result_type;

    float min_;
    float max_;

    bool low_;
    bool high_;

    bool last_;
    uint8_t buffer_[(N + 7) / 8];

    BlockHysteresis(float minimum, float maximum, bool low = false, bool high = true)
    : min_(minimum), max_(maximum), low_(low), high_(high), last_(low)
    , buffer_()
    {}

    result_type operator()(float* value) {

        for (size_t i = 0; i != N; ++i) {
            if (value[i] <= min_) {
                last_ = low_;
            } else if (value[i] >= max_) {
                last_ = high_;
            }
            buffer_[i >> 3] = (last_ << (i & 7));
        }

        return buffer_;
    }
};
*/
}} // mobilinkd::libafsk

#endif // MOBILINKD__HYSTERESIS_H_


