// Copyright 2015 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__IIR_FILTER_H_
#define MOBILINKD__TNC__IIR_FILTER_H_

#include <cstring>
#include <cstddef>

namespace mobilinkd { namespace tnc {

template <size_t N>
struct TIirCoefficients {
    float b[N];
    float a[N];
};

struct IirCoefficients {
    size_t size;
    const float* b;
    const float* a;

    template <size_t N>
    IirCoefficients(const TIirCoefficients<N>& c)
    : size(N), b(c.b), a(c.b)
    {}
};

template <size_t N>
struct IirFilter {

	typedef float float_type;
	const float* numerator_;
	const float* denominator_;
	float history_[N];
	size_t size_;

    IirFilter(const float* b, const float* a)
    : numerator_(b), denominator_(a)
    , history_(), size_(N)
    {
        memset(history_, 0, N);
    }
	
	IirFilter(const float* b, const float* a, size_t size)
	: numerator_(b), denominator_(a)
	, history_(), size_(size)
	{
		memset(history_, 0, N);
	}

	template <size_t M>
    IirFilter(TIirCoefficients<M> c)
    : numerator_(c.b), denominator_(c.a)
    , history_(), size_(M)
    {
        memset(history_, 0, N);
    }

    IirFilter(IirCoefficients c)
    : numerator_(c.b), denominator_(c.a)
    , history_(), size_(c.size)
    {
        memset(history_, 0, N);
    }

    ~IirFilter() {}

	float_type operator()(float_type input)
	{

		for(size_t i = size_ - 1; i != 0; i--) history_[i] = history_[i - 1];
		
		history_[0] = input;

		for (size_t i = 1; i != size_; i++) {
			history_[0] -= denominator_[i] * history_[i];
		}
		
		float_type result = 0;
		for (size_t i = 0; i != size_; i++) {
			result += numerator_[i] * history_[i];
		}
		
		return result;
	}
};

}} // mobilinkd::tnc

#endif // MOBILINKD__TNC__IIR_FILTER_H_


