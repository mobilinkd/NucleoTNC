#ifndef MOBILINKD_DELAY_LINE_H_
#define MOBILINKD_DELAY_LINE_H_

#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <cstring>

namespace mobilinkd { namespace libafsk {

struct DelayLine {
	
	size_t length_;
	std::vector<uint8_t> buffer_;
	size_t pos_;

	DelayLine(double sample_rate, double delay)
	: length_((delay / (1.0 / sample_rate)) + .5), buffer_(length_), pos_(0)
	{}
	
	bool operator()(bool value) {
		bool r = buffer_[pos_];
		buffer_[pos_++] = value;
		if (pos_ == length_) pos_ = 0;
		return r;
	}
};


template <size_t N>
struct FixedDelayLine {
	
	size_t length_;
	char buffer_[N];
	size_t pos_;

	FixedDelayLine(double sample_rate, double delay)
	: length_((delay / (1.0 / sample_rate)) + .5), buffer_(), pos_(0)
	{
		assert(length_ <= N);
		memset(buffer_, 0, N);
	}
	
	bool operator()(bool value) {
		bool r = buffer_[pos_];
		buffer_[pos_++] = value;
		if (pos_ == length_) pos_ = 0;
		return r;
	}
};

#if 0
template <size_t BLOCK_SIZE, size_t DELAY_BITS>
struct BlockDelayLine {

    uint8_t buffer_[(BLOCK_SIZE + DELAY_BITS + 7) / 8];
    size_t pos_;

    BlockDelayLine(double sample_rate, double delay)
    : length_((delay / (1.0 / sample_rate)) + .5), buffer_(), pos_(0)
    {
        assert(length_ <= N);
        memset(buffer_, 0, N);
    }

    bool operator()(bool value) {
        bool r = buffer_[pos_];
        buffer_[pos_++] = value;
        if (pos_ == length_) pos_ = 0;
        return r;
    }
};

#endif
}} // mobilinkd::libafsk

#endif // MOBILINKD_DELAY_LINE_H_

