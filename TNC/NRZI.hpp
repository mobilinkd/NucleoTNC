#ifndef MOBILINKD__NRZI_H_
#define MOBILINKD__NRZI_H_

namespace mobilinkd { namespace libafsk {

struct NRZI {

	bool state_;
	
	NRZI()
	: state_(false)
	{}
	
	bool decode(bool x) {
		bool result = (x == state_);
		state_ = x;
		return result;
	}

	bool encode(bool x) {
	    if (x == 0) {
	        state_ ^= 1;    // Flip the bit.
	    }
	    return state_;
	}
};

}} // mobilinkd::libafsk

#endif // MOBILINKD__NRZI_H_


