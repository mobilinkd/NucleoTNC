#ifndef FILTER_H_
#define FILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 20000 Hz

* 0 Hz - 800 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.37597293671087 dB

* 1100 Hz - 2300 Hz
  gain = 1
  desired ripple = 2 dB
  actual ripple = 1.474114871585135 dB

* 2600 Hz - 10000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.37597293671087 dB

*/

#define FILTER_TAP_NUM 31

extern const float filter_taps[13][FILTER_TAP_NUM];

typedef struct {
    float taps[FILTER_TAP_NUM];
    float history[FILTER_TAP_NUM];
    unsigned int last_index;
} Filter;

void Filter_init(Filter* f, const float* taps);
void Filter_put(Filter* f, float input);
float Filter_get(Filter* f);

#endif
