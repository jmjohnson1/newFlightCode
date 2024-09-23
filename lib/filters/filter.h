#ifndef _FILTER_H_
#define _FILTER_H_

// Adapted from betaflight filter implementation
// https://github.com/betaflight/betaflight
typedef struct biquadFilter_s {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} biquadFilter_t;
void biquadFilter_init(biquadFilter_s *filter, float filterFreq, float sampleFreq);
float biquadFilter_apply(biquadFilter_s *filter, float input);

// Direct form implementation of second order butterworth filter
typedef struct butterworth2_s {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} butterworth2_t;
void butterworth2_init(butterworth2_s *filter, float filterFreq, float sampleFreq);
float butterworth2_apply(butterworth2_s *filter, float input);

class LowpassFilter {
 public:
  LowpassFilter(){};
  LowpassFilter(float cutoffFreq, float sampleFreq) { SetFilterParams(cutoffFreq, sampleFreq); }

  float Apply(float newValue);
  void SetFilterParams(float cutoffFreq_, float sampleFreq_);
  float GetAlpha() { return alpha_; }
  float GetSampleFreq() { return sampleFreq_; }
  float GetCutoffFreq() { return cutoffFreq_; }

 private:
  float alpha_ = 1.0f;
  float cutoffFreq_ = 0.0f;
  float sampleFreq_ = 0.0f;

  // Store previous output
  float output_ = 0.0f;
};
#endif
