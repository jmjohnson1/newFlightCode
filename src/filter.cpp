//#include "Arduino.h"
#include "filter.h"
#include <cmath>

void biquadFilter_init(biquadFilter_s *filter, float filterFreq, float sampleFreq){
	// setup
	const float omega = 2.0f*M_PI*filterFreq/sampleFreq;
	const float sn = sin(omega);
	const float cs = cos(omega);
	const float Q = 1.0f/sqrt(2.0f);
	const float alpha = sn/(2.0f*Q);

	// 2nd order butterworth
	filter->b1 = 1 - cs;
	filter->b0 = filter->b1 * 0.5f;
	filter->b2 = filter->b0;
	filter->a1 = -2*cs;
	filter->a2 = 1 - alpha;
	
	const float a0 = 1 + alpha;


	// Precompute coefficients
	filter->b0 /= a0;
	filter->b1 /= a0;
	filter->b2 /= a0;
	filter->a1 /= a0;
	filter->a2 /= a0;

	filter->x1 = filter->x2 = 0;
	filter->y1 = filter->y2 = 0;
}

float biquadFilter_apply(biquadFilter_s *filter, float input) {
	const float result = filter->b0*input + filter->x1;

	filter->x1 = filter->b1*input - filter->a1*result + filter->x2;
	filter->x2 = filter->b2*input - filter->a2*result;

	return result;
}

