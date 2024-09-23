#include "filter.h"
#include <cmath>

/**
 * @brief Initializes a second order lowpass filter with the filter cutoff
 * frequecy and data sampling frequency
 * @param filter   Pointer to the biquad filter struct that stores the variables
 * @param filterFreq	The filter's cutoff frequency [Hz]
 * @param smapleFreq  The sample rate of data that will be filtered [Hz]
*/
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

/**
 * @brief Applies a biquad filter to a single data point and updates the filter
 * @param input The data point to filter
 * @returns The data point after filtering
*/
float biquadFilter_apply(biquadFilter_s *filter, float input) {
	const float result = filter->b0*input + filter->x1;

	filter->x1 = filter->b1*input - filter->a1*result + filter->x2;
	filter->x2 = filter->b2*input - filter->a2*result;

	return result;
}


/**
 * @brief Initializes a second order butterworht lowpass filter with the filter cutoff
 * frequecy and data sampling frequency
 * @param filter   Pointer to the biquad filter struct that stores the variables
 * @param filterFreq	The filter's cutoff frequency [Hz]
 * @param smapleFreq  The sample rate of data that will be filtered [Hz]
*/
void butterworth2_init(butterworth2_s *filter, float filterFreq, float sampleFreq){
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

/**
 * @brief Applies a second order butterworth filter to a single data point and updates the filter
 * @param input The data point to filter
 * @returns The data point after filtering
*/
float butterworth2_apply(butterworth2_s *filter, float input) {
	// Standard difference equation
	const float result = filter->b0*input + filter->b1*filter->x1 + filter->b2*filter->x2
									 			- filter->a1*filter->y1 - filter->a2*filter->y2;

	// Shift the variables
	filter->x2 = filter->x1;
	filter->x1 = input;
	filter->y2 = filter->y1;
	filter->y1 = result;

	return result;
}

float LowpassFilter::Apply(float newValue) {
	output_ += (newValue - output_)*alpha_;
	return output_;
}
void LowpassFilter::SetFilterParams(float cutoffFreq, float sampleFreq) {
	cutoffFreq_ = cutoffFreq;
	sampleFreq_ = sampleFreq;
	if (sampleFreq_ <= 0.0f || cutoffFreq_ <= 0.0f) {
		alpha_ = 1.0f;
		return;
	}
	alpha_ = 1.0f / (1.0f + sampleFreq_/(2.0f * M_PI * cutoffFreq_));
}
