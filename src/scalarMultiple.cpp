#include "scalarMultiple.h"

void scalarMultiple_init(scalarMultiple_s *scale, float minOutput,
                         float maxOutput, float minInput, float maxInput) {
	scale->yMax = maxOutput;
	scale->yMin = minOutput;
	scale->xMax = maxInput;
	scale->xMin = minInput;
	scale->xRange = maxInput - minInput;
}

float scalarMultiple_apply(scalarMultiple_s *scale, float input) {
	float output = (input - scale->xMin) / scale->xRange * scale->yMax;
	if (output > scale->yMax) {
		return scale->yMax;
	}
	if (output < scale->yMin) {
		return scale->yMin;
	}
	return output;
}
