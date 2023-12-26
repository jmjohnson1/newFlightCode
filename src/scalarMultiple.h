#ifndef _SCALAR_MULTIPLE_H_
#define _SCALAR_MULTIPLE_H_

struct scalarMultiple_s {
	float xMin, xMax, xRange;
	float yMin, yMax;
};

void scalarMultiple_init(scalarMultiple_s *scale, float minOutput, float maxOutput, float minInput, float maxInput);
float scalarMultiple_apply(scalarMultiple_s *scale, float input);

#endif

