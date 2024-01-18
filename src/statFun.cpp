#include "statFun.h"

void LinearInterp(const Ref<const VectorXf> x, const Ref<const VectorXf> y, const Ref<const VectorXf> xq, Ref<VectorXf> yq) {
	/**
	 *  Linearly interpolates the data (x, y) at the points given in xq
	 *
	 *  @param[in] x - data to interpolate
	 *  @param[in] y - data to interpolate
	 *  @param[in] xq - query points
	 *  @param[out] yq - interpolated values at query points
	 */

	for (int i = 0; i < xq.size(); i++) {
		int xMin_index = NearestNeighbor(x, xq[i]);
		if (xMin_index == -1) {
			yq[i] = NAN;
		} else {
			float xMin = x[xMin_index];
			float xMax = x[xMin_index + 1];
			float yMin = y[xMin_index];
			float yMax = y[xMin_index + 1];
			yq[i] = yMin + (xq[i] - xMin)/(xMax - xMin) * (yMax - yMin);
		}
	}
}

float RMSE(const Ref<const VectorXf> x, const Ref<const VectorXf> y) {
	float runningSum = 0.0f;
	for (int i = 0; i < x.size() ; i++) {
		float diff = y(i) - x(i);
		runningSum += diff*diff;
	}
	float mean = runningSum / static_cast<float>(x.size());
	return sqrt(mean);
}

int NearestNeighbor(const Ref<const VectorXf> vec, float point) {
	/**
		*  Returns the index of a vector value that is closest to and less than a given point.
		*
		*  @param vec The vector to compare against
		*  @param point The point of interest
		*
		*  @return index that contains the nearest point or -1 if 'point' is outside the range of vec
		*/

	if (point > vec[vec.size() - 1] || point < vec[0]) {
		return -1;
	}

	int queryIndex = 0;

	while (point - vec(queryIndex) >= 0.0f) {
		queryIndex++;
	}

	int minIndex = queryIndex - 1;
	return minIndex;
}
