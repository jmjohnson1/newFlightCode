#ifndef STATFUN_H
#define STATFUN_H

#include "eigen.h"

using namespace Eigen;

void LinearInterp(const Ref<const VectorXf> x, const Ref<const VectorXf> y, const Ref<const VectorXf> xq, Ref<VectorXf> yq);
float RMSE(const Ref<const VectorXf> x, const Ref<const VectorXf> y);
int NearestNeighbor(const Ref<const VectorXf> vec, float value);

#endif
