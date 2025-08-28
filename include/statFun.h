#ifndef STATFUN_H
#define STATFUN_H

#ifdef BUILD_SITL
#include <Eigen/Dense>
#else
#include "eigen.h"
#endif

using namespace Eigen;

void LinearInterp(const Ref<const VectorXf> x, const Ref<const VectorXf> y, const Ref<const VectorXf> xq, Ref<VectorXf> yq);
float RMSE(const Ref<const VectorXf> x, const Ref<const VectorXf> y);
int NearestNeighbor(const Ref<const VectorXf> vec, float value);

#endif //STATFUN_H
