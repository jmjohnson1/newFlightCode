#ifndef DYNAMICS_MATH_H
#define DYNAMICS_MATH_H

#include <Eigen/Core>

namespace dynamics_math {

const double GRAV = 9.81;
  
template <typename T>
Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1> &v) {
  Eigen::Matrix<T, 3, 3> skew_matrix = Eigen::Matrix<T, 3, 3>::Zero();
  skew_matrix(1, 0) = v(2);
  skew_matrix(2, 0) = -v(1);
  skew_matrix(0, 1) = -v(2);
  skew_matrix(2, 1) = v(0);
  skew_matrix(0, 2) = v(1);
  skew_matrix(1, 2) = -v(0);
  return skew_matrix;
}

} // namespace dynamics_math

#endif
