#ifndef MADGWICK_H
#define MADGWICK_H

#ifdef SITL_BUILD
#include <Eigen/Dense>
#else
#include <eigen.h>
#endif

void Madgwick6DOF(const Eigen::Vector3f &accelMeas, const Eigen::Vector3f &gyroMeas, Eigen::Quaternionf &quatOut,
                  Eigen::Vector3f &eulerOut, float dt);

#endif
