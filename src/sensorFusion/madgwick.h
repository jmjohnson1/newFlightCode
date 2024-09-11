#ifndef MADGWICK_H
#define MADGWICK_H

#include <eigen.h>

void Madgwick6DOF(const Eigen::Vector3f &accelMeas, const Eigen::Vector3f &gyroMeas, Eigen::Quaternionf &quatOut,
                  Eigen::Vector3f &eulerOut, float dt);

#endif
