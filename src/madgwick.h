#ifndef MADGWICK_H
#define MADGWICK_H

#include "IMU.h"
#include "math.h"
#include "commonDefinitions.h"

void Madgwick6DOF(const IMU &imu, Attitude *att, float dt);

#endif
