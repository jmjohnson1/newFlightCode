#ifndef MADGWICK_H
#define MADGWICK_H

#include "IMU.h"
#include "math.h"
#include "common.h"

void Madgwick6DOF(const Generic_IMU &imu, Quadcopter_t &quad, float dt);

#endif
