#ifndef MOTORS_H
#define MOTORS_H

#include <cstdint>

float controlMixer(float throttle, float pitchPID, float rollPID, float yawPID, uint8_t motor);

#endif
