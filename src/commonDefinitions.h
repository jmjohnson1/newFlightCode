#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

typedef struct Attitude {
  float roll, pitch, yaw;
  // Used by Madgwick
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
} Attitude;

namespace globalConstants {
	constexpr float MAX_ANGLE = 30.0f;  // Maximum pitch/roll angle in degrees
	constexpr float MAX_THRUST = 8.9f * 4.0f;  // Maximum total thrust (N)
	constexpr float MIN_THRUST = 0.445f * 4.0f;  // Minimum total thrust (N)
  constexpr float QUAD_MASS = 0.95f;  // Quadcopter mass (kg)
}

#endif
