#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

typedef struct Attitude {
  float roll, pitch, yaw;
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
} Attitude;

#endif
