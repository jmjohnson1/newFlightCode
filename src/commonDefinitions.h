#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

typedef struct attitudeInfo {
  float AccX, AccY, AccZ;
  float AccX_prev, AccY_prev, AccZ_prev;
  float GyroX, GyroY, GyroZ;
  float GyroX_prev, GyroY_prev, GyroZ_prev;
  float MagX, MagY, MagZ;
  float MagX_prev, MagY_prev, MagZ_prev;
  float roll_IMU, pitch_IMU, yaw_IMU;
  float roll_IMU_prev, pitch_IMU_prev;
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
  float AccErrorX = 0.0f;
  float AccErrorY = 0.0f;
  float AccErrorZ = 0.0f;
  float GyroErrorX = 0.0f;
  float GyroErrorY = 0.0f;
  float GyroErrorZ = 0.0f;
  // Filter parameters
  float B_madgwick = 0.04; // Madgwick filter parameter
  float B_accel = 0.14;    // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
  float B_gyro = 0.1;      // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
  float B_mag = 1.0;       // Magnetometer LP filter parameter
} attInfo;

#endif
