#include "madgwick.h"

const float B_madgwick = 0.04f; // Madgwick lowpass parameter

void Madgwick6DOF(const IMU &imu, Quadcopter_t &quad, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  float gx = imu.GetGyroX();
  float gy = imu.GetGyroY();
  float gz = imu.GetGyroZ();
  float ax = imu.GetAccX();
  float ay = imu.GetAccY();
  float az = imu.GetAccZ();

	float q0 = quad.att.quat_madgwick.w();
	float q1 = quad.att.quat_madgwick.x();
	float q2 = quad.att.quat_madgwick.y();
	float q3 = quad.att.quat_madgwick.z();

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = 1/sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    // This has been changed from the original paper to work with the front, starboard, down coordinate frame.
    // The original is here for reference.
    //s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    //s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    //s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    //s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
    s1 = _4q1 * q3q3 + _2q3 * ax + 4.0f * q0q0 * q1 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
    s2 = 4.0f * q0q0 * q2 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
    s3 = 4.0f * q1q1 * q3 + _2q1 * ax + 4.0f * q2q2 * q3 + _2q2 * ay;
    recipNorm = 1/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1*dt;
  q1 += qDot2*dt;
  q2 += qDot3*dt;
  q3 += qDot4*dt;

  // Normalise quaternion
  recipNorm = 1/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

	// Store the new quaternion
  quad.att.quat_madgwick = Eigen::Quaternionf(q0, q1, q2, q3);

  // Compute euler angles (degrees)
  quad.att.eulerAngles_madgwick[0] = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951;
  quad.att.eulerAngles_madgwick[1] = asin(-2.0f*(q1*q3 - q0*q2))*57.29577951;
  quad.att.eulerAngles_madgwick[2] = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951;
}
