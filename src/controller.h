#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "commonDefinitions.h"
#include "eigen.h"

class AngleAttitudeController {
public:
  AngleAttitudeController(const float (&Kp)[3], const float (&Ki)[3],
                          const float (&Kd)[3], float iLimit = 2.0f);
  void Update(const float (&setpoints)[3], const Attitude &att,
              const float (&gyroRates)[3], float dt, bool noIntegral);

  void GetMotorCommands(float motorCommandsNormalized[4],
                        float throttleSetting);

  float GetRollPID() { return rollPID_; }
  float GetPitchPID() { return pitchPID_; }
  float GetYawPID() { return yawPID_; }

  // Need to modify a pointer to an outside array to give out the controller
  // constants
  void GetKp(float KpOut[3]) {
    KpOut[0] = Kp_[0];
    KpOut[1] = Kp_[1];
    KpOut[2] = Kp_[2];
  }
  void GetKi(float KiOut[3]) {
    KiOut[0] = Ki_[0];
    KiOut[1] = Ki_[1];
    KiOut[2] = Ki_[2];
  }
  void GetKd(float KdOut[3]) {
    KdOut[0] = Kd_[0];
    KdOut[1] = Kd_[1];
    KdOut[2] = Kd_[2];
  }

  void SetKp(const float (&KpIn)[3]) {
    Kp_[0] = KpIn[0];
    Kp_[1] = KpIn[1];
    Kp_[2] = KpIn[2];
  }
  void SetKi(const float (&KiIn)[3]) {
    Ki_[0] = KiIn[0];
    Ki_[1] = KiIn[1];
    Ki_[2] = KiIn[2];
  }
  void SetKd(const float (&KdIn)[3]) {
    Kd_[0] = KdIn[0];
    Kd_[1] = KdIn[1];
    Kd_[2] = KdIn[2];
  }

private:
  enum AxisToControl {
    ROLL = 0,
    PITCH = 1,
    YAW = 2,
  };
  float AnglePID(float setpoint, float measuredAngle, float gyroRate, float dt,
                 bool noIntegral, AxisToControl axis);
  float RatePID(float setpoint, float measuredRate, float dt, bool noIntegral,
                AxisToControl axis);

  float rollPID_ = 0.0f;
  float pitchPID_ = 0.0f;
  float yawPID_ = 0.0f;

  float prevIntegralRoll_ = 0.0f;
  float prevIntegralPitch_ = 0.0f;
  float prevIntegralYaw_ = 0.0f;
  float prevErrorYaw_ = 0.0f;

  float Kp_[3];
  float Ki_[3];
  float Kd_[3];

  float iLimit_;
};

class PositionController {
public:
  // Used for floating point comparison later. Generally best to avoid.
  const float EPSILON = 0.1f;

  PositionController(const float (&Kp)[3], const float (&Ki)[3],
                     const float (&Kd)[3], float iLimit = 5.0f);
  void Update(const Eigen::Vector3d &posSetpoints,
              const Eigen::Vector3d &currentPosition, const Attitude &att,
              float dt, bool noIntegral);

	void Reset();

  float GetDesiredThrottle() { return abs(desiredThrust_) / globalConstants::MAX_THRUST; }
  float GetDesiredRoll() { return desiredRoll_; }
  float GetDesiredPitch() { return desiredPitch_; }

  Eigen::Vector3f GetKp() { return Kp_.diagonal(); }
  Eigen::Vector3f GetKi() { return Ki_.diagonal(); }
  Eigen::Vector3f GetKd() { return Kd_.diagonal(); }

  void SetKp(const float (&KpIn)[3]) {
    for (int i = 0; i < 3; i++) {
      Kp_.diagonal()[i] = KpIn[i];
    }
  }
  void SetKi(const float (&KiIn)[3]) {
    for (int i = 0; i < 3; i++) {
      Ki_.diagonal()[i] = KiIn[i];
    }
  }
  void SetKd(const float (&KdIn)[3]) {
    for (int i = 0; i < 3; i++) {
      Kd_.diagonal()[i] = KdIn[i];
    }
  }

private:
  float desiredThrust_ = 0.0f; // Calculated desired thrust
  float desiredPitch_ = 0.0f;  // Calculated desired pitch angle (degrees)
  float desiredRoll_ = 0.0f;   // Calculated desired roll angle (degrees)

  Eigen::Vector3f prevIntegral_; // Save the last integral calculation
  Eigen::Vector3f prevError_;    // Save the last error calculation

  Eigen::Matrix3f Kp_;
  Eigen::Matrix3f Ki_;
  Eigen::Matrix3f Kd_;

  float iLimit_; // Maximum value for the integral portion
};

#endif
