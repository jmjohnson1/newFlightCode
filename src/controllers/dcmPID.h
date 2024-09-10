#ifndef DCM_PID_H
#define DCM_PID_H

#include "eigen.h"

class DCMPositionPID {
public:
  DCMPositionPID(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3], const float iLimit, const float c1,
                 const float mass = 1.0f, const float gravity = 9.81f);
  void Update(const Eigen::Vector3f &posSetpoints,
              const Eigen::Vector3f &velocitySetpoints,
              const Eigen::Vector3f &currentPosition, 
              const Eigen::Vector3f &currentVelocity,
              const Eigen::Vector3f &b1d,
              float dt);
	void Reset() { prevIntegral_.setZero(); }

  float GetDesiredThrust() { return desiredThrust_; } 
  Eigen::Matrix3f GetDesiredDCM() { return desiredDCM_;} 

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
  Eigen::Matrix3f desiredDCM_ = Eigen::Matrix3f::Identity();
  Eigen::Vector3f prevIntegral_; // Save the last integral calculation
  Eigen::Matrix3f Kp_;
  Eigen::Matrix3f Ki_;
  Eigen::Matrix3f Kd_;
  float iLimit_; // Maximum value for the integral portion
  float c1_;  // constant used in integral
	const float mass_;
	const float gravity_;
};

class DCMAttitudeControl {
public:
  DCMAttitudeControl(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3],
                     const float iLimit, const float c2);

  void Update(const Eigen::Matrix3f &currentDCM, const Eigen::Matrix3f &desiredDCM, const Eigen::Vector3f &gyroRates,
              const float dt);

  Eigen::Vector3f GetControlTorque() { return controlTorque_; }

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
  Eigen::Vector3f controlTorque_;
  Eigen::Vector3f prevIntegral_; // Save the last integral calculation
  Eigen::Matrix3f Kp_;
  Eigen::Matrix3f Ki_;
  Eigen::Matrix3f Kd_;
  float iLimit_; // Maximum value for the integral portion
  float c2_;  // constant used in integral
};



#endif
