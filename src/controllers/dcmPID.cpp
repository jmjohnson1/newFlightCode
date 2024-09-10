#include "dcmPID.h"
#include "nav-functions.h"

/**
 * @brief PID position controller from 
 *  Geometric Nonlinear PID Control of a Quadrotor UAV on SE(3)
 *  Goodarzi et al. (2013)
*/
DCMPositionPID::DCMPositionPID(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3], const float iLimit, const float c1, const float mass, const float gravity)
	: mass_(mass),
		gravity_(gravity)
	{
  for (int i = 0; i < 3; i++) {
    Kp_.diagonal()[i] = Kp[i];
    Ki_.diagonal()[i] = Ki[i];
    Kd_.diagonal()[i] = Kd[i];
  }
  iLimit_ = iLimit;
  c1_ = c1;
  prevIntegral_.setZero();
}

/**
 * @brief Update the attitude and thrust setpoints
 *
*/
  void DCMPositionPID::Update(const Eigen::Vector3f &posSetpoints,
              const Eigen::Vector3f &velocitySetpoints,
              const Eigen::Vector3f &currentPosition, 
              const Eigen::Vector3f &currentVelocity,
              const Eigen::Vector3f &b1d,
              float dt) {
  // n3 basis vector (navigation frame)
  Eigen::Vector3f n3 = {0, 0, 1};
  // Position and velocity errors
  Eigen::Vector3f e_x = currentPosition - posSetpoints;
  Eigen::Vector3f e_v = currentVelocity - velocitySetpoints;
  // Integral term 
  Eigen::Vector3f e_i = prevIntegral_ + (e_v + c1_*e_x)*dt;
  // Saturate
  for (int i = 0; i < 3; i++) {
    e_i[i] = constrain(e_i[i], -iLimit_, iLimit_);
  }
  prevIntegral_ = e_i;
  // Compute b3 axis direction
  Eigen::Vector3f b3c = -(-Kp_*e_x - Kd_*e_v - Ki_*e_i - mass_*gravity_*n3).normalized();
  // b1 direction is the projection of b1d on the plane orthogonal to b3c
  Eigen::Vector3f b1c = -1.0f/(b3c.cross(b1d)).norm()*b3c.cross(b3c.cross(b1d));
  desiredDCM_.row(0) = b1c;
  desiredDCM_.row(1) = b3c.cross(b1c);
  desiredDCM_.row(2) = b3c;
  desiredThrust_ = (Kp_*e_x + Kd_*e_v + Ki_*e_i + mass_*gravity_*n3).dot(desiredDCM_.transpose()*n3);
}

DCMAttitudeControl::DCMAttitudeControl(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3],
                                       const float iLimit, const float c2) {
  for (int i = 0; i < 3; i++) {
    Kp_.diagonal()[i] = Kp[i];
    Ki_.diagonal()[i] = Ki[i];
    Kd_.diagonal()[i] = Kd[i];
  }
  iLimit_ = iLimit;
  c2_ = c2;
  prevIntegral_.setZero();
}

void DCMAttitudeControl::Update(const Eigen::Matrix3f &currentDCM, const Eigen::Matrix3f &desiredDCM,
                                const Eigen::Vector3f &gyroRates, const float dt) {
  // Attitude and angular rate error vectors
  Eigen::Matrix3f term = desiredDCM*currentDCM.transpose() - currentDCM*desiredDCM.transpose();
  Eigen::Vector3f e_R = 0.5f*SkewInverse(term);
  Eigen::Vector3f e_w = gyroRates;
  // Integral term
  Eigen::Vector3f e_i = prevIntegral_ + (e_w + c2_*e_R)*dt;
  // Saturate
  for (int i = 0; i < 3; i++) {
    e_i[i] = constrain(e_i[i], -iLimit_, iLimit_);
  }
  prevIntegral_ = e_i;
  // Control torque
  controlTorque_ = -Kp_*e_R - Kd_*e_w - Ki_*e_i;
}
