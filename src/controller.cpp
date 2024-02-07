#include "controller.h"
#include "Arduino.h"
#include "nav-functions.h"

// Angle attitude controller from the original dRehmflight code //
AngleAttitudeController::AngleAttitudeController(const float (&Kp)[3],
                                                 const float (&Ki)[3],
                                                 const float (&Kd)[3],
                                                 float iLimit) {
  for (int i = 0; i < 3; i++) {
    Kp_[i] = Kp[i];
    Ki_[i] = Ki[i];
    Kd_[i] = Kd[i];
  }
  iLimit_ = iLimit;
}

void AngleAttitudeController::Update(const float (&setpoints)[3],
                                     const Attitude &att,
                                     const float (&gyroRates)[3], float dt,
                                     bool noIntegral) {
  rollPID_ =
      AnglePID(setpoints[0], att.roll, gyroRates[0], dt, noIntegral, ROLL);
  pitchPID_ =
      AnglePID(setpoints[1], att.pitch, gyroRates[1], dt, noIntegral, PITCH);
  yawPID_ = RatePID(setpoints[2], gyroRates[2], dt, noIntegral, YAW);
}

void AngleAttitudeController::GetMotorCommands(float motorCommandsNormalized[4],
                                               float throttleSetting) {
  motorCommandsNormalized[0] = throttleSetting + pitchPID_ + rollPID_ - yawPID_;
  motorCommandsNormalized[1] = throttleSetting + pitchPID_ - rollPID_ + yawPID_;
  motorCommandsNormalized[2] = throttleSetting - pitchPID_ - rollPID_ - yawPID_;
  motorCommandsNormalized[3] = throttleSetting - pitchPID_ + rollPID_ + yawPID_;
}

float AngleAttitudeController::AnglePID(float setpoint, float measuredAngle,
                                        float gyroRate, float dt,
                                        bool noIntegral, AxisToControl axis) {
  float *integral_prev = nullptr;
  float Kp = Kp_[axis];
  float Ki = Ki_[axis];
  float Kd = Kd_[axis];
  switch (axis) {
  case ROLL:
    integral_prev = &prevIntegralRoll_;
    break;
  case PITCH:
    integral_prev = &prevIntegralPitch_;
    break;
  default:
    return 0.0f;
  }
  float error = setpoint - measuredAngle;
  float integral = *integral_prev + error * dt;
  if (noIntegral) { // Don't let integrator build if this is set
    integral = 0.0f;
  }
  integral =
      constrain(integral, -iLimit_,
                iLimit_); // Saturate integrator to prevent unsafe buildup
  float derivative =
      -gyroRate; // This is an approximation of the error derivative
  float PIDOutput = 0.01 * (Kp * error + Ki * integral + Kd * derivative);
  *integral_prev = integral;

  return PIDOutput;
}

float AngleAttitudeController::RatePID(float setpoint, float measuredRate,
                                       float dt, bool noIntegral,
                                       AxisToControl axis) {
  float *integral_prev = nullptr;
  float *error_prev = nullptr;
  float Kp = Kp_[axis];
  float Ki = Ki_[axis];
  float Kd = Kd_[axis];
  switch (axis) {
  case YAW:
    integral_prev = &prevIntegralYaw_;
    error_prev = &prevErrorYaw_;
    break;
  default:
    return 0.0f;
  }
  float error = setpoint - measuredRate;
  float integral = *integral_prev + error * dt;
  if (noIntegral) {
    integral = 0.0f;
  }
  integral = constrain(integral, -iLimit_, iLimit_);
  float derivative = (error - *error_prev) / dt;
  float PIDOutput = 0.01f * (Kp * error + Ki * integral + Kd * derivative);
  *integral_prev = integral;
  *error_prev = error;
  return PIDOutput;
}

PositionController::PositionController(const float (&Kp)[3],
                                       const float (&Ki)[3],
                                       const float (&Kd)[3], float iLimit) {
  // Initialize the gain matrices
  Kp_ = Eigen::Matrix3f::Zero();
  Ki_ = Eigen::Matrix3f::Zero();
  Kd_ = Eigen::Matrix3f::Zero();

  // Set the diagonals to the supplied gains
  for (int i = 0; i < 3; i++) {
    Kp_.diagonal()[i] = Kp[i];
    Ki_.diagonal()[i] = Ki[i];
    Kd_.diagonal()[i] = Kd[i];
  }
  iLimit_ = iLimit;
  prevIntegral_ << 0.0, 0.0, 0.0;
  prevError_ << 0.0, 0.0, 0.0;
}

void PositionController::Update(const Eigen::Vector3d &posSetpoints,
                                const Eigen::Vector3d &currentPosition,
                                const Attitude &att, float dt, bool noIntegral) {
  // Declare some variables
  Eigen::Vector3f posError_ned, integral, derivative, desAcc_ned;
  float desAcc_b3; // Desired thrust resolved in the 3 axis of the body frame
  float Cy = cos(att.yaw * DEG_TO_RAD);
  float Sy = sin(att.yaw * DEG_TO_RAD);
  float Cr = cos(att.roll * DEG_TO_RAD);
  float Cp = cos(att.pitch * DEG_TO_RAD);
  float sinRoll, sinPitch; // Sin of the roll and pitch angles
  float maxAngle_sinArg = sin(globalConstants::MAX_ANGLE * DEG_TO_RAD);

  Eigen::Vector3f eul = {att.roll*DEG_TO_RAD, att.pitch*DEG_TO_RAD, att.yaw*DEG_TO_RAD};
  Eigen::Quaternionf quat = Euler2Quat(eul);
  Eigen::Matrix3f Cbn = Quat2DCM(quat);

  posError_ned = (posSetpoints - currentPosition).cast<float>();
  integral = prevIntegral_ + posError_ned * dt;
  // Prevent integral from building up if the thrust is low (noIntegral passed
  // as true).
  if (noIntegral) {
    integral = integral * 0;
  }
  // Prevent integral windup
  for (int i = 0; i < 3; i++) {
    integral[i] = constrain(integral[i], -iLimit_, iLimit_);
  }
  derivative = (posError_ned - prevError_) / dt;

  // Calculate desired acceleration in the NED frame using PID
  desAcc_ned = Kp_ * posError_ned + Ki_ * integral + Kd_ * derivative;

	prevError_ = posError_ned;
	prevIntegral_ = integral;

  // Extract the third term and use it to get the desired thrust in the body
  // frame. Need to add the amount of thrust required to hover with the
  // current attitude.
  desAcc_b3 = desAcc_ned[2]*Cr*Cp - 9.81/Cr/Cp;
  desiredThrust_ = globalConstants::QUAD_MASS * (desAcc_b3);
  desiredThrust_ = constrain(desiredThrust_, -globalConstants::MAX_THRUST, -globalConstants::MIN_THRUST);
  desAcc_b3 =
      desiredThrust_ / globalConstants::QUAD_MASS; // It's easier to constrain the thrust. maybe
  
  // Calculate a new desired attitude based on the amount of thrust available
  // and the desired accelerations in n1 and n2;
  // Let's make sure we can't break arcsin.
  if (abs(desiredThrust_ - globalConstants::MIN_THRUST) > EPSILON) {
    sinRoll = (Sy * desAcc_ned[0] - Cy * desAcc_ned[1]) / desAcc_b3;
    sinRoll = constrain(sinRoll, -maxAngle_sinArg, maxAngle_sinArg);
    desiredRoll_ = asin(sinRoll);

    sinPitch = (Cy * desAcc_ned[0] + Sy * desAcc_ned[1]) / desAcc_b3 /
               sqrt(1 - sinRoll * sinRoll);
    sinPitch = constrain(sinPitch, -maxAngle_sinArg, maxAngle_sinArg);
    desiredPitch_ = asin(sinPitch);

		desiredRoll_ *= RAD_TO_DEG;
		desiredPitch_ *= RAD_TO_DEG;
  } else {
    // If the thrust is small, please don't try to use it.
    desiredRoll_ = 0.0f;
    desiredPitch_ = 0.0f;
  }
}

void PositionController::Reset() {
	prevIntegral_ = Eigen::Vector3f::Zero();
	prevError_ = Eigen::Vector3f::Zero();
}
