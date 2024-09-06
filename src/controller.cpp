#include "controller.h"
#include "Arduino.h"
#include "filter.h"
#include "nav-functions.h"


//===============//
// Control Mixer //
//===============//
/**
 * @brief Control allocator. Maps the control inputs (thrust and moments) to
 * angular rates for each motor.
 * @param inputs 4-element vector that contains the desired thrust in N and
 * moments about each axis in Nm. (T, Mx, My, Mz)
 * @return 4-element vector containing the angular rate setpoints for each motor
*/
Eigen::Vector4f ControlAllocator(Eigen::Vector4f &inputs) {
  Eigen::Vector4f w = (quadProps::ALLOCATION_MATRIX_INV*inputs).cwiseSqrt().real();
  for (int i = 0; i < 4; i++) {
    if (isnan(w[i])) {
      w[i] = 0.0f;
    }
  }
  return w;
}

//==================//
// Attitude Control //
//==================//
/**
 * @brief Create an AngleAttitudeController object with the given PID gains and
 * integral anti-windup limit.
 * @param Kp Reference to a 3-element array containing the P-gain for roll,
 * pitch, and yaw
 * @param Ki Reference to a 3-element array containing the I-gain for roll,
 * pitch, and yaw
 * @param Kd Reference to a 3-element array containing the D-gain for roll,
 * pitch, and yaw
 * @param iLimit The maximum value that the integral term can take. (default = 25.0f)
*/
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

/**
 * @brief Update the reference values for the throttle setting based on current
 * attidude and setpoints
 * @param setpoints  Yaw, pitch, and roll setpoints [deg]
 * @param att  Struct containing current attiude information
 * @param gyroRates The measured angular rates about x, y, z [deg/s]
 * @param dt  Time since last update [s]
 * @param noIntegral  Condition on whether or not to include the integral term.
 * Should be set false when the throttle is very low.
*/

void AngleAttitudeController::Update(const float setpoints[3],
                                     const AttitudeData_t &att,
                                     const Eigen::Vector3f &gyroRates, float dt,
                                     bool noIntegral,
																		 bool AngleForYaw) {
	/*Serial.println(AngleForYaw);*/
  Eigen::Vector3f eulerAngles = *(att.eulerAngles_active);
  rollPID_ = AnglePID(setpoints[0], eulerAngles[0], gyroRates[0], dt, noIntegral, ROLL);
  pitchPID_ = AnglePID(setpoints[1], eulerAngles[1], gyroRates[1], dt, noIntegral, PITCH);
	if (AngleForYaw == true) {
		yawPID_ = AnglePID(setpoints[2], eulerAngles[2], gyroRates[2], dt, noIntegral, YAW);
	} else {
		yawPID_ = RatePID(att.yawRateSetpoint, gyroRates[2], dt, noIntegral, YAW);
	}
	
}

/**
 * @brief Gets normalized motor commands by mixing the throttle setting and
 * roll, pitch, and yaw pid outputs
 * @param[out] motorCommandsNormalized  Motor commands between 0 and 1
 * @param  throttleSetting  The desired throttle setting between 0 and 1
*/
void AngleAttitudeController::GetMotorCommands(float motorCommandsNormalized[4],
                                               float throttleSetting) {
  motorCommandsNormalized[0] = throttleSetting + pitchPID_ + rollPID_ - yawPID_;
  motorCommandsNormalized[1] = throttleSetting + pitchPID_ - rollPID_ + yawPID_;
  motorCommandsNormalized[2] = throttleSetting - pitchPID_ - rollPID_ - yawPID_;
  motorCommandsNormalized[3] = throttleSetting - pitchPID_ + rollPID_ + yawPID_;
  for (int i = 1; i < 4; i++) {
    motorCommandsNormalized[i] = constrain(motorCommandsNormalized[i], 0.0f, 1.0f);
  }
}

/**
 * @brief Performs the PID calculations for an attitude angle given the
 * setpoint, measurement and angular rate.
 * @param setpoint  The angle setpoint
 * @param measuredAngle  The current angle
 * @param gyroRate  The measured angular rate about the axis to control
 * @param dt  The amount of time elapsed since the last update
 * @param noIntegral  Condition on whether to use the integral term
 * @param axis  The axis to control (enum)
 * @returns The pid output
*/
float AngleAttitudeController::AnglePID(float setpoint, float measuredAngle,
                                        float gyroRate, float dt,
                                        bool noIntegral, AxisToControl axis) {
  float *integral_prev = nullptr; // Need to declare this now and assign an address later.
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
	case YAW:
		integral_prev = &prevIntegralYaw_;
		break;
  default:
    return 0.0f;
  }
  float error = setpoint - measuredAngle;
	// Handle wrapping for yaw
	if (axis == YAW) {
		if (error <= -M_PI) {
			error += 2*M_PI;
		} else if (error > M_PI) {
			error -= 2*M_PI;
		}
	}
  float integral = *integral_prev + error * dt;
  if (noIntegral) { // Don't let integrator build if this is set
    integral = 0.0f;
  } 
	// Saturate integrator to prevent unsafe buildup
  integral = constrain(integral, -iLimit_, iLimit_);
	// This is an approximation of the error derivative
  float derivative = -gyroRate; 
  float PIDOutput = (Kp * error + Ki * integral + Kd * derivative);
  *integral_prev = integral;

  return PIDOutput;
}

/**
 * @brief Performs the PI calculations for an attitude angular rate given the
 * setpoint and measured rate.
 * @param setpoint  The angular rate setpoint
 * @param measuredAngle  The current angular rate
 * @param dt  The amount of time elapsed since the last update
 * @param noIntegral  Condition on whether to use the integral term
 * @param axis  The axis to control (enum)
 * @returns The pid output
*/
float AngleAttitudeController::RatePID(float setpoint, float measuredRate,
                                       float dt, bool noIntegral,
                                       AxisToControl axis) {
  float *integral_prev = nullptr;
  float *error_prev = nullptr;
  float Ki = Ki_[axis];
  float Kp = Kd_[axis];
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
  float PIDOutput = (Ki * integral + Kp * error);
  *integral_prev = integral;
  *error_prev = error;
  return PIDOutput;
}

//==================//
// POSITION CONTROL //
//==================//

/**
 * @brief Constructs a PID position controller object with the provided gains
 * and integral limit
 * @param Kp  reference to 3-element array containing proportional gains for
 * position in x, y, z
 * @param Ki  reference to 3-element array containing integral gains for
 * position in x, y, z
 * @param Kd  reference to 3-element array containing derivative gains for
 * position in x, y, z
 * @param iLimit  The largest absolute value that the integral term can build up
 * to
*/
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


	// filterObj, cutofffreq, samplefreq
	biquadFilter_init(&xOutputFilter, 30, 200);
	biquadFilter_init(&yOutputFilter, 30, 200);
	biquadFilter_init(&zOutputFilter, 30, 200);
}

/**
 * @brief Updates the attitude setpoints based on the current position error,
 * velocity, and previous position error
 * @param posSetpoints The position setpoints in the order x, y, z [m]
 * @param currentPosition  The estimated position in x, y, z [m]
 * @param currentVelocity  The estimated velocity in x, y, z [m/s]
 * @param att  Struct containing information on the current attitude
 * @param dt  The elapsed time since the last update [s]
 * @param noIntegral  Condition on whether to use the integral term
*/
void PositionController::Update(const Eigen::Vector3d &posSetpoints,
                                const Eigen::Vector3f &velocitySetpoints,
                                const Eigen::Vector3d &currentPosition,
                                const Eigen::Vector3f &currentVelocity,
                                const AttitudeData_t &att, float dt, bool noIntegral) {
  // Declare some variables
  Eigen::Vector3f posError_ned, integral, derivative, desAcc_ned;
  float desAcc_b3; // Desired thrust resolved in the 3 axis of the body frame
  Eigen::Vector3f eulerAngles = *(att.eulerAngles_active);
  float Cy = cos(eulerAngles[2]);
  float Sy = sin(eulerAngles[2]);
  float Cr = cos(eulerAngles[0]);
  float Cp = cos(eulerAngles[1]);
  float sinRoll, sinPitch; // Sin of the roll and pitch angles
  float maxAngle_sinArg = sin(quadProps::MAX_ANGLE * DEG_TO_RAD);

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
  //derivative = (posError_ned - prevError_) / dt;
  derivative = velocitySetpoints - currentVelocity;

	derivative(0) = biquadFilter_apply(&xOutputFilter, derivative(0));
	derivative(1) = biquadFilter_apply(&yOutputFilter, derivative(1));

  // Calculate desired acceleration in the NED frame using PID
  desAcc_ned = Kp_ * posError_ned + Ki_ * integral + Kd_ * derivative;

	/*desAcc_ned(0) = biquadFilter_apply(&xOutputFilter, desAcc_ned(0));*/
	/*desAcc_ned(1) = biquadFilter_apply(&yOutputFilter, desAcc_ned(1));*/
	/*desAcc_ned(2) = biquadFilter_apply(&zOutputFilter, desAcc_ned(2));*/

	prevError_ = posError_ned;
	prevIntegral_ = integral;

  // Extract the third term and use it to get the desired thrust in the body
  // frame. Need to add the amount of thrust required to hover with the
  // current attitude.
  desAcc_b3 = (desAcc_ned[2] - 9.81)/Cr/Cp;
  desiredThrust_ = quadProps::QUAD_MASS * (-desAcc_b3);
  desiredThrust_ = constrain(desiredThrust_, quadProps::MIN_THRUST, quadProps::MAX_THRUST);
  desAcc_b3 =
      -desiredThrust_ / quadProps::QUAD_MASS; // It's easier to constrain the thrust. maybe
	
  
  // Calculate a new desired attitude based on the amount of thrust available
  // and the desired accelerations in n1 and n2;
  // Let's make sure we can't break arcsin.
  if (abs(desiredThrust_ - quadProps::MIN_THRUST) > EPSILON) {
    sinRoll = (Sy * desAcc_ned[0] - Cy * desAcc_ned[1]) / desAcc_b3;
    sinRoll = constrain(sinRoll, -maxAngle_sinArg, maxAngle_sinArg);
    desiredRoll_ = asin(sinRoll);

    sinPitch = (Cy * desAcc_ned[0] + Sy * desAcc_ned[1]) / desAcc_b3 /
               sqrt(1 - sinRoll * sinRoll);
    sinPitch = constrain(sinPitch, -maxAngle_sinArg, maxAngle_sinArg);
    desiredPitch_ = asin(sinPitch);

  } else {
    // If the thrust is small, please don't try to use it.
    desiredRoll_ = 0.0f;
    desiredPitch_ = 0.0f;
  }
}

/**
 * @brief Resets the integrator and previous error terms used in calculations
*/
void PositionController::Reset() {
	prevIntegral_ = Eigen::Vector3f::Zero();
	prevError_ = Eigen::Vector3f::Zero();
}


/**
 * @brief PID position controller from 
 *  Geometric Nonlinear PID Control of a Quadrotor UAV on SE(3)
 *  Goodarzi et al. (2013)
*/
PositionController2::PositionController2(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3], const float iLimit, const float c1) {
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
  void PositionController2::Update(const Eigen::Vector3f &posSetpoints,
              const Eigen::Vector3f &velocitySetpoints,
              const Eigen::Vector3f &currentPosition, 
              const Eigen::Vector3f &currentVelocity,
              const Eigen::Vector3f &b1d,
              const AttitudeData_t &att,
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
  Eigen::Vector3f b3c = -(-Kp_*e_x - Kd_*e_v - Ki_*e_i - quadProps::QUAD_MASS*quadProps::G*n3).normalized();
  // b1 direction is the projection of b1d on the plane orthogonal to b3c
  Eigen::Vector3f b1c = -1.0f/(b3c.cross(b1d)).norm()*b3c.cross(b3c.cross(b1d));
  desiredDCM_.row(0) = b1c;
  desiredDCM_.row(1) = b3c.cross(b1c);
  desiredDCM_.row(2) = b3c;
  desiredThrust_ = (Kp_*e_x + Kd_*e_v + Ki_*e_i + quadProps::QUAD_MASS*quadProps::G*n3).dot(desiredDCM_.transpose()*n3);
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

void DCMAttitudeControl::Update(const AttitudeData_t &att, const Eigen::Vector3f &gyroRates, const float dt) {
  // Attitude and angular rate error vectors
  Eigen::Matrix3f term = att.desiredDCM*att.currentDCM.transpose() - att.currentDCM*att.desiredDCM.transpose();
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
