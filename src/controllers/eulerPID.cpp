#include "eulerPID.h"
#include "Eigen/src/Core/Matrix.h"

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
 * @param setpoints - Yaw, pitch, and roll setpoints [rad]
 * @param att  Struct containing current attiude information
 * @param gyroRates The measured angular rates about x, y, z [deg/s]
 * @param dt  Time since last update [s]
 * @param noIntegral  Condition on whether or not to include the integral term.
 * Should be set false when the throttle is very low.
*/
void AngleAttitudeController::Update(const Eigen::Vector3f &setpoints,
                                     const Eigen::Vector3f &currentAttitude,
                                     const Eigen::Vector3f &gyroRates, float dt,
                                     bool noIntegral, float yawRateSetpoint,
																		 bool AngleForYaw) {
  rollPID_ = AnglePID(setpoints[0], currentAttitude(0), gyroRates[0], dt, noIntegral, ROLL);
  pitchPID_ = AnglePID(setpoints[1], currentAttitude(1), gyroRates[1], dt, noIntegral, PITCH);
	if (AngleForYaw == true) {
		yawPID_ = AnglePID(setpoints[2], currentAttitude(2), gyroRates[2], dt, noIntegral, YAW);
	} else {
		yawPID_ = RatePID(yawRateSetpoint, gyroRates[2], dt, noIntegral, YAW);
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
PositionController::PositionController(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3],
                                       float angleLimit, float mass, float maxThrust, float minThrust, float iLimit,
                                       const float updateRate, const float xy_P_FilterFreq, const float xy_I_FilterFreq,
                                       const float xy_D_FilterFreq, const float z_P_FilterFreq,
                                       const float z_I_FilterFreq, const float z_D_FilterFreq)
    : angleLimit_(angleLimit), mass_(mass), maxThrust_(maxThrust), minThrust_(minThrust), iLimit_(iLimit)

{
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
  prevIntegral_ << 0.0, 0.0, 0.0;
  prevError_ << 0.0, 0.0, 0.0;


	// filterObj, cutofffreq, samplefreq
	biquadFilter_init(&xy_P_Filter, xy_P_FilterFreq, updateRate);
	biquadFilter_init(&xy_I_Filter, xy_I_FilterFreq, updateRate);
	biquadFilter_init(&xy_D_Filter, xy_D_FilterFreq, updateRate);
	biquadFilter_init(&z_P_Filter, z_P_FilterFreq, updateRate);
	biquadFilter_init(&z_I_Filter, z_I_FilterFreq, updateRate);
	biquadFilter_init(&z_D_Filter, z_D_FilterFreq, updateRate);
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
                                const Eigen::Vector3f &currentAttitude, 
																float dt, bool noIntegral) {
  // Declare some variables
  Eigen::Vector3f posError_ned, integral, derivative, desAcc_ned;
  float desAcc_b3; // Desired thrust resolved in the 3 axis of the body frame
  float Cy = cos(currentAttitude[2]);
  float Sy = sin(currentAttitude[2]);
  float Cr = cos(currentAttitude[0]);
  float Cp = cos(currentAttitude[1]);
  float sinRoll, sinPitch; // Sin of the roll and pitch angles
  float maxAngle_sinArg = sin(angleLimit_ * DEG_TO_RAD);

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
  desiredThrust_ = mass_ * (-desAcc_b3);
  desiredThrust_ = constrain(desiredThrust_, minThrust_, maxThrust_);
  desAcc_b3 =
      -desiredThrust_ / mass_; // It's easier to constrain the thrust. maybe
	
  
  // Calculate a new desired attitude based on the amount of thrust available
  // and the desired accelerations in n1 and n2;
  // Let's make sure we can't break arcsin.
  if (abs(desiredThrust_ - minThrust_) > EPSILON) {
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
