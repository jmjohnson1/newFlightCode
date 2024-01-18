#include "controller.h"
#include "Arduino.h"

// Angle attitude controller from the original dRehmflight code //
AngleAttitudeController::AngleAttitudeController(const float (&Kp)[3], const float (&Ki)[3], const float (&Kd)[3], float iLimit) {
	for (int i = 0; i < 3; i++) {
		Kp_[i] = Kp[i];
		Ki_[i] = Ki[i];
		Kd_[i] = Kd[i];
	}
	iLimit_ = iLimit;
}

void AngleAttitudeController::Update(const float (&setpoints)[3], const Attitude &att, const float (&gyroRates)[3], float dt, bool noIntegral) {
	rollPID_ = AnglePID(setpoints[0], att.roll, gyroRates[0], dt, noIntegral, ROLL);	
	pitchPID_ = AnglePID(setpoints[1], att.pitch, gyroRates[1], dt, noIntegral, PITCH);	
	yawPID_ = RatePID(setpoints[2], gyroRates[2], dt, noIntegral, YAW);
}

void AngleAttitudeController::GetMotorCommands(float motorCommandsNormalized[4], float throttleSetting) {
	//motorCommandsNormalized[0] = throttleSetting - pitchPID_ + rollPID_ + yawPID_;
	//motorCommandsNormalized[1] = throttleSetting - pitchPID_ - rollPID_ - yawPID_;
	//motorCommandsNormalized[2] = throttleSetting + pitchPID_ - rollPID_ + yawPID_;
	//motorCommandsNormalized[3] = throttleSetting + pitchPID_ + rollPID_ - yawPID_;
	motorCommandsNormalized[0] = throttleSetting + pitchPID_ + rollPID_ - yawPID_;
	motorCommandsNormalized[1] = throttleSetting + pitchPID_ - rollPID_ + yawPID_;
	motorCommandsNormalized[2] = throttleSetting - pitchPID_ - rollPID_ - yawPID_;
	motorCommandsNormalized[3] = throttleSetting - pitchPID_ + rollPID_ + yawPID_;
}

float AngleAttitudeController::AnglePID(float setpoint, float measuredAngle, float gyroRate, float dt, bool noIntegral, AxisToControl axis) {
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
  integral = constrain(integral, -iLimit_, iLimit_); // Saturate integrator to prevent unsafe buildup
  float derivative = gyroRate;
  float PIDOutput = 0.01 * (Kp*error + Ki*integral + Kd*derivative);
	*integral_prev = integral;

	return PIDOutput;
}

float AngleAttitudeController::RatePID(float setpoint, float measuredRate, float dt, bool noIntegral, AxisToControl axis) {
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
	float integral = *integral_prev + error*dt;
	if (noIntegral) {
		integral = 0.0f;
	}
	integral = constrain(integral, -iLimit_, iLimit_);
	float derivative = (error - *error_prev)/dt;
	float PIDOutput = 0.01f * (Kp*error + Ki*integral + Kd*derivative);
	*integral_prev = integral;
	*error_prev = error;
	return PIDOutput;
}
