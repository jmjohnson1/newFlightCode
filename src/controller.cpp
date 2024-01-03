#include "controller.h"
#include "Arduino.h"

AnglePID::AnglePID(float Kp, float Ki, float Kd, float iLimit) {
	integral_prev_ = 0.0f;
	PIDOutput_ = 0.0f;
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	iLimit_ = iLimit;
}

float AnglePID::Update(float setpoint, float measuredAngle, float gyroRate, float dt, bool noIntegral) {
	float error = setpoint - measuredAngle;
  float integral = integral_prev_ + error * dt;
  if (noIntegral) { // Don't let integrator build if this is set
    integral = 0.0f;
  }
  integral = constrain(integral, -iLimit_, iLimit_); // Saturate integrator to prevent unsafe buildup
  float derivative = gyroRate;
  PIDOutput_ = 0.01 * (Kp_*error + Ki_*integral + Kd_*derivative);
	integral_prev_ = integral;

	return PIDOutput_;
}

RatePID::RatePID(float Kp, float Ki, float Kd, float iLimit) : AnglePID(Kp, Ki, Kd, iLimit) {
	error_prev_ = 0.0f;
}

float RatePID::Update(float setpoint, float measuredRate, float dt, bool noIntegral) {
	float error = setpoint - measuredRate;
	float integral = integral_prev_ + error*dt;
	if (noIntegral) {
		integral = 0.0f;
	}
	integral = constrain(integral, -iLimit_, iLimit_);
	float derivative = (error - error_prev_)/dt;
	PIDOutput_ = 0.01f * (Kp_*error + Ki_*integral + Kd_*derivative);
	integral_prev_ = integral;
	error_prev_ = error;
	return PIDOutput_;
}
