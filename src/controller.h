#ifndef CONTROLLER_H
#define CONTROLLER_H

class AnglePID {
public:
	AnglePID(float Kp, float Ki, float Kd, float iLimit = 25.0f);
	float Update(float setpoint, float measuredAngle, float dt, float gyroRate, bool noIntegral);
	
	float GetPIDOutput() {return PIDOutput_;}
	float GetKp() {return Kp_;}
	float GetKi() {return Ki_;}
	float GetKd() {return Kd_;}
	void SetKp(float Kp) {Kp_ = Kp;}
	void SetKi(float Ki) {Ki_ = Ki;}
	void SetKd(float Kd) {Kd_ = Kd;}

protected:
	float PIDOutput_;
	float integral_prev_;
	float Kp_;
	float Ki_;
	float Kd_;
	float iLimit_;
};

class RatePID : public AnglePID {
public:
	RatePID(float Kp, float Ki, float Kd, float iLimit = 25.0f);
	float Update(float setpoint, float measuredRate, float dt, bool noIntegral);

protected:
	float error_prev_;
};

#endif
