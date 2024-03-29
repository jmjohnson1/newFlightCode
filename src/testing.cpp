#include "testing.h"
#include "UserDefines.h"

static elapsedMillis sineTime;
static elapsedMillis thrustTime;

float testStand::SineSweep(bool restart) {
	float angle = 0;
	if (restart == true) {
		sineTime = 0;
	}
	float t = sineTime/1000.0f;
	if (t < SWP_TIME) {
		angle = SWP_AMPLITUDE*sin(
			PI*(SWP_MAX_FREQ - SWP_MIN_FREQ)/SWP_TIME*t*t 
			+ 2*PI*SWP_MIN_FREQ*t);
	}
	return angle;
}

float testStand::ThrustSweep(bool restart) {
	float thro = 0;
	if (restart == true) {
		thrustTime = 0;
	}
	float t = thrustTime/1000.0f;
	if (t <= THO_MAX_TIME) {
		thro = t/THO_MAX_TIME * 0.5f;
	}
	return thro;
}

float testStand::Step(RadioChannel &angleCh) {
	float desiredAngle = 0.0f;
	if (angleCh.SwitchPosition() == SwPos::SWITCH_LOW) {
		desiredAngle = STP_ANGLE;
	} else if (angleCh.SwitchPosition() == SwPos::SWITCH_HIGH) {
		desiredAngle = -STP_ANGLE;
	}
	return desiredAngle;
}

float gainTuning::ScaleFactor(RadioChannel &GainCh) {
	float scale = 1.0f + MAX_SCALE_FACTOR*GainCh.NormalizedValue();
	return scale;
}
