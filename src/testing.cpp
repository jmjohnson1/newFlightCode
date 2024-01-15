#include "testing.h"
#include "UserDefines.h"

static float sineTime = 0.0f;

float testStand::SineSweep(unsigned long dt) {
	float angle = 0;
	if (sineTime < 120) {
		angle = SWP_AMPLITUDE*sin(
			PI*(SWP_MAX_FREQ - SWP_MIN_FREQ)/SWP_TIME/SWP_TIME*pow(sineTime, 3) 
			+ 2*PI*SWP_MIN_FREQ*sineTime);
		sineTime = sineTime + static_cast<float>(dt)/1000000.0f;
	} else {
		sineTime = 0;
	}
	return angle;
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
