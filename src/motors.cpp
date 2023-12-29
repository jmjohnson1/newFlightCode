#include "motors.h"

float controlMixer(float throttle, float pitchPID, float rollPID, float yawPID, uint8_t motor) {
	switch (motor) {
		case 1: // Front left
			return throttle - pitchPID + rollPID + yawPID;
		case 2: // Front right
			return throttle - pitchPID - rollPID - yawPID;
		case 3: // Back right
			return throttle + pitchPID - rollPID + yawPID;
		case 4: // Back left
			return throttle + pitchPID + rollPID - yawPID;
		default:
			return 0.0f;
	}
}
