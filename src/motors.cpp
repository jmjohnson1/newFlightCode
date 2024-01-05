#include "motors.h"
#include "UserDefines.h"
#include "Arduino.h"

// TODO: Remove this. It's handled by the attitude controller.
float ControlMixer(float throttle, float pitchPID, float rollPID, float yawPID, uint8_t motor) {
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

/*
* Scales a normalized motor command into a value that can be used with the OneShot125 protocol or
* the servo library. OneShot125 takes values between 125 us and 250 us. Servo takes values between 0
* and 180. Returns the scaled command
*/
int ScaleCommand(float motorCommandNormalized) {
#ifdef USE_ONESHOT
	int motorCommandScaled = motorCommandNormalized * 125 + 125;
	motorCommandScaled = constrain(motorCommandScaled, 125, 250);
#else
	int motorCommandScaled = motorCommandNormalized * 180;
	motorCommandScaled = constrain(motorCommandScaled, 0, 180);
#endif
	return motorCommandScaled;
}

void CommandMotor(TeensyTimerTool::OneShotTimer *timer, int commandValue, uint8_t motorPin) {
	digitalWriteFast(motorPin, HIGH);
	Serial.print("Motor pin ");
	Serial.print(motorPin);
	Serial.print(" written ");
	Serial.println(HIGH);
	timer->trigger(commandValue);
	Serial.print("Timer triggered with value: ");
	Serial.println(commandValue);
}

void ArmMotors(TeensyTimerTool::OneShotTimer *timers, uint8_t *motorPins, uint8_t numberOfMotors) {
	int command = ScaleCommand(0.0f);
	for (int i = 0; i < 10000; i++) {
		for (uint8_t motor = 0; motor < numberOfMotors; motor++) {
			CommandMotor(&timers[motor], command, motorPins[motor]);
		}
		delayMicroseconds(500);
	}
}
