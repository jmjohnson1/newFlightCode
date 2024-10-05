#include "motors.h"
#include "UserDefines.h"
#include "common.h"
#include "Arduino.h"

/**
 * @brief Constructor for Motor class
 * @param motorPins Array containing motor pin assignments
 * @param minPulseDuration The PWM pulse duration corresponding to no output in
 * microseconds
 * @param maxPulseDuration the PWM pulse duration corresponding to maximum
 * output in microseconds
*/
Motors::Motors(const uint8_t motorPins[4], int minPulseDuration, int maxPulseDuration) {
	for (int motor = 0; motor < 4; motor++) {
		motorServos_[motor].attach(motorPins[motor], minPulseDuration, maxPulseDuration);
	}
}

/**
 * @brief Takes in an angular rate for each motor in rad/sec and converts it
 * into a value between 0 and 180 to generate the PWM signal
 * @param angularRates  The desired motor rotation rates in rad/sec
 */
void Motors::ScaleCommand(Eigen::Vector4f &angularRates) {
	// This maps the angular rates to (0, 1)
	for (int i = 0; i < 4; i++) {
		// Yes, it's hideous. Don't try to follow in your head. This formulation may
		// also not work with other motors. If you are using a different type of
		// motor than this was designed for, check if this is your bug.
	/*	motorCommandNormalized_[i] = angularRates[i]/quadProps::K_W1;  */
	/*	motorCommandNormalized_[i] +=*/
	/*	0.25f*quadProps::K_W2*quadProps::K_W2/quadProps::K_W1/quadProps::K_W1;*/
	/*	motorCommandNormalized_[i] = -sqrt(motorCommandNormalized_[i]) -*/
	/*	quadProps::K_W2/quadProps::K_W1/2.0f; */
	/*	motorCommandNormalized_[i] = constrain(motorCommandNormalized_[i], 0, 1);*/
	/*	if (isnan(motorCommandNormalized_[i])) {*/
	/*		motorCommandNormalized_[i] = 0;*/
	/*	}*/
	/*	motorCommandScaled_[i] = map(motorCommandNormalized_[i], 0, 1, 0, 180);*/
	/*}*/

	motorCommandNormalized_[i] = quadProps::K_W1*angularRates[i]*angularRates[i] + quadProps::K_W2*angularRates[i];
		if (isnan(motorCommandNormalized_[i])) {
			motorCommandNormalized_[i] = 0;
		}
		motorCommandNormalized_[i] = constrain(motorCommandNormalized_[i], 0, 1);
		motorCommandScaled_[i] = map(motorCommandNormalized_[i], 0, 1, 0, 180);
	}
}

/**
 * @brief Writes to the motor pins
*/
void Motors::CommandMotor() {
	for (int i = 0; i < 4; i++) {
		motorServos_[i].write(motorCommandScaled_[i]);
	}
}

/**
 * @brief Arms the motors by continually providing idle commands
*/
void Motors::ArmMotors() {
	Eigen::Vector4f zeros = Eigen::Vector4f::Zero();
	ScaleCommand(zeros);  // Want 0 angular rate
	for (int i = 0; i < 10000; i++) {
		for (uint8_t motor = 0; motor < 4; motor++) {
			CommandMotor();
		}
		delayMicroseconds(500);
	}
}
