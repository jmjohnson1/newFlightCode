#include "motors.h"
#include "UserDefines.h"
#include "common.h"
#include "Arduino.h"

/**
 * @brief Scales a normalized motor command into a value that can be used with
 * the OneShot125 protocol or the servo library. OneShot125 takes values between
 * 125 us and 250 us. Servo takes values between 0 and 180. Returns the scaled
 * command
 * @param angularRates  The desired motor rotation rates in rad/sec
 * @returns  The desired motor command scaled for writing to servo or oneshot
 * protocol
 */
Eigen::Vector4i motors::ScaleCommand(Eigen::Vector4f &angularRates) {
	const Eigen::Vector4f ones = Eigen::Vector4f::Ones();
	Eigen::Vector4f motorCommandNormalized;
	Eigen::Vector4i motorCommandScaled;
	motorCommandNormalized = angularRates/quadProps::K_W1;  
	motorCommandNormalized += 0.25f*quadProps::K_W2*quadProps::K_W2/quadProps::K_W1/quadProps::K_W1*ones;
	motorCommandNormalized = -motorCommandNormalized.cwiseSqrt() - quadProps::K_W2/quadProps::K_W1/2.0f*ones; 
	for (int i = 0; i < 4; i++) {
		motorCommandNormalized[i] = constrain(motorCommandNormalized[i], 0, 1);
		if (isnan(motorCommandNormalized[i])) {
			motorCommandNormalized[i] = 0;
		}
	}
#ifdef USE_ONESHOT
	motorCommandScaled = (motorCommandNormalized * 125 + 125*ones).cast<int>();
#else
	motorCommandScaled = (motorCommandNormalized * 180).cast<int>();
#endif
	return motorCommandScaled;
}

/**
 * @brief Generates the pwm signal for the motor. It writes a HIGH value to the
 * motor pint and initiates a hardware timer that will move the pin LOW after
 * the duration specified by commandValue.
 * @param timer  Pointer to the hardware timer object used to bring the pulse LOW
 * @param commandValue  The duration of the PWM HIGH signal. Related to duty cycle [us]
 * @param motorPin  The pin connected to the ESC for the desired motor
*/
void motors::CommandMotor(TeensyTimerTool::OneShotTimer *timer, int commandValue, uint8_t motorPin) {
	digitalWriteFast(motorPin, HIGH);
	timer->trigger(commandValue);
}

/**
 * @brief Arms the motors by continually providing idle commands
 * @param timers  Pointer to an array that contains all of the hardware timer objects
 * @param motorPins  Array containing the pins that connect to each motor's ESC
*/
void motors::ArmMotors(TeensyTimerTool::OneShotTimer **timers, uint8_t *motorPins) {
	Eigen::Vector4f zeros = Eigen::Vector4f::Zero();
	Eigen::Vector4i command = ScaleCommand(zeros);
	for (int i = 0; i < 10000; i++) {
		for (uint8_t motor = 0; motor < 4; motor++) {
			CommandMotor(timers[motor], command[motor], motorPins[motor]);
		}
		delayMicroseconds(500);
	}
}
