#include "motors.h"
#include "UserDefines.h"
#include "Arduino.h"

/**
 * @brief Scales a normalized motor command into a value that can be used with
 * the OneShot125 protocol or the servo library. OneShot125 takes values between
 * 125 us and 250 us. Servo takes values between 0 and 180. Returns the scaled
 * command
 * @param motorCommandNormalized  The desired motor command between 0 and 1
 * @returns  The desired motor command scaled for writing to servo or oneshot
 * protocol
 */
int motors::ScaleCommand(float motorCommandNormalized) {
#ifdef USE_ONESHOT
	int motorCommandScaled = motorCommandNormalized * 125 + 125;
	motorCommandScaled = constrain(motorCommandScaled, 125, 250);
#else
	int motorCommandScaled = motorCommandNormalized * 180;
	motorCommandScaled = constrain(motorCommandScaled, 0, 180);
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
 * @param numberOfMotors	The size of both the motorPins array and timers array
*/
void motors::ArmMotors(TeensyTimerTool::OneShotTimer **timers, uint8_t *motorPins, uint8_t numberOfMotors) {
	int command = ScaleCommand(0.0f);
	for (int i = 0; i < 10000; i++) {
		for (uint8_t motor = 0; motor < numberOfMotors; motor++) {
			CommandMotor(timers[motor], command, motorPins[motor]);
		}
		delayMicroseconds(500);
	}
}
