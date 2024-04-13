#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include "eigen.h"
#include "PWMServo.h"

class Motors {
public:
  Motors(const uint8_t motorPins[4], int minPulseDuration, int maxPulseDuration);
  void ScaleCommand(Eigen::Vector4f &angularRates);
  void CommandMotor();
  void ArmMotors();
  
  // Getters
  void GetMotorCommands(float returnedMotorCommands[4]) {
    for (int motor = 0; motor < 4; motor++) {
      returnedMotorCommands[motor] = motorCommandNormalized_[motor];
    }
  }

private:
  // Using PWMServo to generate the PWM signal for the motors
  PWMServo motorServos_[4];
  int motorCommandScaled_[4];
  float motorCommandNormalized_[4];
};



#endif
