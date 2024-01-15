#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include "TeensyTimerTool.h"

float ControlMixer(float throttle, float pitchPID, float rollPID, float yawPID, uint8_t motor);
int ScaleCommand(float motorCommandNormalized);
void CommandMotor(TeensyTimerTool::OneShotTimer *timer, int commandValue, uint8_t motorPin);
void ArmMotors(TeensyTimerTool::OneShotTimer **timers, uint8_t *motorPins, uint8_t numberOfMotors);


#endif
