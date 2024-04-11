#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include "TeensyTimerTool.h"
#include "eigen.h"

namespace motors{
  Eigen::Vector4i ScaleCommand(Eigen::Vector4f &angularRates);
  void CommandMotor(TeensyTimerTool::OneShotTimer *timer, int commandValue, uint8_t motorPin);
  void ArmMotors(TeensyTimerTool::OneShotTimer **timers, uint8_t *motorPins);
}


#endif
