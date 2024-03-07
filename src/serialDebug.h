#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

namespace serialDebug {
	void PrintRadioData();
  void PrintDesiredState(float desThrottle, float desRoll, float desPitch, float desYaw);
  void PrintGyroData(float gyroX, float gyroY, float gyroZ);
  void PrintAccelData(float accX, float accY, float accZ);
  void PrintRollPitchYaw(float roll, float pitch, float yaw);
  void PrintPIDOutput(float rollPID, float pitchPID, float yawPID);
  void PrintMotorCommands(float m1Command, float m2Command, float m3Command, float m4Command);
  void PrintLoopTime(unsigned long dt);
  void PrintZPosPID(float p, float i, float d);
  void DisplayRoll(float setpoint, float value);
}

#endif
