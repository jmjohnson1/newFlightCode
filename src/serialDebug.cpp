#include "serialDebug.h"
#include "Arduino.h"

void serialDebug::PrintDesiredState(float desThrottle, float desRoll, float desPitch, float desYaw) {
	Serial.print(F("desThrottle: "));
	Serial.print(desThrottle);
	Serial.print(F(" desRoll: "));
	Serial.print(desRoll);
	Serial.print(F(" desPitch: "));
	Serial.print(desPitch);
	Serial.print(F(" desYaw: "));
	Serial.println(desYaw);
}

void serialDebug::PrintGyroData(float gyroX, float gyroY, float gyroZ) {
	Serial.print(F("GyroX: "));
	Serial.print(gyroX);
	Serial.print(F(" GyroY: "));
	Serial.print(gyroY);
	Serial.print(F(" GyroZ: "));
	Serial.println(gyroZ);
}

void serialDebug::PrintAccelData(float accX, float accY, float accZ) {
	Serial.print(F("AccX: "));
	Serial.print(accX);
	Serial.print(F(" AccY: "));
	Serial.print(accY);
	Serial.print(F(" AccZ: "));
	Serial.println(accZ);
}

void serialDebug::PrintRollPitchYaw(float roll, float pitch, float yaw) {
	Serial.print(F("roll: "));
	Serial.print(roll);
	Serial.print(F(" pitch: "));
	Serial.print(pitch);
	Serial.print(F(" yaw: "));
	Serial.println(yaw);
}

void serialDebug::PrintPIDOutput(float rollPID, float pitchPID, float yawPID) {
	Serial.print(F("roll_PID: "));
	Serial.print(rollPID);
	Serial.print(F(" pitch_PID: "));
	Serial.print(pitchPID);
	Serial.print(F(" yaw_PID: "));
	Serial.println(yawPID);
}

void serialDebug::PrintMotorCommands(float m1Command, float m2Command, float m3Command, float m4Command) {
	Serial.print(F("m1Command: "));
	Serial.print(m1Command);
	Serial.print(F(" m2Command: "));
	Serial.print(m2Command);
	Serial.print(F(" m3Command: "));
	Serial.print(m3Command);
	Serial.print(F(" m4Command: "));
	Serial.print(m4Command);
	Serial.println();
}

void serialDebug::PrintLoopTime(unsigned long dt) {
	Serial.print(F("dt = "));
	Serial.println(dt * 1000000.0);
}
