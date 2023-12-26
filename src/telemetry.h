#ifndef _TELEM_H_
#define _TELEM_H_

#include "Arduino.h"

#define HWSERIAL Serial2

#include "src/mavlink/common/mavlink.h"
#include <eigen.h>
#include <cstdint>
#include "commonDefinitions.h"

class Telemetry {
public:
	Telemetry();
	void InitTelemetry();

	void SendHeartbeat();
	void SendPIDGains_core(float P, float I, float D);
	void SendPIDGains_rip(float P, float I, float D);
	void SendAttitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
	void SendScaledIMU(uint64_t timeSinceBoot, attInfo *imuInfo);
	void SendMessage(mavlink_message_t *msg);
	void UpdateReceived();

	// Setters
	void SetSystemMode(uint8_t mode);
	void SetSystemState(uint8_t state);
	// Getters
	uint8_t GetSystemMode() {return systemMode;}
	uint8_t GetSystemState() {return systemState;}

	bool CheckForNewPosition(Eigen::Vector3f& pos);
	
private:
	void HandleMessage(mavlink_message_t *msg);
	void HandleCommandLong(mavlink_message_t *msg);
	void HandleParamRequest(mavlink_message_t *msg);
	void HandleLocalPosNED(mavlink_message_t *msg);


	mavlink_local_position_ned_t localPos;
	bool mostRecentPosRead = true;

	uint8_t systemID = 1;
	uint8_t componentID_core = 1;
	uint8_t componentID_RIP = 2;
	uint8_t systemType = MAV_TYPE_QUADROTOR;
	uint8_t autopilotType = MAV_AUTOPILOT_GENERIC;
	uint8_t systemMode = MAV_MODE_MANUAL_DISARMED;
	uint8_t customMode = 0;
	uint8_t systemState = MAV_STATE_STANDBY;

	// Constants for conversion;
	float deg2rad = PI/180.0f;
	float rad2deg = 180.0f/PI;
	uint32_t baudRate = 57600;
};


#endif
