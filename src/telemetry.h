#ifndef _TELEM_H_
#define _TELEM_H_


#define HWSERIAL Serial2


#include "common/mavlink.h"
#include "ardupilotmega/mavlink_msg_pid_tuning.h"
#include <eigen.h>
#include <stdint.h>
#include "commonDefinitions.h"
#include "IMU.h"
#include "parameters.h"

class Telemetry {
public:
	Telemetry();
	void InitTelemetry(ParameterManager *p);

	void SendHeartbeat();
	void SendPIDGains_core(float P, float I, float D);
	void SendPIDGains_rip(float P, float I, float D);
	void SendAttitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
	void SendScaledIMU(uint64_t timeSinceBoot, const IMU &imu);
	void SendMessage(mavlink_message_t *msg);
	void UpdateReceived();

	// Setters
	void SetSystemMode(uint8_t mode);
	void SetSystemState(uint8_t state);
	// Getters
	uint8_t GetSystemMode() {return systemMode;}
	uint8_t GetSystemState() {return systemState;}

	uint32_t CheckForNewPosition(Eigen::Vector3d& pos, uint32_t *mocapTimestamp, uint32_t tow, uint32_t *quadTimestamp);
	
private:
	void HandleMessage(mavlink_message_t *msg);
	void HandleCommandLong(mavlink_message_t *msg);
	void HandleParamRequest(const mavlink_param_request_list_t &req);
	void HandleParamRequestRead(const mavlink_param_request_read_t &req);
	void HandleParamSet(const mavlink_param_set_t &req);
	void HandleLocalPosNED(mavlink_message_t *msg);
	void HandleViconPosEst(mavlink_message_t *msg);


	mavlink_local_position_ned_t localPos;
	mavlink_vicon_position_estimate_t mocapPos;
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

	// pointer to parameter ParameterManager
	ParameterManager *paramManager = nullptr;

	mavlink_param_request_read_t request_read_;
	mavlink_param_request_list_t request_list_;
	mavlink_param_set_t param_set_;
};


#endif
