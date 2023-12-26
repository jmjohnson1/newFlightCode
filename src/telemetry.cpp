#include "telemetry.h"
#include "src/mavlink/common/mavlink_msg_local_position_ned.h"
#include "src/mavlink/ardupilotmega/mavlink_msg_pid_tuning.h"

Telemetry::Telemetry() {
}

void Telemetry::InitTelemetry() {
  // Mavlink serial ports. See Teensy 4.1 pinouts for a list of available
  // hardware serial ports. Defined in the header file.
  HWSERIAL.begin(baudRate);
}

void Telemetry::SendMessage(mavlink_message_t *msg) {
	uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN];
	// Copy message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(msg_buf, msg);
	// Send message
	HWSERIAL.write(msg_buf, len);
}

void Telemetry::SendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(systemID, componentID_core, &msg, systemType, autopilotType,
                               systemMode, customMode, systemState);
		SendMessage(&msg);
}

void Telemetry::SendPIDGains_core(float P, float I, float D) {
  mavlink_message_t msg;
  mavlink_msg_pid_tuning_pack(systemID, componentID_core, &msg, 0.0f, 0.0f, 0.0f, 0.0f,
                              P, I, D, 0.0f, 0.0f);
	SendMessage(&msg);
}

void Telemetry::SendPIDGains_rip(float P, float I, float D) {
  mavlink_message_t msg;
  mavlink_msg_pid_tuning_pack(systemID, componentID_RIP, &msg, 0.0f, 0.0f, 0.0f, 0.0f,
                              P, I, D, 0.0f, 0.0f);
	SendMessage(&msg);
}

void Telemetry::SendAttitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
	mavlink_message_t msg;

	// Convert to radians and change reference frame
	// The flight controller has Z pointing up, MAVLINK assumes Z points down -> negate y and z
	roll       *=  deg2rad;
	pitch      *= -deg2rad;
	yaw        *= -deg2rad;
	rollspeed  *=  deg2rad;
	pitchspeed *= -deg2rad;
	yawspeed   *= -deg2rad;

	mavlink_msg_attitude_pack(systemID, componentID_core, &msg, 0, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
	SendMessage(&msg);
}

void Telemetry::SendScaledIMU(uint64_t timeSinceBoot, attInfo *imuInfo) {
	int16_t xacc = imuInfo->AccX*1000;  // mG
	int16_t yacc = imuInfo->AccY*1000;
	int16_t zacc = imuInfo->AccZ*1000;
	int16_t xgyro = imuInfo->GyroX*1000 * deg2rad; // mrad/s
	int16_t ygyro = imuInfo->GyroY*1000 * deg2rad;
	int16_t zgyro = imuInfo->GyroZ*1000 * deg2rad;
	//mavlink_msg_scale

}

void Telemetry::SetSystemMode(uint8_t mode) {
	systemMode = mode;
}

void Telemetry::SetSystemState(uint8_t state) {
	systemState = state;
}

void Telemetry::UpdateReceived() {
	mavlink_message_t msg;
	mavlink_status_t status;
	int chan = 0;

	while (HWSERIAL.available()) {
		uint8_t byte = HWSERIAL.read();

		if (mavlink_parse_char(chan, byte, &msg, &status)) {
			HandleMessage(&msg);
			Serial.println("Message received");
		}
	}
}

void Telemetry::HandleMessage(mavlink_message_t *msg) {
	Serial.print("Message ID: ");
	Serial.println(msg->msgid);
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			HandleParamRequest(msg);
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
			HandleCommandLong(msg);
			break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			HandleLocalPosNED(msg);
			break;
		default:
			break;
	}
}

void Telemetry::HandleParamRequest(mavlink_message_t *msg) {
	
}

void Telemetry::HandleCommandLong(mavlink_message_t *msg) {
	mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(msg, &packet);

}

void Telemetry::HandleLocalPosNED(mavlink_message_t *msg) {
	mavlink_msg_local_position_ned_decode(msg, &localPos);
  mostRecentPosRead = false;
}

/**
* Checks for a whether a new position has ben received over mavlink. If so, it modifies the
* referenced position vector and returns true. Else, it returns false.
*
* @param: pos The current position being used by the caller
*
* @returns: true if pos has been modified, false otherwise.
*/
bool Telemetry::CheckForNewPosition(Eigen::Vector3f& pos) {
	if (!mostRecentPosRead) {
		pos[0] = localPos.x;
		pos[1] = localPos.y;
		pos[2] = localPos.z;
		mostRecentPosRead = true;
		return true;
	}
	return false;
}
