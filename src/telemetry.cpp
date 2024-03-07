#ifndef PI
  #define PI               3.14159265358979f
#endif
//#define DEBUG_TELEM

#include "telemetry.h"
#include "Arduino.h"


Telemetry::Telemetry() {
}

void Telemetry::InitTelemetry(ParameterManager *p) {
  // Mavlink serial ports. See Teensy 4.1 pinouts for a list of available
  // hardware serial ports. Defined in the header file.

  // This additional buffer helps the program move on
  static unsigned char biggerWriteBuffer[5000];
	static unsigned char biggerReadBuffer[256];
  size_t biggerWriteBuffer_size = sizeof(biggerWriteBuffer);
	size_t biggerReadBuffer_size = sizeof(biggerReadBuffer);

  HWSERIAL.begin(baudRate);
  HWSERIAL.addMemoryForWrite(biggerWriteBuffer, biggerWriteBuffer_size);
	HWSERIAL.addMemoryForRead(biggerReadBuffer, biggerReadBuffer_size);
  paramManager = p;
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
	roll       *= deg2rad;
	pitch      *= deg2rad;
	yaw        *= deg2rad;
	rollspeed  *= deg2rad;
	pitchspeed *= deg2rad;
	yawspeed   *= deg2rad;

	mavlink_msg_attitude_pack(systemID, componentID_core, &msg, 0, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
	SendMessage(&msg);
}

void Telemetry::SendScaledIMU(uint64_t timeSinceBoot, const IMU &imu) {
	int16_t xacc = imu.GetAccX()*1000;  // mG
	int16_t yacc = imu.GetAccY()*1000;
	int16_t zacc = imu.GetAccZ()*1000;
	int16_t xgyro = imu.GetGyroX()*1000 * deg2rad; // mrad/s
	int16_t ygyro = imu.GetGyroY()*1000 * deg2rad;
	int16_t zgyro = imu.GetGyroZ()*1000 * deg2rad;
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
		}
	}
}

void Telemetry::HandleMessage(mavlink_message_t *msg) {
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			mavlink_msg_param_request_list_decode(msg, &request_list_);
			HandleParamRequest(request_list_);
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			mavlink_msg_param_set_decode(msg, &param_set_);
			HandleParamSet(param_set_);			
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			mavlink_msg_param_request_read_decode(msg, &request_read_);
			HandleParamRequestRead(request_read_);
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
			HandleCommandLong(msg);
			break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			HandleLocalPosNED(msg);
			break;
		case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
			HandleViconPosEst(msg);
			break;
		default:
			break;
	}
}

void Telemetry::HandleParamRequest(const mavlink_param_request_list_t &req) {
	Serial.print("Target_system: ");
	Serial.print(req.target_system);
	Serial.print(" target_component: ");
	Serial.println(req.target_component);

	if ((req.target_system == systemID) && ((req.target_component == componentID_core) || (req.target_component == MAV_COMP_ID_ALL))) {
		if (paramManager->numParameters > 0) {
			Serial.println("Params: ");
			for (int idx = 0; idx < paramManager->numParameters; idx++) {
				#ifdef DEBUG_TELEM
				Serial.print("name: ");
				Serial.print(paramManager->parameters[idx].name);
				Serial.print(" value: ");
				Serial.print(paramManager->parameters[idx].value);
				Serial.print(" type: ");
				Serial.print(paramManager->parameters[idx].mavParamType);
				Serial.println();
				#endif
				mavlink_message_t msg;
				mavlink_msg_param_value_pack(systemID, componentID_core, &msg, 
					paramManager->parameters[idx].name, 
					paramManager->parameters[idx].value,
					paramManager->parameters[idx].mavParamType,
					paramManager->numParameters,
					idx);
				SendMessage(&msg);
			}
		}
	}
}

void Telemetry::HandleParamSet(const mavlink_param_set_t &req) {
	if ((req.target_system == systemID) && ((req.target_component == componentID_core) || (req.target_component == MAV_COMP_ID_ALL))) {
		for (int i = 0; i < paramManager->numParameters; i++) {
			if (strcmp(req.param_id, paramManager->parameters[i].name) == 0) {
				/* Update the param value */
				paramManager->parameters[i].value = req.param_value;
				/* Send the new param back */
				mavlink_message_t msg;
				mavlink_msg_param_value_pack(systemID, componentID_core, &msg, 
					paramManager->parameters[i].name, 
					paramManager->parameters[i].value,
					paramManager->parameters[i].mavParamType,
					paramManager->numParameters,
					i);
				#ifdef DEBUG_TELEM
				Serial.print("Parameter \"");
				Serial.print(paramManager->parameters[i].name);
				Serial.print("\" value changed");
				Serial.println();
				#endif
				SendMessage(&msg);
				return;
			}
		}
		
		// Else
		Serial.println("invalid parameter requested");
		return;
	}
}

void Telemetry::HandleParamRequestRead(const mavlink_param_request_read_t &req) {
	if ((req.target_system == systemID) && ((req.target_component == componentID_core) || (req.target_component == MAV_COMP_ID_ALL))) {
		if (req.param_index > -1) {
			mavlink_message_t msg;
			mavlink_msg_param_value_pack(systemID, componentID_core, &msg, 
				paramManager->parameters[req.param_index].name, 
				paramManager->parameters[req.param_index].value,
				paramManager->parameters[req.param_index].mavParamType,
				paramManager->numParameters,
				req.param_index);
			SendMessage(&msg);
		} else {
			for (int i = 0; i < paramManager->numParameters; i++) {
				if (strcmp(req.param_id, paramManager->parameters[i].name) == 0) {
					mavlink_message_t msg;
					mavlink_msg_param_value_pack(systemID, componentID_core, &msg, 
						paramManager->parameters[i].name, 
						paramManager->parameters[i].value,
						paramManager->parameters[i].mavParamType,
						paramManager->numParameters,
						i);
					SendMessage(&msg);
				}
			}
		}

	}
}

void Telemetry::HandleCommandLong(mavlink_message_t *msg) {
	mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(msg, &packet);

}

void Telemetry::HandleLocalPosNED(mavlink_message_t *msg) {
	mavlink_msg_local_position_ned_decode(msg, &localPos);
  	//mostRecentPosRead = false;
		//Serial.println("Position received");
}

void Telemetry::HandleViconPosEst(mavlink_message_t *msg) {
	mavlink_msg_vicon_position_estimate_decode(msg, &mocapPos);
	mostRecentPosRead = false;
}

uint32_t Telemetry::CheckForNewPosition(Eigen::Vector3d& pos, uint32_t *mocapTimestamp, uint32_t tow, uint32_t *quadTimestamp) {
	if (!mostRecentPosRead) {
		//pos[0] = localPos.x;
		//pos[1] = localPos.y;
		//pos[2] = localPos.z;
		pos[0] = mocapPos.x;
		pos[1] = mocapPos.y;
		pos[2] = mocapPos.z;
		*mocapTimestamp = mocapPos.usec;
		*quadTimestamp = micros();

		#ifdef DEBUG_TELEM
		Serial.print("Measured x position: ");
		Serial.println(localPos.x);
		#endif
		mostRecentPosRead = true;
		tow++;
	}
	return tow;
}
