#include <EEPROM.h>
#include "telemetry.h"
#include "defaultParams.h"
#include "ArduinoLibs/BFSChecksum/checksum.h"

uint8_t param_buf[PARAM_SIZE];
bfs::Fletcher16 param_checksum;
uint16_t chk_computed, chk_read;
uint8_t chk_buf[2];

bool telem::Begin(Quadcopter_t &quadData) {
  static unsigned char biggerWriteBuffer[256*10];
	// static unsigned char biggerReadBuffer[256];
  size_t biggerWriteBuffer_size = sizeof(biggerWriteBuffer);
	// size_t biggerReadBuffer_size = sizeof(biggerReadBuffer);
   Serial2.addMemoryForWrite(biggerWriteBuffer, biggerWriteBuffer_size);
  // Serial2.addMemoryForRead(biggerReadBuffer, biggerReadBuffer_size);

  quadData.telemData.mavlink = new bfs::MavLink<NUM_PARAMS, NUM_UTM>;
  quadData.telemData.mavlink->hardware_serial(&Serial2);
  quadData.telemData.mavlink->aircraft_type(bfs::MULTIROTOR);
  quadData.telemData.mavlink->mission(quadData.missionData.waypoints.data(), quadData.missionData.waypoints.size(), quadData.missionData.temp.data());
  quadData.telemData.mavlink->fence(quadData.missionData.fencePoints.data(), quadData.missionData.fencePoints.size());
  quadData.telemData.mavlink->rally(quadData.missionData.rallyPoints.data(), quadData.missionData.rallyPoints.size());
  quadData.telemData.mavlink->Begin(57600);

  // Parameter handling. Mostly from BFS SPAARO
  /* Load the telemetry parameters from EEPROM */
  for (std::size_t i = 0; i < PARAM_SIZE; i++) {
    param_buf[i] = EEPROM.read(i);
  }
  /* Check whether the parameter store has been initialized */
  /* If it hasn't... */
  if (param_buf[0] != (int)PARAM_HEADER[0] || param_buf[0] == (int)'R') {
    // Populate param_buf with default values
    GetDefaultTelemParams(param_buf);
    /* Compute the checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_HEADER) +
                                          NUM_PARAMS * (sizeof(float) + sizeof(char[16])));
    chk_buf[0] = static_cast<uint8_t>(chk_computed >> 8);
    chk_buf[1] = static_cast<uint8_t>(chk_computed);
    param_buf[PARAM_SIZE - 2] = chk_buf[0];
    param_buf[PARAM_SIZE - 1] = chk_buf[1];
    /* Write to EEPROM */
    for (std::size_t i = 0; i < PARAM_SIZE; i++) {
      EEPROM.write(i, param_buf[i]);
    }
  /* If it has been initialized */
  } else {
    /* Check the checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_HEADER) +
                                          NUM_PARAMS * (sizeof(float) + sizeof(char[16])));
    chk_buf[0] = param_buf[PARAM_SIZE - 2];
    chk_buf[1] = param_buf[PARAM_SIZE - 1];
    chk_read = static_cast<uint16_t>(chk_buf[0]) << 8 |
               static_cast<uint16_t>(chk_buf[1]);
    if (chk_computed != chk_read) {
      /* Parameter store corrupted, reset and warn */
      // Populate param_buf with default values
      GetDefaultTelemParams(param_buf);
      /* Compute the checksum */
      chk_computed = param_checksum.Compute(param_buf,
                                            sizeof(PARAM_HEADER) +
                                            NUM_PARAMS * (sizeof(float) + sizeof(char[16])));
      chk_buf[0] = static_cast<uint8_t>(chk_computed >> 8);
      chk_buf[1] = static_cast<uint8_t>(chk_computed);
      param_buf[PARAM_SIZE - 2] = chk_buf[0];
      param_buf[PARAM_SIZE - 1] = chk_buf[1];
      /* Write to EEPROM */
      for (std::size_t i = 0; i < PARAM_SIZE; i++) {
        EEPROM.write(i, param_buf[i]);
      }
    }
  }
  /* Copy parameter data*/
  memcpy(quadData.telemData.paramValues.data(), &(param_buf[1]),
          NUM_PARAMS * sizeof(float));
  memcpy(quadData.telemData.paramIDs.data(), &(param_buf[1 + NUM_PARAMS*sizeof(float)]), NUM_PARAMS*sizeof(char[16]));
  /* Update the parameter values in MAV Link */
  quadData.telemData.mavlink->params(quadData.telemData.paramValues);
  for (int32_t i = 0; i < NUM_PARAMS; i++) {
    char name[16];
    memcpy(&name, &(quadData.telemData.paramIDs[i*16]), sizeof(char[16]));
    quadData.telemData.mavlink->param_id(i, name);
  }
  quadData.telemData.paramsUpdated = true;

  // Set data stream rate defaults
  quadData.telemData.mavlink->setMinStreamPeriod_ms(MIN_STREAM_PERIOD);
  quadData.telemData.mavlink->raw_sens_stream_period_default_ms(RAW_SENS_STREAM_PERIOD);
  quadData.telemData.mavlink->ext_status_stream_period_default_ms(EXT_STATUS_STREAM_PERIOD);
  quadData.telemData.mavlink->rc_chan_stream_period_default_ms(RC_CHAN_STREAM_PERIOD);
  quadData.telemData.mavlink->extra1_stream_period_default_ms(EXTRA1_STREAM_PERIOD);
  quadData.telemData.mavlink->extra2_stream_period_default_ms(EXTRA2_STREAM_PERIOD);
  quadData.telemData.mavlink->extra3_stream_period_default_ms(EXTRA3_STREAM_PERIOD);
  // Set all data streams to their default rates
  quadData.telemData.mavlink->raw_sens_stream_period_ms(0);
  quadData.telemData.mavlink->ext_status_stream_period_ms(0);
  quadData.telemData.mavlink->rc_chan_stream_period_ms(0);
  quadData.telemData.mavlink->extra1_stream_period_ms(0);
  quadData.telemData.mavlink->extra2_stream_period_ms(0);
  quadData.telemData.mavlink->extra3_stream_period_ms(0);


  // fix this
  return true;
}

void telem::Run(Quadcopter_t &quadData, IMU &quadIMU) {
  // For better readability
  bfs::MavLink<NUM_PARAMS, NUM_UTM> *mavptr = quadData.telemData.mavlink;

  // Update values
  // Raw IMU
  mavptr->imu_accel_x_mps2(quadIMU.GetAccX());
  mavptr->imu_accel_y_mps2(quadIMU.GetAccY());
  mavptr->imu_accel_z_mps2(quadIMU.GetAccZ());
  mavptr->imu_gyro_x_radps(quadIMU.GetGyroX());
  mavptr->imu_gyro_y_radps(quadIMU.GetGyroY());
  mavptr->imu_gyro_z_radps(quadIMU.GetGyroZ());
  // Attitude
  mavptr->nav_roll_rad(quadData.att.eulerAngles_active->coeff(0));
  mavptr->nav_pitch_rad(quadData.att.eulerAngles_active->coeff(1));
  mavptr->nav_hdg_rad(quadData.att.eulerAngles_active->coeff(2));
  float quatSp[4] = {quadData.att.quatSetpoint.w(),
                     quadData.att.quatSetpoint.x(),
                     quadData.att.quatSetpoint.y(),
                     quadData.att.quatSetpoint.z()};
  mavptr->quaternionSetpoint(quatSp);
  // Position & velocity
  mavptr->nav_north_pos_m(quadData.navData.position_NED[0]);
  mavptr->nav_east_pos_m(quadData.navData.position_NED[1]);
  mavptr->nav_down_pos_m(quadData.navData.position_NED[2]);
  mavptr->nav_north_vel_mps(quadData.navData.velocity_NED[0]);
  mavptr->nav_east_vel_mps(quadData.navData.velocity_NED[1]);
  mavptr->nav_down_vel_mps(quadData.navData.velocity_NED[2]);
  mavptr->north_pos_setpoint_m(quadData.navData.positionSetpoint_NED[0]);
  mavptr->east_pos_setpoint_m(quadData.navData.positionSetpoint_NED[1]);
  mavptr->down_pos_setpoint_m(quadData.navData.positionSetpoint_NED[2]);
  mavptr->north_vel_setpoint_m(quadData.navData.velocitySetpoint_NED[0]);
  mavptr->east_vel_setpoint_m(quadData.navData.velocitySetpoint_NED[1]);
  mavptr->down_vel_setpoint_m(quadData.navData.velocitySetpoint_NED[2]);

  quadData.telemData.mavlink->Update();

  // Check if there is a new position setpoint requested by GCS
  if (mavptr->new_setpoint_available() == true) {
    quadData.navData.positionSetpoint_NED[0] = mavptr->new_setpoint_x();
    quadData.navData.positionSetpoint_NED[1] = mavptr->new_setpoint_y();
    quadData.navData.positionSetpoint_NED[2] = mavptr->new_setpoint_z();
  }

  // Handle any parameter updates
  if (mavptr->param_reset() == true) {
    EEPROM.write(0, 'R');
  }
  int32_t param_idx_ = mavptr->updated_param();
  if (param_idx_ >= 0) {
    // Let other places in the program know parameters have changed
    quadData.telemData.paramsUpdated = true;
    // Update the value in common data struct
    quadData.telemData.paramValues[param_idx_] = mavptr->param(param_idx_);
    /* Update the parameter buffer value */
    memcpy(param_buf + sizeof(PARAM_HEADER) + param_idx_*sizeof(float),
           &(quadData.telemData.paramValues[param_idx_]), sizeof(float));
    /* Compute a new checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_HEADER) +
                                          NUM_PARAMS*(sizeof(float) + sizeof(char[16])));
    chk_buf[0] = static_cast<uint8_t>(chk_computed >> 8);
    chk_buf[1] = static_cast<uint8_t>(chk_computed);
    param_buf[PARAM_SIZE - 2] = chk_buf[0];
    param_buf[PARAM_SIZE - 1] = chk_buf[1];
    /* Write to EEPROM */
    for (std::size_t i = 0; i < sizeof(float); i++) {
      std::size_t addr = i + sizeof(PARAM_HEADER) +
                         param_idx_*sizeof(float);
      EEPROM.write(addr, param_buf[addr]);
    }
    // Save the checksum
    EEPROM.write(PARAM_SIZE - 2, param_buf[PARAM_SIZE - 2]);
    EEPROM.write(PARAM_SIZE - 1, param_buf[PARAM_SIZE - 1]);
  }
}
  
// TEMPORARY HACK!! FIX ME!
const float MAX_POS_CHG = 10;
bool firstUpdate = true;

uint32_t telem::CheckForNewPosition(Quadcopter_t &quadData) {
  float viconX = quadData.telemData.mavlink->viconX();
  float viconY = quadData.telemData.mavlink->viconY();
  float viconZ = quadData.telemData.mavlink->viconZ();
  if ((abs(quadData.navData.mocapPosition_NED[0] - viconX) < MAX_POS_CHG &&
      abs(quadData.navData.mocapPosition_NED[1] - viconY) < MAX_POS_CHG &&
      abs(quadData.navData.mocapPosition_NED[2] - viconZ) < MAX_POS_CHG) ||
      firstUpdate == true) {
    firstUpdate = false;
    quadData.navData.mocapPosition_NED[0] = viconX;
    quadData.navData.mocapPosition_NED[1] = viconY;
    quadData.navData.mocapPosition_NED[2] = viconZ;
    quadData.navData.mocapUpdate_mocapTime = quadData.telemData.mavlink->viconTime();
    quadData.navData.mocapUpdate_quadTime = micros();
    return quadData.telemData.mavlink->numViconRX();
  }
  return quadData.navData.numMocapUpdates;
}
