#include <EEPROM.h>
#include "telemetry.h"
#include "defaultParams.h"
#include "ArduinoLibs/BFSChecksum/checksum.h"

uint8_t param_buf[PARAM_SIZE];
bfs::Fletcher16 param_checksum;
uint16_t chk_computed, chk_read;
uint8_t chk_buf[2];

bool telem::Begin(Quadcopter_t &quadData) {
  static unsigned char biggerWriteBuffer[256*2];
	static unsigned char biggerReadBuffer[256];
  size_t biggerWriteBuffer_size = sizeof(biggerWriteBuffer);
	size_t biggerReadBuffer_size = sizeof(biggerReadBuffer);
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
  if (param_buf[0] != (int)PARAM_HEADER[0]) {
    Serial.println("Parameters not initialized");
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

  // fix this
  return true;
}

void telem::Run(Quadcopter_t &quadData) {
  quadData.telemData.mavlink->Update();

  int32_t param_idx_ = mavlink.updated_param();
  if (param_idx_ >= 0) {
    // Update the value in common data struct
    quadData.telemData.paramValues[param_idx_] = mavlink.param(param_idx_);
    /* Update the parameter buffer value */
    memcpy(param_buf + sizeof(PARAM_HEADER) + param_idx_*sizeof(float),
           &(quadData.telemData.paramValues[param_idx_]), sizeof(float));
    /* Compute a new checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_HEADER) +
                                          NUM_PARAMS*sizeof(float));
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
  

uint32_t telem::CheckForNewPosition(Quadcopter_t &quadData) {
  if (isfinite(quadData.telemData.mavlink->viconX())
      &&isfinite(quadData.telemData.mavlink->viconY())
      &&isfinite(quadData.telemData.mavlink->viconZ())
      && quadData.telemData.mavlink->numViconRX() > quadData.navData.numMocapUpdates) {

    quadData.navData.mocapPosition_NED[0] = quadData.telemData.mavlink->viconX();
    quadData.navData.mocapPosition_NED[1] = quadData.telemData.mavlink->viconY();
    quadData.navData.mocapPosition_NED[2] = quadData.telemData.mavlink->viconZ();
    quadData.navData.mocapUpdate_mocapTime = quadData.telemData.mavlink->viconTime();
    quadData.navData.mocapUpdate_quadTime = micros();
    return quadData.telemData.mavlink->numViconRX();
  }
  return quadData.navData.numMocapUpdates;
}