#include "telemetry.h"


bool telem::Begin(Quadcopter_t &quadData) {
  static unsigned char biggerWriteBuffer[5000];
	static unsigned char biggerReadBuffer[256];
  size_t biggerWriteBuffer_size = sizeof(biggerWriteBuffer);
	size_t biggerReadBuffer_size = sizeof(biggerReadBuffer);
  // Serial2.addMemoryForWrite(biggerWriteBuffer, biggerWriteBuffer_size);
  // Serial2.addMemoryForRead(biggerReadBuffer, biggerReadBuffer_size);

  quadData.telemData.mavlink = new bfs::MavLink<NUM_PARAMS, NUM_UTM>;
  quadData.telemData.mavlink->hardware_serial(&Serial2);
  quadData.telemData.mavlink->aircraft_type(bfs::MULTIROTOR);
  quadData.telemData.mavlink->mission(quadData.missionData.waypoints.data(), quadData.missionData.waypoints.size(), quadData.missionData.temp.data());
  quadData.telemData.mavlink->fence(quadData.missionData.fencePoints.data(), quadData.missionData.fencePoints.size());
  quadData.telemData.mavlink->rally(quadData.missionData.rallyPoints.data(), quadData.missionData.rallyPoints.size());
  quadData.telemData.mavlink->Begin(57600);

  // fix this
  return true;
}

void telem::Run(Quadcopter_t &quadData) {
  quadData.telemData.mavlink->Update();
}
  

uint32_t telem::CheckForNewPosition(Quadcopter_t &quadData, Eigen::Vector3d& pos, uint32_t *mocapTimestamp, uint32_t *quadTimestamp) {
  pos[0] = quadData.telemData.mavlink->viconX();
  pos[1] = quadData.telemData.mavlink->viconY();
  pos[2] = quadData.telemData.mavlink->viconZ();
  *mocapTimestamp = quadData.telemData.mavlink->viconTime();
  *quadTimestamp = micros();

  return quadData.telemData.mavlink->numViconRX();
}