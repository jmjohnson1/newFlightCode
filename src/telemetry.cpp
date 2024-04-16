#include "telemetry.h"


static std::array<int, 10000> bigArray;
bool telem::Begin(MissionData_t &missionData) {
  bigArray[1] = 0;
  // mavlink->hardware_serial(&Serial2);
  // mavlink->aircraft_type(bfs::MULTIROTOR);
  // mavlink->mission(missionData.waypoints.data(), missionData.waypoints.size(), missionData.temp.data());
  // mavlink->fence(missionData.fencePoints.data(), missionData.fencePoints.size());
  // mavlink->rally(missionData.rallyPoints.data(), missionData.rallyPoints.size());
  // mavlink->Begin(57600);

  // fix this
  return true;
}

void telem::Run() {
  // mavlink->Update();
}