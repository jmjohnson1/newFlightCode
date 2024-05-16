#include "missionHandler.h"

void MissionHandler::Init(Quadcopter_t *quadData, NavData_t *navData) {
  quadData_ = quadData;
  navData_ = navData;
}

int MissionHandler::Run() {
  // Check if the quad is in mission mode
  uint32_t mode = quadData_->telemData.mavlink->custom_mode();
  bool missionMode = (mode & bfs::CustomMode::MISSION) == bfs::CustomMode::MISSION;
  if (missionMode == true) {
    // Check if a mission has been uploaded
    if (quadData_->telemData.mavlink->num_mission_items() > 0) {
      if (quadData_->flightStatus.missionStarted == false) {
        quadData_->flightStatus.missionStarted = true;
      }
      SetpointHandler(navData_, quadData_);
      return 1;
    }
  }
  return 0;
}