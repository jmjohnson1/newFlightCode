#include "missionHandler.h"

void MissionHandler::Init(Quadcopter_t *quadData, NavData_t *navData) {
  quadData_ = quadData;
  navData_ = navData;
}

int MissionHandler::Run() {
  // Check if the quad is in an armed state
  if (quadData_->flightStatus.mavState == bfs::AircraftState::ACTIVE) {
    // Check if a mission has been uploaded
    if (quadData_->telemData.mavlink->num_mission_items() > 0) {
      if (quadData_->flightStatus.missionStarted == false) {
        quadData_->flightStatus.missionStarted = true;
        quadData_->flightStatus.phase = FlightPhase::TAKEOFF;
      }
      SetpointHandler(navData_, quadData_);
      return 1;
    }
  }
  return 0;
}