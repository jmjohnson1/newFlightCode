#include "navHandler.h"

SetpointHandler::SetpointHandler(QuadType::Quadcopter_t *quadData) :
quadData_(quadData),
quadPos_(&(quadData->navData.position_NED)) {
  posSetpoint_ = &(quadData->navData.positionSetpoint_NED);
  velSetpoint_ = &(quadData->navData.velocitySetpoint_NED);
  inputOverride_ = &(quadData->flightStatus.inputOverride);
}

void SetpointHandler::UpdateSetpoint() {
  // Flight mode check
  uint32_t mode = quadData_->telemData.mavlink->custom_mode();
  if (mode == bfs::CustomMode::TAKEOFF) {
    TakeoffSetpoint();
  } else if (mode == bfs::CustomMode::LANDING) {
    LandingSetpoint();
  } else if (mode == bfs::CustomMode::MISSION) {
    MissionSetpoint();
  } else if (mode == bfs::CustomMode::POSITION) {
    PositionSetpoint();
  }

  // For ALTITUDE, we just use the setpoint is set when the telemetry
  // command is received.
}


void SetpointHandler::TakeoffSetpoint() {
  if (takeoffFlag_ == false) {
    takeoffFlag_ = true;
    // Set all setpoints to the current position
    takeoffSetpoint_ = quadData_->navData.position_NED;
    // Set the third component to the desired altitude (remember Z down)
    takeoffSetpoint_[2] = -0.25;
  }
  *posSetpoint_ = takeoffSetpoint_;
  velSetpoint_->setZero();

  //Check if we're close. Start timer
  if (abs(quadPos_->coeff(2) - posSetpoint_->coeff(2)) < WP_ARRIVED_THRESH) {
    waypointArrived_ = true;
  } else {
    waypointArrived_ = false;
    waypointArrivedTimer_ = 0;
  }

  if (waypointArrivedTimer_ > WP_ARRIVED_TIME) {
    quadData_->telemData.mavlink->custom_mode(bfs::CustomMode::POSITION);
    takeoffFlag_ = false;
  }
}
void SetpointHandler::MissionSetpoint() {
  // TODO: Make this cleaner.

  // Mission items with a local cooridnate frame have the x, y positions as ints
  // representing the position multiplied by 1e4
  Eigen::Vector3f posLeft;
  Eigen::Vector3f velLeft;
  Eigen::Vector3f posRight;
  Eigen::Vector3f velRight;
  float tLeft;
  float tRight;
  int curIdx = quadData_->telemData.mavlink->active_mission_item();
  // If we're not at the first waypoint, go there and come back to this function
  // later.
  if (curIdx == 0) {
    posSetpoint_->data()[0] = static_cast<float>(quadData_->missionData.waypoints[curIdx].x)*1e-4;
    posSetpoint_->data()[1] = static_cast<float>(quadData_->missionData.waypoints[curIdx].y)*1e-4;
    posSetpoint_->data()[2] = quadData_->missionData.waypoints[curIdx].z;
    velSetpoint_->setZero();
    if ((*quadPos_ - *posSetpoint_).norm() > WP_ARRIVED_THRESH) {
      returnToMissionMode_ = true;
      quadData_->telemData.mavlink->custom_mode(bfs::CustomMode::POSITION);
      return;
    }

    missionTime_ = 0;
    quadData_->telemData.mavlink->AdvanceMissionItem();
  }

  float t = static_cast<float>(missionTime_)*1e-6;
  curIdx = quadData_->telemData.mavlink->active_mission_item();
  // Take care of the case where we have moved on to the next waypoint. Includes
  // if we are somehow far behind.
  while (t > quadData_->missionData.waypoints[curIdx].param4 && 
         curIdx < quadData_->telemData.mavlink->num_mission_items()-1) {
    quadData_->telemData.mavlink->AdvanceMissionItem();
    curIdx = quadData_->telemData.mavlink->active_mission_item();
  }
  // If we've reached the end, it's time to land
  if (curIdx == quadData_->telemData.mavlink->num_mission_items() - 1) {
    quadData_->telemData.mavlink->custom_mode(bfs::CustomMode::LANDING);
  }
  // Extract data
  posLeft[0] = static_cast<float>(quadData_->missionData.waypoints[curIdx - 1].x)*1e-4;  
  posLeft[1] = static_cast<float>(quadData_->missionData.waypoints[curIdx - 1].y)*1e-4;
  posLeft[2] = quadData_->missionData.waypoints[curIdx - 1].z;
  posRight[0] = static_cast<float>(quadData_->missionData.waypoints[curIdx].x)*1e-4;  
  posRight[1] = static_cast<float>(quadData_->missionData.waypoints[curIdx].y)*1e-4;
  posRight[2] = quadData_->missionData.waypoints[curIdx].z;
  // Params 1-4 are used for velocity [m/s] and time [s]
  velLeft[0] = quadData_->missionData.waypoints[curIdx - 1].param1; 
  velLeft[1] = quadData_->missionData.waypoints[curIdx - 1].param2; 
  velLeft[2] = quadData_->missionData.waypoints[curIdx - 1].param3; 
  tLeft = quadData_->missionData.waypoints[curIdx - 1].param4;
  velRight[0] = quadData_->missionData.waypoints[curIdx].param1; 
  velRight[1] = quadData_->missionData.waypoints[curIdx].param2; 
  velRight[2] = quadData_->missionData.waypoints[curIdx].param3; 
  tRight = quadData_->missionData.waypoints[curIdx].param4;

  // Interpolate between points
  float frac = (t - tLeft)/(tRight - tLeft);
  *posSetpoint_ = posLeft + (posRight - posLeft)*frac;
  *velSetpoint_ = velLeft + (velRight - velLeft)*frac;
}

void SetpointHandler::LandingSetpoint() {
  if (landingFlag_ == false) {
    landingFlag_ = true;
    // Set all setpoints to the current position
    landingSetpoint_ = *quadPos_;
    // Set the third component to the desired altitude (remember Z down)
    landingSetpoint_[2] = -0.25f;
    waypointArrivedTimer_ = 0;
  }
  *posSetpoint_ = landingSetpoint_;
  velSetpoint_->setZero();

  //Check if we're close. Start timer
  if (abs(quadPos_->coeff(2) - posSetpoint_->coeff(2)) < WP_ARRIVED_THRESH) {
    waypointArrived_ = true;
  } else {
    waypointArrived_ = false;
    waypointArrivedTimer_ = 0;
  }
  if (waypointArrivedTimer_ > WP_ARRIVED_TIME) {
		// Set status to disarm regardless of switch position
		*inputOverride_ = true;
		quadData_->telemData.mavlink->throttle_enabled(false);
		quadData_->telemData.mavlink->custom_mode(bfs::CustomMode::MANUAL);
		landingFlag_ = false;
  }
}

void SetpointHandler::PositionSetpoint() {
  // posSetpoint is already set. We use this function to see if we've arrived,
  // then hold or switch to another flight mode. We also keep the velocity
  // setpoint at zero.
  if ((*quadPos_ - *posSetpoint_).norm() < WP_ARRIVED_THRESH) {
    waypointArrived_ = true;
  } else {
    waypointArrived_ = false;
    waypointArrivedTimer_ = 0;
  }

  if ((waypointArrivedTimer_ > WP_ARRIVED_TIME)
      && returnToMissionMode_ == true) {
    quadData_->telemData.mavlink->custom_mode(bfs::CustomMode::MISSION); 
    returnToMissionMode_ = false;
  }
}

