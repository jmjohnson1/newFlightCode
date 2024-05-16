#include "navSetpoints.h"

void SetpointHandler(NavData_t *navdata, Quadcopter_t *quadData) {
  // Flight mode check
  uint32_t mode = quadData->telemData.mavlink->custom_mode();
  if (mode == bfs::CustomMode::TAKEOFF) {
    TakeoffSetpoints(navdata, quadData);
  } else if (mode == bfs::CustomMode::LANDING) {
    LandingSetpoints(navdata, quadData);
  } else if (mode == bfs::CustomMode::MISSION) {
    MissionSetpoints(navdata, quadData);
  }
  // For POSITION/ALTITUDE, we just use the setpoint is set when the telemetry
  // command is received.
}


void TakeoffSetpoints(NavData_t *navdata, Quadcopter_t *quadData) {
  if (navdata->takeoffFlag == false) {
    navdata->takeoffFlag = true;
    // Set all setpoints to the current position
    navdata->takeoffPosition = navdata->position_NED;
    // Set the third component to the desired altitude (remember Z down)
    navdata->takeoffPosition[2] = -0.75;
  }
  navdata->positionSetpoint_NED = navdata->takeoffPosition;
  navdata->velocitySetpoint_NED.setZero();

  //Check if we're close. Start timer
  if (abs(navdata->position_NED[2] - navdata->positionSetpoint_NED[2]) < waypointArrivedThresh) {
    navdata->waypointArrived = true;
  } else {
    navdata->waypointArrived = false;
    navdata->waypointArrivedTimer = 0;
  }
  if (navdata->waypointArrivedTimer > waypointArrivedTime) {
      quadData->telemData.mavlink->custom_mode(bfs::CustomMode::POSITION);
  }
}
void MissionSetpoints(NavData_t *navdata, Quadcopter_t *quadData) {
  // TODO: Make this cleaner. As is, it makes me want to cry.

  // Mission items with a local cooridnate frame have the x, y positions as ints
  // representing the position multiplied by 1e4
  Eigen::Vector3f posLeft;
  Eigen::Vector3f velLeft;
  Eigen::Vector3f posRight;
  Eigen::Vector3f velRight;
  float tLeft;
  float tRight;
  int curIdx = quadData->telemData.mavlink->active_mission_item();
  if (curIdx == 0) {
    navdata->missionTime = 0;
    quadData->telemData.mavlink->AdvanceMissionItem();
  }
  float t = static_cast<float>(navdata->missionTime)*1e-6;
  curIdx = quadData->telemData.mavlink->active_mission_item();
  // Take care of the case where we have moved on to the next waypoint. Includes
  // if we are somehow far behind.
  while (t > quadData->missionData.waypoints[curIdx].param4 && curIdx < quadData->telemData.mavlink->num_mission_items()-1) {
    quadData->telemData.mavlink->AdvanceMissionItem();
    curIdx = quadData->telemData.mavlink->active_mission_item();
  }
  if (curIdx == quadData->telemData.mavlink->num_mission_items() - 1) {
    quadData->telemData.mavlink->custom_mode(bfs::CustomMode::LANDING);
  }
  // Extract data
  posLeft[0] = static_cast<float>(quadData->missionData.waypoints[curIdx - 1].x)*1e-4;  
  posLeft[1] = static_cast<float>(quadData->missionData.waypoints[curIdx - 1].y)*1e-4;
  posLeft[2] = quadData->missionData.waypoints[curIdx - 1].z;
  posRight[0] = static_cast<float>(quadData->missionData.waypoints[curIdx].x)*1e-4;  
  posRight[1] = static_cast<float>(quadData->missionData.waypoints[curIdx].y)*1e-4;
  posRight[2] = quadData->missionData.waypoints[curIdx].z;
  // Params 1-4 are used for velocity [m/s] and time [s]
  velLeft[0] = quadData->missionData.waypoints[curIdx - 1].param1; 
  velLeft[1] = quadData->missionData.waypoints[curIdx - 1].param2; 
  velLeft[2] = quadData->missionData.waypoints[curIdx - 1].param3; 
  tLeft = quadData->missionData.waypoints[curIdx - 1].param4;
  velRight[0] = quadData->missionData.waypoints[curIdx].param1; 
  velRight[1] = quadData->missionData.waypoints[curIdx].param2; 
  velRight[2] = quadData->missionData.waypoints[curIdx].param3; 
  tRight = quadData->missionData.waypoints[curIdx].param4;

  // Interpolate between points
  float frac = (t - tLeft)/(tRight - tLeft);
  navdata->positionSetpoint_NED = posLeft + (posRight - posLeft)*frac;
  navdata->velocitySetpoint_NED = velLeft + (velRight - velLeft)*frac;
}

void LandingSetpoints(NavData_t *navdata, Quadcopter_t *quadData) {
  if (navdata->landingFlag == false) {
    navdata->landingFlag = true;
    // Set all setpoints to the current position
    navdata->landingPosition = navdata->position_NED;
    // Set the third component to the desired altitude (remember Z down)
    navdata->landingPosition[2] = -0.2f;
    navdata->waypointArrivedTimer = 0;
  }
  navdata->positionSetpoint_NED = navdata->landingPosition;
  navdata->velocitySetpoint_NED.setZero();

  //Check if we're close. Start timer
  if (abs(navdata->position_NED[2] - navdata->positionSetpoint_NED[2]) < waypointArrivedThresh) {
    navdata->waypointArrived = true;
  } else {
    navdata->waypointArrived = false;
    navdata->waypointArrivedTimer = 0;
  }
  if (navdata->waypointArrivedTimer > waypointArrivedTime) {
      // Set status to disarm regardless of switch position
      quadData->flightStatus.inputOverride = true;
      quadData->telemData.mavlink->throttle_enabled(false);
      quadData->telemData.mavlink->custom_mode(bfs::CustomMode::MANUAL);
  }
}