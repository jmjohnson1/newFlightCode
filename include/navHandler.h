#ifndef NAV_HANDLER_H
#define NAV_HANDLER_H

#include "common.h"

class SetpointHandler {
	enum HandlerFlags {
		NONE = 0,
		TAKEOFF_RAMP_UP = 1
	};

public:
  SetpointHandler(QuadType::Quadcopter_t *quadData);
  HandlerFlags UpdateSetpoint();

private:
  void TakeoffSetpoint();
  void LandingSetpoint();
  void MissionSetpoint();
  void PositionSetpoint();
  

  // Keep a constant pointer to the global variables
  const QuadType::Quadcopter_t *quadData_;
  // Because these are used a lot, pointers to these can be pulled out of
  // quadData.
  const Eigen::Vector3f *quadPos_;
  // Pointer to global variables that we can modify here
  Eigen::Vector3f *posSetpoint_ = nullptr;
  Eigen::Vector3f *velSetpoint_ = nullptr;
  bool *inputOverride_ = nullptr;

  // Timer used for making sure we are at a waypoint for long enough to count
  elapsedMicros waypointArrivedTimer_;
  bool waypointArrived_ = false;
  // Timer used when following mission waypoints
  elapsedMicros missionTime_;
  // Tells whether a takeoff or landing sequence was initialized
  bool takeoffFlag_ = false;
  bool landingFlag_ = false;
  // Store the takeoff position setpoint 
  Eigen::Vector3f takeoffSetpoint_;
  // Store the landing position setpoint
  Eigen::Vector3f landingSetpoint_;
  // Used when we leave MISSION and want to return
  bool returnToMissionMode_ = false;
};

#endif
