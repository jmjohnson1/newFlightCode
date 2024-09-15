#ifndef NAV_HANDLER_H
#define NAV_HANDLER_H

#include "common.h"

class SetpointHandler {
  enum HandlerFlags { NONE = 0, TAKEOFF_RAMP_UP = 1 };

 public:
  SetpointHandler(QuadType::Quadcopter_t *quadData);
  void UpdateSetpoint();

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

class TakeoffRamp {
 public:
  TakeoffRamp(float maxThrust, float rampRate, uint64_t waitTime_us)
      : maxThrust_(maxThrust), rate_(rampRate), waitTime_us_(waitTime_us) {}
  bool Done() { return done_; }

  float RampIncrement(float currentThrust, uint64_t dt_us) {
		float updatedThrust;
		float dt = dt_us*1.0e-6;
		// Haven't reached max thrust, keep on incrementing
		if (!maxThrustReached_) {
			updatedThrust = currentThrust + rate_*dt;
			if (updatedThrust >= maxThrust_) {
				updatedThrust = maxThrust_;
				maxThrustReached_ = true;
				// Start the timer (really just zeroing it)
				timer_ = 0;
			}
			return updatedThrust;
		}
		// Have reached max thrust, check timer
		if (timer_ >= waitTime_us_) {
			done_ = true;
		}
		return maxThrust_;
	}

  void Reset() {
		done_ = false;
		maxThrustReached_ = false;
	}

 private:
  bool done_ = false;
	bool maxThrustReached_ = false;
	elapsedMicros timer_;
  const uint64_t waitTime_us_;
  const float maxThrust_;
  const float rate_;
};

#endif
