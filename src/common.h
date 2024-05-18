#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#include "eigen.h"
#include "UserDefines.h"
#include "ArduinoLibs/BFSMavlink/mavlink.h"  // BFS Mavlink implementation
#include "defaultParams.h"

// Define some constants for mission parameters
constexpr std::size_t MAX_WAYPOINTS = 200;
constexpr std::size_t MAX_FENCEPOINTS = 10;
constexpr std::size_t MAX_RALLYPOINTS = 5;
constexpr std::size_t NUM_UTM = 0;

// Minimum distance from waypoint for it to be considered "arrived"
constexpr float waypointArrivedThresh = 0.12;
constexpr uint64_t waypointArrivedTime = 3000000;

typedef struct AttitudeData_s {
  Eigen::Vector3f eulerAngles_madgwick = Eigen::Vector3f::Zero();
  Eigen::Vector3f eulerAngles_ekf = Eigen::Vector3f::Zero();
  Eigen::Quaternionf quat_madgwick = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Quaternionf quat_ekf = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Vector3f * eulerAngles_active = &eulerAngles_madgwick;
  // Setpoints: [roll angle, pitch angle, yaw rate]
  Eigen::Vector3f eulerAngleSetpoint = Eigen::Vector3f::Zero();
  Eigen::Quaternionf quatSetpoint = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Matrix3f currentDCM = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f desiredDCM = Eigen::Matrix3f::Identity();
} AttitudeData_t;

typedef struct MissionData_s {
  std::array<bfs::MissionItem, MAX_WAYPOINTS> waypoints;
  std::array<bfs::MissionItem, MAX_FENCEPOINTS> fencePoints;  // No idea what to do with this yet
  std::array<bfs::MissionItem, MAX_RALLYPOINTS> rallyPoints;  // Nor this
  std::array<bfs::MissionItem, MAX_WAYPOINTS> temp;
  int32_t currentWaypoint;
  uint16_t numWaypoints;
  uint16_t numFencePoints;
  uint16_t numRallyPoints;
} MissionData_t;

typedef struct NavData_s {
  Eigen::Vector3f position_NED;
  Eigen::Vector3f velocity_NED;
  Eigen::Vector3f positionSetpoint_NED = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocitySetpoint_NED = Eigen::Vector3f::Zero();
  Eigen::Vector3f homePosition_NED = Eigen::Vector3f::Zero();
  Eigen::Vector3f mocapPosition_NED = Eigen::Vector3f::Zero();
  uint32_t numMocapUpdates = 0;
  uint32_t mocapUpdate_mocapTime = 0;
  uint32_t mocapUpdate_quadTime = 0;
  // Timers for getting the time since the start of a flight phase
  elapsedMicros missionTime;
  elapsedMicros waypointArrivedTimer;
  bool takeoffFlag = false;
  bool landingFlag = false;
  Eigen::Vector3f takeoffPosition;
  Eigen::Vector3f landingPosition;

  bool waypointArrived = false;
} NavData_t;

typedef struct FilterData_s {
  Eigen::Vector3f accBias;
  Eigen::Vector3f gyroBias;
  Eigen::Vector3f covPos;
  Eigen::Vector3f covVel;
  Eigen::Vector3f covOrient;
  Eigen::Vector3f innovationPos;
  Eigen::Vector3f innovationVel;
} FilterData_t;


typedef struct FlightStatus_s {
  bool missionStarted = false;
  bool inputOverride = false;

  float thrustSetpoint = 0.0f;
  Eigen::Vector4f controlInputs = Eigen::Vector4f::Zero();
  Eigen::Vector4f motorRates = Eigen::Vector4f::Zero();
  uint64_t timeSinceBoot = micros();
} FlightStatus_t;

typedef struct TelemData_s {
  bfs::MavLink<NUM_PARAMS, NUM_UTM> *mavlink = nullptr;
  std::array<float, NUM_PARAMS> paramValues;
  std::array<char, NUM_PARAMS> paramIDs;
  bool paramsUpdated = false;
} TelemData_t;

typedef struct Quadcopter_s {
  // Attitude
  AttitudeData_t att;
  MissionData_t missionData;
  NavData_t navData;
  FlightStatus_t flightStatus;
  TelemData_t telemData;
  FilterData_t filterData;

} Quadcopter_t;


namespace quadProps {
	constexpr float MAX_ANGLE = 30.0f;  // Maximum pitch/roll angle in degrees
	constexpr float MIN_THRUST = 1.0f;  // Minimum total thrust (N)
  constexpr float QUAD_MASS = 1.1f;  // Quadcopter mass (kg)

  constexpr float DIST_X_F = 0.08665f; // x Distance from CoM to front motors [m]
  constexpr float DIST_Y_F = 0.13938f; // x Distance from CoM to front motors [m]
  constexpr float DIST_X_B = 0.10345f; // x Distance from CoM to front motors [m]
  constexpr float DIST_Y_B = 0.11383f; // x Distance from CoM to front motors [m]

  // EMAX ECO II 1300KV with HQProp 7x4x3
  constexpr float K_T = 4.9831e-6;  // Thrust coefficient [N/(rad/s^2)]
  constexpr float K_M = 1.6320e-7;  // Motor torque coefficient [Nm/(rad/s^2)]
  #if defined BAT_3S
	constexpr float MAX_THRUST = 32.0f;  // Maximum total thrust (N)
  constexpr float K_W1 = -6.1875e2;
  constexpr float K_W2 = 1.7412e3;
  #elif defined BAT_4S
	constexpr float MAX_THRUST = 44.0f;  // Maximum total thrust (N)
  constexpr float K_W1 = -1.2349e3;
  constexpr float K_W2 = 2.7856e3;
  #endif

  constexpr float DXF_KT = DIST_X_F*K_T;
  constexpr float DYF_KT = DIST_Y_F*K_T;
  constexpr float DXB_KT = DIST_X_B*K_T;
  constexpr float DYB_KT = DIST_Y_B*K_T;

  static const Eigen::Matrix4f ALLOCATION_MATRIX((Eigen::Matrix4f() << 
                               K_T, K_T, K_T, K_T,
                               DYF_KT, -DYF_KT, -DYB_KT, DYB_KT,
                               DXF_KT, DXF_KT, -DXB_KT, -DXB_KT,
                               -K_M, K_M, -K_M, K_M).finished());
  
  static const Eigen::Matrix4f ALLOCATION_MATRIX_INV = ALLOCATION_MATRIX.inverse();

  constexpr float G = 9.8065;  // Gravity acceleration m/s^2
}

#endif
