#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#include "eigen.h"
#include "UserDefines.h"
#include "ArduinoLibs/BFSMavlink/mavlink.h"  // BFS Mavlink implementation

// Define some constants for mission parameters
constexpr std::size_t MAX_WAYPOINTS = 20;
constexpr std::size_t MAX_FENCEPOINTS = 10;
constexpr std::size_t MAX_RALLYPOINTS = 5;

typedef struct AttitudeData_s {
  Eigen::Vector3f eulerAngles_madgwick = Eigen::Vector3f::Zero();
  Eigen::Vector3f eulerAngles_ekf = Eigen::Vector3f::Zero();
  Eigen::Quaternionf quat_madgwick = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Quaternionf quat_ekf = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Vector3f * eulerAngles_active = &eulerAngles_madgwick;
} AttitudeData_t;

typedef struct MissionData_s {
  std::array<bfs::MissionItem, MAX_WAYPOINTS> waypoints;
  std::array<bfs::MissionItem, MAX_FENCEPOINTS> fencePoints;  // No idea what to do with this yet
  std::array<bfs::MissionItem, MAX_RALLYPOINTS> rallyPoints;  // Nor this
  std::array<bfs::MissionItem, MAX_WAYPOINTS> temp;
  int16_t currentWaypoint;
  uint16_t numWaypoints;
  uint16_t numFencePoints;
  uint16_t numRallyPoints;
} MissionData_t;

typedef struct Quadcopter_s {
  // Attitude
  AttitudeData_t att;
  MissionData_t missionData;
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
}

#endif
