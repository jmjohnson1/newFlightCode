#include "telemetry.h"

static bfs::MavLink<telem::NUM_PARAMS, telem::NUM_UTM> mavlink;

bool telem::Begin(MissionData_t &mission) {
  mavlink.hardware_serial(&Serial2);
  mavlink.aircraft_type(bfs::MULTIROTOR);
  mavlink.mission(mission.waypoints.data(), mission.waypoints.size())
}