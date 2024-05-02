#ifndef MISSION_HANDLER_H
#define MISSION_HANDLER_H

#include "common.h"
#include "navSetpoints.h"

class MissionHandler {
public:
  void Init(Quadcopter_t *quadData, NavData_t *navData);
  int Run();
private:
  Quadcopter_t *quadData_ = nullptr;
  NavData_t *navData_ = nullptr;
};

#endif