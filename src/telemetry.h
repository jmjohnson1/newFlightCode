#ifndef TELEM_H_
#define TELEM_H_

#include "common.h"

namespace telem {
  constexpr size_t NUM_PARAMS = 20;
  constexpr size_t NUM_UTM = 10; 
  bool Begin(MissionData_t &mission);
  void Run();
}

#endif