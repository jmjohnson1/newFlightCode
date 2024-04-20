#ifndef TELEM_H_
#define TELEM_H_

#include "common.h"

namespace telem {
  bool Begin(Quadcopter_t &quadData);
  void Run(Quadcopter_t &quadData);
  uint32_t CheckForNewPosition(Quadcopter_t &quadData, Eigen::Vector3d& pos, uint32_t *mocapTimestamp, uint32_t *quadTimestamp); 
}

#endif