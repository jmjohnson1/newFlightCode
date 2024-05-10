#ifndef TELEM_H_
#define TELEM_H_

#include "common.h"

namespace telem {
  // Telemetry data stream periods (milliseconds)
  constexpr int16_t RAW_SENS_STREAM_PERIOD = 1000;
  constexpr int16_t EXT_STATUS_STREAM_PERIOD = 1000;
  constexpr int16_t RC_CHAN_STREAM_PERIOD = 1000;
  constexpr int16_t POS_STREAM_PERIOD = 250;
  constexpr int16_t EXTRA1_STREAM_PERIOD = 250;
  constexpr int16_t EXTRA2_STREAM_PERIOD = 250;
  constexpr int16_t EXTRA3_STREAM_PERIOD = 250;


  bool Begin(Quadcopter_t &quadData);
  void Run(Quadcopter_t &quadData);
  uint32_t CheckForNewPosition(Quadcopter_t &quadData); 
}

#endif