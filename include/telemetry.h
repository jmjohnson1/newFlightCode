#ifndef TELEM_H_
#define TELEM_H_

#include "common.h"
#include "IMU.h"

namespace telem {
  // Telemetry data stream periods (milliseconds)
  constexpr int16_t RAW_SENS_STREAM_PERIOD = 1000;
  constexpr int16_t EXT_STATUS_STREAM_PERIOD = 1000;
  constexpr int16_t RC_CHAN_STREAM_PERIOD = 1000;
  constexpr int16_t POS_STREAM_PERIOD = 100; // position
  constexpr int16_t EXTRA1_STREAM_PERIOD = 100; // attitude
  constexpr int16_t EXTRA2_STREAM_PERIOD = 1000;
  constexpr int16_t EXTRA3_STREAM_PERIOD = 1000;
  // By default, the minimum stream period is 33 ms. The bandwidth available is
  // limited by the telemetry radios. Increasing the "airrate" allows for more
  // bandwidth at the price of a shorter range.
  constexpr int16_t MIN_STREAM_PERIOD = 50;


  bool Begin(Quadcopter_t &quadData);
  void Run(Quadcopter_t &quadData, Generic_IMU &quadIMU);
  uint32_t CheckForNewPosition(Quadcopter_t &quadData); 
}

#endif
