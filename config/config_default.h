#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

#include <stdint.h>

namespace DroneConfig {

// You can edit these
// Rates in Hz
// The flight controller is designed to run at 2 kHz, best not to change it
constexpr float LOOP_RATE_FC = 2000;
// Determines frequency that controllers update at
constexpr float LOOP_RATE_ATT = 100;
constexpr float LOOP_RATE_POS = 75;
// Frequency of datalogging
constexpr float LOOP_RATE_LOG = 100;
// Flight duration to allocate space for in the datalog (seconds)
constexpr uint32_t LOG_DURATION = 1200;
// Position PID lowpass filter cutoff frequencies (set to 0 for no filtering) (Hz)
constexpr float POS_VERTICAL_P_CUTOFF_FREQ = 0.0f;
constexpr float POS_VERTICAL_I_CUTOFF_FREQ = 0.0f;
constexpr float POS_VERTICAL_D_CUTOFF_FREQ = 0.0f;
constexpr float POS_HORIZONTAL_P_CUTOFF_FREQ = 0.0f;
constexpr float POS_HORIZONTAL_I_CUTOFF_FREQ = 0.0f;
constexpr float POS_HORIZONTAL_D_CUTOFF_FREQ = 0.0f;
// Attitude PID lowpass filter cutoff frequencies (set to 0 for no filtering) (Hz)
constexpr float ATT_ROLL_P_CUTOFF_FREQ = 0.0f;
constexpr float ATT_ROLL_I_CUTOFF_FREQ = 0.0f;
constexpr float ATT_ROLL_D_CUTOFF_FREQ = 0.0f;
constexpr float ATT_PITCH_P_CUTOFF_FREQ = 0.0f;
constexpr float ATT_PITCH_I_CUTOFF_FREQ = 0.0f;
constexpr float ATT_PITCH_D_CUTOFF_FREQ = 0.0f;
constexpr float ATT_YAW_P_CUTOFF_FREQ = 0.0f;
constexpr float ATT_YAW_I_CUTOFF_FREQ = 0.0f;
constexpr float ATT_YAW_D_CUTOFF_FREQ = 0.0f;

// Don't edit these
// Loop periods in microseconds
constexpr uint64_t LOOP_PER_FC = static_cast<uint64_t>((1.0f / LOOP_RATE_FC) * 1.0e6);
constexpr uint64_t LOOP_PER_ATT = static_cast<uint64_t>((1.0f / LOOP_RATE_ATT) * 1.0e6);
constexpr uint64_t LOOP_PER_POS = static_cast<uint64_t>((1.0f / LOOP_RATE_POS) * 1.0e6);
constexpr uint64_t LOOP_PER_LOG = static_cast<uint64_t>((1.0f / LOOP_RATE_LOG) * 1.0e6);

} // namespace DroneConfig

#endif
