#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

#include <TeensyLog.h>
#include <stdint.h>

namespace DroneConfig {

// You can edit these
// Logging
const int LogLevel = LOG_LEVEL_VERBOSE;
constexpr bool LOG_VERBOSE_RADIO_PWM = false;
constexpr bool LOG_VERBOSE_CONTROL_INPUTS = false;
constexpr bool LOG_VERBOSE_MOTOR_COMMANDS = true;
constexpr bool LOG_VERBOSE_FLIGHT_STATUS = false;
// Rates in Hz
// The flight controller is designed to run at 2 kHz, best not to change it
constexpr float LOOP_RATE_FC = 2000;
// Determines frequency that controllers update at
constexpr float LOOP_RATE_ATT = 200;
constexpr float LOOP_RATE_POS = 200;
// Filter update rate
constexpr float LOOP_RATE_EKF = 500;
// Frequency of datalogging
constexpr float LOOP_RATE_LOG = 1000;
// Flight duration to allocate space for in the datalog (seconds)
constexpr uint32_t LOG_DURATION = 1200;
// Position PID lowpass filter cutoff frequencies (set to 0 for no filtering) (Hz)
constexpr float POS_DERIVATIVE_CUTOFF_FREQ = 25.0f;
constexpr float POS_PROPORTIONAL_CUTOFF_FREQ = 25.0f;


// Integrator buildup limits
constexpr float POS_INTEGRAL_LIMIT = 1.0f;
constexpr float ATT_INTEGRAL_LIMIT = 50.0f;

// Don't edit these

// Loop periods in microseconds
constexpr uint64_t LOOP_PER_FC = static_cast<uint64_t>((1.0f / LOOP_RATE_FC) * 1.0e6);
constexpr uint64_t LOOP_PER_ATT = static_cast<uint64_t>((1.0f / LOOP_RATE_ATT) * 1.0e6);
constexpr uint64_t LOOP_PER_POS = static_cast<uint64_t>((1.0f / LOOP_RATE_POS) * 1.0e6);
constexpr uint64_t LOOP_PER_LOG = static_cast<uint64_t>((1.0f / LOOP_RATE_LOG) * 1.0e6);
constexpr uint64_t LOOP_PER_EKF = static_cast<uint64_t>((1.0f / LOOP_RATE_EKF) * 1.0e6);

} // namespace DroneConfig

#endif
