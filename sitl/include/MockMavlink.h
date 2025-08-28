#ifndef MOCK_MAVLINK_H
#define MOCK_MAVLINK_H

#include "teensy_hal.h"

namespace bfs {

struct MissionItem {
  bool autocontinue;
  uint8_t frame;
  uint16_t cmd;
  float param1;
  float param2;
  float param3;
  float param4;
  int32_t x;
  int32_t y;
  float z;
};

enum CustomMode : uint32_t {
  MANUAL,
  ALTITUDE,
  POSITION,
  MISSION,
  TAKEOFF, 
  LANDING,
};

template <std::size_t N, std::size_t M>
class MavLink {
 public:
  // Do-nothing functions
  inline void hardware_serial(SITLSerial *bus) { (void)bus; }
  inline void aircraft_type(const int8_t type) { (void)type; }
  inline void mission(MissionItem *const mission, const std::size_t mission_size, MissionItem *const temp) {
    (void)mission;
    (void)mission_size;
    (void)temp;
  }
  inline void fence(MissionItem *const fence, const std::size_t fence_size) {
    (void)fence;
    (void)fence_size;
  }
  inline void rally(MissionItem *const rally, const std::size_t rally_size) {
    (void)rally;
    (void)rally_size;
  }
  void Begin(uint32_t baud) { (void)baud; }
  void Update() {}

  inline void raw_sens_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void raw_sens_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t raw_sens_stream_period_ms() const { return 0; }
  inline void ext_status_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void ext_status_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t ext_status_stream_period_ms() const { return 0; }
  inline void rc_chan_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void rc_chan_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t rc_chan_stream_period_ms() const { return 0; }
  inline void pos_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void pos_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t pos_stream_period_ms() const { return 0; }
  inline void extra1_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void extra1_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t extra1_stream_period_ms() const { return 0; }
  inline void extra2_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void extra2_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t extra2_stream_period_ms() const { return 0; }
  inline void extra3_stream_period_default_ms(const int32_t val) { (void)val; }
  inline void extra3_stream_period_ms(const int16_t val) { (void)val; }
  inline int16_t extra3_stream_period_ms() const { return 0; }
  inline void setMinStreamPeriod_ms(const int16_t val) { (void)val; }

  // Sharing position & sensors with GCS
  inline void imu_accel_x_mps2(const float val) { (void)val; }
  inline void imu_accel_y_mps2(const float val) { (void)val; }
  inline void imu_accel_z_mps2(const float val) { (void)val; }
  inline void imu_gyro_x_radps(const float val) { (void)val; }
  inline void imu_gyro_y_radps(const float val) { (void)val; }
  inline void imu_gyro_z_radps(const float val) { (void)val; }
  inline void nav_north_pos_m(const float val) { (void)val; }
  inline void nav_east_pos_m(const float val) { (void)val; }
  inline void nav_down_pos_m(const float val) { (void)val; }
  inline void nav_north_vel_mps(const float val) { (void)val; }
  inline void nav_east_vel_mps(const float val) { (void)val; }
  inline void nav_down_vel_mps(const float val) { (void)val; }
  inline void north_pos_setpoint_m(const float val) { (void)val; }
  inline void east_pos_setpoint_m(const float val) { (void)val; }
  inline void down_pos_setpoint_m(const float val) { (void)val; }
  inline void north_vel_setpoint_m(const float val) { (void)val; }
  inline void east_vel_setpoint_m(const float val) { (void)val; }
  inline void down_vel_setpoint_m(const float val) { (void)val; }
  inline void quaternionSetpoint(const float val[4]) { (void)val; }
  inline void nav_pitch_rad(const float val) { (void)val; }
  inline void nav_roll_rad(const float val) { (void)val; }
  inline void nav_hdg_rad(const float val) { (void)val; }

  // Always return false (don't reset parameters)
  inline bool param_reset() { return false; }
  // Always return true for sim (for now)
  inline bool throttle_enabled() { return true; }
  inline bool throttle_enabled(bool val) { (void)val; }


  // Mission
  inline int32_t active_mission_item() const {return mission_current_index_;}
  inline std::size_t num_mission_items() const {return mission_current_count_;}
  inline void AdvanceMissionItem() {};

  // Position updates
  inline float viconX() {return viconX_;}
  inline float viconY() {return viconY_;}
  inline float viconZ() {return viconZ_;}
  inline uint32_t viconTime() {return viconTime_;}
  inline uint32_t numViconRX() {return numViconRX_;}


  // Commanding setpoints from GCS
  inline bool new_setpoint_available() {return newSetpointAvailable_; }
  inline float new_setpoint_x() { return newPositionSetpointRequestX_; }
  inline float new_setpoint_y() { return newPositionSetpointRequestY_; }
  inline float new_setpoint_z() { return newPositionSetpointRequestZ_; }

  // Parameters
  inline void param(const int32_t i, const float val) {
    (void)i;
    (void)val;
  }
  inline int32_t updated_param() {return -1;}

  // Mode
  inline void custom_mode(const uint32_t val) {
    custom_mode_ = val;
  }
  inline uint32_t custom_mode() {
    return custom_mode_;
  }

 private:
  int32_t mission_current_index_ = 0;
  std::size_t mission_current_count_ = 0;

  float viconX_ = 0.0f;
  float viconY_ = 0.0f;
  float viconZ_ = 0.0f;
  uint32_t viconTime_ = 0.0f;
  uint32_t numViconRX_ = 0;
  
  bool newSetpointAvailable_ = false;
  float newPositionSetpointRequestX_ = 0.0f;
  float newPositionSetpointRequestY_ = 0.0f;
  float newPositionSetpointRequestZ_ = 0.0f;

  uint32_t custom_mode_ = 0;
};

}  // namespace bfs

#endif  // MOCK_MAVLINK_H
