#ifdef SITL_BUILD

#include "sensor_models.h"
#include <cmath>
#include <algorithm>

// IMU Model Implementation
SITLIMUModel::SITLIMUModel(const NoiseParams& params) 
    : params_(params), gen_(rd_()), normal_dist_(0.0, 1.0) {
    resetBiases();
}

void SITLIMUModel::update(double dt, 
                          const Eigen::Vector3d& true_accel_body,
                          const Eigen::Vector3d& true_gyro_body,
                          const Eigen::Vector3d& true_mag_body) {
    
    // Update biases (slowly varying)
    updateBiases(dt);
    
    // Add noise and bias to accelerometer
    for (int i = 0; i < 3; ++i) {
        double noise = normal_dist_(gen_) * params_.accel_noise_std;
        accel_raw_[i] = true_accel_body[i] + accel_bias_[i] + noise;
    }
    
    // Add noise and bias to gyroscope
    for (int i = 0; i < 3; ++i) {
        double noise = normal_dist_(gen_) * params_.gyro_noise_std;
        gyro_raw_[i] = true_gyro_body[i] + gyro_bias_[i] + noise;
    }
    
    // Add noise to magnetometer
    for (int i = 0; i < 3; ++i) {
        double noise = normal_dist_(gen_) * params_.mag_noise_std;
        mag_reading_[i] = true_mag_body[i] + noise;
    }
    
    // For now, filtered = raw (your flight code will do the filtering)
    accel_reading_ = accel_raw_;
    gyro_reading_ = gyro_raw_;
}

void SITLIMUModel::updateBiases(double dt) {
    // First-order Markov process for bias evolution
    double accel_alpha = exp(-dt / params_.accel_bias_correlation_time);
    double gyro_alpha = exp(-dt / params_.gyro_bias_correlation_time);
    
    for (int i = 0; i < 3; ++i) {
        // Accelerometer bias evolution
        double accel_noise = normal_dist_(gen_) * params_.accel_bias_std * sqrt(1.0 - accel_alpha * accel_alpha);
        accel_bias_[i] = accel_alpha * accel_bias_[i] + accel_noise;
        
        // Gyroscope bias evolution  
        double gyro_noise = normal_dist_(gen_) * params_.gyro_bias_std * sqrt(1.0 - gyro_alpha * gyro_alpha);
        gyro_bias_[i] = gyro_alpha * gyro_bias_[i] + gyro_noise;
    }
}

void SITLIMUModel::resetBiases() {
    accel_bias_ = Eigen::Vector3d::Zero();
    gyro_bias_ = Eigen::Vector3d::Zero();
}

// RC Model Implementation
SITLRCModel::SITLRCModel() {
    // Initialize default channel configurations
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        channels_[i].name = "CH" + std::to_string(i + 1);
        channels_[i].min_pwm = 1000;
        channels_[i].max_pwm = 2000;
        channels_[i].center_pwm = 1500;
        channels_[i].current_pwm = 1500;
        channels_[i].is_switch = false;
    }
    
    // Configure typical channels
    channels_[0].name = "Throttle";
    channels_[0].center_pwm = 1000;  // Throttle low
    channels_[0].current_pwm = 1000;
    
    channels_[4].name = "Throttle Cut";
    channels_[4].is_switch = true;
    channels_[4].current_pwm = 2000;  // Switch high (cut enabled)
    
    channels_[5].name = "Mode Switch";  
    channels_[5].is_switch = true;
}

void SITLRCModel::setChannel(int channel, double value) {
    if (channel < 0 || channel >= MAX_CHANNELS) return;
    
    if (channels_[channel].is_switch) {
        // Switch: 0 = low, 1 = mid, 2 = high
        int switch_pos = std::max(0, std::min(2, static_cast<int>(value)));
        switch (switch_pos) {
            case 0: channels_[channel].current_pwm = channels_[channel].min_pwm; break;
            case 1: channels_[channel].current_pwm = channels_[channel].center_pwm; break;
            case 2: channels_[channel].current_pwm = channels_[channel].max_pwm; break;
        }
    } else {
        // Normal channel: -1.0 to 1.0
        value = std::max(-1.0, std::min(1.0, value));
        
        if (channel == 0) {  // Throttle: 0.0 to 1.0
            value = std::max(0.0, value);
            channels_[channel].current_pwm = static_cast<uint16_t>(
                channels_[channel].min_pwm + value * (channels_[channel].max_pwm - channels_[channel].min_pwm)
            );
        } else {  // Other channels: -1.0 to 1.0 around center
            channels_[channel].current_pwm = static_cast<uint16_t>(
                channels_[channel].center_pwm + value * (channels_[channel].max_pwm - channels_[channel].center_pwm) / 2.0
            );
        }
    }
}

void SITLRCModel::setChannelPWM(int channel, uint16_t pwm) {
    if (channel < 0 || channel >= MAX_CHANNELS) return;
    
    pwm = std::max(channels_[channel].min_pwm, 
                   std::min(channels_[channel].max_pwm, pwm));
    channels_[channel].current_pwm = pwm;
}

bool SITLRCModel::getChannels(uint16_t* channels, bool* failsafe, bool* lost_frame) {
    if (!channels || !failsafe || !lost_frame) return false;
    
    // Copy current PWM values
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        channels[i] = channels_[i].current_pwm;
    }
    
    *failsafe = failsafe_;
    *lost_frame = lost_frame_;
    
    return true;  // Always successful in simulation
}

void SITLRCModel::configureChannel(int channel, const ChannelConfig& config) {
    if (channel < 0 || channel >= MAX_CHANNELS) return;
    channels_[channel] = config;
}

// GPS Model Implementation  
SITLGPSModel::SITLGPSModel() 
    : gen_(rd_()), pos_noise_(0.0, 3.0), vel_noise_(0.0, 0.1) {
    // Initialize with default home position
    reading_.latitude = home_lat_;
    reading_.longitude = home_lon_;
    reading_.altitude = home_alt_;
}

void SITLGPSModel::update(const Eigen::Vector3d& ned_position, 
                          const Eigen::Vector3d& ned_velocity) {
    
    // Convert NED position to GPS coordinates
    // Simple approximation for small areas
    const double earth_radius = 6378137.0;  // meters
    const double lat_scale = 180.0 / (M_PI * earth_radius);
    const double lon_scale = 180.0 / (M_PI * earth_radius * cos(home_lat_ * M_PI / 180.0));
    
    // Add GPS noise
    double lat_noise = pos_noise_(gen_) * lat_scale;
    double lon_noise = pos_noise_(gen_) * lon_scale;
    double alt_noise = pos_noise_(gen_);
    
    reading_.latitude = home_lat_ + ned_position[1] * lat_scale + lat_noise;
    reading_.longitude = home_lon_ + ned_position[0] * lon_scale + lon_noise;  
    reading_.altitude = home_alt_ - ned_position[2] + alt_noise;  // NED z is down
    
    // Convert NED velocity to ground speed and course
    double vel_north = ned_velocity[1] + vel_noise_(gen_);
    double vel_east = ned_velocity[0] + vel_noise_(gen_);
    
    reading_.ground_speed = sqrt(vel_north * vel_north + vel_east * vel_east);
    reading_.course = atan2(vel_east, vel_north) * 180.0 / M_PI;
    if (reading_.course < 0) reading_.course += 360.0;
    
    // Simulate occasional GPS dropouts
    static int update_counter = 0;
    if (++update_counter % 1000 == 0) {  // Brief dropout every 1000 updates
        reading_.fix_valid = false;
        reading_.num_satellites = 3;
    } else {
        reading_.fix_valid = true;
        reading_.num_satellites = 12;
    }
}

void SITLGPSModel::setHome(double lat, double lon, double alt) {
    home_lat_ = lat;
    home_lon_ = lon;
    home_alt_ = alt;
}

#endif // SITL_BUILD