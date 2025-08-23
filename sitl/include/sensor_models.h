#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

#ifdef UNIX_COMPILE
#include "Eigen/Dense"
#else
#include "eigen.h"
#endif

#include <random>

/**
 * @brief IMU sensor simulation with realistic noise characteristics
 * This will replace the actual IMU readings in your flight code
 */
class SITLIMUModel {
public:
    struct NoiseParams {
        // Accelerometer noise (m/s²)
        double accel_noise_std = 0.01;
        double accel_bias_std = 0.001;
        double accel_bias_correlation_time = 3600.0; // seconds
        
        // Gyroscope noise (rad/s)
        double gyro_noise_std = 0.001;
        double gyro_bias_std = 0.0001;
        double gyro_bias_correlation_time = 3600.0; // seconds
        
        // Magnetometer noise (Tesla)
        double mag_noise_std = 1e-6;
        
        // Update rate
        double update_rate = 1000.0; // Hz
    };
    
    SITLIMUModel(const NoiseParams& params = NoiseParams());
    
    // Update sensor with true values from physics engine
    void update(double dt, 
                const Eigen::Vector3d& true_accel_body,
                const Eigen::Vector3d& true_gyro_body,
                const Eigen::Vector3d& true_mag_body = Eigen::Vector3d::Zero());
    
    // Get sensor readings (what your flight code will see)
    Eigen::Vector3f getAccel() const { return accel_reading_.cast<float>(); }
    Eigen::Vector3f getGyro() const { return gyro_reading_.cast<float>(); }
    Eigen::Vector3f getMag() const { return mag_reading_.cast<float>(); }
    
    // Get raw (unfiltered) readings
    Eigen::Vector3f getAccelRaw() const { return accel_raw_.cast<float>(); }
    Eigen::Vector3f getGyroRaw() const { return gyro_raw_.cast<float>(); }
    
    // Reset biases (simulate IMU calibration)
    void resetBiases();
    
private:
    NoiseParams params_;
    
    // Current readings
    Eigen::Vector3d accel_reading_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_reading_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mag_reading_ = Eigen::Vector3d::Zero();
    
    // Raw readings (before your flight code's filtering)
    Eigen::Vector3d accel_raw_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_raw_ = Eigen::Vector3d::Zero();
    
    // Biases (slowly changing)
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    
    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> normal_dist_;
    
    // Bias evolution
    void updateBiases(double dt);
};

/**
 * @brief Virtual RC transmitter for testing
 */
class SITLRCModel {
public:
    struct ChannelConfig {
        std::string name;
        uint16_t min_pwm = 1000;
        uint16_t max_pwm = 2000;
        uint16_t center_pwm = 1500;
        uint16_t current_pwm = 1500;
        bool is_switch = false;
    };
    
    SITLRCModel();
    
    // Set channel values (-1.0 to 1.0 for normal channels, 0/1/2 for switches)
    void setChannel(int channel, double value);
    void setChannelPWM(int channel, uint16_t pwm);
    
    // Get PWM values (what SBUS.read() will return)
    bool getChannels(uint16_t* channels, bool* failsafe, bool* lost_frame);
    
    // Simulate failsafe condition
    void setFailsafe(bool failsafe) { failsafe_ = failsafe; }
    
    // Configure channels
    void configureChannel(int channel, const ChannelConfig& config);
    
private:
    static const int MAX_CHANNELS = 16;
    ChannelConfig channels_[MAX_CHANNELS];
    bool failsafe_ = false;
    bool lost_frame_ = false;
};

/**
 * @brief GPS simulation (if you use GPS)
 */
class SITLGPSModel {
public:
    struct GPSReading {
        double latitude = 0.0;   // degrees
        double longitude = 0.0;  // degrees
        double altitude = 0.0;   // meters
        double ground_speed = 0.0; // m/s
        double course = 0.0;     // degrees
        uint8_t num_satellites = 12;
        bool fix_valid = true;
    };
    
    SITLGPSModel();
    
    // Update with true position from physics
    void update(const Eigen::Vector3d& ned_position, 
                const Eigen::Vector3d& ned_velocity);
    
    // Get GPS reading
    const GPSReading& getReading() const { return reading_; }
    
    // Set home position (GPS origin)
    void setHome(double lat, double lon, double alt);
    
private:
    GPSReading reading_;
    double home_lat_ = 40.0; // Default home position
    double home_lon_ = -74.0;
    double home_alt_ = 0.0;
    
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> pos_noise_;
    std::normal_distribution<double> vel_noise_;
};

#endif // SENSOR_MODELS_H