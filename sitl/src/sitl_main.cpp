#ifdef SITL_BUILD

#include "teensy_hal.h"
#include "sensor_models.h"

// Include your physics engine header here
// #include "your_physics_engine.h"

// Include adapted flight code
// You'll need to create a wrapper that includes your main flight code
// but excludes Teensy-specific initialization

#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <signal.h>

// Global simulation components
SITLIMUModel* imu_sim = nullptr;
SITLRCModel* rc_sim = nullptr;
// YourPhysicsEngine* physics_sim = nullptr;

// Simulation control
bool sim_running = true;
std::ofstream log_file;

// Signal handler for clean shutdown
void signal_handler(int signal) {
    std::cout << "\nShutting down SITL simulation...\n";
    sim_running = false;
}

// Initialize simulation
void initializeSITL() {
    std::cout << "=== Quadcopter SITL Simulation ===" << std::endl;
    
    // Initialize sensor models
    SITLIMUModel::NoiseParams imu_params;
    imu_params.accel_noise_std = 0.02;  // Adjust based on your real IMU
    imu_params.gyro_noise_std = 0.002;
    imu_sim = new SITLIMUModel(imu_params);
    
    rc_sim = new SITLRCModel();
    
    // Configure RC channels to match your setup
    SITLRCModel::ChannelConfig throttle_config;
    throttle_config.name = "throttle";
    throttle_config.min_pwm = 1000;
    throttle_config.max_pwm = 1965;
    throttle_config.current_pwm = 1000;
    rc_sim->configureChannel(0, throttle_config);
    
    // Initialize physics simulation
    // physics_sim = new YourPhysicsEngine();
    
    // Open log file
    log_file.open("sitl_log.csv");
    log_file << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,p,q,r,"
             << "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
             << "motor1,motor2,motor3,motor4\n";
    
    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "SITL initialized. Press Ctrl+C to stop.\n";
}

// Simulation step
void simulationStep() {
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;
    
    // Limit dt for numerical stability
    dt = std::min(dt, 0.01);
    
    // TODO: Update your physics simulation here
    // physics_sim->update(dt, motor_commands);
    
    // For now, create dummy physics data
    Eigen::Vector3d true_accel_body(0, 0, -9.81); // Hovering
    Eigen::Vector3d true_gyro_body(0, 0, 0);      // No rotation
    
    // Update sensor models with physics data
    imu_sim->update(dt, true_accel_body, true_gyro_body);
    
    // Log data
    static int log_counter = 0;
    if (++log_counter % 100 == 0) { // Log at 20Hz
        log_file << micros() << ",";
        // Add position, attitude, sensor data, motor commands
        log_file << "0,0,0,0,0,0,0,0,0,0,0,0,"; // Dummy physics data
        
        auto accel = imu_sim->getAccel();
        auto gyro = imu_sim->getGyro();
        log_file << accel[0] << "," << accel[1] << "," << accel[2] << ",";
        log_file << gyro[0] << "," << gyro[1] << "," << gyro[2] << ",";
        log_file << "0,0,0,0\n"; // Dummy motor commands
        log_file.flush();
    }
}

// Mock implementations for sensors that your flight code will call
extern "C" {
    // Mock IMU functions - replace the actual sensor calls
    float mock_accel_x = 0.0f, mock_accel_y = 0.0f, mock_accel_z = -9.81f;
    float mock_gyro_x = 0.0f, mock_gyro_y = 0.0f, mock_gyro_z = 0.0f;
    
    // These would be called by your IMU.cpp code instead of real sensor reads
    void updateMockIMUData() {
        if (imu_sim) {
            auto accel = imu_sim->getAccel();
            auto gyro = imu_sim->getGyro();
            
            mock_accel_x = accel[0];
            mock_accel_y = accel[1]; 
            mock_accel_z = accel[2];
            
            mock_gyro_x = gyro[0];
            mock_gyro_y = gyro[1];
            mock_gyro_z = gyro[2];
        }
    }
}

// Mock SBUS implementation
void SITLSBUS::begin() {
    std::cout << "SBUS initialized (SITL)\n";
    initialized = true;
}

bool SITLSBUS::read(uint16_t* channels, bool* failSafe, bool* lostFrame) {
    if (!initialized || !rc_sim) {
        return false;
    }
    
    return rc_sim->getChannels(channels, failSafe, lostFrame);
}

// Main simulation loop
int main(int argc, char* argv[]) {
    initializeSITL();
    
    // Start your flight code in a separate thread
    std::thread flight_thread([]() {
        // This is where you'd call your main flight code
        // You'll need to modify your main flight loop to work with SITL
        // Setup(); // Your flight code setup
        // while(sim_running) {
        //     Loop(); // Your flight code loop
        // }
        
        // For now, just run a simple loop
        while (sim_running) {
            updateMockIMUData();
            std::this_thread::sleep_for(std::chrono::microseconds(500)); // 2kHz
        }
    });
    
    // Main simulation loop
    while (sim_running) {
        simulationStep();
        
        // Run at ~1kHz for physics/sensors
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // Cleanup
    flight_thread.join();
    
    if (imu_sim) delete imu_sim;
    if (rc_sim) delete rc_sim;
    // if (physics_sim) delete physics_sim;
    
    log_file.close();
    
    std::cout << "SITL simulation ended.\n";
    return 0;
}

#endif // SITL_BUILD