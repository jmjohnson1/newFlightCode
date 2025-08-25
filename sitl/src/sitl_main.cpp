#ifdef SITL_BUILD

#include <signal.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "quadcopter.h"
#include "sensor_models.h"
#include "teensy_hal.h"
#include "nav-functions.h"

// Global simulation components
SITLIMUModel* imu_sim = nullptr;
SITLRCModel* rc_sim = nullptr;

QuadcopterModel* quad_sim = nullptr;

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
  imu_params.accel_noise_std = 0.02;
  imu_params.gyro_noise_std = 0.002;
  imu_sim = new SITLIMUModel(imu_params);

  rc_sim = new SITLRCModel();

  // Define initial quad state and configuration
  QuadState quad_state;
  QuadConfig quad_config;
  quad_config.mass = 1.3;  // kg
  quad_config.body_moi  << 6.14438E-03, 5.12080E-06,  2.15539E-04,
                        5.12080E-06, 4.61604E-03,  -6.45560E-07,
                        2.15539E-04, -6.45560E-07, 9.75832E-03;
  quad_config.body_moi_inv = quad_config.body_moi.inverse();
  quad_config.prop_moi_z = 1.5e-5;
  quad_config.tau_motor = 0.015*4;  // sec
  quad_config.min_motor_rate = 277;
  quad_config.max_motor_rate = 1500;
  

  double kt = 4.8e-6;
  double km = 7.7e-8;
  double dxmf = 0.08665;
  double dymf = 0.13938;
  double dxmb = 0.10345;
  double dymb = 0.11383;
  double dzm = 0.021;

  quad_config.mixer_coeffs << kt,      kt,       kt,       kt,
                           dymf*kt, -dymf*kt, -dymb*kt, dymb*kt,
                           dxmf*kt, dxmf*kt,  -dxmb*kt, -dxmb*kt,
                           -km,     km,       -km,      km;
  quad_sim = new QuadcopterModel(quad_state, quad_config);

  // Configure RC channels
  SITLRCModel::ChannelConfig throttle_config;
  throttle_config.name = "throttle";
  throttle_config.min_pwm = 1000;
  throttle_config.max_pwm = 1965;
  throttle_config.current_pwm = 1000;
  rc_sim->configureChannel(0, throttle_config);

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

  quad_sim->Integrate(0.0, dt); // for now, passing in 0 for t

  // For now, create dummy physics data
  /*Eigen::Vector3d true_accel_body(0, 0, -9.81);  // Hovering*/
  /*Eigen::Vector3d true_gyro_body(0, 0, 0);       // No rotation*/

  // Update sensor models with physics data
  imu_sim->update(dt, quad_sim->GetTrueAccel(), quad_sim->GetTrueGyro());

  // Log data
  static int log_counter = 0;
  if (++log_counter % 100 == 0) {  // Log at 20Hz
    log_file << micros() << ",";

    // Convert attitude to euler angles
    Eigen::Vector3f attitude_euler = DCM2Euler(quad_sim->GetDCM_b_n().cast<float>())

    // Add position, attitude, sensor data, motor commands
    log_file << "0,0,0,0,0,0,0,0,0,0,0,0,";  // Dummy physics data
    log_file << quad_sim->GetPositionNED() << ","
             << quad_sim->GetVelocityNED() << ","
             << attitude_euler() << ","
             << quad_sim->GetOmega_b_n() << "\n";

    auto accel = imu_sim->getAccel();
    auto gyro = imu_sim->getGyro();
    log_file << accel[0] << "," << accel[1] << "," << accel[2] << ",";
    log_file << gyro[0] << "," << gyro[1] << "," << gyro[2] << ",";
    log_file << "0,0,0,0\n";  // Dummy motor commands
    log_file.flush();
  }
}

// Mock implementations for sensors
extern "C" {
// Mock IMU functions - replace the actual sensor calls
float mock_accel_x = 0.0f, mock_accel_y = 0.0f, mock_accel_z = -9.81f;
float mock_gyro_x = 0.0f, mock_gyro_y = 0.0f, mock_gyro_z = 0.0f;

// These would be called IMU.cpp code instead of real sensor reads
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
      std::this_thread::sleep_for(std::chrono::microseconds(500));  // 2kHz
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

#endif  // SITL_BUILD
