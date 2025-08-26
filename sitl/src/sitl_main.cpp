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
    Eigen::Vector3f attitude_euler = DCM2Euler(quad_sim->GetDCM_b_n().cast<float>());
    Eigen::Vector3d pos = quad_sim->GetPositionNED();
    Eigen::Vector3d vel = quad_sim->GetVelocityNED();
    Eigen::Vector3d w_bn = quad_sim->GetOmega_b_n();

    // Add position, attitude, sensor data, motor commands
    log_file << pos[0] << "," << pos[1] << "," << pos[2] << ","
             << vel[0] << "," << vel[1] << "," << vel[2] << ","
             << attitude_euler[0] << "," << attitude_euler[1] << "," << attitude_euler[2] << ","
             << w_bn[0] << "," << w_bn[1] << "," << w_bn[2] << ",";

    auto accel = imu_sim->getAccel();
    auto gyro = imu_sim->getGyro();
    log_file << accel[0] << "," << accel[1] << "," << accel[2] << ",";
    log_file << gyro[0] << "," << gyro[1] << "," << gyro[2] << ",";
    log_file << "0,0,0,0\n";  // Dummy motor commands
    log_file.flush();
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

// Flight code loop (Copy from flightCode.cpp, removed unwanted functions)
void Loop() {
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
	quadData.flightStatus.timeSinceBoot = micros();

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  if (isnan(quadData.navData.position_NED[0])) {
    ins.Initialize(quadIMU.GetGyro(), quadIMU.GetAcc(), quadData.navData.mocapPosition_NED.cast<double>());
  }


  /*telem::Run(quadData, quadIMU);*/
  /*// Check if parameters have updated*/
  /*if(quadData.telemData.paramsUpdated == true) {*/
  /*  quadData.telemData.paramsUpdated = false;*/
  /*  // Attitude PID gains*/
  /*  Kp_array[0] = quadData.telemData.paramValues[0]; */
  /*  Ki_array[0] = quadData.telemData.paramValues[1];*/
  /*  Kd_array[0] = quadData.telemData.paramValues[2];*/
  /*  Kp_array[1] = quadData.telemData.paramValues[3]; */
  /*  Ki_array[1] = quadData.telemData.paramValues[4];*/
  /*  Kd_array[1] = quadData.telemData.paramValues[5];*/
  /*  Kp_array[2] = quadData.telemData.paramValues[6]; */
  /*  Ki_array[2] = quadData.telemData.paramValues[7];*/
  /*  Kd_array[2] = quadData.telemData.paramValues[8];*/
  /*  // Position PID gains*/
  /*  Kp_pos[0] = quadData.telemData.paramValues[9];*/
  /*  Ki_pos[0] = quadData.telemData.paramValues[10];*/
  /*  Kd_pos[0] = quadData.telemData.paramValues[11];*/
  /*  Kp_pos[1] = quadData.telemData.paramValues[9];*/
  /*  Ki_pos[1] = quadData.telemData.paramValues[10];*/
  /*  Kd_pos[1] = quadData.telemData.paramValues[11];*/
  /*  Kp_pos[2] = quadData.telemData.paramValues[12];*/
  /*  Ki_pos[2] = quadData.telemData.paramValues[13];*/
  /*  Kd_pos[2] = quadData.telemData.paramValues[14];*/
  /**/
  /*  Kp2_array[0] = quadData.telemData.paramValues[21];*/
  /*  Ki2_array[0] = quadData.telemData.paramValues[23];*/
  /*  Kd2_array[0] = quadData.telemData.paramValues[22];*/
  /*  Kp2_array[1] = quadData.telemData.paramValues[21];*/
  /*  Ki2_array[1] = quadData.telemData.paramValues[23];*/
  /*  Kd2_array[1] = quadData.telemData.paramValues[22];*/
  /*  Kp2_array[2] = quadData.telemData.paramValues[21];*/
  /*  Ki2_array[2] = quadData.telemData.paramValues[23];*/
  /*  Kd2_array[2] = quadData.telemData.paramValues[22];*/
  /**/
  /**/
  /*  angleController.SetKp(Kp_array);*/
  /*  angleController.SetKi(Ki_array);*/
  /*  angleController.SetKd(Kd_array);*/
  /*  posControl.SetKp(Kp_pos);*/
  /*  posControl.SetKi(Ki_pos);*/
  /*  posControl.SetKd(Kd_pos);*/
  /*  dcmAttControl.SetKp(Kp2_array);*/
  /*  dcmAttControl.SetKi(Ki2_array);*/
  /*  dcmAttControl.SetKd(Kd2_array);*/
  /*}*/

#ifdef USE_EKF
    if (EKFUpdateTimer > DroneConfig::LOOP_PER_EKF) {
      EKFUpdateTimer = 0;
      quadData.navData.numMocapUpdates = telem::CheckForNewPosition(quadData);
      ins.Update(micros(), quadData.navData.numMocapUpdates, quadIMU.GetGyro(), quadIMU.GetAcc(), quadData.navData.mocapPosition_NED.cast<double>());
      quadData.navData.position_NED = ins.Get_PosEst().cast<float>();
      quadData.navData.velocity_NED = ins.Get_VelEst();
      quadData.attitudeData.eulerAngles_ekf = ins.Get_OrientEst();
      quadData.attitudeData.currentDCM = Euler2DCM(quadData.attitudeData.eulerAngles_ekf);
    }
    Madgwick6DOF(quadIMU.GetAcc(), quadIMU.GetGyro(), quadData.attitudeData.quat_madgwick, quadData.attitudeData.eulerAngles_madgwick, dt);
#else
    if (EKFUpdateTimer > DroneConfig::LOOP_PER_EKF) {
      Madgwick6DOF(imu_sim.GetAcc(), imu_sim.GetGyro(), quadData.attitudeData.quat_madgwick, quadData.attitudeData.eulerAngles_madgwick, dt);
    }
#endif

  // Flight boundary limit
  if (bndryOnOff == 1) {
    if (quadData.navData.position_NED[0] > FLIGHT_AREA_X_MAX ||
        quadData.navData.position_NED[0] < FLIGHT_AREA_X_MIN ||
        quadData.navData.position_NED[1] > FLIGHT_AREA_Y_MAX ||
        quadData.navData.position_NED[1] < FLIGHT_AREA_Y_MIN ||
        quadData.navData.position_NED[2] > FLIGHT_AREA_Z_MAX ||
        quadData.navData.position_NED[2] < FLIGHT_AREA_Z_MIN) {
        quadData.flightStatus.inputOverride = true;
        quadData.telemData.mavlink->throttle_enabled(false);
        throttleEnabled = false;
      }
  }
#ifdef USE_POSITION_CONTROLLER
    // TODO: Be better
    // Check if position Controller enabled
    Eigen::Vector3f currentPosCovariance = ins.Get_CovPos();
    if (currentPosCovariance[0] < positionCovarianceLimit &&
        currentPosCovariance[1] < positionCovarianceLimit &&
        currentPosCovariance[2] < positionCovarianceLimit) {
      positionFix = true;
      quadData.attitudeData.eulerAngles_active = &(quadData.attitudeData.eulerAngles_ekf);
    } else {
      positionFix = false;
      /*positionFix = true;*/
      quadData.attitudeData.eulerAngles_active = &(quadData.attitudeData.eulerAngles_madgwick);
    }
    if (positionCtrlTimer >= DroneConfig::LOOP_PER_POS) {
      getDesState(); // Convert raw commands to normalized values based on saturated control limits
      positionCtrlTimer = 0;
      if (positionFix == true) {
        customMode = quadData.telemData.mavlink->custom_mode();
        if (quadData.telemData.mavlink->throttle_enabled()) {
          if (customMode == bfs::CustomMode::MANUAL) {
            posControl.Reset();
          /*} else if (customMode == bfs::CustomMode::TAKEOFF && !TakeoffRampUp.Done()) {*/
          /*	quadData.flightStatus.thrustSetpoint = TakeoffRampUp.RampIncrement(quadData.flightStatus.thrustSetpoint, DroneConfig::LOOP_PER_POS);*/
          /*	quadData.attitudeData.eulerAngleSetpoint = *(quadData.attitudeData.eulerAngles_active);*/
          /*	posControl.Reset();*/
          } else {
            spHandler.UpdateSetpoint();
            posControl.Update(quadData.navData.positionSetpoint_NED.cast<double>(), quadData.navData.velocitySetpoint_NED,
                              ins.Get_PosEst(), ins.Get_VelEst(), *(quadData.attitudeData.eulerAngles_active), dt, false);
            if (customMode == bfs::CustomMode::TAKEOFF || customMode == bfs::CustomMode::LANDING) {
              quadData.flightStatus.thrustSetpoint = posControl.GetDesiredThrust();
              quadData.attitudeData.eulerAngleSetpoint[0] = posControl.GetDesiredRoll();
              quadData.attitudeData.eulerAngleSetpoint[1] = posControl.GetDesiredPitch();
            } else if (customMode == bfs::CustomMode::POSITION || customMode == bfs::CustomMode::MISSION) {
              // This is a little dirty.
              // Reset this because we know takeoff is done and this won't cause problems with the earlier if statement.
              TakeoffRampUp.Reset();
              quadData.flightStatus.thrustSetpoint = posControl.GetDesiredThrust();
              quadData.attitudeData.eulerAngleSetpoint[0] = posControl.GetDesiredRoll();
              quadData.attitudeData.eulerAngleSetpoint[1] = posControl.GetDesiredPitch();

            } else if (customMode == bfs::CustomMode::ALTITUDE) {
              quadData.flightStatus.thrustSetpoint = posControl.GetDesiredThrust();
            }
          }
        }
      } else {
        posControl.Reset();
      }
      // We can set the thrust input now
      quadData.flightStatus.controlInputs[0] = quadData.flightStatus.thrustSetpoint;
      // Save the setpoint as a quaternion too
      quadData.attitudeData.quatSetpoint = Euler2Quat(quadData.attitudeData.eulerAngleSetpoint);
    }
    # else
      // Compute desired state based on radio inputs
      getDesState(); // Convert raw commands to normalized values based on saturated control limits
      quadData.flightStatus.controlInputs[0] = quadData.flightStatus.thrustSetpoint;
      quadData.att.quatSetpoint = Euler2Quat(quadData.att.eulerAngleSetpoint);
#endif
    

    bool noIntegral = false;
    if (quadData.flightStatus.thrustSetpoint < 0.5f || throttleEnabled == false) {
      noIntegral = true;
    }
    
    if (attitudeCtrlTimer >= DroneConfig::LOOP_PER_ATT) {
      attitudeCtrlTimer = 0;
      Eigen::Vector3f gyroRates = {quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ()};
      /*if (customMode == bfs::CustomMode::MANUAL ||*/
      /*    customMode == bfs::CustomMode::ALTITUDE) {*/
      /*  // FIXME: Make the function take Eigen::Vector or give up on it*/
      /*  angleController.Update(quadData.att.eulerAngleSetpoint.data(), quadData.att, gyroRates, dt, noIntegral);*/
      /*  quadData.flightStatus.controlInputs(lastN(3)) = angleController.GetMoments();*/
      /*} else {*/
      /*  angleController.Update(quadData.att.eulerAngleSetpoint.data(), quadData.att, gyroRates, dt, noIntegral);*/
      /*  quadData.flightStatus.controlInputs(lastN(3)) = angleController.GetMoments();*/
      /*  // dcmAttControl.Update(quadData.att, gyroRates, dt);*/
      /*  // quadData.flightStatus.controlInputs(lastN(3)) = dcmAttControl.GetControlTorque();*/
      /*}*/
      angleController.Update(quadData.attitudeData.eulerAngleSetpoint, *(quadData.attitudeData.eulerAngles_active),
                             gyroRates, dt, noIntegral, quadData.attitudeData.yawRateSetpoint, positionFix);
      quadData.flightStatus.controlInputs(lastN(3)) = angleController.GetMoments();
    }

    // Convert thrust and moments from controller to angular rates
    if (quadData.telemData.mavlink->throttle_enabled()) {
      quadData.flightStatus.motorRates = ControlAllocator(quadData.flightStatus.controlInputs, quadProps::ALLOCATION_MATRIX_INV);
    } else {
      quadData.flightStatus.motorRates = Eigen::Vector4f::Zero();
    }
    // Convert angular rates to PWM commands
    motors.ScaleCommand(quadData.flightStatus.motorRates);

    float motorCommands_norm[4];
    motors.GetMotorCommands(motorCommands_norm);
    for (int i = 0; i < 4; i++) {
      quadData.flightStatus.motorRates_norm(i) = motorCommands_norm[i];
    }

    motors.CommandMotor();


    // Get vehicle commands for next loop iteration
    getCommands(); // Pulls current available radio commands
    failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup


    // Regulate loop rate
    loopRate(DroneConfig::LOOP_RATE_FC); 
}
// Main simulation loop
int main(int argc, char* argv[]) {
  initializeSITL();

  // Start your flight code in a separate thread
  std::thread flight_thread([]() {
    // Setup();
    // while(sim_running) {
    //     Loop();
    // }

    // For now, just run a simple loop
    while (sim_running) {
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
