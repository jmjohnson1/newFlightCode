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


// Needed for flight loop
#include "UserDefines.h"
#include "dcmPID.h"
#include "eulerPID.h"
#include "EKF.h"
#include "common.h"
#include "madgwick.h"
#include "navHandler.h"
#include "telemetry.h"
#include "radio.h"
#include "IMU.h"

#include "config_default.h"


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

// Variables for flight code
// Controller yaw rate deadzone (deg/s)
const float YAW_DEADZONE = 5.0f * DEG_TO_RAD;

// Radio channel definitions
// Syntax: ("name", channel, slider neutral point (meaningless for switches), failsafe value, true
// (if channel is critical))
RadioChannel throttleChannel ("throttle",     1, 1000, 1000, true, 1000, 1965);
RadioChannel rollChannel     ("roll",         2, 1500, 1500, true);
RadioChannel pitchChannel    ("pitch",        3, 1500, 1500, true);
RadioChannel yawChannel      ("yaw",          4, 1500, 1500, true);
RadioChannel throCutChannel  ("throttle_cut", 5, 1000, 2000);
RadioChannel boundaryOnOff   ("boundary_sw",  6, 1000, 2000);

#ifdef TEST_STAND
RadioChannel aux0 ("none",           6, 1500, 1500);
RadioChannel aux1 ("sine_sweep",     7, 1000, 1000);
RadioChannel aux2 ("step_axis_sel",  8, 1000, 1000);
RadioChannel aux3 ("step_angle_sel", 9, 1000, 1500);
#elif defined(USE_POSITION_CONTROLLER)
RadioChannel aux0 ("position_toggle", 6, 1500, 1500);
RadioChannel aux1 ("xSel",            7, 1000, 1500);
RadioChannel aux2 ("ySel",            8, 1000, 1500);
RadioChannel aux3 ("zSel",            9, 1000, 1500);
#else
RadioChannel aux0 ("none", 6, 1500, 1500);
RadioChannel aux1 ("none", 7, 1000, 1500);
RadioChannel aux2 ("none", 8, 1000, 1500);
RadioChannel aux3 ("none", 9, 1000, 1500);
#endif

RadioChannel KpScaleChannel  ("Kp_scale",  10, 1500, 1500);
RadioChannel KiScaleChannel  ("Ki_scale",  11, 1500, 1500);
RadioChannel KdScaleChannel  ("Kd_scale",  12, 1500, 1500);
RadioChannel scaleAllChannel ("scale_all", 13, 1500, 1500);
RadioChannel resetChannel    ("reset",     14, 1000, 1000);


// Array of pointers the the radio channels. This is useful for datalogging and updating the raw
// values.
const uint8_t numChannels = 15;
RadioChannel *radioChannels[numChannels] = 
	{
		&throttleChannel, 
		&rollChannel,
		&pitchChannel,
		&yawChannel,
		&throCutChannel,
		&boundaryOnOff,											
		&aux1,
		&aux2,
		&aux3,
		&KpScaleChannel,
		&KiScaleChannel,
		&KdScaleChannel,
		&scaleAllChannel,
		&resetChannel,
		&aux0
	};

// Max roll/pitch angles in degrees for angle mode
float maxRoll = quadProps::MAX_ANGLE*DEG_TO_RAD;
float maxPitch = quadProps::MAX_ANGLE*DEG_TO_RAD;
// Max yaw rate in deg/sec
float maxYawRate = 160.0*DEG_TO_RAD;

// ANGLE MODE PID GAINS //
// SCALE FACTORS FOR PID //
float pScale_att = 1.0f;
float iScale_att = 1.0f;
float dScale_att = 1.0f;
float allScale_att = 1.0f;

float Kp_roll_angle = 0.0f;
float Ki_roll_angle = 4.81f;
float Kd_roll_angle = 0.34f;
float Kp_pitch_angle = 1.66f;
float Ki_pitch_angle = 4.81f;
float Kd_pitch_angle = 0.34f;

// YAW PID GAINS //
float Kp_yaw = 0.11f;
float Ki_yaw = 0.00f;
float Kd_yaw = 0.00f;

// POSITION PID GAINS //
float Kp_pos[3] = {0.0f, 0.0f, 0.0f};
float Ki_pos[3] = {0.0f, 0.0f, 0.0f};
float Kd_pos[3] = {0.0f, 0.0f, 0.0f};

// MPU6050 Null shift //
float mpuNS_ax = 0.38f;
float mpuNS_ay = 0.50f;
float mpuNS_az = -0.43f;
float mpuNS_gx = -0.04f;
float mpuNS_gy = 0.01f;
float mpuNS_gz = 0.0f;

//================================================================================================//
//                                      DECLARE PINS                                              //
//================================================================================================//

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the
// 			 MPU6050 IMU for default setup

// Motor pin outputs:
const uint8_t motorPins[4] = {2, 3, 4, 5};
// LED indicator
const uint8_t ledPin = 10;
// Pins for BMI088 IMU
const uint8_t bmiAccCS = 6;
const uint8_t bmiGyrCS = 9;

//================================================================================================//

// SD card setup
// Interval between points (usec) for 100 samples/sec
#define LOG_INTERVAL_USEC 10000

// DECLARE GLOBAL VARIABLES
// General stuff
float dt;
QuadType::Quadcopter_t quadData;

unsigned long current_time, prev_time;
unsigned long print_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
unsigned long print_counterSD = 200000;

SITLSBUS sbus;
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

Eigen::Vector3f accNS  = Eigen::Vector3f::Zero();
Eigen::Vector3f gyroNS = Eigen::Vector3f::Zero();
SITL_IMU quadIMU(accNS, gyroNS);

Eigen::Vector3f accNS2  = Eigen::Vector3f::Zero();
Eigen::Vector3f gyroNS2 = Eigen::Vector3f::Zero();

SetpointHandler spHandler(&quadData);

// Controller:
float Kp_array[3] = {0.0f, 0.0f, 0.0f};
float Ki_array[3] = {0.0f, 0.0f, 0.0f};
float Kd_array[3] = {0.0f, 0.0f, 0.0f};
float Kp2_array[3] = {0.0f, 0.0f, 0.0f};
float Ki2_array[3] = {0.0f, 0.0f, 0.0f};
float Kd2_array[3] = {0.0f, 0.0f, 0.0f};
AngleAttitudeController angleController = AngleAttitudeController(Kp_array, Ki_array, Kd_array, 50.0f);
PositionController posControl = PositionController(Kp_pos, Ki_pos, Kd_pos, quadProps::MAX_ANGLE, quadProps::QUAD_MASS,
                                                   quadProps::MIN_THRUST, quadProps::MAX_THRUST, 1.0f,
                                                   DroneConfig::LOOP_RATE_POS, DroneConfig::POS_DERIVATIVE_CUTOFF_FREQ);
DCMAttitudeControl dcmAttControl = DCMAttitudeControl(Kp2_array, Ki2_array, Kd2_array, 1.0f, 0.8f);
Eigen::Vector3f b1d = {-1.0f, 0.0f, 0.0f};
uint32_t customMode = bfs::CustomMode::MANUAL;

bool SD_is_present = 0;
bool doneWithSetup = 0;
uint16_t failureFlag = 0;
int throttleCutCount = 0;
// Flag to check if the flight loop has started yet, prevents lock in main loop when throttle killed
bool logRunning = 0;
int loopCount = 0;
bool throttleEnabled = true;

// Position vector taken from mocap
bool positionFix = false;  // Set true if the position covariance from the EKF is less than a certain value.
const float positionCovarianceLimit = 1.0f;	// Maximum allowable position covariance from EKF. [m^2]

// EKF
EKF ins;

// Datalogging
/*Datalogger logging;*/

bool wasTrueLastLoop = false; // This will be renamed at some point

// Handles the thrust ramp up for takeoff
TakeoffRamp TakeoffRampUp(quadProps::QUAD_MASS*quadProps::G/2.0f, 1.0f, 0);

// Various timers
elapsedMicros EKFUpdateTimer;
elapsedMicros sdCardUpdateTimer;
elapsedMicros attitudeCtrlTimer;
elapsedMicros positionCtrlTimer;
elapsedMicros IMUUpdateTimer;

const unsigned long imuUpdatePeriod = 0; // microseconds (1000 Hz)

// Defining the flight area (Shepherd Drone Lab)
const float FLIGHT_AREA_X_MAX = 1;
const float FLIGHT_AREA_X_MIN = -1;
const float FLIGHT_AREA_Y_MAX = 1;
const float FLIGHT_AREA_Y_MIN = -1;
const float FLIGHT_AREA_Z_MAX = 0;
const float FLIGHT_AREA_Z_MIN = -2.5;
int bndryOnOff;


//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

/**
 * @brief Updates the desired states (setpoints) based on the raw transmitter
 * inputs obtained in getCommands(). Some of these values can be overwritten
 * later by other functions.
*/
void getDesState() {
	float thrust_des; float roll_des; float pitch_des; float yawRate_des;
  thrust_des = throttleChannel.NormalizedValue(); // Between 0 and 1
  roll_des = rollChannel.NormalizedValue();  // Between -1 and 1
  pitch_des = -pitchChannel.NormalizedValue(); // Between 0 and 1
  yawRate_des = -yawChannel.NormalizedValue();   // Between -1 and 1

  // Constrain within normalized bounds
  quadData.flightStatus.thrustSetpoint = constrain(thrust_des, 0.0, 1.0)*quadProps::MAX_THRUST;
  quadData.attitudeData.eulerAngleSetpoint[0] = constrain(roll_des, -1.0, 1.0) * maxRoll;
  quadData.attitudeData.eulerAngleSetpoint[1] = constrain(pitch_des, -1.0, 1.0) * maxPitch;
  quadData.attitudeData.yawRateSetpoint = constrain(yawRate_des, -1.0, 1.0) * maxYawRate;
	if (abs(yawRate_des) > YAW_DEADZONE) {
    /*Serial.print("Yaw rate: ");*/
    /*Serial.println(quadData.att.yawRateSetpoint);*/
		quadData.attitudeData.eulerAngleSetpoint[2] += quadData.attitudeData.yawRateSetpoint/2000.0f; 
    /*Serial.print("Yaw setpoint: ");*/
    /*Serial.println(quadData.att.eulerAngleSetpoint[2]);*/
	}
}

/**
 * @brief Gets the raw commands from the sbus radio receiver
*/
void getCommands() {

  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
    // sBus scaling below is for Taranis-Plus and X4R-SB
    float scale = 0.615;
    float bias = 895.0;
		for (int i = 0; i < numChannels; i++) {
			uint8_t channel = radioChannels[i]->GetChannel();
			radioChannels[i]->Update(sbusChannels[channel - 1]*scale + bias);
		}
  }
}

/**
 * @brief Checks for anomolies in the raw transmitter command signals and
 * activates failsafe mode for all channels if present.
*/
void failSafe() {
  failureFlag = 0;

  // Triggers for failure criteria
	for (int i = 0; i < numChannels; i++) {
		radioChannels[i]->FailureCheck(&failureFlag);
	}

  // If any failures, set to default failsafe values
  if (failureFlag) {
		for (int i = 0; i < numChannels; i++) {
			radioChannels[i]->TriggerFailsafe();
		}
  }
}


/**
 * @brief Commands the motors not to rotate if the transmitter switch for
 * throttle cut is high
*/
int throttleCut() {
  if (throCutChannel.SwitchPosition() == SwPos::SWITCH_HIGH) {
		Eigen::Vector4f zerosCommand = Eigen::Vector4f::Zero();
		/*motors.ScaleCommand(zerosCommand);*/
    quad_sim->SetMotorRates(zerosCommand.cast<double>());
    return 1;
  }
  return 0;
}

/**
 * @brief Regulates main loop rate to keep it at 2 kHz
 * @param freq The frequency to try to maintain [Hz]
*/
void loopRate(int freq) {
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(ledPin, blinkAlternate); // Pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink(int numBlinks, int upTime, int downTime) {
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(ledPin, LOW);
    delay(downTime);
    digitalWrite(ledPin, HIGH);
    delay(upTime);
  }
}

//=========================================================================================//

void radioSetup() {
    sbus.begin();
}

/**
 * @brief Provides throttle pass through to the motors for the purpose of
 * calibrating the ESC.
*/
void calibrateESCs() {
  while (true) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    digitalWrite(ledPin, HIGH); // LED on to indicate we are not in main loop
    getCommands();
		quadIMU.Update();
    getDesState();
		quadData.flightStatus.controlInputs << quadData.flightStatus.thrustSetpoint, 0, 0, 0;
		// Convert thrust and moments from controller to angular rates
		quadData.flightStatus.motorRates = ControlAllocator(quadData.flightStatus.controlInputs, quadProps::ALLOCATION_MATRIX_INV);
		Serial.print(quadData.flightStatus.motorRates[0]);
		Serial.print(",");
		Serial.print(quadData.flightStatus.motorRates[0]);
		Serial.print(",");
		Serial.print(quadData.flightStatus.motorRates[0]);
		Serial.print(",");
		Serial.println(quadData.flightStatus.motorRates[0]);
		// Convert angular rates to PWM commands
		/*motors.ScaleCommand(quadData.flightStatus.motorRates);*/
		/*motors.CommandMotor();*/
    quad_sim->SetMotorRates(quadData.flightStatus.motorRates.cast<double>());
    loopRate(2000);
  }
}

/**
 * @brief Calculates the constant null shift in the accelerometers and gyros.
 * Prints the values over serial so they can be given in the setup() function.
 * @param imu Pointer to the imu object to get the error from
 * @param att Pointer to the structure that contains attitude info for the IMU
*/
Eigen::Vector<float, 6> calculate_IMU_error(Generic_IMU *imu) {

	// Initialize these to hold the error terms
	float errorAcc[3] = {0, 0, 0};
	float errorGyro[3] = {0, 0, 0};

	// First set the null shift to zero
	Eigen::Vector3f nullShiftArray = Eigen::Vector3f::Zero();
	imu->SetAccNullShift(nullShiftArray);
	imu->SetGyroNullShift(nullShiftArray);

  Serial.println("Calibrating IMU. Please wait ~12 seconds.");
  // Read IMU values 12000 times (why 12000? idk)
  int c = 0;
  while (c < 12000) {
		imu->Update();

		errorAcc[0] += imu->GetAccX();
		errorAcc[1] += imu->GetAccY();
		errorAcc[2] += imu->GetAccZ() + 9.80665f; // Need to subtract gravity (but it's negative because that's the convention we use)

		errorGyro[0] += imu->GetGyroX();
		errorGyro[1] += imu->GetGyroY();
		errorGyro[2] += imu->GetGyroZ();

		// Add a delay here so we don't exceed the bandwidth of the IMU
		delayMicroseconds(1000);
    c++;
  }
  // Divide the sum by 12000 to get the error value
	for (int i = 0; i < 3; i++) {
		errorAcc[i] = errorAcc[i]/c;
		errorGyro[i] = errorGyro[i]/c;
	}

	Serial.println("Accelerometers: ");
	Serial.print("{");
  Serial.print(errorAcc[0]);
	Serial.print(",");
  Serial.print(errorAcc[1]);
	Serial.print(",");
  Serial.print(errorAcc[2]);
	Serial.println("}");

	Serial.println("Gyros: ");
	Serial.print("{");
  Serial.print(errorGyro[0]);
	Serial.print(",");
  Serial.print(errorGyro[1]);
	Serial.print(",");
  Serial.print(errorGyro[2]);
	Serial.println("}");

	Serial.println("Values have been saved to Parameters. Comment out 'calculate_IMU_error()' in Setup() to prevent this from running at startup");
	return Eigen::Vector<float, 6>(errorAcc[0], errorAcc[1], errorAcc[2], errorGyro[0], errorGyro[1], errorGyro[2]);
}

//===========================//
//========== SETUP ==========//
//===========================//
void Setup() {
  Serial.begin(500000); // USB serial (baud rate doesn't actually matter for Teensy)
  delay(500); // Give Serial some time to initialize
	
	// Serial port logging
	/*Log.begin(DroneConfig::LogLevel, &Serial);*/

  // Initialize all pins
  pinMode(ledPin, OUTPUT); // Pin 13 LED blinker on board, do not modify

  // Set built in LED to turn on to signal startup
  digitalWrite(ledPin, HIGH);

  delay(5);

  // Initialize radio communication
  radioSetup();

  // Begin mavlink telemetry module
	telem::Begin(quadData);

	// Initialize IMUs
	bool IMU_initSuccessful = quadIMU.Init(imu_sim);	
  quadIMU.Update(); // Get an initial reading. If not, initial attitude estimate will be NaN
	/*if (IMU_initSuccessful == false) {*/
	/*	Log.warningln("IMU failed to initialize");*/
	/*}*/

	// Initialize EKF
#ifdef USE_EKF
  ins.Configure();
	ins.Initialize(quadIMU.GetGyro(), quadIMU.GetAcc(), quadData.navData.mocapPosition_NED.cast<double>());
#endif

	// Putting this here for now. Initialize the yaw angle setpoint to 180
	quadData.attitudeData.eulerAngleSetpoint[2] = M_PI;

  // Initialize the SD card
	/*LoggingSetup();*/

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is
  // level when powered up Calibration parameters printed to serial monitor.
  // Paste these in the user specified variables section, then comment this out
  // forever.
	// BEGIN
	/*Eigen::Vector<float, 6> temp;*/
	/*temp = calculate_IMU_error(&quadIMU);*/
	/*for (int32_t i = 0; i < 6; i++) {*/
	/*	telem::UpdateParam(quadData, i+24, temp(i));*/
	/*}*/
	/*temp = calculate_IMU_error(&quadIMU2);*/
	/*for (int32_t i = 0; i < 6; i++) {*/
	/*	telem::UpdateParam(quadData, i+30, temp(i));*/
	/*}*/
	// END 
	// Get IMU null shift values from params
	for (int32_t i = 0; i < 3; i++) {
		accNS(i)   = quadData.telemData.paramValues[i+24];
		gyroNS(i)  = quadData.telemData.paramValues[i+27];
		accNS2(i)  = quadData.telemData.paramValues[i+30];
		gyroNS2(i) = quadData.telemData.paramValues[i+33];
	}
	quadIMU.SetAccNullShift(accNS);
	quadIMU.SetGyroNullShift(gyroNS);
	/*quadIMU2.SetAccNullShift(accNS2);*/
	/*quadIMU2.SetGyroNullShift(gyroNS2);*/

  delay(5);

  // PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick
  // to max, powering on, and lowering throttle to zero after the beeps
  /*calibrateESCs();*/
  // Code will not proceed past here if this function is uncommented!

  /*motors.ArmMotors(); // Loop over commandMotors() until ESCs happily arm*/

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)

  doneWithSetup = 1;
	/*Log.verboseln("Setup done");*/
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


  telem::Run(quadData, quadIMU);
  // Check if parameters have updated
  if(quadData.telemData.paramsUpdated == true) {
    quadData.telemData.paramsUpdated = false;
    // Attitude PID gains
    Kp_array[0] = quadData.telemData.paramValues[0]; 
    Ki_array[0] = quadData.telemData.paramValues[1];
    Kd_array[0] = quadData.telemData.paramValues[2];
    Kp_array[1] = quadData.telemData.paramValues[3]; 
    Ki_array[1] = quadData.telemData.paramValues[4];
    Kd_array[1] = quadData.telemData.paramValues[5];
    Kp_array[2] = quadData.telemData.paramValues[6]; 
    Ki_array[2] = quadData.telemData.paramValues[7];
    Kd_array[2] = quadData.telemData.paramValues[8];
    // Position PID gains
    Kp_pos[0] = quadData.telemData.paramValues[9];
    Ki_pos[0] = quadData.telemData.paramValues[10];
    Kd_pos[0] = quadData.telemData.paramValues[11];
    Kp_pos[1] = quadData.telemData.paramValues[9];
    Ki_pos[1] = quadData.telemData.paramValues[10];
    Kd_pos[1] = quadData.telemData.paramValues[11];
    Kp_pos[2] = quadData.telemData.paramValues[12];
    Ki_pos[2] = quadData.telemData.paramValues[13];
    Kd_pos[2] = quadData.telemData.paramValues[14];

    Kp2_array[0] = quadData.telemData.paramValues[21];
    Ki2_array[0] = quadData.telemData.paramValues[23];
    Kd2_array[0] = quadData.telemData.paramValues[22];
    Kp2_array[1] = quadData.telemData.paramValues[21];
    Ki2_array[1] = quadData.telemData.paramValues[23];
    Kd2_array[1] = quadData.telemData.paramValues[22];
    Kp2_array[2] = quadData.telemData.paramValues[21];
    Ki2_array[2] = quadData.telemData.paramValues[23];
    Kd2_array[2] = quadData.telemData.paramValues[22];


    angleController.SetKp(Kp_array);
    angleController.SetKi(Ki_array);
    angleController.SetKd(Kd_array);
    posControl.SetKp(Kp_pos);
    posControl.SetKi(Ki_pos);
    posControl.SetKd(Kd_pos);
    dcmAttControl.SetKp(Kp2_array);
    dcmAttControl.SetKi(Ki2_array);
    dcmAttControl.SetKd(Kd2_array);
  }

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
    /*motors.ScaleCommand(quadData.flightStatus.motorRates);*/
    /**/
    /*float motorCommands_norm[4];*/
    /*motors.GetMotorCommands(motorCommands_norm);*/
    /*for (int i = 0; i < 4; i++) {*/
    /*  quadData.flightStatus.motorRates_norm(i) = motorCommands_norm[i];*/
    /*}*/
    /**/
    /*motors.CommandMotor();*/
    quad_sim->SetMotorRates(quadData.flightStatus.motorRates.cast<double>());


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
    Setup();
    while(sim_running) {
        Loop();
    }

    // For now, just run a simple loop
    /*while (sim_running) {*/
    /*  std::this_thread::sleep_for(std::chrono::microseconds(500));  // 2kHz*/
    /*}*/
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
