//================================================================================================//

#include "UserDefines.h"
#include <cstdio>
#include <stdint.h>

#include "eigen.h"  	// Linear algebra
#include "SBUS.h"     //sBus interface

#include "common.h"
#include "heartbeat.h"
#include "serialDebug.h"
#include "telemetry.h"
#include "IMU.h"
#include "madgwick.h"
#include "controller.h"
#include "motors.h"
#include "radio.h"
#include "testing.h"
#include "EKF.h"
#include "navHandler.h"
#include "datalogger.h"

//================================================================================================//
//                                     USER-SPECIFIED VARIABLES                                   //
//================================================================================================//

// Radio channel definitions
// Syntax: ("name", channel, slider neutral point (meaningless for switches), failsafe value, true
// (if channel is critical))
RadioChannel throttleChannel("throttle", 1, 1000, 1000, true, 1000, 1965);
RadioChannel rollChannel("roll", 2, 1500, 1500, true);
RadioChannel pitchChannel("pitch", 3, 1500, 1500, true);
RadioChannel yawChannel("yaw", 4, 1500, 1500, true);
RadioChannel throCutChannel("throttle_cut", 5, 1000, 2000);
RadioChannel boundaryOnOff("boundary_sw", 6, 1000, 2000);

#ifdef TEST_STAND
RadioChannel aux0("none", 6, 1500, 1500);
RadioChannel aux1("sine_sweep", 7, 1000, 1000);
RadioChannel aux2("step_axis_sel", 8, 1000, 1000);
RadioChannel aux3("step_angle_sel", 9, 1000, 1500);
#elif defined(USE_POSITION_CONTROLLER)
RadioChannel aux0("position_toggle", 6, 1500, 1500);
RadioChannel aux1("xSel", 7, 1000, 1500);
RadioChannel aux2("ySel", 8, 1000, 1500);
RadioChannel aux3("zSel", 9, 1000, 1500);
#else
RadioChannel aux0("none", 6, 1500, 1500);
RadioChannel aux1("none", 7, 1000, 1500);
RadioChannel aux2("none", 8, 1000, 1500);
RadioChannel aux3("none", 9, 1000, 1500);
#endif

RadioChannel KpScaleChannel("Kp_scale", 10, 1500, 1500);
RadioChannel KiScaleChannel("Ki_scale", 11, 1500, 1500);
RadioChannel KdScaleChannel("Kd_scale", 12, 1500, 1500);
RadioChannel scaleAllChannel("scale_all", 13, 1500, 1500);
RadioChannel resetChannel("reset", 14, 1000, 1000);


// Array of pointers the the radio channels. This is useful for datalogging and updating the raw
// values.
const uint8_t numChannels = 15;
RadioChannel *radioChannels[numChannels] = {&throttleChannel, 
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
float maxYaw = 160.0*DEG_TO_RAD;

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
const uint8_t ledPin = 10;
// 4 2 5 3 pin assignments for new drones, 0 1 2 3 for old drone
// Can write high or low to check conditions when debugging.
//* const uint8_t debugPin = 5; *//

//================================================================================================//

// Logic needed to restart teensy
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

// SD card setup
// Interval between points (usec) for 100 samples/sec
#define LOG_INTERVAL_USEC 10000

// DECLARE GLOBAL VARIABLES
// General stuff
float dt;
Quadcopter_t quadData;

unsigned long current_time, prev_time;
unsigned long print_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
unsigned long print_counterSD = 200000;

SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

Eigen::Vector3f accNS = {mpuNS_ax, mpuNS_ay, mpuNS_az};
Eigen::Vector3f gyroNS = {mpuNS_gx, mpuNS_gy, mpuNS_gz};
mpu6050 quadIMU = mpu6050(accNS, gyroNS);

SetpointHandler spHandler(&quadData);

// Controller:
float Kp_array[3] = {0.0f, 0.0f, 0.0f};
float Ki_array[3] = {0.0f, 0.0f, 0.0f};
float Kd_array[3] = {0.0f, 0.0f, 0.0f};
float Kp2_array[3] = {0.0f, 0.0f, 0.0f};
float Ki2_array[3] = {0.0f, 0.0f, 0.0f};
float Kd2_array[3] = {0.0f, 0.0f, 0.0f};
AngleAttitudeController angleController = AngleAttitudeController(Kp_array, Ki_array, Kd_array, 50.0f);
PositionController posControl = PositionController(Kp_pos, Ki_pos, Kd_pos, 1.0f);
PositionController2 posControl2 = PositionController2(Kp_pos, Ki_pos, Kd_pos, 1.0f, 3.6f);
DCMAttitudeControl dcmAttControl = DCMAttitudeControl(Kp2_array, Ki2_array, Kd2_array, 1.0f, 0.8f);
Eigen::Vector3f b1d = {-1.0f, 0.0f, 0.0f};
uint32_t customMode = bfs::CustomMode::MANUAL;

// Motor object
#ifdef USE_ONESHOT
Motors motors(motorPins, 125, 250);
#else
Motors motors(motorPins, 1000, 2000);
#endif

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
Datalogger logging;

bool wasTrueLastLoop = false; // This will be renamed at some point

// Various timers
elapsedMillis heartbeatTimer;
elapsedMillis fastMavlinkTimer;
elapsedMillis EKFUpdateTimer;
elapsedMicros sdCardUpdateTimer;
elapsedMillis attitudeCtrlTimer;
elapsedMillis positionCtrlTimer;
elapsedMicros IMUUpdateTimer;

// These define some loop rates
const unsigned long EKFPeriod = 2; // milliseconds  (500 Hz)
const unsigned long attitudeCtrlPeriod = 5; // milliseconds (200 Hz)
const unsigned long positionCtrlPeriod = 5; // milliseconds (200 Hz)
const unsigned long imuUpdatePeriod = 0; // microseconds (1000 Hz)

#ifdef TEST_STAND
bool restartSineSweep = true;
#endif

// Defining the flight area (Shepherd Drone Lab)
const float FLIGHT_AREA_X_MAX = 1.5;
const float FLIGHT_AREA_X_MIN = -6;
const float FLIGHT_AREA_Y_MAX = 4;
const float FLIGHT_AREA_Y_MIN = -1;
const float FLIGHT_AREA_Z_MAX = 0;
const float FLIGHT_AREA_Z_MIN = -2.5;
int bndryOnOff;

//========================================================================================================================//
//                                                      FUNCTIONS //
//========================================================================================================================//

/**
 * @brief Updates the desired states (setpoints) based on the raw transmitter
 * inputs obtained in getCommands(). Some of these values can be overwritten
 * later by other functions.
*/
void getDesState() {
	float thrust_des; float roll_des; float pitch_des; float yaw_des;
  thrust_des = throttleChannel.NormalizedValue(); // Between 0 and 1
  roll_des = rollChannel.NormalizedValue();  // Between -1 and 1
  pitch_des = -pitchChannel.NormalizedValue(); // Between -1 and 1
  yaw_des = -yawChannel.NormalizedValue();   // Between -1 and 1

  // Constrain within normalized bounds
  quadData.flightStatus.thrustSetpoint = constrain(thrust_des, 0.0, 1.0)*quadProps::MAX_THRUST;
  quadData.att.eulerAngleSetpoint[0] = constrain(roll_des, -1.0, 1.0) * maxRoll;
  quadData.att.eulerAngleSetpoint[1] = constrain(pitch_des, -1.0, 1.0) * maxPitch;
  quadData.att.eulerAngleSetpoint[2] = constrain(yaw_des, -1.0, 1.0) * maxYaw;
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
		motors.ScaleCommand(zerosCommand);
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
    failSafe();  
    getDesState();
	quadIMU.Update();
    Madgwick6DOF(quadIMU, quadData, dt);
    getDesState();
		quadData.flightStatus.controlInputs << quadData.flightStatus.thrustSetpoint, 0, 0, 0;
		// Convert thrust and moments from controller to angular rates
		quadData.flightStatus.motorRates = ControlAllocator(quadData.flightStatus.controlInputs);
		Serial.print(quadData.flightStatus.motorRates[0]);
		Serial.print(",");
		Serial.print(quadData.flightStatus.motorRates[0]);
		Serial.print(",");
		Serial.print(quadData.flightStatus.motorRates[0]);
		Serial.print(",");
		Serial.println(quadData.flightStatus.motorRates[0]);
		// Convert angular rates to PWM commands
		motors.ScaleCommand(quadData.flightStatus.motorRates);
		motors.CommandMotor();
    loopRate(2000);
  }
}

/**
 * @brief Calculates the constant null shift in the accelerometers and gyros.
 * Prints the values over serial so they can be given in the setup() function.
 * @param imu Pointer to the imu object to get the error from
 * @param att Pointer to the structure that contains attitude info for the IMU
*/
void calculate_IMU_error(Generic_IMU *imu) {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
	
	float errorAcc[3] = {0, 0, 0};
	float errorGyro[3] = {0, 0, 0};

	// First set the null shift to zero
	float nullShiftArray[3] = {0, 0, 0};
	imu->SetAccNullShift(nullShiftArray);
	imu->SetGyroNullShift(nullShiftArray);

  // Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
		imu->Update();

		errorAcc[0] += imu->GetAccX();
		errorAcc[1] += imu->GetAccY();
		errorAcc[2] += imu->GetAccZ() + 9.80665f; // Need to subtract gravity

		errorGyro[0] += imu->GetGyroX();
		errorGyro[1] += imu->GetGyroY();
		errorGyro[2] += imu->GetGyroZ();

    c++;
  }
  // Divide the sum by 12000 to get the error value
	for (int i = 0; i < 3; i++) {
		errorAcc[i] = errorAcc[i]/c;
		errorGyro[i] = errorGyro[i]/c;
	}


  Serial.print("float AccErrorX = ");
  Serial.print(errorAcc[0]);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(errorAcc[1]);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(errorAcc[2]);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(errorGyro[0]);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(errorGyro[1]);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(errorGyro[2]);
  Serial.println(";");

  Serial.println("Paste these values in Setup() function and "
                 "comment out calculate_IMU_error() in void setup.");
  for (;;);
}

// Adds items to the datalogger
// FIXME: PID GAINS NOT YET INCLUDED
void LoggingSetup() {
	// Attitude
	logging.AddItem(quadData.att.eulerAngles_madgwick, "euler_madgwick", 4);
	logging.AddItem(quadData.att.eulerAngles_ekf, "euler_ekf", 4);
	logging.AddItem(quadData.att.eulerAngleSetpoint, "euler_setpoint", 4);
	logging.AddItem(&quadData.flightStatus.thrustSetpoint, "thrust_setpoint", 4);
	// Raw radio PWM values (1000-2000)
	for (int i = 0; i < numChannels; i++) {
		logging.AddItem(&(radioChannels[i]->rawValue_), radioChannels[i]->GetName(), 10);
	}
	// LP-filtered IMU data
	logging.AddItem(quadIMU.GetAccXPtr(), "Acc1", 4);
	logging.AddItem(quadIMU.GetAccYPtr(), "Acc2", 4);
	logging.AddItem(quadIMU.GetAccZPtr(), "Acc3", 4);
	logging.AddItem(quadIMU.GetGyroXPtr(), "Gyro1", 4);
	logging.AddItem(quadIMU.GetGyroYPtr(), "Gyro2", 4);
	logging.AddItem(quadIMU.GetGyroZPtr(), "Gyro3", 4);
	// Control inputs and motor rates
	logging.AddItem(quadData.flightStatus.controlInputs, "u", 4);
	logging.AddItem(quadData.flightStatus.motorRates, "w", 4);
	// Timers
	logging.AddItem(&(quadData.flightStatus.timeSinceBoot), "timeSinceBoot", 10);
	logging.AddItem(&(quadData.navData.mocapUpdate_mocapTime), "mocapTime", 10);
	logging.AddItem(&(quadData.navData.mocapUpdate_quadTime), "quadTime", 10);
	// Nav
	logging.AddItem(quadData.navData.position_NED, "positionNED_", 6);
	logging.AddItem(quadData.navData.velocity_NED, "velocityNED_", 6);
	logging.AddItem(quadData.navData.positionSetpoint_NED, "positionSetpointNED_", 6);
	logging.AddItem(quadData.navData.velocitySetpoint_NED, "velocitySetpointNED_", 6);
	logging.AddItem(quadData.navData.mocapPosition_NED, "mocapPositionNED_", 6);
	logging.AddItem(&(quadData.navData.numMocapUpdates), "numMocapUpdates", 10);
	logging.AddItem(&pScale_att, "pScale_att", 4);
	logging.AddItem(&iScale_att, "iScale_att", 4);
	logging.AddItem(&dScale_att, "dScale_att", 4);
	// PID values
	logging.AddItem(Kp_array, "Kp_array", 3);
	logging.AddItem(Ki_array, "Ki_array", 3);
	logging.AddItem(Kd_array, "Kd_array", 3);
	logging.AddItem(Kp_pos, "Kp_pos", 3);
	logging.AddItem(Ki_pos, "Ki_pos", 3);
	logging.AddItem(Kd_pos, "Kd_pos", 3);
	logging.AddItem(Kp2_array, "Kp2_array", 3);
	logging.AddItem(Ki2_array, "Ki2_array", 3);
	logging.AddItem(Kd2_array, "Kd2_array", 3);
}

//===========================//
//========== SETUP ==========//
//===========================//
void setup() {
  Serial.begin(500000); // USB serial (baud rate doesn't actually matter for Teensy)
  delay(500); // Give Serial some time to initialize

  // Initialize all pins
  pinMode(ledPin, OUTPUT); // Pin 13 LED blinker on board, do not modify
  //*pinMode(5, OUTPUT);*//

  // Set built in LED to turn on to signal startup
  digitalWrite(ledPin, HIGH);

  delay(5);

  // Initialize radio communication
  radioSetup();

  // Begin mavlink telemetry module
	telem::Begin(quadData);


	bool IMU_initSuccessful = quadIMU.Init(&Wire);	
  quadIMU.Update(); // Get an initial reading. If not, initial attitude estimate will be NaN
	Serial.print("IMU initialization successful: ");
	Serial.println(IMU_initSuccessful);

#ifdef USE_EKF
  ins.Configure();
	ins.Initialize(quadIMU.GetGyro(), quadIMU.GetAcc(), quadData.navData.mocapPosition_NED.cast<double>());
#endif


  // Initialize the SD card, returns 1 if no sd card is detected or it can't be
  // initialized. So it's negated to make SD_is_present true when everything is OK
	LoggingSetup();

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is
  // level when powered up Calibration parameters printed to serial monitor.
  // Paste these in the user specified variables section, then comment this out
  // forever.
	// calculate_IMU_error(&quadIMU);


  delay(5);

  // PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick
  // to max, powering on, and lowering throttle to zero after the beeps
  //calibrateESCs();
  // Code will not proceed past here if this function is uncommented!

  //motors.ArmMotors(); // Loop over commandMotors() until ESCs happily arm

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)

  doneWithSetup = 1;
}

//==========================//
//========== Loop ==========//
//==========================//
void loop() {
  // digitalWriteFast(5, HIGH);
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
	quadData.flightStatus.timeSinceBoot = micros();

	while (Serial.available()) {
		char rc = Serial.read();
		if (rc == 'r') {
			CPU_RESTART;
		} else if (rc == 's') {
			if (!logRunning) {
				logRunning = true;
  			SD_is_present = !(logging.Setup());
			}
		} else if (rc == 't') {
			if (logRunning==true) {
				logRunning = false;
				logging.End();
				SD_is_present = false;
			}
		}
	}

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds
  //
  //
  if (isnan(quadData.navData.position_NED[0])) {
    ins.Initialize(quadIMU.GetGyro(), quadIMU.GetAcc(), quadData.navData.mocapPosition_NED.cast<double>());
  }

  //  Print data at 100hz (uncomment one at a time for troubleshooting)
	if (current_time - print_counter > 100000) {
		print_counter = current_time;
		//serialDebug::PrintRadioData(); // Currently does nothing
		// serialDebug::PrintDesiredState(thrust_des, roll_des, pitch_des, yaw_des);
		//serialDebug::PrintGyroData(quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ());
		/*serialDebug::PrintAccelData(quadIMU.GetAccX(), quadIMU.GetAccY(), quadIMU.GetAccZ());*/
		/*serialDebug::PrintRollPitchYaw(quadData.att.eulerAngles_active->coeff(0), quadData.att.eulerAngles_active->coeff(1), quadData.att.eulerAngles_active->coeff(2));*/
		//serialDebug::PrintPIDOutput(angleController.GetRollPID(), angleController.GetPitchPID(), angleController.GetYawPID());
		// float motorCommands[4] = {0, 0, 0, 0};
		// motors.GetMotorCommands(motorCommands);
		// serialDebug::PrintMotorCommands(motorCommands[0], motorCommands[1], motorCommands[2], motorCommands[3]);
		//serialDebug::PrintLoopTime(dt);
		//serialDebug::PrintZPosPID(posControl.GetTmpPropo()[2],
		//posControl.GetTmpInteg()[2],posControl.GetTmpDeriv()[2]);
		// serialDebug::DisplayRoll(roll_des, quadIMU_info.roll);
	}

	// Mode checking
	// Check the status of the armed switch
	switch(throCutChannel.SwitchPosition()) {
		case SwPos::SWITCH_LOW:
      if (quadData.flightStatus.inputOverride == false) {
        quadData.telemData.mavlink->throttle_enabled(true);
        throttleEnabled = true;
      }
			/*if (!logRunning) {*/
			/*	logRunning = true;*/
			/*		SD_is_present = !(logging.Setup());*/
			/*}*/
			break;
		case SwPos::SWITCH_HIGH:
      quadData.telemData.mavlink->throttle_enabled(false);
      throttleEnabled = false;
      quadData.flightStatus.inputOverride = false;
			/*if (logRunning==true) {*/
			/*	logRunning = false;*/
			/*	logging.End();*/
			/*	SD_is_present = false;*/
			/*}*/
			break;
		default:
			break;
	}

  // Check boundary On-Off switch
	switch(boundaryOnOff.SwitchPosition()) {
		case SwPos::SWITCH_HIGH:
	  if (1) {bndryOnOff = 1;}
	  		break;
		case SwPos::SWITCH_MID:
	  if (1) {bndryOnOff = 1;}
	  		break;
		case SwPos::SWITCH_LOW:
	  if (1) {bndryOnOff = 0;}
	  		break;
		default:
			break;
	}

  // Check autopilot mode


	// Check autopilot switch
	// switch(aux0.SwitchPosition()) {
	// 	case SwPos::SWITCH_HIGH:
	// 		// 
	// 		quadData.UpdateMode(bfs::CustomMode::POSITION);
	// 		break;
	// 	case SwPos::SWITCH_MID:
	// 		// Altitude assistance
	// 		quadData.UpdateMode(bfs::CustomMode::ALTITUDE);
	// 		break;
	// 	case SwPos::SWITCH_LOW:
	// 		// Manual flight
	// 		quadData.UpdateMode(bfs::CustomMode::MANUAL);
	// 		break;
	// 	default:
	// 		break;

	// }

	// Handle restart request
  if (throttleEnabled == false) {
		if (resetChannel.SwitchPosition() == SwPos::SWITCH_HIGH) {
			CPU_RESTART;
		}
	}

  if (SD_is_present && (current_time - print_counterSD) > LOG_INTERVAL_USEC) {
  	// Write to SD card buffer
		print_counterSD = micros();
		logging.Write();
  }

	// TODO: Be better


if (IMUUpdateTimer >= imuUpdatePeriod) {
	IMUUpdateTimer = 0;
  quadIMU.Update(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
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



  // ins.Set_AccelSigma(quadData.telemData.paramValues[15]*Eigen::Vector3f::Ones());
  // ins.Set_AccelMarkov(quadData.telemData.paramValues[16]*Eigen::Vector3f::Ones());
  // ins.Set_AccelTau(quadData.telemData.paramValues[17]*Eigen::Vector3f::Ones());
  // ins.Set_RotRateSigma(quadData.telemData.paramValues[18]*Eigen::Vector3f::Ones());
  // ins.Set_RotRateMarkov(quadData.telemData.paramValues[19]*Eigen::Vector3f::Ones());
  // ins.Set_RotRateTau(quadData.telemData.paramValues[20]*Eigen::Vector3f::Ones());
}

#ifdef USE_EKF
	if (EKFUpdateTimer > EKFPeriod) {
		EKFUpdateTimer = 0;
		quadData.navData.numMocapUpdates = telem::CheckForNewPosition(quadData);
  	ins.Update(micros(), quadData.navData.numMocapUpdates, quadIMU.GetGyro(), quadIMU.GetAcc(), quadData.navData.mocapPosition_NED.cast<double>());
		quadData.navData.position_NED = ins.Get_PosEst().cast<float>();
		quadData.navData.velocity_NED = ins.Get_VelEst();
  	quadData.att.eulerAngles_ekf = ins.Get_OrientEst();
    quadData.att.currentDCM = Euler2DCM(quadData.att.eulerAngles_ekf);
	}
	Madgwick6DOF(quadIMU, quadData, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates
#else
	if (EKFUpdateTimer > EKFPeriod) {
		Madgwick6DOF(quadIMU, quadData, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates
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
	// Check if position Controller enabled
	Eigen::Vector3f currentPosCovariance = ins.Get_CovPos();
	if (currentPosCovariance[0] < positionCovarianceLimit &&
			currentPosCovariance[1] < positionCovarianceLimit &&
			currentPosCovariance[2] < positionCovarianceLimit) {
		positionFix = true;
		quadData.att.eulerAngles_active = &(quadData.att.eulerAngles_ekf);
	} else {
		positionFix = false;
    // positionFix = true;
		quadData.att.eulerAngles_active = &(quadData.att.eulerAngles_madgwick);
	}
	if (positionCtrlTimer >= positionCtrlPeriod) {
		getDesState(); // Convert raw commands to normalized values based on saturated control limits
		positionCtrlTimer = 0;
		if (positionFix == true) {
      customMode = quadData.telemData.mavlink->custom_mode();
      if (quadData.telemData.mavlink->throttle_enabled()) {
        if (customMode == bfs::CustomMode::MANUAL) {
					posControl.Reset();
        } else {
          spHandler.UpdateSetpoint();
          posControl.Update(quadData.navData.positionSetpoint_NED.cast<double>(), 
                            quadData.navData.velocitySetpoint_NED,
                            ins.Get_PosEst(), ins.Get_VelEst(), quadData.att, dt, false);
          // posControl2.Update(quadData.navData.positionSetpoint_NED, quadData.navData.velocitySetpoint_NED, 
          //                    ins.Get_PosEst().cast<float>(), ins.Get_VelEst(), b1d, quadData.att, dt);
          if (customMode == bfs::CustomMode::MISSION ||
              customMode == bfs::CustomMode::POSITION ||
              customMode == bfs::CustomMode::TAKEOFF ||
              customMode == bfs::CustomMode::LANDING) {
            quadData.flightStatus.thrustSetpoint = posControl.GetDesiredThrust();
            quadData.att.eulerAngleSetpoint[0] = posControl.GetDesiredRoll();
            quadData.att.eulerAngleSetpoint[1] = posControl.GetDesiredPitch();
            // quadData.flightStatus.thrustSetpoint = posControl2.GetDesiredThrust();
            // quadData.att.desiredDCM = posControl2.GetDesiredDCM();
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
    quadData.att.quatSetpoint = Euler2Quat(quadData.att.eulerAngleSetpoint);
	}
	# else
		// Compute desired state based on radio inputs
		getDesState(); // Convert raw commands to normalized values based on saturated control limits
		quadData.flightStatus.controlInputs[0] = quadData.flightStatus.thrustSetpoint;
    quadData.att.quatSetpoint = Euler2Quat(quadData.att.eulerAngleSetpoint);
#endif
	
#ifdef TEST_STAND
	// Sine sweep
  if (aux1.SwitchPosition() == SwPos::SWITCH_HIGH) {
		roll_des = testStand::SineSweep(restartSineSweep);
		restartSineSweep = false;
  } else {
		restartSineSweep = true;
	}

	// if (aux1.SwitchPosition() == SwPos::SWITCH_HIGH) {
	// 	thrust_des = testStand::ThrustSweep(restartSineSweep);
	// 	restartSineSweep = false;
	// } else {
	// 	restartSineSweep = true;
	// }

	// Step inputs
  if (aux2.SwitchPosition() == SwPos::SWITCH_MID) {
		roll_des = testStand::Step(aux3);
  } else if (aux2.SwitchPosition() == SwPos::SWITCH_HIGH) {
		pitch_des = testStand::Step(aux3);
	}
#endif


	// PID Gain scaling
#ifdef PID_TUNING
	pScale_att = gainTuning::ScaleFactor(KpScaleChannel);
	iScale_att = gainTuning::ScaleFactor(KiScaleChannel);
	dScale_att = gainTuning::ScaleFactor(KdScaleChannel);
	allScale_att = gainTuning::ScaleFactor(scaleAllChannel);

	pScale_att *= allScale_att;
	iScale_att *= allScale_att;
	dScale_att *= allScale_att;

	float KpScaled[3] = {Kp_roll_angle*pScale_att, Kp_pitch_angle*pScale_att, Kp_yaw};
	float KiScaled[3] = {Ki_roll_angle*iScale_att, Ki_pitch_angle*iScale_att, Ki_yaw};
	float KdScaled[3] = {Kd_roll_angle*dScale_att, Kd_pitch_angle*dScale_att, Kd_yaw};
	angleController.SetKp(KpScaled);
	angleController.SetKi(KiScaled);
	angleController.SetKd(KdScaled);
#endif

	bool noIntegral = false;
	if (quadData.flightStatus.thrustSetpoint < 0.5f || throttleEnabled == false) {
		noIntegral = true;
	}
	
	if (attitudeCtrlTimer >= attitudeCtrlPeriod) {
		attitudeCtrlTimer = 0;
    Eigen::Vector3f gyroRates = {quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ()};
    if (customMode == bfs::CustomMode::MANUAL ||
        customMode == bfs::CustomMode::ALTITUDE) {
      // FIXME: Make the function take Eigen::Vector or give up on it
      angleController.Update(quadData.att.eulerAngleSetpoint.data(), quadData.att, gyroRates, dt, noIntegral);
      quadData.flightStatus.controlInputs(lastN(3)) = angleController.GetMoments();
    } else {
      angleController.Update(quadData.att.eulerAngleSetpoint.data(), quadData.att, gyroRates, dt, noIntegral);
      quadData.flightStatus.controlInputs(lastN(3)) = angleController.GetMoments();
      // dcmAttControl.Update(quadData.att, gyroRates, dt);
      // quadData.flightStatus.controlInputs(lastN(3)) = dcmAttControl.GetControlTorque();
    }
	}

	// Convert thrust and moments from controller to angular rates
	if (quadData.telemData.mavlink->throttle_enabled()) {
		quadData.flightStatus.motorRates = ControlAllocator(quadData.flightStatus.controlInputs);
	} else {
		quadData.flightStatus.motorRates = Eigen::Vector4f::Zero();
	}
	// Convert angular rates to PWM commands
	motors.ScaleCommand(quadData.flightStatus.motorRates);

	motors.CommandMotor();
  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands
  failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup


  // Regulate loop rate
  loopRate(2000); 
  // digitalWriteFast(5, LOW);
}

//========== MAIN FUNCTION ==========//

int main() {
	setup();
	while(1) {
		loop();
	}
	return 0;
}
