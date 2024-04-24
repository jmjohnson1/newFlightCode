//================================================================================================//

#include "UserDefines.h"

#include <stdint.h>

#include "TeensyTimerTool.h"  // Oneshot timer
#include "PWMServo.h" // Commanding any extra actuators, installed with teensyduino installer
#include "SPI.h"      // SPI communication
#include "Wire.h"     // I2c communication
#include "eigen.h"  	// Linear algebra
#include "SBUS.h"     //sBus interface

#include "common.h"
#include "serialDebug.h"
#include "telemetry.h"
#include "IMU.h"
#include "madgwick.h"
#include "controller.h"
#include "motors.h"
#include "radio.h"
#include "testing.h"
#include "uNavINS.h"
#include "missionHandler.h"
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
const uint8_t numChannels = 14;
RadioChannel *radioChannels[numChannels] = {&throttleChannel, 
											&rollChannel, 
											&pitchChannel, 
											&yawChannel, 
											&throCutChannel, 
											&aux0,
											&aux1,
											&aux2,
											&aux3,
											&KpScaleChannel,
											&KiScaleChannel,
											&KdScaleChannel,
											&scaleAllChannel,
											&resetChannel
											};

// Max roll/pitch angles in degrees for angle mode
float maxRoll = quadProps::MAX_ANGLE;
float maxPitch = quadProps::MAX_ANGLE;
// Max yaw rate in deg/sec
float maxYaw = 160.0;

// ANGLE MODE PID GAINS //
// SCALE FACTORS FOR PID //
float pScale_att = 1.0f;
float iScale_att = 1.0f;
float dScale_att = 1.0f;
float allScale_att = 1.0f;
// DIVIDE BY 100 IF USING AGAIN
// float Kp_roll_angle = 0.7f;
// float Ki_roll_angle = 0.47f;
// float Kd_roll_angle = 0.12f;
// float Kp_pitch_angle = 0.7f;
// float Ki_pitch_angle = 0.47f;
// float Kd_pitch_angle = 0.12f;

// // YAW PID GAINS //
// float Kp_yaw = 0.3;
// float Ki_yaw = 0.06;
// float Kd_yaw = 0.00015;

float Kp_roll_angle = 0.007f*1.5;
float Ki_roll_angle = 0.001645f*1.5;
float Kd_roll_angle = 0.001305f*1.5;
float Kp_pitch_angle = 0.007f*1.5;
float Ki_pitch_angle = 0.001645f*1.5;
float Kd_pitch_angle = 0.001305f*1.5;

// YAW PID GAINS //
float Kp_yaw = 0.002f;
float Ki_yaw = 0.0f;
float Kd_yaw = 0.0f;

// POSITION PID GAINS //
float Kp_pos[3] = {5.0f, 5.0f, 29.0f};
float Ki_pos[3] = {3.0f, 3.0f, 8.0f};
float Kd_pos[3] = {8.0f, 8.0f, 16.0f};

//================================================================================================//
//                                      DECLARE PINS                                              //
//================================================================================================//

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the
// 			 MPU6050 IMU for default setup

// Motor pin outputs:
const uint8_t motorPins[4] = {0, 1, 2, 3};


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

IMU quadIMU = IMU(0.00f, 0.00f, 0.09f, 0.04f, 2.78f, 0.35f);

MissionHandler mission;

// Controller:
float Kp_array[3] = {Kp_roll_angle, Kp_pitch_angle, Kp_yaw};
float Ki_array[3] = {Ki_roll_angle, Ki_pitch_angle, Ki_yaw};
float Kd_array[3] = {Kd_roll_angle, Kd_pitch_angle, Kd_yaw};
AngleAttitudeController controller = AngleAttitudeController(Kp_array, Ki_array, Kd_array);
PositionController posControl = PositionController(Kp_pos, Ki_pos, Kd_pos);

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
bool flightLoopStarted = 0;
int loopCount = 0;

// Position vector taken from mocap
bool positionFix = false;  // Set true if the position covariance from the EKF is less than a certain value.
const float positionCovarianceLimit = 1.0f;	// Maximum allowable position covariance from EKF. [m^2]

// EKF
uNavINS ins;

// Datalogging
Datalogger logging;

bool wasTrueLastLoop = false; // This will be renamed at some point

// Constants for unit conversion
const float DEG_2_RAD = PI/180.0f;
const float RAD_2_DEG = 1.0f/DEG_2_RAD;
const float G = 9.81f;  // m/s/s

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
    digitalWrite(13, blinkAlternate); // Pin 13 is built in LED

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
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
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
    digitalWrite(13, HIGH); // LED on to indicate we are not in main loop
    getCommands();
    failSafe();  
    getDesState();
		quadIMU.Update();
    Madgwick6DOF(quadIMU, quadData, dt);
    getDesState();
		quadData.flightStatus.controlInputs << quadData.flightStatus.thrustSetpoint, 0, 0, 0;
		// Convert thrust and moments from controller to angular rates
		quadData.flightStatus.motorRates = ControlAllocator(quadData.flightStatus.controlInputs);
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
void calculate_IMU_error(IMU *imu) {
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
		errorAcc[2] += imu->GetAccZ() + 1.0f; // Need to subtract gravity

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
	
	logging.AddItem(&(quadData.telemData.lastSequenceRX), "lastSequenceRX", 10);
	logging.AddItem(&(quadData.telemData.lastSequenceTx), "lastSequenceTX", 10);
	logging.AddItem(&(quadData.telemData.numMissionItemsRX), "numMissionItemsRX", 10);
	logging.AddItem(&(quadData.telemData.numMissionRequestTX), "numMissionRequestTX", 10);

}

//===========================//
//========== SETUP ==========//
//===========================//
void setup() {
  Serial.begin(500000); // USB serial (baud rate doesn't actually matter for Teensy)
  delay(500); // Give Serial some time to initialize

  // Initialize all pins
  pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify

  // Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  // Initialize radio communication
  radioSetup();

  // Begin mavlink telemetry module
	telem::Begin(quadData);
	mission.Init(&quadData, &quadData.navData);


	bool IMU_initSuccessful = quadIMU.Init(&Wire);	
  quadIMU.Update(); // Get an initial reading. If not, initial attitude estimate will be NaN
	Serial.print("IMU initialization successful: ");
	Serial.println(IMU_initSuccessful);

#ifdef USE_EKF
  ins.Configure();
	ins.Initialize(quadIMU.GetGyro()*DEG_2_RAD, quadIMU.GetAcc()*G, quadData.navData.mocapPosition_NED.cast<double>());
#endif

  // Initialize the SD card, returns 1 if no sd card is detected or it can't be
  // initialized. So it's negated to make SD_is_present true when everything is OK
	LoggingSetup();
  SD_is_present = !(logging.Setup());

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is
  // level when powered up Calibration parameters printed to serial monitor.
  // Paste these in the user specified variables section, then comment this out
  // forever.
	//calculate_IMU_error(&quadIMU);


  delay(5);

  // PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick
  // to max, powering on, and lowering throttle to zero after the beeps
  // calibrateESCs();
  // Code will not proceed past here if this function is uncommented!

  motors.ArmMotors(); // Loop over commandMotors() until ESCs happily arm

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)

  doneWithSetup = 1;
}

//==========================//
//========== Loop ==========//
//==========================//
void loop() {
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
	quadData.flightStatus.timeSinceBoot = micros();

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  //  Print data at 100hz (uncomment one at a time for troubleshooting)
	if (current_time - print_counter > 100000) {
		print_counter = current_time;
		//serialDebug::PrintRadioData(); // Currently does nothing
		// serialDebug::PrintDesiredState(thrust_des, roll_des, pitch_des, yaw_des);
		//serialDebug::PrintGyroData(quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ());
		//serialDebug::PrintAccelData(quadIMU.GetAccX(), quadIMU.GetAccY(), quadIMU.GetAccZ());
		//serialDebug::PrintRollPitchYaw(quadIMU_info.roll, quadIMU_info.pitch, quadIMU_info.yaw);
		//serialDebug::PrintPIDOutput(controller.GetRollPID(), controller.GetPitchPID(), controller.GetYawPID());
		// float motorCommands[4] = {0, 0, 0, 0};
		// motors.GetMotorCommands(motorCommands);
		// serialDebug::PrintMotorCommands(motorCommands[0], motorCommands[1], motorCommands[2], motorCommands[3]);
		//serialDebug::PrintLoopTime(dt);
		//serialDebug::PrintZPosPID(posControl.GetTmpPropo()[2],
		//posControl.GetTmpInteg()[2],posControl.GetTmpDeriv()[2]);
		// serialDebug::DisplayRoll(roll_des, quadIMU_info.roll);
	}

  // Check if rotors should be armed
  if (!flightLoopStarted && (throCutChannel.SwitchPosition() == SwPos::SWITCH_LOW)) {
    flightLoopStarted = 1;
		// Let the GCS know that things are running
    // telem.SetSystemState(MAV_STATE_ACTIVE);
    // telem.SetSystemMode(MAV_MODE_MANUAL_ARMED);
		quadData.telemData.mavlink->aircraft_state(bfs::AircraftState::ACTIVE);
		quadData.telemData.mavlink->aircraft_mode(bfs::AircraftMode::MANUAL);
		quadData.flightStatus.mavState = bfs::AircraftState::ACTIVE;
		quadData.flightStatus.mavMode = bfs::AircraftMode::MANUAL;
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

quadData.navData.numMocapUpdates = telem::CheckForNewPosition(quadData);

#ifdef USE_EKF
	if (EKFUpdateTimer > EKFPeriod) {
	telem::Run(quadData);
		EKFUpdateTimer = 0;
  	ins.Update(micros(), quadData.navData.numMocapUpdates, quadIMU.GetGyro()*DEG_2_RAD, quadIMU.GetAcc()*G, quadData.navData.mocapPosition_NED.cast<double>());
	}
  quadData.att.eulerAngles_ekf = ins.Get_OrientEst()*RAD_2_DEG;
	Madgwick6DOF(quadIMU, quadData, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
#else
	if (EKFUpdateTimer > EKFPeriod) {
  	quadIMU.Update(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
		Madgwick6DOF(quadIMU, quadData, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
	}
#endif

	
#ifdef USE_POSITION_CONTROLLER
	// Check if position Controller enabled
	Eigen::Vector3f currentPosCovariance = ins.Get_CovPos();
	if (currentPosCovariance[0] < positionCovarianceLimit &&
			currentPosCovariance[1] < positionCovarianceLimit &&
			currentPosCovariance[2] < positionCovarianceLimit) {
		positionFix = true;
	} else {
		positionFix = false;
	}
	if (positionCtrlTimer >= positionCtrlPeriod) {
		getDesState(); // Convert raw commands to normalized values based on saturated control limits
		positionCtrlTimer = 0;
		if ((aux0.SwitchPosition() == SwPos::SWITCH_HIGH || aux0.SwitchPosition() == SwPos::SWITCH_MID)
				&& positionFix==true) {
			if (!wasTrueLastLoop) {
				wasTrueLastLoop = true;
				posControl.Reset();
			}
			mission.Run();
			posControl.Update(quadData.navData.positionSetpoint_NED.cast<double>(), ins.Get_PosEst(), ins.Get_VelEst(), quadData.att, dt, false);
			quadData.flightStatus.thrustSetpoint = posControl.GetDesiredThrust();
			if (aux0.SwitchPosition() == SwPos::SWITCH_HIGH) {
				quadData.att.eulerAngleSetpoint[0] = posControl.GetDesiredRoll();
				quadData.att.eulerAngleSetpoint[0] = posControl.GetDesiredPitch();
			}
		} else {
			wasTrueLastLoop = false; // Ensures the position controller is reset next time it's enabled
		}
		quadData.flightStatus.controlInputs[0] = quadData.flightStatus.thrustSetpoint;
	}
	# else
		// Compute desired state based on radio inputs
		getDesState(); // Convert raw commands to normalized values based on saturated control limits
		quadData.flightStatus.controlInputs[0] = quadData.flightStatus.thrustSetpoint;
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
	controller.SetKp(KpScaled);
	controller.SetKi(KiScaled);
	controller.SetKd(KdScaled);
#endif

	// TODO: be better
	bool noIntegral = false;
	if (throttleChannel.GetRawValue() < 1060) {
		noIntegral = true;
	}
	
	if (attitudeCtrlTimer >= attitudeCtrlPeriod) {
		attitudeCtrlTimer = 0;
		// FIXME: Make the function take Eigen::Vector or give up on it
		float gyroRates[3] = {quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ()};
		controller.Update(quadData.att.eulerAngleSetpoint.data(), quadData.att, gyroRates, dt, noIntegral);
		quadData.flightStatus.controlInputs(lastN(3)) = controller.GetMoments();
	}
	// Convert thrust and moments from controller to angular rates
	quadData.flightStatus.motorRates = ControlAllocator(quadData.flightStatus.controlInputs);
	// Convert angular rates to PWM commands
	motors.ScaleCommand(quadData.flightStatus.motorRates);

  // Throttle cut check
  bool killThrottle = throttleCut(); // Directly sets motor commands to low based on state of ch5
	

	motors.CommandMotor();
  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands
  failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  if (killThrottle && (throttleCutCount > 100) && flightLoopStarted) {
		logging.End();
    while (1) {
      getCommands();

			motors.CommandMotor();
      if (resetChannel.SwitchPosition() == SwPos::SWITCH_HIGH) {
        CPU_RESTART;
      }
    }
  } else if (killThrottle && flightLoopStarted) {
    throttleCutCount++;
  } else {
    throttleCutCount = 0;
  }

  // Regulate loop rate
  loopRate(2000); 
}

//========== MAIN FUNCTION ==========//

int main() {
	setup();
	while(1) {
		loop();
	}
	return 0;
}
