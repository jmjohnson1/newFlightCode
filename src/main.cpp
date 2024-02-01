//================================================================================================//

#include "UserDefines.h"

#include <stdint.h>

#include "RingBuf.h"  // Ring buffer used to store values for SD card
#include "SdFat.h"    // Library used for SD card
#include "TeensyTimerTool.h"  // Oneshot timer
#include "PWMServo.h" // Commanding any extra actuators, installed with teensyduino installer
#include "SPI.h"      // SPI communication
#include "Wire.h"     // I2c communication
#include "eigen.h"  	// Linear algebra
#include "SBUS.h" //sBus interface

#include "commonDefinitions.h"
#include "serialDebug.h"
#include "telemetry.h"
#include "IMU.h"
#include "madgwick.h"
#include "controller.h"
#include "motors.h"
#include "radio.h"
#include "testing.h"
#include "uNavINS.h"
#include "trajectory.h"
#include "parameters.h"




//================================================================================================//
//                                     USER-SPECIFIED VARIABLES 																	//
//================================================================================================//

// Radio channel definitions
// Syntax: ("name", channel, slider neutral point (meaningless for switches), failsafe value, true
// (if channel is critical))
RadioChannel throttleChannel("throttle", 1, 1000, 1000, true);
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
float maxRoll = globalConstants::MAX_ANGLE;
float maxPitch = globalConstants::MAX_ANGLE;
// Max yaw rate in deg/sec
float maxYaw = 160.0;

// ANGLE MODE PID GAINS //
// SCALE FACTORS FOR PID //
float pScale_att = 1.0f;
float iScale_att = 1.0f;
float dScale_att = 1.0f;
float allScale_att = 1.0f;

float Kp_roll_angle = 0.56;
float Ki_roll_angle = 0.176;
float Kd_roll_angle = 0.063;
float Kp_pitch_angle = 0.56;
float Ki_pitch_angle = 0.176;
float Kd_pitch_angle = 0.063;

// YAW PID GAINS //
float Kp_yaw = 0.3;
float Ki_yaw = 0.06;
float Kd_yaw = 0.00015;

// POSITION PID GAINS //
float Kp_pos[3] = {1.0f, 1.0f, 10.0f};
float Ki_pos[3] = {0.0f, 0.0f, 2.0f};
float Kd_pos[3] = {0.0f, 0.0f, 8.0f};

//================================================================================================//
//                                      DECLARE PINS 																							//
//================================================================================================//

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the
// 			 MPU6050 IMU for default setup

// Motor pin outputs:
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;

// Create servo objects to control a servo or ESC with PWM
#ifndef USE_ONESHOT
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
#endif

//================================================================================================//

// SD card setup
// Logic needed to restart teensy
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points (usec) for 100 samples/sec
#define LOG_INTERVAL_USEC 10000

// Size to log 256 byte lines at 100 Hz for a while
#define LOG_FILE_SIZE 256 * 100 * 600 * 10 /*~1,500,000,000 bytes.*/

// Space to hold more than 1 second of 256-byte lines at 100 Hz in the buffer
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.csv"

SdFs sd;
FsFile file;

// Ring buffer for filetype FsFile (The filemanager that will handle the data stream)
RingBuf<FsFile, RING_BUF_CAPACITY> buffer;
// DECLARE GLOBAL VARIABLES

// General stuff
float dt;

unsigned long current_time, prev_time;
unsigned long print_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
unsigned long print_counterSD = 200000;

SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

IMU quadIMU = IMU(-0.0121f, -0.0126f, -0.0770f, -4.7787f, 2.1795f, 0.6910f);
Attitude quadIMU_info;

ParameterManager paramManager;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:
float Kp_array[3] = {Kp_roll_angle, Kp_pitch_angle, Kp_yaw};
float Ki_array[3] = {Ki_roll_angle, Ki_pitch_angle, Ki_yaw};
float Kd_array[3] = {Kd_roll_angle, Kd_pitch_angle, Kd_yaw};
AngleAttitudeController controller = AngleAttitudeController(Kp_array, Ki_array, Kd_array);

PositionController posControl = PositionController(Kp_pos, Ki_pos, Kd_pos);


// Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

// Timers for each motor
TeensyTimerTool::OneShotTimer m1_timer(TeensyTimerTool::TMR1);
TeensyTimerTool::OneShotTimer m2_timer(TeensyTimerTool::TMR1);
TeensyTimerTool::OneShotTimer m3_timer(TeensyTimerTool::TMR1);
TeensyTimerTool::OneShotTimer m4_timer(TeensyTimerTool::TMR1);

// Flag for whether the motors are busy writing
bool m1_writing = false;
bool m2_writing = false;
bool m3_writing = false;
bool m4_writing = false;

// SD Card settings
String filePrefix = "flight_data";
String fileExtension = ".csv";
String fileName;

bool SD_is_present = 0;

bool doneWithSetup = 0;

uint16_t failureFlag = 0;

int throttleCutCount = 0;

// Flag to check if the flight loop has started yet, prevents lock in main loop when throttle killed
bool flightLoopStarted = 0;

int loopCount = 0;

// Telemetry
Telemetry telem;

// Position vector taken from mocap
Eigen::Vector3d mocapPosition(0, 0, 0);
uint32_t EKF_tow = 0; // Time of week used with the EKF. It increments whenever a new position is received from the transmitter

// EKF
uNavINS ins;

// Trajectory thing (this is temporary)
trajectory traj;
Eigen::Vector3d homePosition(0, 0.5, -0.5);
Eigen::Vector3d posSetpoint;

bool wasTrueLastLoop = false; // This will be renamed at some point

// Constants for unit conversion
const float DEG_2_RAD = PI/180.0f;
const float RAD_2_DEG = 1.0f/DEG_2_RAD;
const float G = 9.807f;  // m/s/s


// Various timers
elapsedMillis heartbeatTimer;
elapsedMillis fastMavlinkTimer;
elapsedMillis positionUpdateTimer;

const unsigned long EKFPeriod = 10; // milliseconds




//========================================================================================================================//
//                                                      FUNCTIONS //
//========================================================================================================================//




void getDesState() {
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and
   * yaw_des. These are computed by using the raw RC pwm commands and scaling
   * them to be within our limits defined in setup. thro_des stays within 0 to 1
   * range. roll_des and pitch_des are scaled to be within max roll/pitch amount
   * in either degrees (angle mode) or degrees/sec (rate mode). yaw_des is
   * scaled to be within max yaw in degrees/sec. Also creates roll_passthru,
   * pitch_passthru, and yaw_passthru variables, to be used in commanding
   * motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = throttleChannel.NormalizedValue(); // Between 0 and 1
  roll_des = rollChannel.NormalizedValue();  // Between -1 and 1
  pitch_des = -pitchChannel.NormalizedValue(); // Between -1 and 1
  yaw_des = -yawChannel.NormalizedValue();   // Between -1 and 1
  roll_passthru = roll_des / 2.0;               // Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0;             // Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0;                 // Between -0.5 and 0.5

  // Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);               // Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       // Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}


void getCommands() {
  // DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands.
   * channel_x_pwm is the raw command used in the rest of the loop. If using a
   * PWM or PPM receiver, the radio commands are retrieved from a function in
   * the readPWM file separate from this one which is running a bunch of
   * interrupts to continuously update the radio readings. If using an SBUS
   * receiver, the alues are pulled from the SBUS library directly. The raw
   * radio commands are filtered with a first order low-pass filter to eliminate
   * any really high frequency noise.
   */

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

void m1_EndPulse() {
  digitalWrite(m1Pin, LOW);
  m1_writing = false;
  //Serial.println("Timer triggered");
}
 
void m2_EndPulse() {
  digitalWrite(m2Pin, LOW);
  m2_writing = false;
}

void m3_EndPulse() {
  digitalWrite(m3Pin, LOW);
  m3_writing = false;
}

void m4_EndPulse() {
  digitalWrite(m4Pin, LOW);
  m4_writing = false;
}


int throttleCut() {
  // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command channel_5_pwm and directly sets the
   * mx_command_PWM values to minimum (120 is minimum for oneshot125 protocol, 0
   * is minimum for standard PWM servo library used) if channel 5 is high. This
   * is the last function called before commandMotors() is called so that the
   * last thing checked is if the user is giving permission to command the
   * motors to anything other than minimum value. Safety first.
   */
  if (throCutChannel.SwitchPosition() == SwPos::SWITCH_HIGH) {
		#ifdef USE_ONESHOT
    m1_command_PWM = 125;
    m2_command_PWM = 125;
    m3_command_PWM = 125;
    m4_command_PWM = 125;
		#else
    m1_command_PWM = 0;
    m2_command_PWM = 0;
    m3_command_PWM = 0;
    m4_command_PWM = 0;
		#endif
    return 1;
  }
  return 0;
}

void loopRate(int freq) {
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable
   * and whatnot. Interrupt routines running in the background cause the loop
   * rate to fluctuate. This function basically just waits at the end of every
   * loop iteration until the correct time has passed since the start of the
   * current loop for the desired loop rate in Hz. 2kHz is a good rate to be at
   * because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us
   * have a little room to add extra computations and remain above 2kHz, without
   * needing to retune all of our filtering parameters.
   */
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


void MadgwickWarmup() {
  // DESCRIPTION: Used to warm up the main loop to allow the madwick filter to
  // converge before commands can be sent to the actuators Assuming vehicle is
  // powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is
   * what causes startup to take a few seconds to boot.
   */
  // Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
		quadIMU.Update();
    Madgwick6DOF(quadIMU, &quadIMU_info, dt);
    loopRate(2000); // do not exceed 2000Hz
  }
}

void calibrateESCs() {
  // DESCRIPTION: Used in void setup() to allow standard ESC calibration
  // procedure with the radio to take place.
  /*
   *  Simulates the void loop(), but only for the purpose of providing throttle
   * pass through to the motors, so that you can power up with throttle at full,
   * let ESCs begin arming sequence, and lower throttle to zero. This function
   * should only be uncommented when performing an ESC calibration.
   */
  while (true) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    digitalWrite(13, HIGH); // LED on to indicate we are not in main loop
    getCommands();
    failSafe();  
    getDesState();
		quadIMU.Update();
    Madgwick6DOF(quadIMU, &quadIMU_info, dt);
    getDesState();
    m1_command_scaled = thro_des;
    m2_command_scaled = thro_des;
    m3_command_scaled = thro_des;
    m4_command_scaled = thro_des;
		m1_command_PWM = ScaleCommand(m1_command_scaled);
		m2_command_PWM = ScaleCommand(m2_command_scaled);
		m3_command_PWM = ScaleCommand(m3_command_scaled);
		m4_command_PWM = ScaleCommand(m4_command_scaled);
	CommandMotor(&m2_timer, m2_command_PWM, m2Pin);
    loopRate(2000);
  }
}

void calculate_IMU_error(IMU *imu, Attitude *att) {
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
		errorAcc[2] += imu->GetAccZ() - 1.0f; // Need to subtract gravity

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

  Serial.println("Paste these values in user specified variables section and "
                 "comment out calculate_IMU_error() in void setup.");
  for (;;);
}

//=====================================//
//========== SD Card Logging ==========//
//=====================================//
namespace datalogger {
	void PrintCSVHeader() {
		buffer.print("roll");
		buffer.write(",");
		buffer.print("pitch");
		buffer.write(",");
		buffer.print("yaw");
		buffer.write(",");
		buffer.print("roll_des");
		buffer.write(",");
		buffer.print("pitch_des");
		buffer.write(",");
		buffer.print("yaw_des");
		buffer.write(",");
		buffer.print("throttle_des");
		buffer.write(",");
		buffer.print("roll_PID");
		buffer.write(",");
		buffer.print("pitch_PID");
		buffer.write(",");
		buffer.print("yaw_PID");
		buffer.write(",");

		// radio channels
		for (int i = 0; i < numChannels; i++) {
			buffer.print(radioChannels[i]->GetName());
			buffer.write(",");
		}

		buffer.print("GyroX");
		buffer.write(",");
		buffer.print("GyroY");
		buffer.write(",");
		buffer.print("GyroZ");
		buffer.write(",");
		buffer.print("AccX");
		buffer.write(",");
		buffer.print("AccY");
		buffer.write(",");
		buffer.print("AccZ");
		buffer.write(",");
		buffer.print("m1_command");
		buffer.write(",");
		buffer.print("m2_command");
		buffer.write(",");
		buffer.print("m3_command");
		buffer.write(",");
		buffer.print("m4_command");
		buffer.write(",");
		buffer.print("kp_rp");
		buffer.write(",");
		buffer.print("ki_rp");
		buffer.write(",");
		buffer.print("kd_rp");
		buffer.write(",");
		buffer.print("kp_yaw");
		buffer.write(",");
		buffer.print("ki_yaw");
		buffer.write(",");
		buffer.print("kd_yaw");
		buffer.write(",");
		buffer.print("failsafeTriggered");
		buffer.write(",");
		buffer.print("timeSinceBoot");
		buffer.write(",");
		buffer.print("EKF_tow");
		buffer.write(",");
		buffer.print("MocapPositionX");
		buffer.write(",");
		buffer.print("MocapPositionY");
		buffer.write(",");
		buffer.print("MocapPositionZ");

  #ifdef USE_EKF
		buffer.write(",");
    buffer.print("rollEstEKF");
		buffer.write(",");
    buffer.print("pitchEstEKF");
		buffer.write(",");
    buffer.print("yawEstEKF");
		buffer.write(",");
    buffer.print("xEstEKF");
		buffer.write(",");
    buffer.print("yEstEKF");
		buffer.write(",");
    buffer.print("zEstEKF");

		buffer.write(",");
		buffer.print("vxEstEKF");
		buffer.write(",");
		buffer.print("vyEstEKF");
		buffer.write(",");
		buffer.print("vzEstEKF");
		buffer.write(",");
		buffer.print("AccelBias1");
		buffer.write(",");
		buffer.print("AccelBias2");
		buffer.write(",");
		buffer.print("AccelBias3");
		buffer.write(",");
		buffer.print("GyroBias1");
		buffer.write(",");
		buffer.print("GyroBias2");
		buffer.write(",");
		buffer.print("GyroBias3");
		

  #endif

	#ifdef USE_POSITION_CONTROLLER
		buffer.write(",");
		buffer.print("setpointX");
		buffer.write(",");
		buffer.print("setpointY");
		buffer.write(",");
		buffer.print("setpointZ");

		buffer.write(",");
		buffer.print("kp_xy");
		buffer.write(",");
		buffer.print("ki_xy");
		buffer.write(",");
		buffer.print("kd_xy");
		buffer.write(",");
		buffer.print("kp_z");
		buffer.write(",");
		buffer.print("ki_z");
		buffer.write(",");
		buffer.print("kd_z");
	#endif


		buffer.println();
	}

	int Setup() {
		// Initialize the SD
		if (!sd.begin(SD_CONFIG)) {
			sd.initErrorPrint(&Serial); // Prints message to serial if SD can't init
			return 1;
		}
		// Determine logfile name
		int fileIncrement = 0;
		fileName = filePrefix + String(fileIncrement) + fileExtension;
		while(sd.exists(fileName)) {
			// Increment file name if it exists and try again
			fileIncrement++;
			fileName = filePrefix + String(fileIncrement) + fileExtension;
		}
		//Open or create file - truncate existing
		if (!file.open(fileName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
			Serial.println("open failed\n");
			return 1;
		}
		// Initialize ring buffer
		buffer.begin(&file);
		Serial.println("Buffer initialized");
		datalogger::PrintCSVHeader();
		return 0;
	}


	int WriteBuffer() {
		size_t amtDataInBuf = buffer.bytesUsed();
		
		if ((amtDataInBuf + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
			Serial.println("Log file full -- No longer writing");
			return 1;
		}
		if (amtDataInBuf >= 512 && !file.isBusy()) {
			// One sector (512 bytes) can be printed before busy wait
			// Write from buffer to file
			if (512 != buffer.writeOut(512)) {
				Serial.println("Write to file from buffer failed -- breaking");
				return 1;
			}
		}

		buffer.print(quadIMU_info.roll, 4);
		buffer.write(",");
		buffer.print(quadIMU_info.pitch, 4);
		buffer.write(",");
		buffer.print(quadIMU_info.yaw, 4);
		buffer.write(",");
		buffer.print(roll_des, 4);
		buffer.write(",");
		buffer.print(pitch_des, 4);
		buffer.write(",");
		buffer.print(yaw_des, 4);
		buffer.write(",");
		buffer.print(thro_des, 4);
		buffer.write(",");
		buffer.print(controller.GetRollPID(), 4);
		buffer.write(",");
		buffer.print(controller.GetPitchPID(), 4);
		buffer.write(",");
		buffer.print(controller.GetYawPID(), 4);
		buffer.write(",");

		// radio channels
		for (int i = 0; i < numChannels; i++) {
			buffer.print(radioChannels[i]->GetRawValue());
			buffer.write(",");
		}


		buffer.print(quadIMU.GetGyroX(), 4);
		buffer.write(",");
		buffer.print(quadIMU.GetGyroY(), 4);
		buffer.write(",");
		buffer.print(quadIMU.GetGyroZ(), 4);
		buffer.write(",");
		buffer.print(quadIMU.GetAccX(), 4);
		buffer.write(",");
		buffer.print(quadIMU.GetAccY(), 4);
		buffer.write(",");
		buffer.print(quadIMU.GetAccZ(), 4);
		buffer.write(",");
		buffer.print(m1_command_scaled, 4);
		buffer.write(",");
		buffer.print(m2_command_scaled, 4);
		buffer.write(",");
		buffer.print(m3_command_scaled, 4);
		buffer.write(",");
		buffer.print(m4_command_scaled, 4);
		buffer.write(",");
		buffer.print(Kp_roll_angle*pScale_att, 4);
		buffer.write(",");
		buffer.print(Ki_roll_angle*iScale_att, 4);
		buffer.write(",");
		buffer.print(Kd_roll_angle*dScale_att, 4);
		buffer.write(",");
		buffer.print(Kp_yaw, 4);
		buffer.write(",");
		buffer.print(Ki_yaw, 4);
		buffer.write(",");
		buffer.print(Kd_yaw, 4);
		buffer.write(",");
		buffer.print(failureFlag);
		buffer.write(",");
		buffer.print(micros());
		buffer.write(",");
		buffer.print(EKF_tow);
		buffer.write(",");
		buffer.print(mocapPosition[0], 10);
		buffer.write(",");
		buffer.print(mocapPosition[1], 10);
		buffer.write(",");
		buffer.print(mocapPosition[2], 10);
  #ifdef USE_EKF
		buffer.write(",");
    buffer.print(ins.Get_OrientEst()[0], 5);
		buffer.write(",");
    buffer.print(ins.Get_OrientEst()[1], 5);
		buffer.write(",");
    buffer.print(ins.Get_OrientEst()[2], 5);
		buffer.write(",");
    buffer.print(ins.Get_PosEst()[0], 10);
		buffer.write(",");
    buffer.print(ins.Get_PosEst()[1], 10);
		buffer.write(",");
    buffer.print(ins.Get_PosEst()[2], 10);
		buffer.write(",");
		buffer.print(ins.Get_VelEst()[0], 4);
		buffer.write(",");
		buffer.print(ins.Get_VelEst()[1], 4);
		buffer.write(",");
		buffer.print(ins.Get_VelEst()[2], 4);
		buffer.write(",");
		buffer.print(ins.Get_AccelBias()[0], 4);
		buffer.write(",");
		buffer.print(ins.Get_AccelBias()[1], 4);
		buffer.write(",");
		buffer.print(ins.Get_AccelBias()[2], 4);
		buffer.write(",");
		buffer.print(ins.Get_RotRateBias()[0], 4);
		buffer.write(",");
		buffer.print(ins.Get_RotRateBias()[1], 4);
		buffer.write(",");
		buffer.print(ins.Get_RotRateBias()[2], 4);
  #endif
	#ifdef USE_POSITION_CONTROLLER
		buffer.write(",");
		buffer.print(traj.GetSetpoint()[0]);
		buffer.write(",");
		buffer.print(traj.GetSetpoint()[1]);
		buffer.write(",");
		buffer.print(traj.GetSetpoint()[2]);
		buffer.write(",");
		buffer.print(posControl.GetKp()[0]);
		buffer.write(",");
		buffer.print(posControl.GetKi()[0]);
		buffer.write(",");
		buffer.print(posControl.GetKd()[0]);
		buffer.write(",");
		buffer.print(posControl.GetKp()[2]);
		buffer.write(",");
		buffer.print(posControl.GetKi()[2]);
		buffer.write(",");
		buffer.print(posControl.GetKd()[2]);
	#endif
		buffer.println();

		if (buffer.getWriteError()) {
			Serial.println("WriteError");
			return 1;
		}
		return 0;
	}

	void EndProcess() {
		// Write any remaining buffer data to file
		buffer.sync();
		file.truncate();
		file.rewind();
		file.close();
		
		Serial.println("logging ended peacefully");
	}
}

//===========================//
//========== SETUP ==========//
//===========================//
void setup() {
  Serial.begin(500000); // USB serial
  delay(500);

  // Initialize all pins
  pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify

  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);

	#ifdef USE_ONESHOT
  // Initialize timers for OneShot125
  m1_timer.begin(m1_EndPulse);
  m2_timer.begin(m2_EndPulse);
  m3_timer.begin(m3_EndPulse);
  m4_timer.begin(m4_EndPulse);
	#else
  servo1.attach(m1Pin, 1000, 2100); // Pin, min PWM value, max PWM value
  servo2.attach(m2Pin, 1000, 2100);
  servo3.attach(m3Pin, 1000, 2100);
  servo4.attach(m4Pin, 1000, 2100);
	#endif

  // Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  // Initialize radio communication (defined in header file)
  radioSetup();

  // Begin mavlink telemetry module
	// TODO: Find a better way to add and manage params. No hardcode for index
	paramManager.addParameter("kp_xy", Kp_pos[0], MAV_PARAM_TYPE_REAL32);
	paramManager.addParameter("ki_xy", Ki_pos[0], MAV_PARAM_TYPE_REAL32);
	paramManager.addParameter("kd_xy", Kd_pos[0], MAV_PARAM_TYPE_REAL32);
	paramManager.addParameter("kp_z", Kp_pos[2], MAV_PARAM_TYPE_REAL32);
	paramManager.addParameter("ki_z", Ki_pos[2], MAV_PARAM_TYPE_REAL32);
	paramManager.addParameter("kd_z", Kd_pos[2], MAV_PARAM_TYPE_REAL32);
  telem.InitTelemetry(&paramManager);


	bool IMU_initSuccessful = quadIMU.Init(&Wire);	
  quadIMU.Update(); // Get an initial reading. If not, initial attitude estimate will be NaN
	Serial.print("IMU initialization successful: ");
	Serial.println(IMU_initSuccessful);

#ifdef USE_EKF
  ins.Configure();
	ins.Initialize(quadIMU.GetGyro()*DEG_2_RAD, quadIMU.GetAcc()*G, mocapPosition);
#endif

  // Initialize the SD card, returns 1 if no sd card is detected or it can't be
  // initialized
  SD_is_present = !(datalogger::Setup());

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is
  // level when powered up Calibration parameters printed to serial monitor.
  // Paste these in the user specified variables section, then comment this out
  // forever.

  // calculate_IMU_error(&quadIMU_info, &quadIMU);


  // Arm servo channels
#ifndef USE_ONESHOT
  servo1.write(0); // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); // Set these to 90 for servos if you do not want them to
                   // briefly max out on startup
  servo3.write(0); // Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
#endif

  delay(5);

  // PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick
  // to max, powering on, and lowering throttle to zero after the beeps
  //calibrateESCs();
  // Code will not proceed past here if this function is uncommented!

#ifdef USE_ONESHOT
  // Arm OneShot125 motors
  m1_command_PWM = 125; // Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
	TeensyTimerTool::OneShotTimer *motorTimers[4] =  {&m1_timer, &m2_timer, &m3_timer, &m4_timer};
	uint8_t motorPins[4] = {m1Pin, m2Pin, m3Pin, m4Pin};
  ArmMotors(motorTimers, motorPins, 4); // Loop over commandMotors() until ESCs happily arm
#endif

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

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  //  Print data at 100hz (uncomment one at a time for troubleshooting)
	if (current_time - print_counter > 100000) {
		print_counter = current_time;
		//serialDebug::PrintRadioData(); // Currently does nothing
		//serialDebug::PrintDesiredState(thro_des, roll_des, pitch_des, yaw_des);
		//serialDebug::PrintGyroData(quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ());
		//serialDebug::PrintAccelData(quadIMU.GetAccX(), quadIMU.GetAccY(), quadIMU.GetAccZ());
		//serialDebug::PrintRollPitchYaw(quadIMU_info.roll, quadIMU_info.pitch, quadIMU_info.yaw);
		//serialDebug::PrintPIDOutput(controller.GetRollPID(), controller.GetPitchPID(), controller.GetYawPID());
		//serialDebug::PrintMotorCommands(m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM);
		//serialDebug::PrintLoopTime(dt);
	}

  // Check if rotors should be armed
  if (!flightLoopStarted && (throCutChannel.SwitchPosition() == SwPos::SWITCH_LOW)) {
    flightLoopStarted = 1;
    telem.SetSystemState(MAV_STATE_ACTIVE);
    telem.SetSystemMode(MAV_MODE_MANUAL_ARMED);
  }

  // Write to SD card buffer
  if (SD_is_present && (current_time - print_counterSD) > LOG_INTERVAL_USEC) {
    print_counterSD = micros();
	datalogger::WriteBuffer();
  }

	// TODO: Be better
  if (heartbeatTimer > 1000) {
		heartbeatTimer = 0;
    telem.SendHeartbeat();
    telem.SendPIDGains_core(Kp_roll_angle * pScale_att, Ki_roll_angle * iScale_att, Kd_roll_angle * dScale_att);
  }
  if (fastMavlinkTimer > 100) {
		fastMavlinkTimer = 0;
    telem.SendAttitude(quadIMU_info.roll, quadIMU_info.pitch, 0.0f, quadIMU.GetGyroX(), quadIMU.GetGyroY(), 0.0f);
  }

  telem.UpdateReceived();
  EKF_tow = telem.CheckForNewPosition(mocapPosition, EKF_tow);
	// TODO: Implement some way to not do this every loop and do this without
	// hardcoded indecies
	// Update position PID gains
	Kp_pos[0] = paramManager.parameters[0].value;
	Ki_pos[0] = paramManager.parameters[1].value;
	Kd_pos[0] = paramManager.parameters[2].value;
	Kp_pos[1] = paramManager.parameters[0].value;
	Ki_pos[1] = paramManager.parameters[1].value;
	Kd_pos[1] = paramManager.parameters[2].value;
	Kp_pos[2] = paramManager.parameters[3].value;
	Ki_pos[2] = paramManager.parameters[4].value;
	Kd_pos[2] = paramManager.parameters[5].value;
	posControl.SetKp(Kp_pos);
	posControl.SetKi(Ki_pos);
	posControl.SetKd(Kd_pos);

  // Get vehicle state
  quadIMU.Update(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick6DOF(quadIMU, &quadIMU_info, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

#ifdef USE_EKF
	if (positionUpdateTimer > EKFPeriod) {
		positionUpdateTimer = 0;
  	ins.Update(micros(), EKF_tow, quadIMU.GetGyro()*DEG_2_RAD, quadIMU.GetAcc()*G, mocapPosition);
	}
  quadIMU_info.roll = ins.Get_OrientEst()[0]*RAD_2_DEG;
  quadIMU_info.pitch = ins.Get_OrientEst()[1]*RAD_2_DEG;
  quadIMU_info.yaw = ins.Get_OrientEst()[2]*RAD_2_DEG;
#endif

  // Compute desired state based on radio inputs
  getDesState(); // Convert raw commands to normalized values based on saturated control limits
	
#ifdef USE_POSITION_CONTROLLER
	// Check if position Controller enabled
	if (aux0.SwitchPosition() == SwPos::SWITCH_HIGH || aux0.SwitchPosition() == SwPos::SWITCH_MID) {
		if (!wasTrueLastLoop) {
			wasTrueLastLoop = true;
			posControl.Reset();
		}
		posSetpoint = homePosition;
		if (aux3.SwitchPosition() == SwPos::SWITCH_HIGH) {
			posSetpoint(2) = -1.0;
		}
		if (aux2.SwitchPosition() == SwPos::SWITCH_HIGH) {
			posSetpoint(1) = 1.0;
		}
		if (aux1.SwitchPosition() == SwPos::SWITCH_HIGH) {
			posSetpoint(0) = 1.0;
		}
		posControl.Update(posSetpoint, ins.Get_PosEst(), quadIMU_info, dt, false);
		thro_des = posControl.GetDesiredThrottle();
		if (aux0.SwitchPosition() == SwPos::SWITCH_MID) {
			roll_des = posControl.GetDesiredRoll();
			pitch_des = posControl.GetDesiredPitch();
		}
	} else {
		wasTrueLastLoop = false; // Ensures the position controller is reset next time it's enabled
	}
	
#endif
	
#ifdef TEST_STAND
	// Sine sweep
  if (aux1.SwitchPosition() == SwPos::SWITCH_HIGH) {
		testStand::SineSweep(dt);
  }
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
	
	float setpoints[3] = {roll_des, pitch_des, yaw_des};
	float gyroRates[3] = {quadIMU.GetGyroX(), quadIMU.GetGyroY(), quadIMU.GetGyroZ()};
	controller.Update(setpoints, quadIMU_info, gyroRates, dt, noIntegral);
	float motorCommandsNormalized[4];
	controller.GetMotorCommands(motorCommandsNormalized, thro_des);

	// TODO: Fix these god awful names and maybe put things in arrays
	m1_command_scaled = motorCommandsNormalized[0];
	m2_command_scaled = motorCommandsNormalized[1];
	m3_command_scaled = motorCommandsNormalized[2];
	m4_command_scaled = motorCommandsNormalized[3];

	m1_command_PWM = ScaleCommand(m1_command_scaled);
	m2_command_PWM = ScaleCommand(m2_command_scaled);
	m3_command_PWM = ScaleCommand(m3_command_scaled);
	m4_command_PWM = ScaleCommand(m4_command_scaled);

  // Throttle cut check
  bool killThrottle = throttleCut(); // Directly sets motor commands to low based on state of ch5
	
	CommandMotor(&m1_timer, m1_command_PWM, m1Pin);
	CommandMotor(&m2_timer, m2_command_PWM, m2Pin);
	CommandMotor(&m3_timer, m3_command_PWM, m3Pin);
	CommandMotor(&m4_timer, m4_command_PWM, m4Pin);

  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands
  failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  if (killThrottle && (throttleCutCount > 100) && flightLoopStarted) {
		datalogger::EndProcess();
    while (1) {
      getCommands();

			CommandMotor(&m1_timer, m1_command_PWM, m1Pin);
			CommandMotor(&m2_timer, m2_command_PWM, m2Pin);
			CommandMotor(&m3_timer, m3_command_PWM, m3Pin);
			CommandMotor(&m4_timer, m4_command_PWM, m4Pin);

      if (resetChannel.SwitchPosition() == SwPos::SWITCH_HIGH) {
        CPU_RESTART;
      }
    }
  } else if (killThrottle && flightLoopStarted) {
    throttleCutCount++;
    Serial.print("throttleCutCount: ");
    Serial.println(throttleCutCount);
  } else {
    throttleCutCount = 0;
  }

  // Regulate loop rate
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

//========== MAIN FUNCTION ==========//

int main() {
	setup();
	while(1) {
		loop();
	}
	return 0;
}
