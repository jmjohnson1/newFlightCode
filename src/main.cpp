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

#include "commonDefinitions.h"
#include "serialDebug.h"
#include "telemetry.h"
#include "IMU.h"
#include "madgwick.h"
#include "controller.h"
#include "motors.h"

#include "SBUS.h" //sBus interface

IMU quadIMU = IMU(-0.0121f, 0.0126f, 0.0770f, -4.7787f, -2.1795f, -0.6910f);

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
#define LOG_FILE_SIZE 256 * 100 * 600 * 10 // ~1,500,000,000 bytes.

// Space to hold more than 1 second of 256-byte lines at 100 Hz in the buffer
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.csv"

SdFs sd;
FsFile file;

// Ring buffer for filetype FsFile (The filemanager that will handle the data stream)
RingBuf<FsFile, RING_BUF_CAPACITY> buffer;

//================================================================================================//
//                                     USER-SPECIFIED VARIABLES 																	//
//================================================================================================//

// Radio failsafe values for every channel in the event that bad reciever data
// is detected. Recommended defaults:
unsigned long channel_1_fs = 1000;  // thro cut
unsigned long channel_2_fs = 1500;  // ail neutral
unsigned long channel_3_fs = 1500;  // elev neutral
unsigned long channel_4_fs = 1500;  // rudd neutral
unsigned long channel_5_fs = 2000;  // gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 1000;  // Iris toggle (closed)
unsigned long channel_7_fs = 1000;  // Conduct sine sweep (Don't do a sine sweep)
unsigned long channel_8_fs = 1000;  // Perform step in pitch or roll (No step commands)
unsigned long channel_9_fs = 1500;  // Step angle (+15, 0, -15) (0 degrees)
unsigned long channel_10_fs = 1500; // P gain scale (no scaling)
unsigned long channel_11_fs = 1500; // I gain scale
unsigned long channel_12_fs = 1500; // D gain scale
unsigned long channel_13_fs = 1500; // Pitch and roll pid offset
unsigned long channel_14_fs = 1000; // Reset switch

// Controller parameters (take note of defaults before modifying!):
// Integrator saturation level, mostly for safety (default 25.0)
float i_limit = 25.0;
// Max roll/pitch angles in degrees for angle mode (maximum ~70 degrees),
// deg/sec for rate mode
float maxRoll = 30.0;
float maxPitch = 30.0;
// Max yaw rate in deg/sec
float maxYaw = 160.0;

// ANGLE MODE PID GAINS //
// SCALE FACTORS FOR PID //
float pScaleRoll = 1.0f;
float pScalePitch = 1.0f;
float pScaleYaw = 1.0f;
float iScaleRoll = 1.0f;
float iScalePitch = 1.0f;
float iScaleYaw = 1.0f;
float dScaleRoll = 1.0f;
float dScalePitch = 1.0f;
float dScaleYaw = 1.0f;

float Kp_roll_angle = 0.56;
float Ki_roll_angle = 0.176;
float Kd_roll_angle = -0.063;
float Kp_pitch_angle = 0.56;
float Ki_pitch_angle = 0.176;
float Kd_pitch_angle = -0.063;

// Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float B_loop_roll = 0.9;
// Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float B_loop_pitch = 0.9;

// RATE MODE PID GAINS //
float Kp_roll_rate = 0.15;
float Ki_roll_rate = 0.2;
float Kd_roll_rate = 0.0002;
float Kp_pitch_rate = 0.15;
float Ki_pitch_rate = 0.2;
float Kd_pitch_rate = 0.0002;

// YAW PID GAINS //
float Kp_yaw = 0.3;
float Ki_yaw = 0.06;
float Kd_yaw = 0.00015;

// Options for controlling the quad using user input values written over the
// serial line Sets whether or not to allow direct input of pitch or roll angles
// during a test flight. Setting this to true
bool useSerialAngleCommands = 0;
// The axis to rotate about in the setDesStateSerial() function: 1 = roll, 2 =
// pitch
int axisToRotate = 1;
// Determines whether or not to use a sine wave in the setDesStateSerial()
// function. If so, then the input from the serial line is taken to be the
// frequency of this sine wave in Hz.
bool useSineWave = 1;

// Options for sine sweep
// Option for whether or not to do a sine sweep as part of a test flight.
// Setting this to true will result in pilot control of the axis connected to
// axisToRotate being removed, and a sine sweep conducted between the maximum
// and minimum frequencies specified.
bool conductSineSweep = 0;
float maxFreq = 3.0f;  // Maximum frequency of the sine sweep in Hz 		FIXME: Goes slightly beyond
											 // 																											maxFreq
float minFreq = 0.5f;  // Minimum frequency of the sine sweep in Hz
float sweepTime = 120; // How long to run the sweep for in seconds

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

// DECLARE GLOBAL VARIABLES

// General stuff
float dt;

unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
unsigned long print_counterSD = 200000;

// Radio communication:
int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm,
    channel_8_pwm, channel_9_pwm, channel_10_pwm, channel_11_pwm, channel_12_pwm, channel_13_pwm, channel_14_pwm;
int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
int channel_1_pwm_pre, channel_2_pwm_pre, channel_3_pwm_pre, channel_4_pwm_pre;

SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

Attitude quadIMU_info;
Attitude imu2_info;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:

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

// Values for the setDesStateSerial() function
float serialInputValue = 0.0f; // User input over the serial line
float sineFrequency = 0.0f;    // If using sine wave, its frequency in Hz
float sineTime = 0.0f;         // Counter used to determine time in the sine functions (seconds)

// A flag for whether or not the sine sweep should be conducted. User input
// while the program is running sets this. DON'T SET THIS YOURSELF!
bool sweepFlag = 0;

// SD Card settings
String filePrefix = "flight_data";
String fileExtension = ".csv";
String fileName;

bool SD_is_present = 0;

int cutoff_val = 150;
int d_ch1;
int d_ch2;
int d_ch3;
int d_ch4;

int ch1_CutCounter = 0;
int ch2_CutCounter = 0;
int ch3_CutCounter = 0;
int ch4_CutCounter = 0;

bool doneWithSetup = 0;
int servoLoopCounter = 0;

// Number of loops before a sustained large change in values are accepted
int radioChCutoffTimeout = 10;
bool failureFlag = 0;

int throttleCutCount = 0;

// For keeping track of loop times
float max_loopTime = 0;

// Flag to check if the flight loop has started yet, prevents lock in main loop when throttle killed
bool flightLoopStarted = 0;

int loopCount = 0;

// Telemetry
Telemetry telem;

// Position vector taken from mocap
Eigen::Vector3f mocapPosition(0, 0, 0);
bool newPositionReceived;

AnglePID rollPID = AnglePID(Kp_roll_angle, Ki_roll_angle, Kd_roll_angle);
AnglePID pitchPID = AnglePID(Kp_pitch_angle, Ki_pitch_angle, Kd_pitch_angle);
RatePID yawPID = RatePID(Kp_yaw, Ki_yaw, Kd_yaw);

float Kp_array[3] = {Kp_roll_angle, Kp_pitch_angle, Kp_yaw};
float Ki_array[3] = {Ki_roll_angle, Ki_pitch_angle, Ki_yaw};
float Kd_array[3] = {Kd_roll_angle, Kd_pitch_angle, Kd_yaw};

AngleAttitudeController controller = AngleAttitudeController(Kp_array, Ki_array, Kd_array);


//========================================================================================================================//
//                                                      FUNCTIONS //
//========================================================================================================================//


//void calculate_IMU_error(attInfo *imu, MPU6050 *mpu6050) {
//  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
//  imu->AccErrorX = 0.0;
//  imu->AccErrorY = 0.0;
//  imu->AccErrorZ = 0.0;
//  imu->GyroErrorX = 0.0;
//  imu->GyroErrorY = 0.0;
//  imu->GyroErrorZ = 0.0;
//
//  // Read IMU values 12000 times
//  int c = 0;
//  while (c < 12000) {
//    mpu6050->getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
//
//    imu->AccX = AcX / ACCEL_SCALE_FACTOR;
//    imu->AccY = AcY / ACCEL_SCALE_FACTOR;
//    imu->AccZ = AcZ / ACCEL_SCALE_FACTOR;
//    imu->GyroX = GyX / GYRO_SCALE_FACTOR;
//    imu->GyroY = GyY / GYRO_SCALE_FACTOR;
//    imu->GyroZ = GyZ / GYRO_SCALE_FACTOR;
//
//    // Sum all readings
//    imu->AccErrorX = imu->AccErrorX + imu->AccX;
//    imu->AccErrorY = imu->AccErrorY + imu->AccY;
//    imu->AccErrorZ = imu->AccErrorZ + imu->AccZ;
//    imu->GyroErrorX = imu->GyroErrorX + imu->GyroX;
//    imu->GyroErrorY = imu->GyroErrorY + imu->GyroY;
//    imu->GyroErrorZ = imu->GyroErrorZ + imu->GyroZ;
//    c++;
//  }
//  // Divide the sum by 12000 to get the error value
//  imu->AccErrorX = imu->AccErrorX / c;
//  imu->AccErrorY = imu->AccErrorY / c;
//  imu->AccErrorZ = imu->AccErrorZ / c - 1.0;
//  imu->GyroErrorX = imu->GyroErrorX / c;
//  imu->GyroErrorY = imu->GyroErrorY / c;
//  imu->GyroErrorZ = imu->GyroErrorZ / c;
//
//  Serial.print("float AccErrorX = ");
//  Serial.print(imu->AccErrorX);
//  Serial.println(";");
//  Serial.print("float AccErrorY = ");
//  Serial.print(imu->AccErrorY);
//  Serial.println(";");
//  Serial.print("float AccErrorZ = ");
//  Serial.print(imu->AccErrorZ);
//  Serial.println(";");
//
//  Serial.print("float GyroErrorX = ");
//  Serial.print(imu->GyroErrorX);
//  Serial.println(";");
//  Serial.print("float GyroErrorY = ");
//  Serial.print(imu->GyroErrorY);
//  Serial.println(";");
//  Serial.print("float GyroErrorZ = ");
//  Serial.print(imu->GyroErrorZ);
//  Serial.println(";");
//
//  Serial.println("Paste these values in user specified variables section and "
//                 "comment out calculate_IMU_error() in void setup.");
//  for (;;);
//}


void setDesStateSerial(int controlledAxis) {
  // DESCRIPTION: Sets the desired pitch and roll angles based on user input
  // over USB
  if (Serial.available()) {
    serialInputValue = Serial.parseFloat();
    while (Serial.available() != 0) {
      Serial.read();
    }
  }

  float desiredAngle = 0;

  if (useSineWave) {
    sineFrequency = static_cast<float>(serialInputValue);
    desiredAngle = 10 * sin(2 * PI * sineFrequency * sineTime); // Set the output to be a sin wave
                                                                // between -5 and 5 degrees
    sineTime = sineTime + 1 / 2000.0f;
  } else {
    desiredAngle = static_cast<float>(serialInputValue);
  }

  switch (controlledAxis) {
  case 1:
    roll_des = desiredAngle;
    break;
  case 2:
    pitch_des = desiredAngle;
    break;
  default:
    break;
  }
}

void performSineSweep(int controlledAxis) {
  // DESCRIPTION: Performs a sine sweep from minFreq (Hz) to maxFreq (Hz) over sweepTime (seconds)
  float desiredAngle = 0;
  float amp = 10; // Sine wave amplitude in degrees

  // if (Serial.available()) {
  //   sweepFlag = 1;
  //   while (Serial.available() !=0) {
  //       Serial.read();
  //     }
  // }

  // if (sweepFlag){
  desiredAngle =
      amp * sin(PI * (maxFreq - minFreq) / pow(sweepTime, 2) * pow(sineTime, 3) + 2 * PI * minFreq * sineTime);
  if (sineTime > sweepTime) {
    desiredAngle = 0;
  }
  sineTime = sineTime + 1 / 2000.0f;
  //}

  switch (controlledAxis) {
  case 1:
    roll_des = desiredAngle;
    break;
  case 2:
    pitch_des = desiredAngle;
    break;
  default:
    break;
  }
}

void rollStep() {
  float desiredAngle;
  if (channel_9_pwm < 1250) {
    desiredAngle = 10.0f;
  } else if (channel_9_pwm > 1750) {
    desiredAngle = -10.0f;
  } else {
    desiredAngle = 0.0f;
  }
  roll_des = desiredAngle;
}
void pitchStep() {
  float desiredAngle;
  if (channel_9_pwm < 1250) {
    desiredAngle = 3.0f;
  } else if (channel_9_pwm > 1750) {
    desiredAngle = -3.0f;
  } else {
    desiredAngle = 0.0f;
  }
  pitch_des = desiredAngle;
}

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
  thro_des = (channel_1_pwm - 1000.0) / 1000.0; // Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0) / 500.0;  // Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0) / 500.0; // Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0) / 500.0;   // Between -1 and 1
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
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias;
    channel_7_pwm = sbusChannels[6] * scale + bias;
    channel_8_pwm = sbusChannels[7] * scale + bias;
    channel_9_pwm = sbusChannels[8] * scale + bias;
    channel_10_pwm = sbusChannels[9] * scale + bias;
    channel_11_pwm = sbusChannels[10] * scale + bias;
    channel_12_pwm = sbusChannels[11] * scale + bias;
    channel_13_pwm = sbusChannels[12] * scale + bias;
    channel_14_pwm = sbusChannels[13] * scale + bias;
  }

  // Low-pass the critical commands and update previous values
  float b = 0.7; // Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;

  // Update prev values
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  // DESCRIPTION: If radio gives garbage values, set all commands to default
  // values
  /*
   * Radio connection failsafe used to check if the getCommands() function is
   * returning acceptable pwm values. If any of the commands are lower than 800
   * or higher than 2200, then we can be certain that there is an issue with the
   * radio connection (most likely hardware related). If any of the channels
   * show this failure, then all of the radio commands channel_x_pwm are set to
   * default failsafe values specified in the setup. Comment out this function
   * when troubleshooting your radio connection in case any extreme values are
   * triggering this function to overwrite the printed variables.
   */
  int minVal = 800;
  int maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
  failureFlag = 0;

  // Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal)
    check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal)
    check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal)
    check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal)
    check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal)
    check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal)
    check6 = 1;

  // If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
    channel_7_pwm = channel_7_fs;
    channel_8_pwm = channel_8_fs;
    channel_9_pwm = channel_9_fs;
    channel_10_pwm = channel_10_fs;
    channel_11_pwm = channel_11_fs;
    channel_12_pwm = channel_12_fs;
    channel_13_pwm = channel_13_fs;
    channel_14_pwm = channel_14_fs;

    failureFlag = 1;
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


//void armMotors() {
//  // DESCRIPTION: Sends many command pulses to the motors, to be used to arm
//  // motors in the void setup()
//  /*
//   *  Loops over the commandMotors() function 1500 times with a delay in between,
//   * simulating how the commandMotors() function is used in the main loop.
//   * Ensures motors arm within the void setup() where there are some delays for
//   * other processes that sometimes prevent motors from arming.
//   */
//  for (int i = 0; i <= 10000; i++) {
//    commandMotors();
//    delayMicroseconds(500);
//  }
//}


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
  if (channel_5_pwm > 1500) {
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

void getPScale() {
  float scaleVal;
  scaleVal = 1.0f + (channel_10_pwm - 1000.0f) / 1000.0f * 1.0f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  pScaleRoll = scaleVal;
  pScalePitch = scaleVal;
}

void getDScale() {
  float scaleVal;
  scaleVal = 1.0f + (channel_12_pwm - 1000.0f) / 1000.0f * 2.0f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  dScaleRoll = scaleVal;
  dScalePitch = scaleVal;
}

void getIScale() {
  float scaleVal;
  scaleVal = 1.0f + (channel_11_pwm - 1000.0f) / 1000.0f * 2.0f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  iScaleRoll = scaleVal;
  iScalePitch = scaleVal;
}

void scaleBoth() {
  float scaleMultiplier;
  scaleMultiplier = 1.0f + (channel_13_pwm - 1015.0f) / 1000.0f * 0.25f;
  pScaleRoll *= scaleMultiplier;
  iScaleRoll *= scaleMultiplier;
  dScaleRoll *= scaleMultiplier;
  pScalePitch *= scaleMultiplier;
  iScalePitch *= scaleMultiplier;
  dScalePitch *= scaleMultiplier;
}

//=========================================================================================//

void radioSetup() {
    sbus.begin();
}


//void calibrateAttitude() {
//  // DESCRIPTION: Used to warm up the main loop to allow the madwick filter to
//  // converge before commands can be sent to the actuators Assuming vehicle is
//  // powered up on level surface!
//  /*
//   * This function is used on startup to warm up the attitude estimation and is
//   * what causes startup to take a few seconds to boot.
//   */
//  // Warm up IMU and madgwick filter in simulated main loop
//  for (int i = 0; i <= 10000; i++) {
//    prev_time = current_time;
//    current_time = micros();
//    dt = (current_time - prev_time) / 1000000.0;
//    getIMUData(&quadIMU_info, &quadIMU);
//    Madgwick6DOF(&quadIMU_info, dt);
//    loopRate(2000); // do not exceed 2000Hz
//  }
//}

//void calibrateESCs() {
//  // DESCRIPTION: Used in void setup() to allow standard ESC calibration
//  // procedure with the radio to take place.
//  /*
//   *  Simulates the void loop(), but only for the purpose of providing throttle
//   * pass through to the motors, so that you can power up with throttle at full,
//   * let ESCs begin arming sequence, and lower throttle to zero. This function
//   * should only be uncommented when performing an ESC calibration.
//   */
//  while (true) {
//    prev_time = current_time;
//    current_time = micros();
//    dt = (current_time - prev_time) / 1000000.0;
//
//    digitalWrite(13, HIGH); // LED on to indicate we are not in main loop
//
//    getCommands();                       // Pulls current available radio commands
//    failSafe();                          // Prevent failures in event of bad receiver connection,
//                                         // defaults to failsafe values assigned in setup
//    getDesState();                       // Convert raw commands to normalized values based on
//                                         // saturated control limits
//    getIMUData(&quadIMU_info, &quadIMU); // Pulls raw gyro, accelerometer, and magnetometer data from
//                                         // IMU and LP filters to remove noise
//    Madgwick6DOF(&quadIMU_info, dt);
//    getDesState(); // Convert raw commands to normalized values based on
//                   // saturated control limits
//
//    m1_command_scaled = thro_des;
//    m2_command_scaled = thro_des;
//    m3_command_scaled = thro_des;
//    m4_command_scaled = thro_des;
//    scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125
//                     // protocol) and servo PWM commands to 0 to 180 (for servo
//                     // library)
//
//    // throttleCut(); //Directly sets motor commands to low based on state of ch5
//
//		#ifdef USE_ONESHOT
//    commandMotors();
//		#else
//    servo1.write(m1_command_PWM);
//    servo2.write(m2_command_PWM);
//    servo3.write(m3_command_PWM);
//    servo4.write(m4_command_PWM);
//		#endif
//
//    // printRadioData(); //Radio pwm values (expected: 1000 to 2000)
//
//    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to
//                    // 2000Hz by default
//  }
//}


//=====================================//
//========== SD Card Logging ==========//
//=====================================//
namespace datalogger {
	void PrintCSVHeader() {
		buffer.print("roll_imu");
		buffer.write(",");
		buffer.print("pitch_imu");
		buffer.write(",");
		buffer.print("yaw_imu");
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
		buffer.print("radio_ch1");
		buffer.write(",");
		buffer.print("radio_ch2");
		buffer.write(",");
		buffer.print("radio_ch3");
		buffer.write(",");
		buffer.print("radio_ch4");
		buffer.write(",");
		buffer.print("radio_ch5");
		buffer.write(",");
		buffer.print("radio_ch6");
		buffer.write(",");
		buffer.print("radio_ch7");
		buffer.write(",");
		buffer.print("radio_ch8");
		buffer.write(",");
		buffer.print("radio_ch9");
		buffer.write(",");
		buffer.print("radio_ch10");
		buffer.write(",");
		buffer.print("radio_ch11");
		buffer.write(",");
		buffer.print("radio_ch12");
		buffer.write(",");
		buffer.print("radio_ch13");
		buffer.write(",");
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
		buffer.print("kp_roll");
		buffer.write(",");
		buffer.print("ki_roll");
		buffer.write(",");
		buffer.print("kd_roll");
		buffer.write(",");
		buffer.print("kp_pitch");
		buffer.write(",");
		buffer.print("ki_pitch");
		buffer.write(",");
		buffer.print("kd_pitch");
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
		buffer.print("newPositionReceived");
		buffer.write(",");
		buffer.print("PositionX");
		buffer.write(",");
		buffer.print("PositionY");
		buffer.write(",");
		buffer.print("PositionZ");

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
		buffer.print(channel_1_pwm);
		buffer.write(",");
		buffer.print(channel_2_pwm);
		buffer.write(",");
		buffer.print(channel_3_pwm);
		buffer.write(",");
		buffer.print(channel_4_pwm);
		buffer.write(",");
		buffer.print(channel_5_pwm);
		buffer.write(",");
		buffer.print(channel_6_pwm);
		buffer.write(",");
		buffer.print(channel_7_pwm);
		buffer.write(",");
		buffer.print(channel_8_pwm);
		buffer.write(",");
		buffer.print(channel_9_pwm);
		buffer.write(",");
		buffer.print(channel_10_pwm);
		buffer.write(",");
		buffer.print(channel_11_pwm);
		buffer.write(",");
		buffer.print(channel_12_pwm);
		buffer.write(",");
		buffer.print(channel_13_pwm);
		buffer.write(",");
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
		buffer.print(Kp_roll_angle*pScaleRoll, 4);
		buffer.write(",");
		buffer.print(Ki_roll_angle*iScaleRoll, 4);
		buffer.write(",");
		buffer.print(Kd_roll_angle*dScaleRoll, 4);
		buffer.write(",");
		buffer.print(Kp_pitch_angle*pScalePitch, 4);
		buffer.write(",");
		buffer.print(Ki_pitch_angle*iScalePitch, 4);
		buffer.write(",");
		buffer.print(Kd_pitch_angle*dScalePitch, 4);
		buffer.write(",");
		buffer.print(Kp_yaw*pScaleYaw, 4);
		buffer.write(",");
		buffer.print(Ki_yaw*iScaleYaw, 4);
		buffer.write(",");
		buffer.print(Kd_yaw*dScaleYaw, 4);
		buffer.write(",");
		buffer.print(failureFlag);
		buffer.write(",");
		buffer.print(micros());
		buffer.write(",");
		buffer.print(newPositionReceived);
		buffer.write(",");
		buffer.print(mocapPosition[0], 6);
		buffer.write(",");
		buffer.print(mocapPosition[1], 6);
		buffer.write(",");
		buffer.print(mocapPosition[2], 6);
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
  telem.InitTelemetry();

  // Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
  channel_7_pwm = channel_7_fs;
  channel_8_pwm = channel_8_fs;
  channel_9_pwm = channel_9_fs;
  channel_10_pwm = channel_10_fs;
  channel_11_pwm = channel_11_fs;
  channel_12_pwm = channel_12_fs;
  channel_13_pwm = channel_13_fs;
  channel_14_pwm = channel_14_fs;

	bool IMU_initSuccessful = quadIMU.Init(&Wire);	
	Serial.print("IMU initialization successful: ");
	Serial.println(IMU_initSuccessful);

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
  //  calibrateESCs();
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
  if (!flightLoopStarted && channel_5_pwm < 1500) {
    flightLoopStarted = 1;
    telem.SetSystemState(MAV_STATE_ACTIVE);
    telem.SetSystemMode(MAV_MODE_MANUAL_ARMED);
  }

  // Sine sweep check
  if (channel_7_pwm > 1750) {
    conductSineSweep = 1;
  } else {
    conductSineSweep = 0;
    sineTime = 0;
  }

  // Write to SD card buffer
  if (SD_is_present && (current_time - print_counterSD) > LOG_INTERVAL_USEC) {
    print_counterSD = micros();
		datalogger::WriteBuffer();
    Serial.println("logged");
  }

  if (loopCount > 2000) {
    telem.SendHeartbeat();
    loopCount = 0;
  }
  if ((loopCount % 250) == 0) {
		telem.UpdateReceived();
    telem.SendAttitude(quadIMU_info.roll, quadIMU_info.pitch, 0.0f, quadIMU.GetGyroX(), quadIMU.GetGyroY(), 0.0f);
    telem.SendPIDGains_core(Kp_roll_angle * pScaleRoll, Ki_roll_angle * iScaleRoll, Kd_roll_angle * dScaleRoll);
  }
  loopCount++;

	// Check for a new position value
	if (telem.CheckForNewPosition(mocapPosition)) {
		newPositionReceived = true;
	} else {
		newPositionReceived = false;
	}

  // Get vehicle state
  quadIMU.Update(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick6DOF(quadIMU, &quadIMU_info, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  // Compute desired state based on radio inputs
  getDesState(); // Convert raw commands to normalized values based on saturated control limits

  if (useSerialAngleCommands) {
    // Overwrites axisToRotate in getDesState()
    setDesStateSerial(axisToRotate);
  }

  if (conductSineSweep) {
    // Overwrites axisToRotate in getDesState()
    performSineSweep(axisToRotate);
  }

  if (channel_8_pwm > 1250 && channel_8_pwm < 1750) {
    rollStep();
  }

  if (channel_8_pwm > 1750) {
    pitchStep();
  }

  getPScale();
  getIScale();
  getDScale();
  scaleBoth();

	// TODO: be better
	bool noIntegral = false;
	if (channel_1_pwm < 1060) {
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

      if (channel_14_pwm > 1500) {
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
