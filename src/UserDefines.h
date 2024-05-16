#ifndef USER_DEFINE_H
#define USER_DEFINE_H

// Type of battery
//#define BAT_3S
#define BAT_4S

/**Uncomment only one full scale gyro range (deg/sec)**/
// #define GYRO_250DPS
#define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

/**Uncomment only one full scale accelerometer range (G's)**/
// #define ACCEL_2G
 #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

/**Define whether tuning RIP gains or core PID gains**/
// #define TUNE_RIP
#define TUNE_CORE

/**Don't change this for now**/
#define USE_MADGWICK
#define USE_EKF

/**For position control**/
#define USE_POSITION_CONTROLLER
#ifdef USE_POSITION_CONTROLLER
	#ifndef USE_EKF
		#error "Position controller requires USE_EKF to be defined!"
	#endif
#endif

/**Use OneShot125 comment out to use PWM**/
#define USE_ONESHOT

/**Uncomment to have access to step inputs and sine sweeps**/
// #define TEST_STAND
// Parameters for sine sweep
#define SWP_MIN_FREQ 0.5 /*Hz*/
#define SWP_MAX_FREQ 3.0 /*Hz*/
#define SWP_TIME 120 /*Duration of sweep (seconds)*/
#define SWP_AMPLITUDE 10 /*degrees*/
// Parameters for steps
#define STP_ANGLE 10 /*degrees*/

// Thrust sweep
#define THO_MAX_TIME 60

/**Uncomment to be able to live tune the PID gains with the transmitter**/
// #define PID_TUNING
// The limits of the gain tuning knob are (1 +/- MAX_SCALE_FACTOR). Probably shouldn't be greater
// than 1, but the output is not allowed to be negative in any case.
// #define MAX_SCALE_FACTOR 9.0f;
constexpr float MAX_SCALE_FACTOR = 0.5f;

#endif
