#ifndef USER_DEFINE_H
#define USER_DEFINE_H

/**Uncomment only one full scale gyro range (deg/sec)**/
#define GYRO_250DPS // Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

/**Uncomment only one full scale accelerometer range (G's)**/
#define ACCEL_2G // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

/**Define whether tuning RIP gains or core PID gains**/
// #define TUNE_RIP
#define TUNE_CORE

/**Use OneShot125 comment out to use PWM**/ //TODO: Make PWM work again
#define USE_ONESHOT

#endif
