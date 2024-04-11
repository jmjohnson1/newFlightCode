#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "Wire.h"
#include "MPU6050.h"
#include "eigen.h"
#include "UserDefines.h"
#include "filter.h"


// Setup gyro and accel full scale value selection and scale factor

#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

class IMU {
public:
	IMU(float accNSX, float accNSY, float accNSZ, float gyroNSX, float gyroNSY, float gyroNSZ);
	~IMU();

	void Update();
	bool Init(TwoWire *bus);
	float GetAccX() const {return accX_;}
	float GetAccY() const {return accY_;}
	float GetAccZ() const {return accZ_;}
	float GetGyroX() const {return gyroX_;}
	float GetGyroY() const {return gyroY_;}
	float GetGyroZ() const {return gyroZ_;}
	Eigen::Vector3f GetAcc() const {return Eigen::Vector3f(accX_, accY_, accZ_);}
	Eigen::Vector3f GetGyro() const {return Eigen::Vector3f(gyroX_, gyroY_, gyroZ_);}


	void SetAccNullShift(float NullShift[3]) {
		accNullShiftX_ = NullShift[0];
		accNullShiftY_ = NullShift[1];
		accNullShiftZ_ = NullShift[2];
	}
	void SetGyroNullShift(float NullShift[3]) {
		gyroNullShiftX_ = NullShift[0];
		gyroNullShiftY_ = NullShift[1];
		gyroNullShiftZ_ = NullShift[2];
	}

private:
	MPU6050 *mpu6050_ = NULL;
	TwoWire *bus_ = NULL;

	float accX_, accY_, accZ_;
	float gyroX_, gyroY_, gyroZ_;
	float accNullShiftX_, accNullShiftY_, accNullShiftZ_;
	float gyroNullShiftX_, gyroNullShiftY_, gyroNullShiftZ_;

	// Lowpass filters
	butterworth2_t accelFilter_1;
	butterworth2_t accelFilter_2;
	butterworth2_t accelFilter_3;
	butterworth2_t gyroFilter_1;
	butterworth2_t gyroFilter_2;
	butterworth2_t gyroFilter_3;
	float gyroFilterCutoff = 50;  // Hz
	float accelFilterCutoff = 50;  // Hz
	// FIXME: don't hardcode this
	float sampleFreq = 1000;  // Hz
};

#endif