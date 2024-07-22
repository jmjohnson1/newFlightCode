#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "Wire.h"
#include "MPU6050.h"
#include "eigen.h"
#include "UserDefines.h"
#include "filter.h"
#include "BMI088.h"


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

class Generic_IMU {
public:
    Generic_IMU(const Eigen::Vector3f &accNullShift, const Eigen::Vector3f &gyroNullShift);

	float GetAccX() const {return accX_;}
	float GetAccY() const {return accY_;}
	float GetAccZ() const {return accZ_;}
	float GetGyroX() const {return gyroX_;}
	float GetGyroY() const {return gyroY_;}
	float GetGyroZ() const {return gyroZ_;}

	const float* GetAccXPtr() {return &(accX_);}
	const float* GetAccYPtr() {return &(accY_);}
	const float* GetAccZPtr() {return &(accZ_);}
	const float* GetGyroXPtr() {return &(gyroX_);}
	const float* GetGyroYPtr() {return &(gyroY_);}
	const float* GetGyroZPtr() {return &(gyroZ_);}

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

	bool Init();

protected:
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
	float sampleFreq = 2000;  // Hz

};

class mpu6050 : public Generic_IMU {
public:
	mpu6050(const Eigen::Vector3f &accNullShift, const Eigen::Vector3f &gyroNullShift);
	~mpu6050();
	void Update();
	bool Init(TwoWire *i2c);

private:
	MPU6050 *mpu6050_ = NULL;
	TwoWire *i2c_ = NULL;
};


class bmi088 : public Generic_IMU {
public:
  bmi088(const Eigen::Vector3f &accNullShift,
         const Eigen::Vector3f &gyroNullShift, SPIClass &spi, uint8_t accel_cs,
         uint8_t gyro_cs, const int accRange, const int gyroRange);
  ~bmi088();
  bool Init();
  void Update();

private:
	Bmi088 *bmi088_ = NULL;
	Bmi088::AccelRange accRange = Bmi088::ACCEL_RANGE_6G;
	Bmi088::GyroRange gyroRange = Bmi088::GYRO_RANGE_500DPS;

};
#endif
