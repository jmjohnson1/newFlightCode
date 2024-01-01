#include "IMU.h"

IMU::IMU(float accNSX, float accNSY, float accNSZ, float gyroNSX, float gyroNSY, float gyroNSZ){
	mpu6050_ = new MPU6050;
	accNullShiftX_ = accNSX;
	accNullShiftY_ = accNSY;
	accNullShiftZ_ = accNSZ;
	gyroNullShiftX_ = gyroNSX;
	gyroNullShiftY_ = gyroNSY;
	gyroNullShiftZ_ = gyroNSZ;
}

IMU::~IMU() {
	delete[] mpu6050_;
}
