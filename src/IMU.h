#ifndef IMU_H
#define IMU_H

#include "MPU6050.h"

class IMU {
public:
	IMU(float accNSX, float accNSY, float accNSZ, float gyroNSX, float gyroNSY, float gyroNSZ);
	~IMU();

	void Update();
	float GetAccX() {return accX_;}
	float GetAccY() {return accY_;}
	float GetAccZ() {return accZ_;}
	float GetGyroX() {return gyroX_;}
	float GetGyroY() {return gyroY_;}
	float GetGyroZ() {return gyroZ_;}

private:
	MPU6050 *mpu6050_ = NULL;

	float accX_, accY_, accZ_;
	float gyroX_, gyroY_, gyroZ_;
	float accNullShiftX_, accNullShiftY_, accNullShiftZ_;
	float gyroNullShiftX_, gyroNullShiftY_, gyroNullShiftZ_;
};

#endif
