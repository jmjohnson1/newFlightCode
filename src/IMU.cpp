#include "IMU.h"


IMU::IMU(float accNSX, float accNSY, float accNSZ, float gyroNSX, float gyroNSY, float gyroNSZ){
	mpu6050_ = new MPU6050;
	accNullShiftX_ = accNSX;
	accNullShiftY_ = accNSY;
	accNullShiftZ_ = accNSZ;
	gyroNullShiftX_ = gyroNSX;
	gyroNullShiftY_ = gyroNSY;
	gyroNullShiftZ_ = gyroNSZ;

	previousAccX_ = 0.0f;
	previousAccY_ = 0.0f;
	previousAccZ_ = 0.0f;
	previousGyroX_ = 0.0f;
	previousGyroY_ = 0.0f;
	previousGyroZ_ = 0.0f;
}

IMU::~IMU() {
	delete[] mpu6050_;
}

bool IMU::Init(TwoWire *bus) {
	bus_ = bus;
	bus->begin();
	bus->setClock(1000000);

	mpu6050_->initialize();

	if (mpu6050_->testConnection() == false) {
		return false;
	}


	mpu6050_->setFullScaleGyroRange(GYRO_SCALE);
	mpu6050_->setFullScaleAccelRange(ACCEL_SCALE);
	return true;
}

void IMU::Update() {
	int16_t accX_raw, accY_raw, accZ_raw, gyroX_raw, gyroY_raw, gyroZ_raw;
	mpu6050_->getMotion6(&accX_raw, &accY_raw, &accZ_raw, &gyroX_raw, &gyroY_raw, &gyroZ_raw);
	// Accelerometer
	accX_ = accX_raw / ACCEL_SCALE_FACTOR; // G's
	accY_ = accY_raw / ACCEL_SCALE_FACTOR; // G's
	accZ_ = accZ_raw / ACCEL_SCALE_FACTOR; // G's
	// Remove the supplied null shift bias
	accX_ -= accNullShiftX_;
	accY_ -= accNullShiftY_;
	accZ_ -= accNullShiftZ_;
	// Lowpass filter for the accelerometer data. This was done in dRehmflight, but I don't feel great
	// about it. Leaving it here for now.
	accX_ = (1.0f - B_accel_)*previousAccX_ + B_accel_*accX_;
	accY_ = (1.0f - B_accel_)*previousAccY_ + B_accel_*accY_;
	accZ_ = (1.0f - B_accel_)*previousAccZ_ + B_accel_*accZ_;
	previousAccX_ = accX_;
	previousAccY_ = accY_;
	previousAccZ_ = accZ_;

	// Gyro
	gyroX_ = gyroX_raw / GYRO_SCALE_FACTOR;
	gyroY_ = gyroY_raw / GYRO_SCALE_FACTOR;
	gyroZ_ = gyroZ_raw / GYRO_SCALE_FACTOR;
	// Remove the null shift bias
	gyroX_ -= gyroNullShiftX_;
	gyroY_ -= gyroNullShiftY_;
	gyroZ_ -= gyroNullShiftZ_;
	// Lowpass filter for the gyro data. This was done in dRehmflight, but I don't feel great
	// about it. Leaving it here for now.
	gyroX_ = (1.0f - B_gyro_)*previousGyroX_ + B_gyro_*gyroX_;
	gyroY_ = (1.0f - B_gyro_)*previousGyroY_ + B_gyro_*gyroY_;
	gyroZ_ = (1.0f - B_gyro_)*previousGyroZ_ + B_gyro_*gyroZ_;
	previousGyroX_ = gyroX_;
	previousGyroY_ = gyroY_;
	previousGyroZ_ = gyroZ_;
}