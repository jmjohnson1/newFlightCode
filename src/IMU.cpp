#include "IMU.h"

/**
 * The caller must pass in constant null shift values for the IMU.
*/
IMU::IMU(float accNSX, float accNSY, float accNSZ, float gyroNSX, float gyroNSY, float gyroNSZ){
	mpu6050_ = new MPU6050;
	accNullShiftX_ = accNSX;
	accNullShiftY_ = accNSY;
	accNullShiftZ_ = accNSZ;
	gyroNullShiftX_ = gyroNSX;
	gyroNullShiftY_ = gyroNSY;
	gyroNullShiftZ_ = gyroNSZ;

}

/**
 * We need a destructor here to make sure that the mpu6050 object doesn't stick around if
 * this IMU object is deleted.
*/
IMU::~IMU() {
	delete[] mpu6050_;
}

/**
 * Initializes the IMU(s) using the MPU6050 library. If there is an issue with the initialization, this function
 * returns false.
*/
bool IMU::Init(TwoWire *bus) {
	bus_ = bus;
	bus->begin();
	bus->setClock(1000000);

	mpu6050_->initialize();

	if (mpu6050_->testConnection() == false) {
		return false;
	}

	butterworth2_init(&accelFilter_1, accelFilterCutoff, sampleFreq);
	butterworth2_init(&accelFilter_2, accelFilterCutoff, sampleFreq);
	butterworth2_init(&accelFilter_3, accelFilterCutoff, sampleFreq);
	butterworth2_init(&gyroFilter_1, gyroFilterCutoff, sampleFreq);
	butterworth2_init(&gyroFilter_2, gyroFilterCutoff, sampleFreq);
	butterworth2_init(&gyroFilter_3, gyroFilterCutoff, sampleFreq);

	mpu6050_->setFullScaleGyroRange(GYRO_SCALE);
	mpu6050_->setFullScaleAccelRange(ACCEL_SCALE);
	return true;
}
/**
 * Updates the accelerometer and gyro values with new measurements taken via the MPU6050 library. Note that the
 * sensor is mounted with the y-axis facing left and z-axis facing up. For consistency with other algorithms, those
 * two axes are rotated (the sign is flipped) in order to use the conventional front, starboard, down body frame.
*/
void IMU::Update() {
	int16_t accX_raw, accY_raw, accZ_raw, gyroX_raw, gyroY_raw, gyroZ_raw;
	mpu6050_->getMotion6(&accX_raw, &accY_raw, &accZ_raw, &gyroX_raw, &gyroY_raw, &gyroZ_raw);
	// Accelerometer
	accX_ = accX_raw / ACCEL_SCALE_FACTOR * 9.80665; // m/s^2
	accY_ = -accY_raw / ACCEL_SCALE_FACTOR * 9.80665; // See description for why negative
	accZ_ = -accZ_raw / ACCEL_SCALE_FACTOR * 9.80665;
	// Remove the supplied null shift bias
	accX_ -= accNullShiftX_;
	accY_ -= accNullShiftY_;
	accZ_ -= accNullShiftZ_;
	// Lowpass filter for the accelerometer data.
	accX_ = butterworth2_apply(&accelFilter_1, accX_);
	accY_ = butterworth2_apply(&accelFilter_2, accY_);
	accZ_ = butterworth2_apply(&accelFilter_3, accZ_);

	// Gyro
	gyroX_ = gyroX_raw / GYRO_SCALE_FACTOR * 0.01745329252f; // rad/s
	gyroY_ = -gyroY_raw / GYRO_SCALE_FACTOR * 0.01745329252f;
	gyroZ_ = -gyroZ_raw / GYRO_SCALE_FACTOR * 0.01745329252f;
	// Remove the null shift bias
	gyroX_ -= gyroNullShiftX_;
	gyroY_ -= gyroNullShiftY_;
	gyroZ_ -= gyroNullShiftZ_;
	// Lowpass filter for the gyro data.
	gyroX_ = butterworth2_apply(&gyroFilter_1, gyroX_);
	gyroY_ = butterworth2_apply(&gyroFilter_2, gyroY_);
	gyroZ_ = butterworth2_apply(&gyroFilter_3, gyroZ_);

}
