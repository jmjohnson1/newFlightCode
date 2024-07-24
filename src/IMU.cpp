#include "IMU.h"


Generic_IMU::Generic_IMU(const Eigen::Vector3f &accNullShift, const Eigen::Vector3f &gyroNullShift){
	accNullShiftX_ = accNullShift[0];
	accNullShiftY_ = accNullShift[1];
	accNullShiftZ_ = accNullShift[2];
	gyroNullShiftX_ = gyroNullShift[0];
	gyroNullShiftY_ = gyroNullShift[1];
	gyroNullShiftZ_ = gyroNullShift[2];

}

bool Generic_IMU::Init() {
	butterworth2_init(&accelFilter_1, accelFilterCutoff, sampleFreq);
	butterworth2_init(&accelFilter_2, accelFilterCutoff, sampleFreq);
	butterworth2_init(&accelFilter_3, accelFilterCutoff, sampleFreq);
	butterworth2_init(&gyroFilter_1, gyroFilterCutoff, sampleFreq);
	butterworth2_init(&gyroFilter_2, gyroFilterCutoff, sampleFreq);
	butterworth2_init(&gyroFilter_3, gyroFilterCutoff, sampleFreq);
	return true;
}

/**
 * The caller must pass in constant null shift values for the IMU.
*/
mpu6050::mpu6050(const Eigen::Vector3f &accNullShift, const Eigen::Vector3f &gyroNullShift) : Generic_IMU::Generic_IMU(accNullShift, gyroNullShift){
	mpu6050_ = new MPU6050;

}

/**
 * We need a destructor here to make sure that the mpu6050 object doesn't stick around if
 * this IMU object is deleted.
*/
mpu6050::~mpu6050() {
	delete[] mpu6050_;
}

/**
 * Initializes the IMU(s) using the MPU6050 library. If there is an issue with the initialization, this function
 * returns false.
*/
bool mpu6050::Init(TwoWire *i2c) {
	i2c_ = i2c;
	i2c->begin();
	i2c->setClock(1000000);

	Generic_IMU::Init();
	mpu6050_->initialize();

	if (mpu6050_->testConnection() == false) {
		return false;
	}

	mpu6050_->setFullScaleGyroRange(GYRO_SCALE);
	mpu6050_->setFullScaleAccelRange(ACCEL_SCALE);
	return true;
}
/**
 * Updates the accelerometer and gyro values with new measurements taken via the MPU6050 library. Note that the
 * sensor is mounted with the y-axis facing left and z-axis facing up. For consistency with other algorithms, those
 * two axes are rotated (the sign is flipped) in order to use the conventional front, starboard, down body frame.
*/
void mpu6050::Update() {
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


bmi088::bmi088(const Eigen::Vector3f &accNullShift, const Eigen::Vector3f &gyroNullShift, SPIClass &spi, uint8_t accel_cs,uint8_t gyro_cs,
const int accRange, const int gyroRange) : Generic_IMU::Generic_IMU(accNullShift, gyroNullShift){
	bmi088_ = new Bmi088(spi, accel_cs, gyro_cs);

}

bmi088::~bmi088() {
	delete[] bmi088_;
}

bool bmi088::Init() {
	Generic_IMU::Init();
	bmi088_->begin();
	bmi088_->setRange(accRange, gyroRange);
	return true;
}

void bmi088::Update() {
	// Reading sensor data from BMI088.h defined functions
	bmi088_->readSensor();
	accX_ = bmi088_->getAccelX_mss();
	accY_ = bmi088_->getAccelY_mss();
	accZ_ = bmi088_->getAccelZ_mss();
	gyroX_ = bmi088_->getGyroX_rads();
	gyroY_ = bmi088_->getGyroY_rads();
	gyroZ_ = bmi088_->getGyroZ_rads();

	// Applying null shift bias
	accX_ -= accNullShiftX_;
	accY_ -= accNullShiftY_;
	accZ_ -= accNullShiftZ_;

	gyroX_ -= gyroNullShiftX_;
	gyroY_ -= gyroNullShiftY_;
	gyroZ_ -= gyroNullShiftZ_;

	// Applying Filter
	accX_ = butterworth2_apply(&accelFilter_1, accX_);
	accY_ = butterworth2_apply(&accelFilter_2, accY_);
	accZ_ = butterworth2_apply(&accelFilter_3, accZ_);

	gyroX_ = butterworth2_apply(&gyroFilter_1, gyroX_);
	gyroY_ = butterworth2_apply(&gyroFilter_2, gyroY_);
	gyroZ_ = butterworth2_apply(&gyroFilter_3, gyroZ_);

}
