list(APPEND COMMON_LIBS
	Teensy4
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
	Filter
)

## Sensor Fusion ##
add_library(sensorFusion
	sensorFusion/EKF.cpp
	sensorFusion/madgwick.cpp
	sensorFusion/nav-functions.cpp

	sensorFusion/EKF.h
	sensorFusion/madgwick.h
	sensorFusion/nav-functions.h
)

target_include_directories(sensorFusion PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sensorFusion>
)

target_link_libraries(sensorFusion 
	PRIVATE
	Teensy4
	PUBLIC
	Eigen
)

## PID Controllers ##
add_library(PIDController
	${CMAKE_CURRENT_SOURCE_DIR}/controllers/dcmPID.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/controllers/eulerPID.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/controllers/dcmPID.h
	${CMAKE_CURRENT_SOURCE_DIR}/controllers/eulerPID.h
)

target_include_directories(PIDController PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controllers>
)

target_link_libraries(PIDController 
	PUBLIC
	Teensy4
	Eigen
	Filter
	PRIVATE
	TeensyLog
)

