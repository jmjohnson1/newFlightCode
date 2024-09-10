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
	libFilter
)

## Sensor Fusion ##
add_library(sensorFusion
	EKF.cpp
	madgwick.cpp
	nav-functions.cpp
)

target_include_directories(sensorFusion PUBLIC
	"${DRONE_Code_SOURCE_DIR}/include"
)

target_link_libraries(sensorFusion PRIVATE
	${COMMON_LIBS}
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
	libFilter
	PRIVATE
	sensorFusion
)

