# Optionally glob, but only for CMake 3.12 or later:
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/include/*.h")

# Make an automatic library - will be static or dynamic based on user setting
add_library(flightSource 
	controller.cpp
	datalogger.cpp
	EKF.cpp
	filter.cpp
	IMU.cpp
	madgwick.cpp
	flightRunner.cpp
	motors.cpp
	nav-functions.cpp
	navHandler.cpp
	radio.cpp
	serialDebug.cpp
	statFun.cpp
	telemetry.cpp
	testing.cpp

	${HEADER_LIST}
)

# We need this directory, and users of our library will need it too
target_include_directories(flightSource PUBLIC 
"${DRONE_Code_SOURCE_DIR}/include"
)

# This depends on (header only) boost
target_link_libraries(flightSource PRIVATE 
	TeensyLib
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	EEPROM
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
)
